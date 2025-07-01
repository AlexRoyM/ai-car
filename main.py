import os
import uuid
import threading
import signal
import sys
import json
import time
from flask import Flask, render_template, request, jsonify, Response
from werkzeug.utils import secure_filename
import pygame

import config
import state
from unified_camera_handler import UnifiedCamera
from voice_handler import stop_speech_playback
# 根据配置选择导入哪个处理器
if config.LLM_PROVIDER == "OPENAI":
    from openai_handler import process_message_and_get_reply_openai as process_message_and_get_reply
    print("--- [模式] 当前使用 OpenAI API ---")
else:
    from llm_handler import process_message_and_get_reply
    print("--- [模式] 当前使用 LM Studio ---")
from robot_control import available_functions

# --- ROS Integration ---
try:
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy
    from nav_msgs.msg import Odometry
    import robot_control 
    state.ros_enabled = True
except ImportError:
    print("警告：无法导入 rospy。小车控制功能将被禁用。")
    state.ros_enabled = False
    # 创建假的类以便代码在没有ROS的环境下也能运行不报错
    class Joy: pass
    class Twist: pass

# --- Flask App Initialization ---
app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = None # 将在会话开始时设置

# --- Helper Functions ---
def start_new_session():
    """初始化或重置会话"""
    state.conversation_history = []
    state.current_session_id = str(uuid.uuid4())
    session_dir = os.path.join('static', 'sessions', state.current_session_id)
    
    state.UPLOAD_FOLDER = os.path.join(session_dir, 'uploads')
    state.TEMP_AUDIO_PATH = os.path.join(session_dir, 'temp_audio.mp3')
    
    os.makedirs(state.UPLOAD_FOLDER, exist_ok=True)
    app.config['UPLOAD_FOLDER'] = state.UPLOAD_FOLDER
    print(f"新会话已启动，ID: {state.current_session_id}")

def get_video_frames():
    """视频流生成器"""
    if not state.camera_handler:
        return
    
    last_frame_time = 0
    frame_interval = 1.0 / 30  # 限制为15fps
    
    while True:
        current_time = time.time()
        if current_time - last_frame_time < frame_interval:
            time.sleep(0.01)
            continue
            
        frame = state.camera_handler.get_jpeg_frame()
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            last_frame_time = current_time


# --- ROS Joy Callback ---
def joy_callback(data):
    """监听手柄消息以实现快捷键"""
    # L2+R2 停止播放
    l2_pressed = data.axes[config.L2_AXIS_ID] < -0.1
    r2_pressed = data.axes[config.R2_AXIS_ID] < -0.1
    if l2_pressed and r2_pressed:
        if state.joy_stop_tts_lock.acquire(blocking=False):
            try:
                stop_speech_playback()
            finally:
                threading.Timer(1.0, lambda: state.joy_stop_tts_lock.release()).start()
        return

    # L1+R1 拍照发送
    l1_pressed = data.buttons[config.L1_BUTTON_ID] == 1
    r1_pressed = data.buttons[config.R1_BUTTON_ID] == 1
    if not (l1_pressed and r1_pressed):
        return

    tts_is_playing = state.tts_thread and state.tts_thread.is_alive()
    if state.is_autonomous_mode or tts_is_playing:
        return

    if state.joy_trigger_lock.acquire(blocking=False):
        print("--- [快捷键] L1+R1 已触发，后台处理中... ---")
        
        def process_in_background():
            try:
                if not state.camera_handler: return
                prompt = "请根据这张图片进行分析。"
                    
                # 使用app上下文，确保在线程中能访问app.config
                with app.app_context():
                    # Add indentation to this line
                    image_webpath, image_filepath = state.camera_handler.capture_image_to_file(app.config['UPLOAD_FOLDER'], state.current_session_id)

                if not image_filepath:
                    print("[快捷键] 拍照失败，无法发送。")
                    return

                # 在主线程中调用llm处理函数，因为它会修改全局状态
                # Flask的werkzeug库已经处理了多线程请求的上下文问题
                process_message_and_get_reply(prompt, image_filepath, image_webpath, autonomous_mode=False)

            finally:
                threading.Timer(2.0, lambda: state.joy_trigger_lock.release()).start()

        threading.Thread(target=process_in_background).start()

# --- Flask Routes ---
@app.route('/')
def index():
    return render_template('index.html', history=state.conversation_history, is_muted=state.is_muted, slam_url=config.SLAM_PAGE_URL)

@app.route('/send_message', methods=['POST'])
def handle_send_message():
    prompt = request.form.get('prompt', '')
    image_file = request.files.get('image')
    autonomous_mode = request.form.get('autonomous_mode', 'false').lower() == 'true'
    image_filepath, image_webpath = None, None
    
    if image_file and image_file.filename:
        filename = secure_filename(image_file.filename)
        image_filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        image_file.save(image_filepath)
        image_webpath = os.path.join('static', 'sessions', state.current_session_id, 'uploads', filename).replace('\\', '/')
        
    response_data = process_message_and_get_reply(prompt, image_filepath, image_webpath, autonomous_mode)
    return jsonify(response_data)

@app.route('/capture_and_send', methods=['POST'])
def handle_capture_and_send():
    if not state.camera_handler: return jsonify({"error": "摄像头未初始化"}), 500
    
    prompt = request.form.get('prompt', '')
    autonomous_mode = request.form.get('autonomous_mode', 'false').lower() == 'true'
    
    image_webpath, image_filepath = state.camera_handler.capture_image_to_file(app.config['UPLOAD_FOLDER'], state.current_session_id) 
    if not image_filepath: return jsonify({"error": "拍照失败"}), 400
    
    response_data = process_message_and_get_reply(prompt, image_filepath, image_webpath, autonomous_mode)
    return jsonify(response_data)

@app.route('/video_feed')
def video_feed():
    return Response(get_video_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/get_history')
def get_history():
    return jsonify({"history": state.conversation_history})

@app.route('/toggle_mute', methods=['POST'])
def toggle_mute():
    state.is_muted = not state.is_muted
    return jsonify({"is_muted": state.is_muted})

@app.route('/clear_history', methods=['POST'])
def clear_history():
    start_new_session()
    return jsonify({"success": True})

@app.route('/toggle_autonomy_backend', methods=['POST'])
def toggle_autonomy_backend():
    state.is_autonomous_mode = request.json.get('is_autonomous', False)
    print(f"自主模式状态更新为: {state.is_autonomous_mode}")
    return jsonify({"success": True})

@app.route('/tts_status')
def tts_status():
    is_playing = state.tts_thread and state.tts_thread.is_alive()
    return jsonify({"is_playing": is_playing})

@app.route('/stop_tts', methods=['POST'])
def stop_tts():
    stopped = stop_speech_playback()
    return jsonify({"success": stopped})


# --- Initialization and Cleanup ---
def initialize_app():
    """初始化所有服务"""
    # 初始化Pygame Mixer
    pygame.init()
    pygame.mixer.init()

    # 初始化统一摄像头
    try:
        # 尝试初始化我们新的统一摄像头处理器
        state.camera_handler = UnifiedCamera()
        # UnifiedCamera 的构造函数会在失败时自动将 state.ros_enabled 设置为 False
    except Exception as e:
        print(f"严重错误: 统一摄像头初始化失败: {e}。摄像头与自主导航功能将不可用。")
        state.camera_handler = None
        state.ros_enabled = False

    # 初始化ROS (现在这个判断是可靠的)
    if state.ros_enabled:
        print("正在初始化ROS节点...")
        rospy.init_node('ai_car_controller', anonymous=True, disable_signals=True)
        state.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, joy_callback, queue_size=1)
        # 订阅里程计话题以实现精确旋转
        odom_topic = "/odometry/filtered"
        rospy.Subscriber(odom_topic, Odometry, robot_control.odom_callback, queue_size=1)
        print(f"* 里程计监听器已启动，订阅话题: {odom_topic}")
        print("* ROS节点和手柄监听器已启动。")
    
    # 开始第一个会话
    start_new_session()

def cleanup(sig, frame):
    """程序退出前的清理工作"""
    print("\n接收到关停信号 (Ctrl+C)... 正在清理资源...")
    if state.camera_handler: 
        state.camera_handler.release()
    if pygame.mixer.get_init():
        pygame.mixer.quit()
    if pygame.get_init():
        pygame.quit()
    if state.ros_enabled:
        # 发布一个空的Twist消息确保小车停止
        if state.cmd_vel_pub:
            state.cmd_vel_pub.publish(Twist())
        rospy.signal_shutdown("Flask服务器关闭")
        print("ROS节点已请求关闭。")
    print("清理完毕，程序退出。")
    sys.exit(0)
