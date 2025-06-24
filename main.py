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
from flask_socketio import SocketIO

# 导入自定义模块
import config
import state
from camera_handler import Camera
from voice_handler import stop_speech_playback
from llm_handler import process_message_and_get_reply

# --- ROS Integration ---
try:
    import rospy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy
    from nav_msgs.msg import Odometry
    from tf.transformations import euler_from_quaternion
    state.ros_enabled = True
except ImportError:
    print("警告：无法导入 rospy。小车控制功能将被禁用。")
    state.ros_enabled = False
    class Joy: pass
    class Twist: pass
    class Odometry: pass

# --- Flask App & SocketIO Initialization ---
app = Flask(__name__)
app.config['UPLOAD_FOLDER'] = None
# 使用 eventlet 作为异步模式
socketio = SocketIO(app, async_mode='eventlet')

# --- Global Instances ---
camera = None

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
    # 通过WebSocket通知前端清空历史
    socketio.emit('clear_history')


def get_video_frames():
    """视频流生成器"""
    if not camera: return
    while True:
        frame = camera.get_jpeg_frame()
        if frame:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        socketio.sleep(1/30)


# --- ROS Callbacks ---
def odom_callback(msg: Odometry):
    """监听里程计消息以实现闭环控制"""
    with state.odom_lock:
        state.current_pose['x'] = msg.pose.pose.position.x
        state.current_pose['y'] = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, state.current_pose['theta'] = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])

def joy_callback(data: Joy):
    # ... (此函数无变化)
    l2_pressed = data.axes[config.L2_AXIS_ID] < -0.1
    r2_pressed = data.axes[config.R2_AXIS_ID] < -0.1
    if l2_pressed and r2_pressed:
        if state.joy_stop_tts_lock.acquire(blocking=False):
            try:
                stop_speech_playback()
            finally:
                threading.Timer(1.0, lambda: state.joy_stop_tts_lock.release()).start()
        return

    l1_pressed = data.buttons[config.L1_BUTTON_ID] == 1
    r1_pressed = data.buttons[config.R1_BUTTON_ID] == 1
    if not (l1_pressed and r1_pressed): return

    tts_is_playing = state.tts_thread and state.tts_thread.is_alive()
    if state.is_autonomous_mode or tts_is_playing: return

    if state.joy_trigger_lock.acquire(blocking=False):
        print("--- [快捷键] L1+R1 已触发，后台处理中... ---")
        
        def process_in_background():
            try:
                if not camera: return
                prompt = "请根据这张图片进行分析。"
                with app.app_context():
                    image_webpath, image_filepath = camera.capture_image_to_file(app.config['UPLOAD_FOLDER'], state.current_session_id)
                if not image_filepath:
                    print("[快捷键] 拍照失败，无法发送。")
                    return
                process_message_and_get_reply(prompt, image_filepath, image_webpath, autonomous_mode=False)
            finally:
                threading.Timer(2.0, lambda: state.joy_trigger_lock.release()).start()

        socketio.start_background_task(target=process_in_background)


# --- Flask Routes ---
@app.route('/')
def index():
    # 页面加载时传递完整历史记录
    return render_template('index.html', history=state.conversation_history, is_muted=state.is_muted)

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
        
    socketio.start_background_task(process_message_and_get_reply, prompt, image_filepath, image_webpath, autonomous_mode)
    return jsonify({"success": True, "message": "请求已在后台处理"})

@app.route('/capture_and_send', methods=['POST'])
def handle_capture_and_send():
    if not camera: return jsonify({"error": "摄像头未初始化"}), 500
    prompt = request.form.get('prompt', '')
    autonomous_mode = request.form.get('autonomous_mode', 'false').lower() == 'true'
    image_webpath, image_filepath = camera.capture_image_to_file(app.config['UPLOAD_FOLDER'], state.current_session_id)
    if not image_filepath: return jsonify({"error": "拍照失败"}), 400
    
    socketio.start_background_task(process_message_and_get_reply, prompt, image_filepath, image_webpath, autonomous_mode)
    return jsonify({"success": True, "message": "请求已在后台处理"})

@app.route('/video_feed')
def video_feed():
    return Response(get_video_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# --- 移除轮询路由 /get_history 和 /tts_status ---

# --- SocketIO aevents ---
@socketio.on('connect')
def handle_connect():
    print('客户端已连接')

@socketio.on('disconnect')
def handle_disconnect():
    print('客户端已断开')

@socketio.on('toggle_mute_req')
def handle_toggle_mute():
    state.is_muted = not state.is_muted
    socketio.emit('mute_status', {'is_muted': state.is_muted})

@socketio.on('clear_history_req')
def handle_clear_history():
    start_new_session() # 内部会emit 'clear_history'

@socketio.on('toggle_autonomy_req')
def handle_toggle_autonomy(data):
    state.is_autonomous_mode = data.get('is_autonomous', False)
    print(f"自主模式状态更新为: {state.is_autonomous_mode}")
    # 可以选择性地向所有客户端广播这个状态变化
    # socketio.emit('autonomy_status_changed', {'is_autonomous': state.is_autonomous_mode})

@socketio.on('stop_tts_req')
def handle_stop_tts():
    stop_speech_playback()

# --- Initialization and Cleanup ---
def initialize_app():
    """初始化所有服务"""
    global camera
    pygame.init()
    pygame.mixer.init()
    
    try:
        camera = Camera()
    except IOError as e:
        print(f"严重错误: 摄像头初始化失败: {e}。部分功能将不可用。")
        camera = None

    if state.ros_enabled:
        print("正在初始化ROS节点...")
        rospy.init_node('ai_car_controller', anonymous=True, disable_signals=True)
        state.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, joy_callback, queue_size=1)
        # 订阅里程计信息
        rospy.Subscriber('/odom', Odometry, odom_callback, queue_size=1)
        print("* ROS节点、手柄和里程计监听器已启动。")
    
    start_new_session()

def cleanup(sig, frame):
    # ... (此函数无变化)
    print("\n接收到关停信号 (Ctrl+C)... 正在清理资源...")
    if camera: camera.release()
    if pygame.mixer.get_init(): pygame.mixer.quit()
    if pygame.get_init(): pygame.quit()
    if state.ros_enabled:
        if state.cmd_vel_pub: state.cmd_vel_pub.publish(Twist())
        rospy.signal_shutdown("Flask服务器关闭")
        print("ROS节点已请求关闭。")
    print("清理完毕，程序退出。")
    sys.exit(0)
