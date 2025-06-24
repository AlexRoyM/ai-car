import threading
import math

# --- 会话与文件管理 ---
current_session_id = None
conversation_history = []
UPLOAD_FOLDER = None
TEMP_AUDIO_PATH = None

# --- 应用状态 ---
is_muted = False
tts_thread = None
is_autonomous_mode = False

# --- 线程锁 ---
joy_trigger_lock = threading.Lock()
joy_stop_tts_lock = threading.Lock()

# --- ROS & 小车控制状态 ---
# 将ROS相关对象放在这里，方便跨模块访问
ros_enabled = False
cmd_vel_pub = None
car_control_lock = threading.Lock()
odom_lock = threading.Lock()
# 当前姿态和位置
current_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
# 开始移动或转向时的初始姿态和位置
initial_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}


def get_pose_diff(pose1, pose2):
    """计算两个姿态之间的距离和角度差"""
    dx = pose2['x'] - pose1['x']
    dy = pose2['y'] - pose1['y']
    distance = math.sqrt(dx**2 + dy**2)
    
    # 计算角度差，处理角度环绕问题(-pi, pi)
    d_theta = pose2['theta'] - pose1['theta']
    while d_theta > math.pi: d_theta -= 2 * math.pi
    while d_theta < -math.pi: d_theta += 2 * math.pi
    
    return distance, d_theta

