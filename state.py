import threading

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
tts_lock = threading.Lock()
# --- ROS & 小车控制状态 ---
# 将ROS相关对象放在这里，方便跨模块访问
ros_enabled = False
camera_handler = None
cmd_vel_pub = None
car_control_lock = threading.Lock()
# --- 用于“原路返回”功能的动作记忆 ---
last_executed_actions = []