# --- 服务与端口配置 ---
FLASK_PORT = 17958
SLAM_PAGE_URL = "http://192.168.10.102:3000"

# --- LLM 服务提供商选择 ---
# 可选项: "LM_STUDIO" 或 "OPENAI"
LLM_PROVIDER = "OPENAI"  

# --- OpenAI API 配置 ---
OPENAI_API_KEY = "AIzaSyCrtI8Mw1y5xAKQCVTWgA6BkUd4ftZX-G0" # 这特么我的api-key
OPENAI_MODEL_NAME = "gemini-2.5-flash-lite-preview-06-17" 
OPENAI_BASE_URL = "https://generativelanguage.googleapis.com/v1beta/openai/" # gemini2.5

# --- LM Studio API 与模型配置 ---
LM_STUDIO_URL = "http://199.168.137.135:9568/v1/chat/completions"
MODEL_NAME = "lmstudio-community/Qwen2.5-7B-Instruct-GGUF"
MODEL_MAX_TOKENS = 4096
MODEL_TEMPERATURE = 0.7

# --- 语音配置 ---
VOICES = {'female': 'zh-CN-XiaoxiaoNeural', 'male': 'zh-CN-YunxiNeural'}
SELECTED_VOICE = VOICES['male']

# --- 小车物理参数 ---
MOVE_SPEED = 0.3
ANGULAR_SPEED = 0.5
MAX_MOVE_DISTANCE = 2.0   # 单次指令最大移动距离(米)
MAX_TURN_ANGLE = 28.0     # 单次指令最大转向角度(度)


# --- ROS 手柄按钮ID定义 ---
L1_BUTTON_ID = 4
R1_BUTTON_ID = 5
L2_AXIS_ID = 2
R2_AXIS_ID = 5
