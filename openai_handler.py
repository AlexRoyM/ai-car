# openai_handler.py

import openai
import json
import base64
import threading
import cv2
import config
import state
import robot_control
from voice_handler import speak_text_threaded

# 初始化 OpenAI 客户端
# 如果 config.OPENAI_API_KEY 提供了值，则使用它，否则客户端会自动从环境变量读取
client = openai.OpenAI(api_key=config.OPENAI_API_KEY, base_url=config.OPENAI_BASE_URL)
LOCATE_OBJECT_TOOL = [
    {
        "type": "function",
        "function": {
            "name": "locate_object_in_image",
            "description": "根据用户的自然语言描述，在图像中定位目标物体，并返回其中心点的像素坐标。",
            "parameters": {
                "type": "object",
                "properties": {
                    "object_description": {
                        "type": "string",
                        "description": "用户想要定位的物体的简短描述, 例如 '那个粉色的物体' 或 '左边的瓶子'。",
                    },
                    "center_x": {
                        "type": "integer",
                        "description": "识别出的物体边界框的中心点 x 坐标 (像素)。"
                    },
                    "center_y": {
                        "type": "integer",
                        "description": "识别出的物体边界框的中心点 y 坐标 (像素)。"
                    }
                },
                "required": ["object_description", "center_x", "center_y"]
            }
        }
    }
]
def image_to_base64(image_path):
    """将图片文件转换为Base64编码"""
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

def send_to_openai(messages_history, tools=None):
    """向 OpenAI API 发送请求"""
    try:
        # OpenAI 的工具选择逻辑与LM Studio不同，这里我们用 tool_choice="auto"
        # 它会自动判断是否需要调用工具
        response = client.chat.completions.create(
            model=config.OPENAI_MODEL_NAME,
            messages=messages_history,
            tools=tools,
            tool_choice="auto", 
            temperature=config.MODEL_TEMPERATURE,
            max_tokens=config.MODEL_MAX_TOKENS,
        )
        return response
    except Exception as e:
        print(f"OpenAI API 请求错误: {e}")
        # 返回一个字典以保持与现有错误处理逻辑的兼容性
        return {"error": f"调用 OpenAI API 失败: {e}"}

def process_message_and_get_reply_openai(prompt, image_filepath=None, image_webpath=None, autonomous_mode=False):
    """处理用户输入，与OpenAI LLM交互并返回结果的完整流程"""
    # 如果是带图像的自主模式，执行新的精确导航流程
    if autonomous_mode and image_filepath and state.depth_camera_handler:
        print("--- 进入精确自主导航模式 ---")
        
        # 1. 准备给LLM的消息和指令
        system_like_instruction = (
            "你是一个AI小车的视觉分析助手。你的任务是严格按照用户的指令，在提供的图片中找到目标物体。"
            "一旦找到，你必须且只能使用 `locate_object_in_image` 工具来返回该物体中心点的像素坐标 (center_x, center_y)。"
            "不要进行任何对话、道歉或提供额外信息。直接调用工具即可。"
            "例如，如果用户说 '靠近那个粉色的盒子'，你就找到粉色盒子并返回其坐标。"
        )
        final_prompt = system_like_instruction + "\n\n用户指令是：" + prompt
        
        current_user_content = [{"type": "text", "text": final_prompt}]
        base64_image = image_to_base64(image_filepath)
        current_user_content.append({
            "type": "image_url",
            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
        })
        
        user_message = {"role": "user", "content": current_user_content, "image_path": image_webpath}
        state.conversation_history.append(user_message)

        # 2. 调用LLM进行物体定位
        # 注意：这里我们只给LLM提供定位工具，而不是移动工具
        temp_history_for_locating = state.conversation_history[-1:] # 只发送当前用户的指令
        response = send_to_openai(temp_history_for_locating, tools=LOCATE_OBJECT_TOOL)
        
        if isinstance(response, dict) and "error" in response:
            state.conversation_history.pop()
            return {"history": state.conversation_history, "is_muted": state.is_muted, "error": response["error"]}

        message = response.choices[0].message

        # 3. 处理LLM的工具调用结果
        if message.tool_calls:
            tool_call = message.tool_calls[0]
            if tool_call.function.name == "locate_object_in_image":
                try:
                    args = json.loads(tool_call.function.arguments)
                    u, v = args['center_x'], args['center_y']
                    u, v = int(v * 0.48), int(u * 0.64)
                    print(f"LLM成功定位到物体，中心点坐标: ({u}, {v})")
                    
                    # 4. 从深度相机获取精确数据
                    coords = state.depth_camera_handler.get_distance_and_angle(u, v)
                    
                    if coords:
                        # 5. 构建并执行移动指令
                        distance_m = coords['distance_m']-0.2
                        angle_deg = -coords['angle_deg']
                        
                        # 创建动作序列，先转向，再前进
                        actions = [
                            {"command": "turn", "value": angle_deg},
                            {"command": "move_forward", "value": distance_m}
                        ]
                        
                        # 调用小车控制函数
                        result = robot_control.execute_sequence(actions)
                        reply_text = f"好的，已定位到目标。正在执行移动指令: {result}"
                        
                    else:
                        reply_text = "定位到了物体，但在获取它的深度信息时失败了，无法移动。"

                except (json.JSONDecodeError, KeyError, TypeError) as e:
                    reply_text = f"模型返回的坐标格式有误，无法解析: {e}"
            else:
                reply_text = f"模型调用了未知的工具: {tool_call.function.name}"
        else:
            reply_text = "抱歉，我没能在图片中识别出您指定的物体，无法执行自主移动。"

        # 将最终结果告知用户
        assistant_message = {"role": "assistant", "content": reply_text}
        state.conversation_history.append(assistant_message)
        if not state.is_muted:
            state.tts_thread = threading.Thread(target=speak_text_threaded, args=(reply_text,))
            state.tts_thread.start()
            
        return {"history": state.conversation_history, "is_muted": state.is_muted}
    
    # --- 如果不是带图像的自主模式，则执行旧的通用逻辑 ---
    else:
        print("--- 进入通用对话或非视觉自主模式 ---")
        # 这部分代码保持原样，处理纯文本对话或旧的自主模式（如果还想保留的话）
        final_prompt = prompt
        current_user_content = [{"type": "text", "text": final_prompt}]
        if image_filepath:
            base64_image = image_to_base64(image_filepath)
            current_user_content.append({
                "type": "image_url",
                "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
            })
        
        user_message = {"role": "user", "content": current_user_content}
        if image_webpath:
            user_message['image_path'] = image_webpath
        
        state.conversation_history.append(user_message)
        
        # 在非精确导航模式下，不给LLM任何工具，让它只聊天
        tools_for_llm = None 
        
        response = send_to_openai(state.conversation_history, tools=tools_for_llm)

        if isinstance(response, dict) and "error" in response:
            state.conversation_history.pop()
            return {"history": state.conversation_history, "is_muted": state.is_muted, "error": response["error"]}

        message = response.choices[0].message
        reply_text = message.content

        if reply_text:
            assistant_message = {"role": "assistant", "content": reply_text}
            state.conversation_history.append(assistant_message)
            if not state.is_muted:
                state.tts_thread = threading.Thread(target=speak_text_threaded, args=(reply_text,))
                state.tts_thread.start()
        
        return {"history": state.conversation_history, "is_muted": state.is_muted}

