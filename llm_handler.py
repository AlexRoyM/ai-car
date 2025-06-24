import requests
import json
import base64
import threading

import config
import state
import robot_control
from voice_handler import speak_text_threaded

def image_to_base64(image_path):
    """将图片文件转换为Base64编码"""
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

def send_to_lm_studio(messages_history, tools=None):
    # ... (此函数无变化)
    payload = {
        "model": config.MODEL_NAME,
        "messages": messages_history,
        "temperature": config.MODEL_TEMPERATURE,
        "max_tokens": config.MODEL_MAX_TOKENS,
        "stream": False,
    }
    if tools:
        payload['tools'] = tools
        payload['tool_choice'] = "auto"
    
    headers = {"Content-Type": "application/json"}
    try:
        response = requests.post(config.LM_STUDIO_URL, headers=headers, data=json.dumps(payload), timeout=120)
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"API请求错误: {e}")
        return {"error": f"调用大模型API失败: {e}"}

def process_message_and_get_reply(prompt, image_filepath=None, image_webpath=None, autonomous_mode=False):
    """
    处理用户输入，与LLM交互，并通过WebSocket发送更新。
    此函数现在不再直接返回JSON给HTTP请求，而是通过socketio发送事件。
    """
    from main import socketio # 导入socketio实例

    final_prompt = prompt

    if autonomous_mode and image_filepath:
        system_like_instruction = (
            "你是一个AI小车的控制系统。你的唯一任务是结合图像和以下的用户文字指令，"
            "生成一个包含具体数值（角度和米）的动作序列来控制小车物理移动。"
            "你必须调用 `execute_sequence` 工具来完成任务。请仔细分析图像内容，估算完成指令所需的转向角度和移动距离。"
            "例如，如果目标在右边，你应该估算出一个正数角度。不要进行任何与工具调用无关的对话或道歉。\n\n"
            "用户指令是： "
        )
        final_prompt = system_like_instruction + prompt
        print(f"视觉自主模式 - 增强后的指令: {final_prompt}")

    # 准备用户消息并立即发送到前端
    current_user_content = [{"type": "text", "text": final_prompt}]
    if image_filepath:
        base64_image = image_to_base64(image_filepath)
        current_user_content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}})
    
    user_message = {"role": "user", "content": current_user_content}
    if image_webpath:
        user_message['image_path'] = image_webpath
        
    state.conversation_history.append(user_message)
    socketio.emit('new_message', user_message)
    socketio.emit('status_update', {'message': '已收到，正在思考中...'})

    tools_for_llm = robot_control.CAR_TOOLS if autonomous_mode and state.ros_enabled else None
    
    response_json = send_to_lm_studio(state.conversation_history, tools=tools_for_llm)

    if "error" in response_json:
        state.conversation_history.pop() # 移除失败的用户消息
        error_msg = response_json["error"]
        socketio.emit('error_message', {'error': error_msg})
        socketio.emit('status_update', {'message': ''}) # 清除状态
        return

    message = response_json.get('choices', [{}])[0].get('message', {})
    
    if message.get("tool_calls"):
        socketio.emit('status_update', {'message': '正在解析指令...'})
        state.conversation_history.append(message)
        # 将LLM的工具调用请求也发给前端，让UI可以显示“正在执行”
        socketio.emit('new_message', message) 
        
        tool_call = message["tool_calls"][0]
        function_name = tool_call['function']['name']
        
        if function_name in robot_control.available_functions:
            try:
                arguments = json.loads(tool_call['function']['arguments'])
                function_to_call = robot_control.available_functions[function_name]
                # 函数内部会通过socketio发送实时状态
                function_response = function_to_call(**arguments)
            except Exception as e:
                function_response = f"错误: 执行工具时发生异常: {e}"
        else:
             function_response = f"错误: 模型请求了未知的工具 '{function_name}'"

        state.conversation_history.append({
            "role": "tool", "tool_call_id": tool_call['id'],
            "name": function_name, "content": function_response,
        })
        
        final_response_json = send_to_lm_studio(state.conversation_history, tools=None)
        reply_text = final_response_json.get('choices', [{}])[0].get('message', {}).get('content', "工具执行完毕，但在生成最终回复时出错。")
    else:
        reply_text = message.get('content', '我不知道该说些什么。')

    # 处理最终回复
    socketio.emit('status_update', {'message': ''}) # 清空状态栏
    if reply_text and not reply_text.startswith("错误"):
        assistant_message = {"role": "assistant", "content": reply_text}
        state.conversation_history.append(assistant_message)
        socketio.emit('new_message', assistant_message)
        
        if not state.is_muted:
            state.tts_thread = socketio.start_background_task(speak_text_threaded, reply_text)
