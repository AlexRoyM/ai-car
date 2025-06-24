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
    """向LM Studio发送请求"""
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
    """处理用户输入，与LLM交互并返回结果的完整流程"""
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

    # 准备用户消息
    current_user_content = [{"type": "text", "text": final_prompt}]
    if image_filepath:
        base64_image = image_to_base64(image_filepath)
        current_user_content.append({"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}})
    
    user_message = {"role": "user", "content": current_user_content}
    if image_webpath:
        user_message['image_path'] = image_webpath
    state.conversation_history.append(user_message)

    # 根据模式决定是否提供工具
    tools_for_llm = robot_control.CAR_TOOLS if autonomous_mode and state.ros_enabled else None
    
    # 发送给LLM
    response_json = send_to_lm_studio(state.conversation_history, tools=tools_for_llm)

    if "error" in response_json:
        state.conversation_history.pop() # 如果出错，移除刚刚添加的用户消息
        return {"history": state.conversation_history, "is_muted": state.is_muted, "error": response_json["error"]}

    message = response_json.get('choices', [{}])[0].get('message', {})
    if not message:
        # 如果 message 为空字典，说明API返回结构异常，构造一个错误信息
        error_msg = "调用大模型成功，但返回数据结构异常。"
        print(f"API Response Error: {response_json}")
        # 稳妥起见，不把用户消息和空回复计入历史
        state.conversation_history.pop()
        return {"history": state.conversation_history, "is_muted": state.is_muted, "error": error_msg}
    
    # 处理工具调用
    if message.get("tool_calls"):
        state.conversation_history.append(message)
        tool_call = message["tool_calls"][0]
        function_name = tool_call['function']['name']
        
        if function_name in robot_control.available_functions:
            try:
                arguments = json.loads(tool_call['function']['arguments'])
                function_to_call = robot_control.available_functions[function_name]
                function_response = function_to_call(**arguments)
            except Exception as e:
                function_response = f"错误: 执行工具时发生异常: {e}"
        else:
             function_response = f"错误: 模型请求了未知的工具 '{function_name}'"

        # 将工具执行结果加入历史记录
        state.conversation_history.append({
            "role": "tool", "tool_call_id": tool_call['id'],
            "name": function_name, "content": function_response,
        })
        
        # 再次调用LLM，让它根据工具结果生成最终回复
        final_response_json = send_to_lm_studio(state.conversation_history, tools=None)
        reply_text = final_response_json.get('choices', [{}])[0].get('message', {}).get('content', "工具执行完毕，但在生成最终回复时出错。")
    else:
        # 没有工具调用，直接获取回复
        reply_text = message.get('content', '我不知道该说些什么。')

    # 处理最终回复
    if reply_text and not reply_text.startswith("错误"):
        assistant_message = {"role": "assistant", "content": reply_text}
        state.conversation_history.append(assistant_message)
        if not state.is_muted:
            # 启动语音播放线程
            state.tts_thread = threading.Thread(target=speak_text_threaded, args=(reply_text,))
            state.tts_thread.start()
    
    return {"history": state.conversation_history, "is_muted": state.is_muted}
