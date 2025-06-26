# openai_handler.py

import openai
import json
import base64
import threading

import config
import state
import robot_control
from voice_handler import speak_text_threaded

# 初始化 OpenAI 客户端
# 如果 config.OPENAI_API_KEY 提供了值，则使用它，否则客户端会自动从环境变量读取
client = openai.OpenAI(api_key=config.OPENAI_API_KEY, base_url=config.OPENAI_BASE_URL)

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
        print(f"视觉自主模式 (OpenAI) - 增强后的指令: {final_prompt}")

    # 准备用户消息
    current_user_content = [{"type": "text", "text": final_prompt}]
    if image_filepath:
        base64_image = image_to_base64(image_filepath)
        current_user_content.append({
            "type": "image_url",
            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
        })
    
    user_message = {"role": "user", "content": current_user_content}
    if image_webpath:
        # 将web路径附加到消息对象上，以便在前端渲染
        user_message['image_path'] = image_webpath
    
    #with state.history_lock: # 使用在上一轮建议中提到的线程锁保护历史记录
    state.conversation_history.append(user_message)

    # 根据模式决定是否提供工具 (OpenAI的'tools'格式与LM Studio兼容)
    tools_for_llm = robot_control.CAR_TOOLS if autonomous_mode and state.ros_enabled else None
    
    # 发送给LLM
    response = send_to_openai(state.conversation_history, tools=tools_for_llm)

    if isinstance(response, dict) and "error" in response:
        #with state.history_lock:
        state.conversation_history.pop() # 如果出错，移除刚刚添加的用户消息
        return {"history": state.conversation_history, "is_muted": state.is_muted, "error": response["error"]}

    message = response.choices[0].message
    
    # 处理工具调用
    if message.tool_calls:
        #with state.history_lock:
        state.conversation_history.append(message.model_dump()) # 记录模型的工具调用请求
        
        tool_results_for_next_call = []
        for tool_call in message.tool_calls:
            function_name = tool_call.function.name
            
            if function_name in robot_control.available_functions:
                try:
                    arguments = json.loads(tool_call.function.arguments)
                    function_to_call = robot_control.available_functions[function_name]
                    function_response = function_to_call(**arguments)
                except Exception as e:
                    function_response = f"错误: 执行工具时发生异常: {e}"
            else:
                 function_response = f"错误: 模型请求了未知的工具 '{function_name}'"

            tool_results_for_next_call.append({
                "tool_call_id": tool_call.id,
                "role": "tool",
                "name": function_name,
                "content": function_response,
            })
        
        # 将所有工具的执行结果都加入历史记录
        #with state.history_lock:
        state.conversation_history.extend(tool_results_for_next_call)
        
        # 再次调用LLM，让它根据工具结果生成最终回复
        final_response_obj = send_to_openai(state.conversation_history, tools=None)
        if isinstance(final_response_obj, dict) and "error" in final_response_obj:
             reply_text = "工具执行完毕，但在生成最终回复时出错。"
        else:
            reply_text = final_response_obj.choices[0].message.content
    else:
        # 没有工具调用，直接获取回复
        reply_text = message.content

    # 处理最终回复
    if reply_text and not reply_text.startswith("错误"):
        assistant_message = {"role": "assistant", "content": reply_text}
        #with state.history_lock:
        state.conversation_history.append(assistant_message)
        if not state.is_muted:
            # 启动语音播放线程
            state.tts_thread = threading.Thread(target=speak_text_threaded, args=(reply_text,))
            state.tts_thread.start()
    
    return {"history": state.conversation_history, "is_muted": state.is_muted}
