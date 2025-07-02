# openai_handler.py

import openai
import json
import base64
import threading
import cv2 # 导入OpenCV来读取图片尺寸
import config
import state
import robot_control
from voice_handler import speak_text_threaded
from PIL import Image, ImageDraw # 导入Pillow库用于图像绘制

# 初始化 OpenAI 客户端
client = openai.OpenAI(api_key=config.OPENAI_API_KEY, base_url=config.OPENAI_BASE_URL)

def image_to_base64(image_path):
    """将图片文件转换为Base64编码"""
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')
def get_refined_system_prompt():
    """获取系统角色指令"""
    return (
        "你是一个部署在工业环境（如工厂或仓库或室外工业场景）中的AI智能安全巡检车。你的核心任务是担任“现场安全员”的角色。除非特殊要求，不然中文回答。"
        "在与用户的所有交互中，你必须优先考虑安全。你的职责包括：\n"
        "1. **主动观察与分析**：仔细分析摄像头画面和用户指令，识别潜在的安全隐患。这包括但不限于：警告标识、危险设备、"
        "不当操作、人员安全防护（如是否佩戴安全帽）以及任何可能对人员或设备造成伤害的物体。\n"
        "2. **清晰沟通**：用简洁、清晰、专业的语言与用户沟通。在执行指令或报告观察结果时，附带安全提醒。但是你回复的方式绝对不能过于机械，不要死板地念出每个条目，要像个正常人类。\n"
        "3. **精确执行**：准确理解并执行识别任务。"
    )
def get_autonomous_json_instructions():
    """获取在自主模式下指导LLM输出特定JSON格式的指令"""
    return (
        "根据用户的指令和提供的图片（如果有），以严格的JSON格式返回你的分析和行动计划。JSON对象必须包含以下字段：\n"
        "1. `detection_required` (boolean): 如果需要通过分析图片中的物体来确定移动目标，则为 `true`。如果指令是纯文本移动（如“前进1米”或者“后退1米”或者“向左转20.0度”）或非移动任务（如“描述图片”），则为 `false`。\n"
        "2. `actions` (array): 一个包含移动指令的列表。如果 `detection_required` 为 `true`，此列表应为空，因为具体动作将由代码根据检测结果计算。如果 `detection_required` 为 `false` 且用户指令包含明确的移动要求，请在此填充动作，例如 `[{\"command\": \"turn\", \"value\": -90.0}, {\"command\": \"move_forward\", \"value\": 2.0}]` (角度单位为度，距离单位为米；左转为负，右转为正)。如果无需移动，则为空列表 `[]`。\n"
        "3. `target_object` (object | null): 如果 `detection_required` 为 `true`，此对象必须包含 `label` (string，物体描述) 和 `box_2d` (array，格式为 `[ymin, xmin, ymax, xmax]` 的归一化坐标 0-1000)。否则，此值为 `null`。\n"
        "4. `response` (string): 你要对用户说的自然语言回复。这段回复应该结合你的观察和将要执行的动作，并包含必要的安全提醒。\n"
        "重要提示: 绝对不要在JSON格式之外输出任何文本。"
    )
def process_message_and_get_reply_openai(prompt, image_filepath=None, image_webpath=None, autonomous_mode=False):
    """
    处理用户输入、与LLM交互并返回结果的全新统一流程。
    """
    # 1. 构建用户消息体
    user_message_content = [{"type": "text", "text": prompt}]
    if image_filepath:
        base64_image = image_to_base64(image_filepath)
        user_message_content.append({
            "type": "image_url",
            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
        })
    
    # 将用户消息存入完整历史记录
    user_message_for_history = {"role": "user", "content": user_message_content}
    if image_webpath:
        user_message_for_history['image_path'] = image_webpath
    state.conversation_history.append(user_message_for_history)

    # 2. 根据模式选择不同的处理逻辑
    if autonomous_mode:
        # --- 自主移动模式 ---
        print("--- [模式] 开启自主移动 ---")
        return handle_autonomous_mode(user_message_content)
    else:
        # --- 普通对话模式 ---
        print("--- [模式] 普通对话 ---")
        return handle_conversation_mode()

def handle_conversation_mode():
    """处理普通对话，不涉及小车自主移动"""
    messages = [
        {"role": "system", "content": get_refined_system_prompt()},
        *state.conversation_history
    ]
    try:
        response = client.chat.completions.create(
            model=config.OPENAI_MODEL_NAME,
            messages=messages,
            temperature=config.MODEL_TEMPERATURE,
            max_tokens=config.MODEL_MAX_TOKENS,
        )
        reply_text = response.choices[0].message.content or "抱歉，我没有理解您的意思。"
    except Exception as e:
        print(f"OpenAI API 请求错误 (通用模式): {e}")
        reply_text = f"调用API时出错: {e}"
        state.conversation_history.pop() # 如果出错，移除刚刚添加的用户消息

    if reply_text:
        assistant_message = {"role": "assistant", "content": reply_text}
        state.conversation_history.append(assistant_message)
        if not state.is_muted:
            state.tts_thread = threading.Thread(target=speak_text_threaded, args=(reply_text,))
            state.tts_thread.start()
            
    return {"history": state.conversation_history, "is_muted": state.is_muted}

def handle_autonomous_mode(user_content):
    """处理自主移动模式，核心逻辑在此"""
    # 构建专门用于获取JSON的请求
    messages_for_json = [
        {"role": "system", "content": get_autonomous_json_instructions()},
        {"role": "user", "content": user_content}
    ]
    
    try:
        response = client.chat.completions.create(
            model=config.OPENAI_MODEL_NAME,
            messages=messages_for_json,
            response_format={"type": "json_object"},
            temperature=0.1,
            max_tokens=config.MODEL_MAX_TOKENS,
        )
        content_str = response.choices[0].message.content
        print(f"LLM返回的原始JSON字符串: {content_str}")
        data = json.loads(content_str)

        final_reply_text = data.get("response", "好的，任务已收到。")
        actions_to_execute = []
        action_result_summary = "无需移动。"

        # 逻辑分支1: 需要检测图片来决定动作
        if data.get("detection_required") and data.get("target_object"):
            print("--- [自主导航] 模式: 图像目标检测 ---")
            target = data["target_object"]
            box_normalized = target.get("box_2d")
            
            # 使用图像坐标计算移动动作
            calculated_actions = calculate_actions_from_box(box_normalized)
            if calculated_actions:
                actions_to_execute = calculated_actions
            else:
                final_reply_text = f"已在图中定位到'{target.get('label', '目标')}'，但无法获取其空间坐标，无法移动。"

        # 逻辑分支2: 根据LLM直接提供的动作指令移动
        elif data.get("actions"):
            print("--- [自主导航] 模式: 文本指令解析 ---")
            actions_to_execute = data["actions"]

        # 执行动作
        if actions_to_execute:
            action_result_summary = robot_control.execute_sequence(actions_to_execute)
        
        # 组合最终回复
        final_response_message = f"{final_reply_text} (执行结果: {action_result_summary})"
        
    except (Exception) as e:
        print(f"处理自主模式时出错: {e}")
        final_response_message = f"在自主模式下处理时遇到错误: {e}"
        state.conversation_history.pop()

    # 将最终结果告知用户
    assistant_message = {"role": "assistant", "content": final_response_message}
    state.conversation_history.append(assistant_message)
    if not state.is_muted:
        # 我们只播报LLM生成的response部分，不播报括号里的执行结果
        text_to_speak = data.get("response", "指令已处理。") if 'data' in locals() else "指令已处理。"
        state.tts_thread = threading.Thread(target=speak_text_threaded, args=(text_to_speak,))
        state.tts_thread.start()
        
    return {"history": state.conversation_history, "is_muted": state.is_muted}


def calculate_actions_from_box(box_normalized):
    """根据归一化的bbox计算转动和移动的动作列表"""
    if not box_normalized or len(box_normalized) != 4:
        print(f"错误: 'box_2d' 格式不正确: {box_normalized}")
        return None

    # 获取最后一张用户发的图片路径用于计算
    image_path = None
    for msg in reversed(state.conversation_history):
        if msg['role'] == 'user' and msg.get('image_path'):
            # 在服务器上的文件路径是 'static/...' 开头，需要转为绝对或相对路径
            image_path = msg['image_path'] 
            break
    
    if not image_path or not state.camera_handler:
        print("错误: 找不到图片路径或摄像头未初始化，无法计算坐标。")
        return None

    image = cv2.imread(image_path)
    if image is None:
        print(f"错误: 无法读取图片 {image_path}")
        return None
    height, width, _ = image.shape
    
    y_min_norm, x_min_norm, y_max_norm, x_max_norm = box_normalized
    u = int(((x_min_norm + x_max_norm) / 2 / 1000.0) * width)
    v = int(((y_min_norm + y_max_norm) / 2 / 1000.0) * height)
    
    print(f"--- [计算] 反归一化后, 目标中心点像素坐标: (u={u}, v={v}) ---")
    coords = state.camera_handler.get_distance_and_angle(u, v)

    if coords:
        distance_m = max(0, coords['distance_m'] - 0.35) # 减去20cm安全距离
        angle_deg = coords['angle_deg']
        
        # 只有在距离大于0时才移动
        actions = [{"command": "turn", "value": angle_deg}]
        if distance_m > 0:
            actions.append({"command": "move_forward", "value": distance_m})
        
        return actions
    else:
        print(f"--- [计算] 无法从点 ({u}, {v}) 获取深度信息 ---")
        return None

