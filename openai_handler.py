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

def send_to_openai_for_json(messages_history):
    """向 OpenAI API 发送请求，并强制其返回JSON对象"""
    try:
        # 核心修正：移除 tools 参数，仅使用 response_format 来获取JSON
        response = client.chat.completions.create(
            model=config.OPENAI_MODEL_NAME,
            messages=messages_history,
            response_format={"type": "json_object"}, # 强制JSON输出
            temperature=0.1, # 对于JSON输出，使用较低的温度以保证格式稳定
            max_tokens=config.MODEL_MAX_TOKENS,
        )
        return response
    except Exception as e:
        print(f"OpenAI API 请求错误 (JSON模式): {e}")
        return {"error": f"调用 OpenAI API 失败: {e}"}

def process_message_and_get_reply_openai(prompt, image_filepath=None, image_webpath=None, autonomous_mode=False):
    """处理用户输入，与OpenAI LLM交互并返回结果的完整流程"""
    # 仅在带图像的自主模式下，执行目标定位逻辑
    if autonomous_mode and image_filepath and state.depth_camera_handler:
        print("--- 进入精确自主导航模式 (归一化坐标JSON模式) ---")
        
        # 1. 准备给LLM的指令，严格按照你提供的示例逻辑，要求返回归一化坐标
        system_instruction = (
            "你是一个AI小车的视觉定位助手。你的任务是分析图片和用户指令，并以JSON格式返回结果。"
            "JSON对象必须包含一个名为'detections'的列表。列表中的每个对象都应包含一个'label'和一个'box_2d'。"
            "'box_2d'是一个包含4个整数的列表，代表物体边界框的坐标，格式为 [ymin, xmin, ymax, xmax]，"
            "并且所有坐标值都必须被归一化到 0-1000 的范围。"
            "如果找到多个物体，请返回最符合用户描述的那一个。"
            "不要输出任何其他文字或解释，只输出一个JSON对象。"
            '例如: {"detections": [{"label": "红色物体", "box_2d": [100, 150, 800, 850]}]}'
        )
        final_prompt = "用户指令是：" + prompt

        current_user_content = [{"type": "text", "text": final_prompt}]
        base64_image = image_to_base64(image_filepath)
        current_user_content.append({
            "type": "image_url",
            "image_url": {"url": f"data:image/jpeg;base64,{base64_image}"}
        })
        
        # 构建本次请求的消息历史
        messages_for_locating = [
            {"role": "system", "content": system_instruction},
            {"role": "user", "content": current_user_content}
        ]
        
        # 仅将用户消息存入完整历史记录
        user_message_for_history = {"role": "user", "content": current_user_content, "image_path": image_webpath}
        state.conversation_history.append(user_message_for_history)
        
        # 2. 调用LLM进行物体定位
        response = send_to_openai_for_json(messages_for_locating)
        
        if isinstance(response, dict) and "error" in response:
            state.conversation_history.pop()
            return {"history": state.conversation_history, "is_muted": state.is_muted, "error": response["error"]}

        # 3. 解析模型返回的JSON内容
        try:
            content_str = response.choices[0].message.content
            print(f"LLM返回的原始JSON字符串: {content_str}")
            data = json.loads(content_str)
            
            # 从返回结果中获取第一个检测到的物体
            if not data.get("detections") or not isinstance(data["detections"], list) or len(data["detections"]) == 0:
                 raise KeyError("模型返回的JSON中没有找到'detections'列表或列表为空。")
            
            item = data["detections"][0]
            box_normalized = item.get("box_2d")

            # 4. **核心逻辑：反归一化坐标**
            # 读取实际图片尺寸，用于坐标换算
            image = cv2.imread(image_filepath)
            height, width, _ = image.shape
            print(f"--- [计算] 图片实际尺寸: 宽={width}, 高={height} ---")

            if not box_normalized or len(box_normalized) != 4:
                raise ValueError("JSON中'box_2d'格式不正确。")
            
            # [ymin, xmin, ymax, xmax]
            y_min_norm, x_min_norm, y_max_norm, x_max_norm = box_normalized
            
            abs_x_min = (x_min_norm / 1000.0) * width
            abs_y_min = (y_min_norm / 1000.0) * height
            abs_x_max = (x_max_norm / 1000.0) * width
            abs_y_max = (y_max_norm / 1000.0) * height

            # 计算绝对像素坐标的中心点
            u = int((abs_x_min + abs_x_max) / 2)
            v = int((abs_y_min + abs_y_max) / 2)
            # 定义一个包含所有关键点的字典
            points_of_interest = {
                "center": (u, v),
                "top_right": (abs_x_max, abs_y_min),
                "top_left": (abs_x_min, abs_y_min),
                "bottom_left": (abs_x_min, abs_y_max),
                "bottom_right": (abs_x_max, abs_y_max)
            }

            print(f"--- [计算] 反归一化后, 目标中心点像素坐标: (u={u}, v={v}) ---")

            # 5. 画框并保存图片的功能
            if image_filepath:
                try:
                    pil_image = Image.open(image_filepath)
                    draw = ImageDraw.Draw(pil_image)
                    # 绘制反归一化后的边界框
                    draw.rectangle([abs_x_min, abs_y_min, abs_x_max, abs_y_max], outline="lime", width=3)
                    # 绘制中心点
                    draw.ellipse((u - 5, v - 5, u + 5, v + 5), fill="red", outline="red")
                    
                    output_image_path = image_filepath.replace('.jpg', '_with_box.jpg')
                    pil_image.save(output_image_path)
                    print(f"--- [调试] 带边界框的图片已保存到: {output_image_path} ---")
                except Exception as e:
                    print(f"--- [错误] 绘制调试图片时出错: {e} ---")

            # 6. 从深度相机获取精确数据
            coords = state.depth_camera_handler.get_distance_and_angle(points_of_interest)
            
            if coords:
                distance_m = coords['distance_m'] - 0.15 # 保留一定安全距离
                angle_deg = coords['angle_deg'] 
                
                actions = [
                    {"command": "turn", "value": angle_deg},
                    {"command": "move_forward", "value": distance_m}
                ]
                
                result = robot_control.execute_sequence(actions)
                reply_text = f"好的，已定位到目标。正在执行移动指令: {result}"
            else:
                reply_text = "定位到了物体，但在获取它的深度信息时失败了，无法移动。"

        except (json.JSONDecodeError, KeyError, ValueError, TypeError) as e:
            reply_text = f"处理模型返回数据时出错: {e}。模型原始回复: {response.choices[0].message.content}"
        
        # 将最终结果告知用户
        assistant_message = {"role": "assistant", "content": reply_text}
        state.conversation_history.append(assistant_message)
        if not state.is_muted:
            state.tts_thread = threading.Thread(target=speak_text_threaded, args=(reply_text,))
            state.tts_thread.start()
            
        return {"history": state.conversation_history, "is_muted": state.is_muted}

    # --- 如果不是带图像的自主模式，则执行旧的通用对话逻辑 ---
    else:
        print("--- 进入通用对话模式 ---")
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
        
        try:
            response = client.chat.completions.create(
                model=config.OPENAI_MODEL_NAME,
                messages=state.conversation_history,
                temperature=config.MODEL_TEMPERATURE,
                max_tokens=config.MODEL_MAX_TOKENS,
            )
            reply_text = response.choices[0].message.content if response.choices[0].message.content else "抱歉，我不知道该如何回应。"

        except Exception as e:
            print(f"OpenAI API 请求错误 (通用模式): {e}")
            reply_text = f"调用OpenAI API时出错: {e}"
            state.conversation_history.pop()

        if reply_text:
            assistant_message = {"role": "assistant", "content": reply_text}
            state.conversation_history.append(assistant_message)
            if not state.is_muted:
                state.tts_thread = threading.Thread(target=speak_text_threaded, args=(reply_text,))
                state.tts_thread.start()
        
        return {"history": state.conversation_history, "is_muted": state.is_muted}

