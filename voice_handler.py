import asyncio
import edge_tts
import pygame
import os
import time
import config
import state
import uuid
import re
def preprocess_text_for_speech(text: str) -> str:
    """
    对将要播报的文本进行预处理，优化语音效果。
    1. 移除特殊符号，如 *, {}, [], () 等。
    2. 将 "-数字" 的格式转换为 "负数字" 的读法。
    """
    # 移除不希望被读出的标点和符号
    text = re.sub(r'[*#\[\]{}()"`]', '', text)
    
    # 将 "-15" 或 "-20.5" 这样的格式替换为 "负15" 或 "负20.5"
    # \g<1> 代表第一个捕获组 (\d+(\.\d+)?)
    text = re.sub(r'-(\d+(\.\d+)?)', r'负\g<1>', text)
    
    return text
def speak_text_threaded(text):
    """在后台线程中生成并播放语音"""
    session_dir = os.path.dirname(state.TEMP_AUDIO_PATH)
    temp_audio_path = os.path.join(session_dir, f"tts_{uuid.uuid4()}.mp3")
    try:
        if not temp_audio_path:
            print("错误: 临时音频路径未设置。")
            return
        processed_text = preprocess_text_for_speech(text)
        print(f"--- [语音播报] 原始文本: '{text}' ---")
        print(f"--- [语音播报] 处理后文本: '{processed_text}' ---")
        # 确保目录存在
        os.makedirs(os.path.dirname(temp_audio_path), exist_ok=True)

        # 异步生成语音文件
        asyncio.run(generate_speech_async(processed_text, temp_audio_path))
        
        # 播放语音
        pygame.mixer.music.load(temp_audio_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            if not pygame.mixer.get_init(): break
            time.sleep(0.1)
            
    except Exception as e:
        print(f"后台语音线程错误: {e}")
    finally:
        # 清理临时文件
        if os.path.exists(temp_audio_path):
            try:
                time.sleep(0.1)
                os.remove(temp_audio_path)
            except Exception as e:
                print(f"删除临时音频文件失败: {e}")

async def generate_speech_async(text, output_file):
    """使用edge-tts异步生成语音文件"""
    communicate = edge_tts.Communicate(text, config.SELECTED_VOICE)
    await communicate.save(output_file)

def stop_speech_playback():
    """停止当前播放的语音并打印日志"""
    if pygame.mixer.get_init() and pygame.mixer.music.get_busy():
        pygame.mixer.music.stop()
        print("--- 语音播放已通过指令停止 ---")
        return True
    return False
