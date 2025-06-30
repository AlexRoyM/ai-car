# unified_camera_handler.py

import cv2
import numpy as np
import math
import threading
import time
import os
from datetime import datetime

# 导入所有需要的 pyorbbecsdk 组件
from pyorbbecsdk import (Config, Pipeline, OBSensorType, OBAlignMode, 
                         OBError, OBFormat, VideoFrame)

import state
import config

def _convert_color_frame_to_bgr_image(color_frame: VideoFrame):
    """
    内部辅助函数：将Orbbec SDK的彩色帧高效地转换为OpenCV BGR图像。
    这个函数主要用于拍照保存，而不是实时视频流。
    """
    if color_frame is None:
        return None
    try:
        # 直接利用np.asanyarray获取数据，避免不必要的数据拷贝
        data = np.asanyarray(color_frame.get_data())
        width = color_frame.get_width()
        height = color_frame.get_height()
        color_format = color_frame.get_format()

        if color_format == OBFormat.RGB:
            return cv2.cvtColor(data.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
        elif color_format == OBFormat.BGR:
            return data.reshape((height, width, 3))
        elif color_format == OBFormat.YUYV:
            return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_YUY2)
        elif color_format == OBFormat.MJPG:
            # MJPG格式需要先解码
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        else:
            print(f"警告: 不支持的彩色帧格式: {color_format}")
            return None
    except Exception as e:
        print(f"转换彩色帧时出错: {e}")
        return None

class UnifiedCamera:
    def __init__(self):
        self.pipeline = Pipeline()
        self.config = Config()
        self.lock = threading.Lock()
        
        # 存储最新的帧数据
        self.latest_color_frame = None
        self.latest_depth_map = None # 存储处理好的深度图 (np.array)
        self.camera_params = None
        
        self.is_running = False
        
        print("--- 正在配置统一摄像头... ---")
        try:
            # 1. 配置彩色流 - 优先使用 MJPG 格式以优化Web视频流
            color_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            if not color_profile_list:
                raise RuntimeError("错误: 未找到彩色传感器!")
            
            try:
                # 明确请求 MJPG 格式，这是解决网页闪烁的关键
                color_profile = color_profile_list.get_video_stream_profile(640, 480, OBFormat.MJPG, 30)
            except OBError:
                print("警告: 无法获取 640x480 MJPG@30fps 配置，回退到默认配置。")
                color_profile = color_profile_list.get_default_video_stream_profile()
            self.config.enable_stream(color_profile)
            print(f"彩色流已配置: {color_profile.get_width()}x{color_profile.get_height()} @ {color_profile.get_fps()}fps, 格式={color_profile.get_format()}")

            # 2. 配置深度流
            depth_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            if not depth_profile_list:
                raise RuntimeError("错误: 未找到深度传感器!")
            
            try:
                depth_profile = depth_profile_list.get_video_stream_profile(640, 480, OBFormat.Y16, 30)
            except OBError:
                print("警告: 无法获取 640x480 Y16@30fps 配置，回退到默认配置。")
                depth_profile = depth_profile_list.get_default_video_stream_profile()
            self.config.enable_stream(depth_profile)
            print(f"深度流已配置: {depth_profile.get_width()}x{depth_profile.get_height()} @ {depth_profile.get_fps()}fps, 格式={depth_profile.get_format()}")

            # 3. 设置硬件对齐模式 - 这是保证深度和彩色像素对应的关键
            self.config.set_align_mode(OBAlignMode.HW_MODE)

        except (OBError, RuntimeError) as e:
            print(f"严重错误: 配置数据流时出错: {e}")
            state.ros_enabled = False
            raise e # 抛出异常，让上层知道初始化失败

        # 4. 启动 Pipeline
        self.pipeline.start(self.config)
        self.camera_params = self.pipeline.get_camera_param()
        print("--- 摄像头 Pipeline 已启动 ---")

        self.is_running = True
        self.thread = threading.Thread(target=self._update_frames, daemon=True)
        self.thread.start()
        print("--- 统一摄像头处理线程已启动 ---")

    def _update_frames(self):
        """后台线程，持续获取并缓存对齐后的彩色帧和深度图"""
        while self.is_running:
            try:
                frames = self.pipeline.wait_for_frames(1000)
                if frames is None:
                    continue

                with self.lock:
                    # 获取对齐后的帧
                    color_frame = frames.get_color_frame()
                    depth_frame = frames.get_depth_frame()

                    if color_frame:
                        # 只缓存原始帧对象，不做任何处理
                        self.latest_color_frame = color_frame
                    
                    if depth_frame:
                        # 将深度帧处理成可直接用于计算的 numpy 数组
                        scale = depth_frame.get_depth_scale()
                        depth_buffer = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                        depth_uint16 = depth_buffer.reshape((depth_frame.get_height(), depth_frame.get_width()))
                        self.latest_depth_map = depth_uint16.astype(np.float32) * scale
            except Exception as e:
                print(f"更新帧数据时出错: {e}")
            time.sleep(1/60) # 稍微降低循环频率

    def get_jpeg_frame(self):
        """
        为网页视频流提供JPEG编码的帧。
        如果彩色流是MJPG格式，直接返回原始数据，效率极高。
        """
        with self.lock:
            if self.latest_color_frame is None:
                return None
            
            # [!INFO] 核心优化：如果格式是MJPG，直接返回数据
            if self.latest_color_frame.get_format() == OBFormat.MJPG:
                return np.asanyarray(self.latest_color_frame.get_data()).tobytes()
            
            # [!INFO] 回退方案：如果不是MJPG，则进行编码
            else:
                bgr_image = _convert_color_frame_to_bgr_image(self.latest_color_frame)
                if bgr_image is None:
                    return None
                ret, jpeg_buffer = cv2.imencode('.jpg', bgr_image)
                return jpeg_buffer.tobytes() if ret else None

    def capture_image_to_file(self, upload_folder, session_id):
        """
        捕获当前帧，将彩色图像保存到文件，并返回对应的深度图副本。
        这是“点击发送”功能的核心。
        返回: (web_path, file_path, depth_map_copy) 或 (None, None, None)
        """
        with self.lock:
            if self.latest_color_frame is None or self.latest_depth_map is None:
                return None, None, None
            # 创建副本以保证线程安全
            color_frame_copy = self.latest_color_frame
            depth_map_copy = self.latest_depth_map.copy()

        bgr_image = _convert_color_frame_to_bgr_image(color_frame_copy)
        if bgr_image is None:
            return None, None, None

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"capture_{timestamp}.jpg"
        filepath = os.path.join(upload_folder, filename)
        
        os.makedirs(upload_folder, exist_ok=True)
        cv2.imwrite(filepath, bgr_image)
        
        web_path = os.path.join('static', 'sessions', session_id, 'uploads', filename).replace('\\', '/')
        return web_path, filepath, depth_map_copy

    def get_distance_and_angle(self, u, v, depth_map):
        """
        根据给定的像素坐标(u, v)和【特定的深度图】计算距离和角度。
        这保证了计算用的是拍照瞬间的深度数据。
        """
        if depth_map is None or self.camera_params is None:
            print("错误: 深度数据或相机参数尚未就绪。")
            return None

        h, w = depth_map.shape
        if not (0 <= v < h and 0 <= u < w):
            return None

        depth_in_mm = depth_map[v, u]
        if depth_in_mm <= 0:
            return None

        center_u_pixel = w / 2
        angle_per_pixel = config.HORIZONTAL_FOV / w
        camera_yaw_rad = math.radians((u - center_u_pixel) * angle_per_pixel)

        z_mm = float(depth_in_mm)
        x_mm = z_mm * math.tan(camera_yaw_rad)
        
        z_compensated_mm = z_mm + (config.CAR_RADIUS_M * 1000)
        corrected_yaw_rad = math.atan2(x_mm, z_compensated_mm)
        
        return {
            "distance_m": z_mm / 1000.0,
            "angle_deg": math.degrees(corrected_yaw_rad)
        }

    def release(self):
        if self.is_running:
            print("--- 正在释放统一摄像头资源... ---")
            self.is_running = False
            if hasattr(self, 'thread') and self.thread.is_alive():
                self.thread.join(timeout=2)
            self.pipeline.stop()
            print("--- 统一摄像头已释放。 ---")
