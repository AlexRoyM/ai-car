# unified_camera_handler.py

import cv2
import numpy as np
import math
import threading
import time
import os
from datetime import datetime
# 严格遵守您提供的导入列表
from pyorbbecsdk import Config, OBSensorType, Pipeline, OBError, OBCameraParam

import state
import config

class UnifiedCamera:
    def __init__(self):
        self.pipeline = Pipeline()
        self.config = Config()
        self.lock = threading.Lock()
        
        # 存储最新的帧数据
        self.color_frame_data = None
        self.depth_data_map = None
        self.camera_params = None
        
        self.is_running = False
        
        try:
            # 1. 配置并使能深度流 (逻辑与您提供的示例完全一致)
            depth_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            if depth_profile_list is None:
                raise RuntimeError("未找到深度传感器!")
            depth_profile = depth_profile_list.get_default_video_stream_profile()
            if depth_profile is None:
                raise RuntimeError("未找到默认的深度流配置!")
            self.config.enable_stream(depth_profile)
            print(f"默认深度流已配置: {depth_profile}")

            # 2. 配置并使能彩色流 (采用与深度流相同的逻辑模式)
            color_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            if color_profile_list is None:
                print("警告: 未找到彩色传感器。")
            else:
                color_profile = color_profile_list.get_default_video_stream_profile()
                if color_profile is None:
                    print("警告: 未找到默认的彩色流配置。")
                else:
                    self.config.enable_stream(color_profile)
                    print(f"默认彩色流已配置: {color_profile}")

        except (OBError, AttributeError, RuntimeError) as e:
            print(f"严重错误: 配置数据流时出错: {e}")
            state.ros_enabled = False
            return
            
        # 启动 Pipeline
        self.pipeline.start(self.config)
        self.camera_params = self.pipeline.get_camera_param()

        self.is_running = True
        self.thread = threading.Thread(target=self._update_frames, daemon=True)
        self.thread.start()
        print("--- 统一摄像头处理线程已启动 ---")

    def _update_frames(self):
        while self.is_running:
            try:
                frames = self.pipeline.wait_for_frames(100)
                if frames is None:
                    continue

                # 将所有的数据处理都包裹在同一个锁内，确保原子性
                with self.lock:
                    # 处理深度帧
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        height = depth_frame.get_height()
                        width = depth_frame.get_width()
                        scale = depth_frame.get_depth_scale()
                        depth_buffer = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                        depth_uint16 = depth_buffer.reshape((height, width))
                        self.depth_data_map = depth_uint16.astype(np.float32) * scale

                    # 处理彩色帧
                    color_frame = frames.get_color_frame()
                    if color_frame:
                        color_image_buffer = np.asanyarray(color_frame.get_data())
                        decoded_frame = cv2.imdecode(color_image_buffer, cv2.IMREAD_COLOR)
                        # 【核心修正】
                        # 必须将解码后的帧的【副本】存入共享变量中。
                        # 这可以保证任何时候其他线程读取到的都是一张完整的、不受干扰的图像。
                        self.color_frame_data = decoded_frame.copy()
            except Exception as e:
                print(f"更新帧数据时出错: {e}")
            time.sleep(0.01)

    # 以下函数不再需要进行修改，因为写入端已经确保了数据的线程安全
    def get_jpeg_frame(self):
        with self.lock:
            if self.color_frame_data is None: return None
            frame_copy = self.color_frame_data.copy()
        ret, jpeg_buffer = cv2.imencode('.jpg', frame_copy)
        return jpeg_buffer.tobytes() if ret else None

    def capture_image_to_file(self, upload_folder, session_id):
        with self.lock:
            if self.color_frame_data is None: return None, None
            frame_copy = self.color_frame_data.copy()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"capture_{timestamp}.jpg"
        filepath = os.path.join(upload_folder, filename)
        
        os.makedirs(upload_folder, exist_ok=True)
        cv2.imwrite(filepath, frame_copy)
        
        web_path = os.path.join('static', 'sessions', session_id, 'uploads', filename).replace('\\', '/')
        return web_path, filepath

    def get_distance_and_angle(self, u, v):
        depth_in_mm = 0
        image_width = 0

        with self.lock:
            if self.depth_data_map is None: return None
            
            h, w = self.depth_data_map.shape
            if not (0 <= v < h and 0 <= u < w): return None
            
            depth_in_mm = self.depth_data_map[v, u]
            image_width = w

        if depth_in_mm <= 0: return None

        center_u_pixel = image_width / 2
        angle_per_pixel = config.HORIZONTAL_FOV / image_width
        camera_yaw_rad = math.radians((u - center_u_pixel) * angle_per_pixel)

        z_mm = float(depth_in_mm)
        x_mm = z_mm * math.tan(camera_yaw_rad)
        
        z_compensated_mm = z_mm + (config.CAR_RADIUS_M * 1000)
        
        corrected_yaw_rad = math.atan2(x_mm, z_compensated_mm)
        corrected_yaw_deg = math.degrees(corrected_yaw_rad)
        
        distance_m = z_mm / 1000.0

        return {"distance_m": distance_m, "angle_deg": corrected_yaw_deg}

    def release(self):
        if self.is_running:
            print("--- 正在释放统一摄像头资源... ---")
            self.is_running = False
            if hasattr(self, 'thread') and self.thread.is_alive():
                self.thread.join(timeout=2)
            self.pipeline.stop()
            print("--- 统一摄像头已释放。 ---")
