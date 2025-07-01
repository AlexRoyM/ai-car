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
    """
    if color_frame is None:
        return None
    
    width = color_frame.get_width()
    height = color_frame.get_height()
    color_format = color_frame.get_format()
    data = np.asanyarray(color_frame.get_data())

    try:
        if color_format == OBFormat.RGB:
            return cv2.cvtColor(data.reshape((height, width, 3)), cv2.COLOR_RGB2BGR)
        elif color_format == OBFormat.BGR:
            return data.reshape((height, width, 3))
        elif color_format == OBFormat.YUYV:
            return cv2.cvtColor(data.reshape((height, width, 2)), cv2.COLOR_YUV2BGR_YUY2)
        elif color_format == OBFormat.MJPG:
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
        self.latest_depth_frame = None
        self.camera_params = None
        self.latest_jpeg_bytes = None        
        
        self.is_running = False
        
        print("--- 正在配置统一摄像头... ---")
        try:
            # 1. 配置彩色流 - 优先使用 MJPG 格式
            color_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
            if not color_profile_list:
                raise RuntimeError("错误: 未找到彩色传感器!")
            
            try:
                color_profile = color_profile_list.get_video_stream_profile(640, 480, OBFormat.MJPG, 30)
            except OBError:
                color_profile = color_profile_list.get_default_video_stream_profile()
            self.config.enable_stream(color_profile)
            print(f"彩色流已配置: {color_profile.get_width()}x{color_profile.get_height()} @ {color_profile.get_fps()}fps, 格式={color_profile.get_format()}")

            # 2. 配置深度流
            #depth_profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            #if not depth_profile_list:
                #raise RuntimeError("错误: 未找到深度传感器!")
            
            #try:
                #depth_profile = depth_profile_list.get_video_stream_profile(640, 480, OBFormat.Y16, 30)
            #except OBError:
                #depth_profile = depth_profile_list.get_default_video_stream_profile()
            #self.config.enable_stream(depth_profile)
            #print(f"深度流已配置: {depth_profile.get_width()}x{depth_profile.get_height()} @ {depth_profile.get_fps()}fps, 格式={depth_profile.get_format()}")

            # 3. 设置硬件对齐模式
            #self.config.set_align_mode(OBAlignMode.HW_MODE)

        except (OBError, RuntimeError) as e:
            print(f"严重错误: 配置数据流时出错: {e}")
            state.ros_enabled = False
            raise e

        # 4. 启动 Pipeline
        self.pipeline.start(self.config)
        #self.camera_params = self.pipeline.get_camera_param()
        print("--- 摄像头 Pipeline 已启动 ---")

        self.is_running = True
        self.thread = threading.Thread(target=self._update_frames, daemon=True)
        self.thread.start()
        print("--- 统一摄像头处理线程已启动 ---")

    def _update_frames(self):
        """后台线程，持续获取对齐后的彩色帧和深度帧"""
        while self.is_running:
            try:
                # 使用wait_for_frames阻塞等待，而不是高频轮询
                frames = self.pipeline.wait_for_frames(100)
                if frames is None:
                    continue

                color_frame = frames.get_color_frame()
                #depth_frame = frames.get_depth_frame()
                #if depth_frame is None or color_frame is None:
                    #continue
                #with self.lock:
                #    # 直接保存帧对象，不做任何处理
                #    if color_frame:
                #        self.latest_color_frame = color_frame
                #    if depth_frame:
                #        self.latest_depth_frame = depth_frame
                bgr_image = _convert_color_frame_to_bgr_image(color_frame)
                if bgr_image is None:
                    continue
                ret, jpeg_buffer = cv2.imencode('.jpg', bgr_image)
                if not ret:
                    continue                                        
                with self.lock:
                    self.latest_color_frame = bgr_image.copy()
                    self.latest_jpeg_bytes = jpeg_buffer.tobytes()
                    #if depth_frame:
                         #self.latest_depth_frame = depth_frame                                        
                        
            except Exception as e:
                print(f"更新帧数据时出错: {e}")

    def get_jpeg_frame(self):
        """
        为网页视频流提供JPEG编码的帧。
        """
        with self.lock:
            return self.latest_jpeg_bytes        
            #if self.latest_color_frame is None:
                #return None
            
            ## 如果格式是MJPG，直接返回原始数据
            #if self.latest_color_frame.get_format() == OBFormat.MJPG:
            #    data = np.asanyarray(self.latest_color_frame.get_data())
            #    return data.tobytes()
            
            # 否则需要转换
            #bgr_image = self.latest_color_frame.copy()
        
        # 在锁外进行转换
        #bgr_image = _convert_color_frame_to_bgr_image(color_frame)
        #if bgr_image is None:
            #return None
        
        #ret, jpeg_buffer = cv2.imencode('.jpg', bgr_image)
        #return jpeg_buffer.tobytes() if ret else None

    def capture_image_to_file(self, upload_folder, session_id):
        """
        捕获当前帧，将彩色图像保存到文件。
        返回: (web_path, file_path) 或 (None, None)
        """
        #with self.lock:
            #if self.latest_color_frame is None:
                #return None, None
            #color_frame_copy = self.latest_color_frame.copy()

        #bgr_image = _convert_color_frame_to_bgr_image(color_frame_copy)
        with self.lock:
            bgr_image = self.latest_color_frame.copy()
        #print(f"111")
        if bgr_image is None:
            return None, None
            

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"capture_{timestamp}.jpg"
        filepath = os.path.join(upload_folder, filename)
        
        os.makedirs(upload_folder, exist_ok=True)
        cv2.imwrite(filepath, bgr_image)
        
        web_path = os.path.join('static', 'sessions', session_id, 'uploads', filename).replace('\\', '/')
        return web_path, filepath

    def get_distance_and_angle(self, u, v):
        """
        根据给定的像素坐标(u, v)计算距离和角度。
        """
        with self.lock:
            if self.latest_depth_frame is None:
                print("错误: 深度帧尚未就绪。")
                return None
            depth_frame = self.latest_depth_frame.copy()
        
        #if self.camera_params is None:
            #print("错误: 相机参数尚未就绪。")
            #return None

        # 处理深度数据
        scale = depth_frame.get_depth_scale()
        depth_buffer = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
        depth_map = depth_buffer.reshape((depth_frame.get_height(), depth_frame.get_width()))
        depth_map = depth_map.astype(np.float32) * scale

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
