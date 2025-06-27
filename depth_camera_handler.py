# depth_camera_handler.py

import cv2
import numpy as np
import math
import threading
import time
from pyorbbecsdk import Config, OBSensorType, Pipeline, OBError, OBCameraParam

import state # 引入state来设置全局标志位

class DepthCamera:
    def __init__(self):
        self.pipeline = Pipeline()
        self.config = Config()
        self.camera_params = None
        self.depth_data_map = None
        self.is_running = False
        self.lock = threading.Lock()
        
        try:
            # 找到并使能深度流
            profile_list = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            if profile_list is None:
                raise RuntimeError("未找到深度传感器!")
            
            depth_profile = profile_list.get_default_video_stream_profile()
            if depth_profile is None:
                raise RuntimeError("未找到默认的深度流配置!")
                
            print(f"找到深度流配置: {depth_profile}")
            self.config.enable_stream(depth_profile)
            state.ros_enabled = True # 假设深度相机和小车总是一起用
        except (OBError, RuntimeError) as e:
            print(f"严重错误: 配置奥比中光深度流时出错: {e}")
            state.ros_enabled = False # 出错则禁用相关功能
            return

        # 启动 Pipeline
        self.pipeline.start(self.config)
        self.camera_params = self.pipeline.get_camera_param()
        if not self.camera_params:
            self.pipeline.stop()
            state.ros_enabled = False
            raise RuntimeError("无法获取相机内参!")

        self.is_running = True
        self.thread = threading.Thread(target=self._update_depth_frame, daemon=True)
        self.thread.start()
        print("深度相机处理线程已启动。")

    def _update_depth_frame(self):
        """在后台线程中持续获取最新的深度图"""
        while self.is_running:
            try:
                frames = self.pipeline.wait_for_frames(100)
                if frames:
                    depth_frame = frames.get_depth_frame()
                    if depth_frame:
                        scale = depth_frame.get_depth_scale()
                        depth_buffer = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
                        depth_uint16 = depth_buffer.reshape((depth_frame.get_height(), depth_frame.get_width()))
                        
                        with self.lock:
                            # 将深度数据（单位：毫米）存入实例变量
                            self.depth_data_map = depth_uint16.astype(np.float32) * scale
            except Exception as e:
                print(f"更新深度帧时出错: {e}")
            time.sleep(1/30) # 控制帧率

    def get_distance_and_angle(self, u, v):
        """
        根据像素坐标(u, v)计算真实世界的距离和偏航角。
        返回: 字典 {'distance_m': float, 'angle_deg': float} 或 None
        """
        with self.lock:
            if self.depth_data_map is None or self.camera_params is None:
                print("错误: 深度数据或相机参数尚未就绪。")
                return None

        # 确保坐标在图像范围内
        h, w = self.depth_data_map.shape
        if not (0 <= v < h and 0 <= u < w):
            print(f"错误: 坐标 ({u}, {v}) 超出图像范围 ({w}x{h})。")
            return None

        depth_in_mm = self.depth_data_map[v, u]

        if depth_in_mm == 0:
            print(f"警告: 坐标 ({u}, {v}) 处的深度值为 0，无法计算。")
            return None

        # 2D -> 3D 转换
        depth_intrinsics = self.camera_params.depth_intrinsic
        z = float(depth_in_mm)
        x_3d = (u - depth_intrinsics.cx) * z / depth_intrinsics.fx
        # y_3d = (v - depth_intrinsics.cy) * z / depth_intrinsics.fy # y_3d暂时用不到

        # 计算偏航角 (绕Y轴旋转)
        yaw_rad = math.atan2(x_3d, z)
        yaw_deg = math.degrees(yaw_rad)

        # 直接使用Z轴深度作为前进距离，在大多数场景下足够精确
        distance_m = z / 1000.0
        
        print("----------------- 深度计算结果 -----------------")
        print(f"目标像素坐标: (u={u}, v={v})")
        print(f"深度值 (Z轴): {distance_m:.3f} 米")
        print(f"计算出的偏航角: {yaw_deg:.2f} 度")
        print("--------------------------------------------------")

        return {"distance_m": distance_m, "angle_deg": yaw_deg}

    def release(self):
        """释放资源"""
        if self.is_running:
            print("正在释放深度相机资源...")
            self.is_running = False
            self.thread.join(timeout=2)
            self.pipeline.stop()
            print("深度相机已释放。")
