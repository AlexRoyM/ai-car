# -*- coding: utf-8 -*-

import time
import threading
import signal
import sys
import math

# --- 核心依赖导入 ---
import config
import state

# 尝试导入ROS相关的库
try:
    import rospy
    from geometry_msgs.msg import Twist
    # from sensor_msgs.msg import Imu # <--- 修改：不再需要Imu消息
    from nav_msgs.msg import Odometry # <--- 修改：导入新的Odometry消息类型
except ImportError:
    print("\n[错误] 未找到 ROS 环境或相关库（rospy, geometry_msgs, nav_msgs）。")
    print("请确保您已在正确配置了ROS环境的终端中运行此脚本。")
    sys.exit(1)

# --- 全局变量，用于存储机器人状态 ---
class RobotState:
    def __init__(self):
        self.current_yaw = 0.0
        self.lock = threading.Lock() # <--- 修改：重命名为通用锁

robot_state = RobotState()

# --- 自定义的四元数转偏航角函数 (这个函数保持不变) ---
def quaternion_to_yaw(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

# --- 回调函数修改 ---
def odom_callback(msg): # <--- 修改：函数名改为 odom_callback
    """
    处理来自 Odometry 的数据，更新机器人的当前偏航角。
    """
    # Odometry消息的四元数路径是 msg.pose.pose.orientation
    orientation_q = msg.pose.pose.orientation # <--- 修改：数据结构路径不同
    
    yaw = quaternion_to_yaw(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    
    # 使用线程锁来安全地更新全局偏航角变量
    with robot_state.lock:
        robot_state.current_yaw = yaw

# ... (normalize_angle 函数和 _turn_task_imu 函数保持不变, 这里省略)
def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def _turn_task_imu(angle_deg):
    target_rad = math.radians(angle_deg)
    tolerance_rad = math.radians(1.0)
    rate = rospy.Rate(50)
    rospy.sleep(0.1)
    with robot_state.lock:
        initial_yaw = robot_state.current_yaw
    target_yaw = normalize_angle(initial_yaw + target_rad)
    print(f"  [控制] 初始角度: {math.degrees(initial_yaw):.2f}°")
    print(f"  [控制] 目标角度: {math.degrees(target_yaw):.2f}°")
    twist_msg = Twist()
    if angle_deg < 0:
        twist_msg.angular.z = -abs(config.ANGULAR_SPEED)
    else:
        twist_msg.angular.z = abs(config.ANGULAR_SPEED)
    while not rospy.is_shutdown():
        with robot_state.lock:
            current_yaw = robot_state.current_yaw
        state.cmd_vel_pub.publish(twist_msg)
        remaining_rad = normalize_angle(target_yaw - current_yaw)
        if abs(remaining_rad) < tolerance_rad:
            print("  [控制] 已到达目标角度。")
            break
        rate.sleep()
    state.cmd_vel_pub.publish(Twist())
    rospy.sleep(0.1)


def initialize_test_environment():
    """
    初始化ROS环境。
    """
    print("--- 正在初始化ROS测试环境... ---")
    try:
        rospy.init_node('rotation_controller_odom', anonymous=True, disable_signals=True) # <--- 修改：节点名更新
        
        state.ros_enabled = True
        state.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # --- 修改：订阅新的话题和使用新的回调函数 ---
        odom_topic_name = "/odometry/filtered" # <--- 修改：设定新的话题名
        rospy.Subscriber(odom_topic_name, Odometry, odom_callback) # <--- 修改：使用Odometry类型和odom_callback
        
        print(f"* ROS 节点 'rotation_controller_odom' 已启动")
        print(f"* 已连接到速度话题 '/cmd_vel'")
        print(f"* 已订阅里程计话题 '{odom_topic_name}'")
        print(f"* 当前配置的角速度(ANGULAR_SPEED): {config.ANGULAR_SPEED} rad/s")
        print("-------------------------------------\n")
        
        print("正在等待里程计数据...")
        time.sleep(1.5) # 等待建立连接和接收第一条消息
        
        with robot_state.lock:
            if robot_state.current_yaw == 0.0:
                 # 启动时角度为0很正常，但如果一直为0就有问题。这里的警告可能需要调整。
                 # 我们暂时假设它可以工作，如果仍然失败，再增加更复杂的检查。
                 print("[提示] 已开始监听里程计数据。如果机器人静止，初始角度可能为0。")
            else:
                 print("里程计数据接收正常。")
        return True
    except rospy.ROSInitException as e:
        print(f"[严重错误] ROS Master 连接失败: {e}")
        print("请确保 'roscore' 正在运行中。")
        return False

# ... (stop_robot 和 main 函数保持不变, 这里省略)
def stop_robot(sig=None, frame=None):
    print("\n--- 接收到退出信号，正在停止小车... ---")
    if state.ros_enabled and hasattr(state, 'cmd_vel_pub') and state.cmd_vel_pub:
        state.cmd_vel_pub.publish(Twist())
        print("小车已停止。")
    sys.exit(0)

def main():
    if not initialize_test_environment():
        return
    signal.signal(signal.SIGINT, stop_robot)
    signal.signal(signal.SIGTERM, stop_robot)
    print("\n--- 里程计精确旋转控制工具 ---")
    print("输入一个角度值 (如 90, -45)，然后按 Enter。")
    print("正数代表向右转(顺时针)，负数代表向左转(逆时针)。")
    print("输入 'q' 或 按 Ctrl+C 退出程序。\n")
    while not rospy.is_shutdown():
        try:
            angle_str = input("请输入要旋转的角度（°）: ")
            if angle_str.lower() == 'q':
                break
            angle_deg = float(angle_str)
            print(f"\n>>> 准备执行转向: {angle_deg}°...")
            _turn_task_imu(angle_deg)
            print("<<< 转向执行完毕。\n")
            state.cmd_vel_pub.publish(Twist())
            time.sleep(0.5)
        except ValueError:
            print("[错误] 无效输入，请输入一个数字或 'q'。")
        except Exception as e:
            print(f"[程序异常] 发生错误: {e}")
            break
    stop_robot()

if __name__ == '__main__':
    main()
