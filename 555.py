import time
import threading
import signal
import sys

# --- 核心依赖导入 ---
# 从您的项目中导入必要的模块和函数
import config
import state
from robot_control import _turn_task

# 尝试导入ROS相关的库，如果失败则优雅退出
try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:
    print("\n[错误] 未找到 ROS 环境或 'rospy' 库。")
    print("请确保您已在正确配置了ROS环境的终端中运行此脚本。")
    print("例如，先执行 'source /opt/ros/noetic/setup.bash'")
    sys.exit(1)


def initialize_test_environment():
    """
    初始化一个最小化的ROS环境，用于测试小车控制。
    """
    print("--- 正在初始化ROS测试环境... ---")
    try:
        # 初始化ROS节点
        rospy.init_node('rotation_calibrator', anonymous=True, disable_signals=True)
        
        # 设置全局状态
        state.ros_enabled = True
        state.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        print(f"* ROS 节点 'rotation_calibrator' 已启动")
        print(f"* 已连接到话题 '/cmd_vel'")
        print(f"* 当前配置的角速度(ANGULAR_SPEED): {config.ANGULAR_SPEED} rad/s")
        print("-------------------------------------\n")
        # 等待发布者建立连接
        time.sleep(1)
        return True
    except rospy.ROSInitException as e:
        print(f"[严重错误] ROS Master 连接失败: {e}")
        print("请确保 'roscore' 正在运行中。")
        return False

def stop_robot(sig=None, frame=None):
    """
    确保在退出时停止机器人。
    """
    print("\n--- 接收到退出信号，正在停止小车... ---")
    if state.ros_enabled and state.cmd_vel_pub:
        state.cmd_vel_pub.publish(Twist()) # 发布一个全零速度指令
        print("小车已停止。")
    sys.exit(0)

def main():
    """
    主函数，用于接收用户输入的角度并执行转向测试。
    """
    if not initialize_test_environment():
        return

    # 绑定Ctrl+C信号到停止函数
    signal.signal(signal.SIGINT, stop_robot)
    signal.signal(signal.SIGTERM, stop_robot)

    print("--- 旋转角度校准工具 ---")
    print("输入一个角度值 (如 90, -45)，然后按 Enter。")
    print("正数代表向右转，负数代表向左转。")
    print("输入 'q' 或 按 Ctrl+C 退出程序。\n")

    while not rospy.is_shutdown():
        try:
            # 获取用户输入
            angle_str = input("请输入要旋转的角度（°）: ")

            if angle_str.lower() == 'q':
                break

            # 将输入转换为浮点数
            angle_deg = float(angle_str)
            
            # 调用您项目中的转向函数
            print(f"\n>>> 准备执行转向: {angle_deg}°...")
            _turn_task(angle_deg)
            print("<<< 转向执行完毕。\n")

            # 确保小车在两次指令之间是停止的
            state.cmd_vel_pub.publish(Twist())
            time.sleep(0.5)

        except ValueError:
            print("[错误] 无效输入，请输入一个数字（如 90, -45.5）或 'q'。")
        except Exception as e:
            print(f"[程序异常] 发生错误: {e}")
            break

    stop_robot()

if __name__ == '__main__':
    main()
