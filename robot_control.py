import math
import time
import config
import state
import threading
# ROS消息类型
try:
    import rospy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
except ImportError:
    # 即使ROS未启用，也定义一个假的Twist以便类型提示
    class Twist: pass
    class Odometry: pass
# --- 用于存储基于里程计的机器人状态 ---
class RobotState:
    def __init__(self):
        self.current_yaw = 0.0
        self.lock = threading.Lock()

robot_state = RobotState()

# --- 从四元数计算偏航角的辅助函数 ---
def quaternion_to_yaw(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

# --- 处理里程计消息的回调函数 ---
def odom_callback(msg: Odometry):
    """
    处理来自 /odometry/filtered 的数据，更新机器人的当前偏航角。
    """
    orientation_q = msg.pose.pose.orientation
    yaw = quaternion_to_yaw(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    
    with robot_state.lock:
        robot_state.current_yaw = yaw

# --- 将角度归一化到-pi到pi之间的辅助函数 ---
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# --- Tool Definition ---
CAR_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "execute_sequence",
            "description": "执行一个由多个移动和转向组成的动作序列来控制小车。用于处理包含多个步骤的指令，例如“先左转15.0度，再前进0.8米”。",
            "parameters": {
                "type": "object",
                "properties": {
                    "actions": {
                        "type": "array",
                        "description": "一个包含多个动作指令的列表，将按顺序执行。",
                        "items": {
                            "type": "object",
                            "properties": {
                                "command": {
                                    "type": "string",
                                    "description": "要执行的命令类型。",
                                    "enum": ["turn", "move_forward", "move_backward"]
                                },
                                "value": {
                                    "type": "number",
                                    "description": "命令的参数值。对于'turn'是角度(°)，正数右转，负数左转。对于'move_forward'或'move_backward'是距离(m)。"
                                }
                            },
                            "required": ["command", "value"]
                        }
                    }
                },
                "required": ["actions"]
            }
        }
    }
]

# --- 内部物理控制函数 ---
def _move_forward_task(distance: float):
    if not state.ros_enabled: return "错误: ROS未启用。"
    
    # 使用config中的参数
    duration = abs(distance) / config.MOVE_SPEED
    twist_msg = Twist()
    twist_msg.linear.x = config.MOVE_SPEED if distance > 0 else -config.MOVE_SPEED
    
    print(f"  - 执行移动: 距离={distance:.2f}m, 速度={twist_msg.linear.x}m/s, 持续时间={duration:.2f}s")
    start_time = rospy.Time.now()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and rospy.Time.now() - start_time < rospy.Duration.from_sec(duration):
        state.cmd_vel_pub.publish(twist_msg)
        rate.sleep()
    return f"已向前移动{distance:.2f}米"

def _turn_task(angle_degrees: float):
    if not state.ros_enabled: return "错误: ROS未启用。"

    target_rad = math.radians(angle_degrees)
    # 容忍误差
    tolerance_rad = math.radians(1.2)
    rate = rospy.Rate(50)

    # 短暂延时，确保回调函数已经更新了初始角度
    rospy.sleep(0.1)

    with robot_state.lock:
        initial_yaw = robot_state.current_yaw

    target_yaw = normalize_angle(initial_yaw + target_rad)
    
    print(f"  - [精确转向] 初始角度: {math.degrees(initial_yaw):.2f}°")
    print(f"  - [精确转向] 目标角度: {math.degrees(target_yaw):.2f}°")
    
    twist_msg = Twist()
    # 根据角度正负决定旋转方向
    if angle_degrees < 0:
        twist_msg.angular.z = abs(config.ANGULAR_SPEED) # 左转
    else:
        twist_msg.angular.z = -abs(config.ANGULAR_SPEED)  # 右转
    
    while not rospy.is_shutdown():
        with robot_state.lock:
            current_yaw = robot_state.current_yaw
        
        # 持续发布速度指令
        state.cmd_vel_pub.publish(twist_msg)
        
        # 计算剩余角度
        remaining_rad = normalize_angle(target_yaw - current_yaw)
        
        # 如果剩余角度小于容忍误差，则停止
        if abs(remaining_rad) < tolerance_rad:
            print(f"  - [精确转向] 已到达目标角度，剩余误差: {math.degrees(remaining_rad):.2f}°")
            break
            
        rate.sleep()
    
    # 确保循环结束后机器人完全停止
    state.cmd_vel_pub.publish(Twist())
    rospy.sleep(0.1) # 短暂延时确保停止指令被发送
    
    return f"已旋转{angle_degrees:.1f}度"

# --- Tool Executor ---
def execute_sequence(actions: list):
    """工具执行器，由LLM调用"""
    if not state.ros_enabled or not state.car_control_lock.acquire(timeout=2):
        return "错误：无法获取小车控制权或ROS未启用。"
    
    print(f"开始执行动作序列，共 {len(actions)} 个动作...")
    tool_results = []
    try:
        for i, action in enumerate(actions):
            command = action.get("command")
            value = action.get("value", 0)
            print(f" 步骤 {i+1}/{len(actions)}: command={command}, value={value}")

            if command == "move_forward":
                # 使用config中的参数进行限制
                value = max(min(value, config.MAX_MOVE_DISTANCE), -config.MAX_MOVE_DISTANCE)
                result = _move_forward_task(value)
                tool_results.append(result)
            elif command == "move_backward":
                value = max(min(abs(value), config.MAX_MOVE_DISTANCE), 0)
                result = _move_forward_task(-value)
                tool_results.append(result)
            elif command == "turn":
                original_angle = value

                # 逻辑分支 1: 检查角度是否小于容忍值，若是则跳过
                if abs(original_angle) <= config.TURN_TOLERANCE_DEG:
                    result = f"转向角度 {original_angle:.1f}° 小于容忍值 {config.TURN_TOLERANCE_DEG}°，已跳过。"
                    print(f"  - {result}")
                    tool_results.append(result)
                    time.sleep(0.2)
                    continue # 直接进行下一个动作

                # 应用最大角度限制
                angle_to_execute = max(min(original_angle, config.MAX_TURN_ANGLE), -config.MAX_TURN_ANGLE)

                # 逻辑分支 2: 检查是否需要执行补偿动作
                if abs(angle_to_execute) <= 5.0 * config.TURN_TOLERANCE_DEG:
                    print(f"  - 执行补偿转向: 目标角度 {angle_to_execute:.1f}°")
                    
                    # 补偿动作1: 反向转动10度
                    backward_angle = -math.copysign(10.0, angle_to_execute)
                    print(f"    - 补偿动作1: 反向转动 {backward_angle:.1f}°")
                    result1 = _turn_task(backward_angle)
                    time.sleep(0.2)

                    # 补偿动作2: 正向转动 (目标角度 + 10度补偿)
                    forward_angle = angle_to_execute + math.copysign(10.0, angle_to_execute)
                    print(f"    - 补偿动作2: 正向转动 {forward_angle:.1f}°")
                    result2 = _turn_task(forward_angle)
                    
                    result = f"{result1}；{result2}"
                    tool_results.append(result)
                
                # 逻辑分支 3: 执行常规转向 (角度较大时)
                else:
                    print(f"  - 执行常规转向: 目标角度 {angle_to_execute:.1f}°")
                    result = _turn_task(angle_to_execute)
                    tool_results.append(result)
            else:
                tool_results.append(f"未知命令: {command}")
            time.sleep(0.2) # 动作间短暂延迟
            
        final_summary = "序列执行完毕: " + "；".join(tool_results)
        print(final_summary)
        return final_summary
    except Exception as e:
        print(f"执行序列时出错: {e}")
        return f"执行序列时出错: {e}"
    finally:
        # 确保动作结束后小车停止
        if state.ros_enabled:
            state.cmd_vel_pub.publish(Twist())
        state.car_control_lock.release()

# 将可用的函数映射到一个字典，方便LLM处理器调用
available_functions = {
    "execute_sequence": execute_sequence,
}
