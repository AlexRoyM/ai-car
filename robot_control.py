import math
import time
import config
import state

# ROS消息类型
try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:
    # 即使ROS未启用，也定义一个假的Twist以便类型提示
    class Twist: pass

# --- Tool Definition ---
CAR_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "execute_sequence",
            "description": "执行一个由多个移动和转向组成的动作序列来控制小车。用于处理包含多个步骤的指令，例如“先左转30.0度，再前进0.8米”。",
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
    
    angle_radians = math.radians(angle_degrees)
    duration = abs(angle_radians) / config.ANGULAR_SPEED
    twist_msg = Twist()

    twist_msg.angular.z = -math.copysign(config.ANGULAR_SPEED, angle_radians)
    
    print(f"  - 执行转向: 角度={angle_degrees:.1f}°, 目标角速度={twist_msg.angular.z:.2f}rad/s, 持续时间={duration:.2f}s")
    rate = rospy.Rate(50)
    start_time = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() - start_time < rospy.Duration.from_sec(duration):
        state.cmd_vel_pub.publish(twist_msg)
        rate.sleep()
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
                # 使用config中的参数进行限制
                value = max(min(value, config.MAX_TURN_ANGLE), -config.MAX_TURN_ANGLE)
                result = _turn_task(value)
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
