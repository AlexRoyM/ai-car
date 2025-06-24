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
            "description": "执行一个由多个移动和转向组成的动作序列来控制小车。用于处理包含多个步骤的指令，例如“先左转30度，再前进0.8米”。",
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
    """使用里程计进行闭环移动控制"""
    if not state.ros_enabled: return "错误: ROS未启用。"
    
    print(f"  - 执行闭环移动: 目标距离={distance:.2f}m")
    
    with state.odom_lock:
        state.initial_pose = state.current_pose.copy()

    twist_msg = Twist()
    twist_msg.linear.x = config.MOVE_SPEED if distance > 0 else -config.MOVE_SPEED
    
    rate = rospy.Rate(20)
    start_time = rospy.Time.now()
    timeout_duration = rospy.Duration.from_sec(abs(distance) / config.MOVE_SPEED * 2.5 + 2) # 设置一个合理的超时

    while not rospy.is_shutdown():
        # 检查超时
        if rospy.Time.now() - start_time > timeout_duration:
            print("  - 移动超时！")
            break
            
        with state.odom_lock:
            dist_moved, _ = state.get_pose_diff(state.initial_pose, state.current_pose)

        if dist_moved >= abs(distance):
            print(f"  - 到达目标距离: 已移动 {dist_moved:.2f}m")
            break
        
        state.cmd_vel_pub.publish(twist_msg)
        rate.sleep()
        
    return f"已向前移动约{distance:.2f}米"

def _turn_task(angle_degrees: float):
    """使用里程计进行闭环转向控制"""
    if not state.ros_enabled: return "错误: ROS未启用。"

    print(f"  - 执行闭环转向: 目标角度={angle_degrees:.1f}°")
    
    angle_radians = math.radians(angle_degrees)
    
    with state.odom_lock:
        state.initial_pose = state.current_pose.copy()

    twist_msg = Twist()
    # 保持修复后的转向逻辑
    twist_msg.angular.z = -math.copysign(config.ANGULAR_SPEED, angle_radians)

    rate = rospy.Rate(20)
    start_time = rospy.Time.now()
    timeout_duration = rospy.Duration.from_sec(abs(angle_radians) / config.ANGULAR_SPEED * 2.5 + 2)

    while not rospy.is_shutdown():
        if rospy.Time.now() - start_time > timeout_duration:
            print("  - 转向超时！")
            break

        with state.odom_lock:
            _, angle_turned_rad = state.get_pose_diff(state.initial_pose, state.current_pose)

        if abs(angle_turned_rad) >= abs(angle_radians):
            print(f"  - 到达目标角度: 已旋转 {math.degrees(angle_turned_rad):.1f}°")
            break
        
        state.cmd_vel_pub.publish(twist_msg)
        rate.sleep()

    return f"已旋转约{angle_degrees:.1f}度"


# --- Tool Executor ---
def execute_sequence(actions: list):
    """工具执行器，由LLM调用"""
    if not state.ros_enabled or not state.car_control_lock.acquire(timeout=2):
        return "错误：无法获取小车控制权或ROS未启用。"
    
    # 从main模块导入socketio实例，用于发送状态更新
    from main import socketio
    
    print(f"开始执行动作序列，共 {len(actions)} 个动作...")
    socketio.emit('status_update', {'message': '指令解析完毕，开始执行动作序列...'})
    
    tool_results = []
    try:
        for i, action in enumerate(actions):
            command = action.get("command")
            value = action.get("value", 0)
            
            step_message = f"步骤 {i+1}/{len(actions)}: {command} {value}"
            print(f" {step_message}")
            socketio.emit('status_update', {'message': step_message})

            if command == "move_forward":
                value = max(min(value, config.MAX_MOVE_DISTANCE), -config.MAX_MOVE_DISTANCE)
                result = _move_forward_task(value)
                tool_results.append(result)
            elif command == "move_backward":
                value = max(min(abs(value), config.MAX_MOVE_DISTANCE), 0)
                result = _move_forward_task(-value)
                tool_results.append(result)
            elif command == "turn":
                value = max(min(value, config.MAX_TURN_ANGLE), -config.MAX_TURN_ANGLE)
                result = _turn_task(value)
                tool_results.append(result)
            else:
                tool_results.append(f"未知命令: {command}")
            time.sleep(0.2)
            
        final_summary = "序列执行完毕: " + "；".join(tool_results)
        print(final_summary)
        socketio.emit('status_update', {'message': '动作序列执行完毕，正在生成最终回复...'})
        return final_summary
    except Exception as e:
        print(f"执行序列时出错: {e}")
        return f"执行序列时出错: {e}"
    finally:
        if state.ros_enabled:
            state.cmd_vel_pub.publish(Twist())
        state.car_control_lock.release()

available_functions = {
    "execute_sequence": execute_sequence,
}
