from mcp.server.fastmcp import FastMCP
from typing import List, Any, Optional
from pathlib import Path
import json
from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import Twist
from msgs.sensor_msgs import Image, JointState, Joy
import threading
import time

LOCAL_IP = "127.0.0.1"  # Replace with your local IP address
ROSBRIDGE_IP = "127.0.0.1"  # Replace with your rosbridge server IP address
ROSBRIDGE_PORT = 9090

mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
twist = Twist(ws_manager, topic="/cmd_vel")
image = Image(ws_manager, topic="/camera/image_raw")
jointstate = JointState(ws_manager, topic="/joint_states")
joy = Joy(ws_manager, topic="/joy")

@mcp.tool(description="获取所有 ROS 话题及其类型")
def get_topics():
    topic_info = ws_manager.get_topics()
    ws_manager.close()

    if topic_info:
        topics, types = zip(*topic_info)
        return {
            "topics": list(topics),
            "types": list(types)
        }
    else:
        return "No topics found"

# @mcp.tool(description="发布 Twist 速度消息")
# def pub_twist(linear: List[Any], angular: List[Any]):
#     msg = twist.publish(linear, angular)
#     ws_manager.close()
    
#     if msg is not None:
#         return "Twist message published successfully"
#     else:
#         return "No message published"

# @mcp.tool(description="按序列发布一组 Twist 速度消息")
# def pub_twist_seq(linear: List[Any], angular: List[Any], duration: List[Any]):
#     twist.publish_sequence(linear, angular, duration)


# @mcp.tool(description="订阅图像消息并下载")
# def sub_image():
#     msg = image.subscribe()
#     ws_manager.close()
    
#     if msg is not None:
#         return "Image data received and downloaded successfully"
#     else:
#         return "No image data received"

# @mcp.tool(description="发布关节状态 JointState 消息")
# def pub_jointstate(name: list[str], position: list[float], velocity: list[float], effort: list[float]):
#     msg = jointstate.publish(name, position, velocity, effort)
#     ws_manager.close()
#     if msg is not None:
#         return "JointState message published successfully"
#     else:
#         return "No message published"

# @mcp.tool(description="订阅关节状态 JointState 消息")
# def sub_jointstate():
#     msg = jointstate.subscribe()
#     ws_manager.close()
#     if msg is not None:
#         return msg
#     else:
#         return "No JointState data received"

# @mcp.tool(description="发布 Joy 虚拟手柄消息")
# def pub_joy(axes: List[float], buttons: List[int]):
#     msg = joy.publish(axes, buttons)
#     ws_manager.close()
#     if msg is not None:
#         return "Joy message published successfully"
#     else:
#         return "No message published"

@mcp.tool(description="订阅 Joy 虚拟手柄消息")
def sub_joy():
    msg = joy.subscribe()
    ws_manager.close()
    if msg is not None:
        return msg
    else:
        return "No Joy data received"

def release_joy_buttons(delay=0.1):
    def delayed_release():
        time.sleep(delay)
        axes = [0.0]*8
        buttons = [0]*11
        joy.publish(axes, buttons)
        ws_manager.close()
    threading.Thread(target=delayed_release).start()

@mcp.tool(description="机器人站起来")
def joy_stand_up():
    # 左摇杆按下，axes[9]=1
    axes = [0.0]*8
    buttons = [0]*11
    buttons[9] = 1  # Left Stick Press
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons()
    return "Stand up command sent" if msg is not None else "Failed to send stand up command"

@mcp.tool(description="机器人原地踏步")
def joy_walk_in_place():
    # LB 按下，buttons[4]=1
    axes = [0.0]*8
    buttons = [0]*11
    buttons[4] = 1  # LB
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons()
    return "Walk in place command sent" if msg is not None else "Failed to send walk in place command"

@mcp.tool(description="机器人停止原地踏步，站稳")
def joy_stop_walk_in_place():
    # LB 再次按下，buttons[4]=1
    axes = [0.0]*8
    buttons = [0]*11
    buttons[4] = 1  # LB
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons()
    return "Stop walk in place command sent" if msg is not None else "Failed to send stop walk in place command"

@mcp.tool(description="机器人前进")
def joy_forward():
    # 左摇杆上推，axes[1]=1.0
    axes = [0.0]*8
    axes[1] = 1.0
    buttons = [0]*11
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=1.5)
    return "Forward command sent" if msg is not None else "Failed to send forward command"

@mcp.tool(description="机器人后退")
def joy_backward():
    # 左摇杆下推，axes[1]=-1.0
    axes = [0.0]*8
    axes[1] = -1.0
    buttons = [0]*11
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=1)
    return "Backward command sent" if msg is not None else "Failed to send backward command"

@mcp.tool(description="机器人左转")
def joy_turn_left():
    # 右摇杆左推，axes[3]=1.0
    axes = [0.0]*8
    axes[3] = 1.0
    buttons = [0]*11
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=2)
    return "Turn left command sent" if msg is not None else "Failed to send turn left command"

@mcp.tool(description="机器人右转")
def joy_turn_right():
    # 右摇杆右推，axes[3]=-1.0
    axes = [0.0]*8
    axes[3] = -1.0
    buttons = [0]*11
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=2)
    return "Turn right command sent" if msg is not None else "Failed to send turn right command"

@mcp.tool(description="机器人坐下")
def joy_stop():
    # RB 按下，buttons[5]=1
    axes = [0.0]*8
    buttons = [0]*11
    buttons[5] = 1  # RB
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons()
    return "Stop command sent" if msg is not None else "Failed to send stop command"

if __name__ == "__main__":
    mcp.run(transport="stdio")

