from mcp.server.fastmcp import FastMCP
from typing import List, Any, Optional
from pathlib import Path
import json
from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import Twist
from msgs.sensor_msgs import Image, JointState, Joy
import threading
import time
import logging

logger = logging.getLogger('jokes_mcp')
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

LOCAL_IP = "127.0.0.1"  # Replace with your local IP address
ROSBRIDGE_IP = "127.0.0.1"  # Replace with your rosbridge server IP address
ROSBRIDGE_PORT = 9091

mcp = FastMCP("ros-mcp-server",host="0.0.0.0")
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

@mcp.tool(description="站起来")
def joy_stand_up():
    # 左摇杆按下，axes[9]=1
    axes = [0.0]*8
    buttons = [0]*11
    buttons[9] = 1  # Left Stick Press
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons()
    logger.info("机器人 站起来")
    return "Stand up command sent" if msg is not None else "Failed to send stand up command"

@mcp.tool(description="原地踏步")
def joy_walk_in_place():
    # LB 按下，buttons[4]=1
    axes = [0.0]*8
    buttons = [0]*11
    buttons[4] = 1  # LB
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons()
    logger.info("机器人 原地踏步")
    return "Walk in place command sent" if msg is not None else "Failed to send walk in place command"

@mcp.tool(description="停下")
def joy_stop_walk_in_place():
    # LB 再次按下，buttons[4]=1
    axes = [0.0]*8
    buttons = [0]*11
    buttons[4] = 1  # LB
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons()
    logger.info("机器人 停下")
    return "Stop walk in place command sent" if msg is not None else "Failed to send stop walk in place command"

@mcp.tool(description="前进")
def joy_forward():
    # 左摇杆上推，axes[1]=1.0
    axes = [0.0]*8
    axes[1] = 1.0
    buttons = [0]*11
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=2)
    logger.info("机器人 前进")
    return "Forward command sent" if msg is not None else "Failed to send forward command"

@mcp.tool(description="后退")
def joy_backward():
    # 左摇杆下推，axes[1]=-1.0
    axes = [0.0]*8
    axes[1] = -0.5
    buttons = [0]*11
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=1)
    logger.info("机器人 后退")
    return "Backward command sent" if msg is not None else "Failed to send backward command"

@mcp.tool(description="左转")
def joy_turn_left():
    # 右摇杆左推，axes[3]=1.0
    axes = [0.0]*8
    axes[3] = 0.6
    buttons = [0]*11
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=3)
    logger.info("机器人 左转")
    return "Turn left command sent" if msg is not None else "Failed to send turn left command"

@mcp.tool(description="右转")
def joy_turn_right():
    # 右摇杆右推，axes[3]=-1.0
    axes = [0.0]*8
    axes[3] = -0.6
    buttons = [0]*11
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=3)
    logger.info("机器人 右转")
    return "Turn right command sent" if msg is not None else "Failed to send turn right command"

# @mcp.tool(description="机器人坐下")
# def joy_stop():
#     # RB 按下，buttons[5]=1
#     axes = [0.0]*8
#     buttons = [0]*11
#     buttons[5] = 1  # RB
#     msg = joy.publish(axes, buttons)
#     ws_manager.close()
#     release_joy_buttons()
#     return "Stop command sent" if msg is not None else "Failed to send stop command"


@mcp.tool(description="扭腰")
def joy_turn_waist():
    # 右摇杆右推，axes[3]=-1.0
    axes = [0.0]*8
   #axes[2] = -1.0
    axes[5] = -1.0
    buttons = [0]*11
    buttons[0] = 1
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    logger.info("机器人 扭腰")
    return "Turn waist command sent" if msg is not None else "Failed to send turn waist command"

@mcp.tool(description="机器人劈叉、一字马或分腿")
def joy_Split():
    axes = [0.0]*8
    buttons = [0]*11
    axes[5] = -1.0  # RT
    buttons[1] = 1  # B
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=1)
    return "Split command sent" if msg is not None else "Failed to send Split command"

@mcp.tool(description="机器人左右摇摆或平衡")
def joy_balance():
    axes = [0.0]*8
    buttons = [0]*11
    axes[5] = -1.0  # RT
    buttons[2] = 1  # X
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=1)
    return "Maintain balance command sent" if msg is not None else "Failed to send Maintain balance command"

@mcp.tool(description="机器人压腿或拉伸")
def joy_Leg_stretches():
    axes = [0.0]*8
    buttons = [0]*11
    axes[5] = -1.0  # RT
    buttons[3] = 1  # Y
    msg = joy.publish(axes, buttons)
    ws_manager.close()
    release_joy_buttons(delay=1)
    return "Leg stretches command sent" if msg is not None else "Failed to send Leg stretches command"


if __name__ == "__main__":
    mcp.run(transport="stdio")
