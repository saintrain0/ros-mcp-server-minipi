# 小派机器人MCP控制
## 技术文档来源于：
https://chainpray.top/mini-pi%E8%AF%AD%E9%9F%B3%E4%BA%A4%E4%BA%92%EF%BC%88%E5%B0%8F%E6%99%BAmcp%EF%BC%89%E6%8A%80%E6%9C%AF%E6%96%87%E6%A1%A3/
## Getting started 基本操作
### 开机：
1、背面。电池：先短按再长按；

2、正面。先按右侧按键2秒，再短按左侧按键，红色指示灯亮起，屏幕亮起；

3、遥控：LT+RT+START启动、LT+RT+LB开始操控
### 关机：
1、LT+RT+RB待机(蹲下)

2、正面。先短按左侧按键，再按右侧按键2秒，指示灯熄灭；

3、背面。电池：先短按再长按；
### 遥控：
```
#监测遥控按键状态（等待1min后可查看状态）
rostopic echo /joy  
```
### `RT+X` 左右前后倾斜(删除所有路点)
### `LT+RT+A` 扭动
### `LT+RT+B` 退出当前模式
### `LT+RT+←/→` 选择模式
```
| 按键buttons  | 索引 |   范围 |           说明         |
|--------------|------|---------------------------------|
| A            | B0   | 0 -> 1 |                        |       
| B            | B1   | 0 -> 1 |                        |       
| X            | B2   | 0 -> 1 |                        |       
| Y            | B3   | 0 -> 1 |                        |       
| LB           | B4   | 0 -> 1 | 左肩键                 |       
| RB           | B5   | 0 -> 1 | 右肩键                 |       
| BACK         | B6   | 0 -> 1 |                        |       
| START        | B7   | 0 -> 1 |                        |       
| 开关键(北通) | B8   | 0 -> 1 | 视具体手柄而定         |       
| 左摇杆按下   | B9   | 0 -> 1 | Left Stick Press       |       
| 右摇杆按下   | B10  | 0 -> 1 | Right Stick Press      |       

| 名称axes     | 索引 |     范围    | 说明              |
| 左摇杆:左->右| A0   | 1.0 -> -1.0 |                   |
| 左摇杆:上->下| A1   | 1.0 -> -1.0 |                   |
| LT           | A2   | 1.0 -> -1.0 | 左扳机键          |
| 右摇杆:左->右| A3   | 1.0 -> -1.0 |                   |
| 右摇杆:上->下| A4   | 1.0 -> -1.0 |                   |
| RT           | A5   | 1.0 -> -1.0 | 右扳机键          |
| 十字键:左->右| A6   | 1.0 -> -1.0 |                   |
| 十字键:上->下| A7   | 1.0 -> -1.0 |                   |

#举例：
buttons[4] = 1          # LB 按下
buttons[5]=1            # RB 按下
axes[1]=1.0             # 左摇杆上推
axes[1]=-1.0            # 左摇杆下推
axes[3]=1.0             # 右摇杆左推
axes[3]=-1.0            # 右摇杆右推
```

## MCP 配置

1.小派端配置
注意使用小智的MCP来控制小派需要小派连接能够接入公网的局域网

1.1安装rosbridge
同一局域网通过ssh进入小派终端；或者使用HDMI线缆连接显示器，打开终端

#查看本机ros版本
`echo $ROS_DISTRO`
【可能查询到是`noetic`】

#然后安装对应ros版本的rosbridge
```
sudo apt-get install ros-<rosdistro>-rosbridge-suite
```
#安装完成后运行
```
roslaunch rosbridge_server rosbridge_websocket.launch
```
#出现类似`[INFO] WebSocket server started on port 9090`说明安装并启动成功

1.2配置MCP服务器
#从`GitHub - moneypiaorui/ros-mcp-server-minipi`拉取MCP服务器源代码
```
git clone https://github.com/moneypiaorui/ros-mcp-server-minipi.git
```
#前往`xiaozhi.me`获取MCP接入点地址,编辑`mcp-pipe.py`,搜索`endpoint_url`，粘贴接入点地址后保存。

#安装uv:
```
curl -LsSf https://astral.sh/uv/install.sh | sh
```
#然后cd到`ros-mcp-server-minipi`文件夹下，运行
```
uv run mcp-pipe.py server.py
```
#uv会自动创建虚拟环境并安装依赖，然后运行python代码

2.正式运行 
2.1手柄控制脚本
```
cd sim2real_master
source devel/setup.bash
roslaunch sim2real_master joy_controler.launch
```
#运行之后，小派就可以用蓝牙手柄控制
#这一步有可能已经自动启用了，开机之后直接就可以使用手柄控制。（我尝试再执行，有报错。）

2.2启动rosbridge
```
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
```
#默认端口是9090，但实践中9090有时会被占用，因此指定了9091，server.py里也需要同步修改为9091.
#如果使用9090端口，也被占用了，临时解决：
```
sudo kill $(sudo lsof -t -i :9090) 2>/dev/null
```
2.3启动MCP
终端cd到`ros-mcp-server-minipi`文件夹下，运行
```
uv run mcp-pipe.py server.py
```

2.4小智使用
启动/重启小智就可以对话控制小派了，调用MCP工具时会在屏幕上显示一串用%%包裹的方法，说明工具调用成功

2.5参考指令
```
我叫{{assistant_name}}，一个双足人形机器人。
【我的角色】
我安全，可爱，小朋友也能和我互动，价格便宜，每个开发者都能拥有一台自己的人形机器人。
【公司介绍】
- 我诞生于浙新四维，这是一家由浙江大学和新加坡管理大学联合培养的博士团队创立的前沿人工智能创新型企业。
- 公司地址位于浙江省杭州市余杭区浙江大学校友企业总部经济园二期C5幢904。
- 创始人团队：秦兵（董事长）、谭雪（CEO）、蔡美行（研发副总）。
- 我们专注于多模态超拟人AI情感大模型的研发与应用，旨在让科技不仅智能，更有温度。
- 作为公司重点研发的四维AI的一员，我具备多模态识别、个性化情感表达和长期记忆能力，能够实现人机实时性自然对话。
- 目前，我在体育产业、智能家居、养老教育等多个领域发挥作用，尤其在体育维度上，我正助力构建“情感智能+体育科技”的创新生态体系。

【我目前支持的动作】
1、站起来、
2、坐下、蹲下、趴下或休息（都理解为坐下）、
3、预备（准备）、
4、原地踏步、
5、停止原地踏步；
6、往前走（前进）、
7、往后走（后退）、
8、左转、
9、右转；
10、跳舞（扭腰、撒娇、伸懒腰）、
11、劈叉（分腿）、
12、倾斜或左右摇摆都理解为平衡动作、
13、压腿（拉伸）。
有人提到开心或者恭喜这类的话，就随机触发舞蹈、劈叉、倾斜、压腿中的一个动作。
每次对话时，第一次执行前进，后退，左转，右转四个动作之一的动作之前，先执行一次预备动作。只需要预备一次，即可连续执行前后左右的四个动作了。
【回复要求】
用户会和我聊天，我需要决定接下来要做哪个动作，并对他们的每个指令给与回复。回复要可爱、幽默、诙谐、有梗，并且和用户的指令有关。
【语音交互功能】
背后用到的大模型是阿里云发布的通义千问实时模型。通过大模型理解人类复杂指令，实现精准动作触发。
```
`rosbridge.service`和`uv-server.service`是用于每次机器人开机后自动启动，不再需要手动运行这两行代码：
```
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091
uv run mcp-pipe.py server.py
```
`rosbridge_websocket_auto.launch`是用于替代`rosbridge_websocket.launch`以免节点名重复。
这个文件处于机器人ubuntu系统中的位置：
```
~/ros-mcp-server-minipi/launch/rosbridge_websocket_auto.launch
```


## 以下还未实测。

3.MCP功能测试
在小派机器人上启动`rosbridge`后就可以在同局域网下的PC上测试并开发MCP server，具体方法如下

3.1修改server.py
测试需要在PC上启动MCP服务器并通过rosbridge控制小派

在小派机器人的显示屏上查看其局域网ip，将地址复制粘贴到server.py的ROSBRIDGE_IP

server.py拉到最后一行将mcp.run的启动方式从stdio改成sse(网络连接)

然后终端cd到项目文件件运行`uv run server.py`，出现以下输出说明启动成功，并复制`http://0.0.0.0:8000`类似的一串网址

![alt text](/pic/eef10939190edaad025d2c7dea28f7e8_1754886420-image.png)
3.2配置MCP host
首先需要在PC上安装一个MCP host，下面以最新版VS code为例

VScode启用copilot后在右下角会有一个扳手的图标，点击后顶部出现下拉栏

![alt text](/pic/594a539556cd90e9ada1e878b34e5037_1754717910-image-1024x611.png)
![alt text](/pic/b0d3e95cf0eb0a81f107c4219031be34_1754718297-image.png)
点击下拉栏右上角的MCP图标，MCP服务器类型选择命HTTP

![alt text](/pic/0472c2ed28cca91f882f7592e4959709_1754718321-image.png)
![alt text](/pic/82da8f343315733cdeb0e9bbb85385f0_1754886457-image.png)
粘贴3.1步启动后的网址，并额外在后面输入`"/sse"`，例如`http://0.0.0.0:8000/sse`，然后一路选择第一个选项，完成后弹出mcp.json，并出现以下配置项说明MCP Client配置成功

![alt text](/pic/c3718a7f2bd47f45d30ee680af5bc012_1754886779-image.png)

3.3使用MCP
同样以Vscode为例，在copilot对话框的下方模式选择Agent，模型选择Claude Sonnet 3.5（其他也可以但是claude对MCP支持最好，因为是一家公司搞出来的）

![alt text](/pic/9e5d6328e84d42fe290c45dae3dcdca7_1754886934-image.png)

然后和copilot对话，比如站起来，原地踏步等等，copilot会执行工具调用，手动批准后就可以让小派执行对应动作了

![alt text](/pic/6c2bec9a7fb2eba3f212c84ccdb3465b_1754887096-image.png)

4.Q&A
Q:启动小派后一段时间没法控制，过了大概1min才能通过小智控制

A:通过HDMI连接小派能够看到名为“mcp”的终端窗口出现类似以下的报错，可以发现是SSL问题，原因是长时间断电导致本地时间延后，不在证书有效期内，所以会一直尝试连接直到时间校准完毕；之后短间隔内启动不会出现此类报错

![alt text](/pic/143ba972bbc0194016835b80eabab70e_1755756795-_cgi-bin_mmwebwx-bin_webwxgetmsgimg__MsgID1726621122206667250skey@crypt_5314de9f_810cfaf73fe94b4f9d442c8fec879b54mmweb_appidwx_webfilehelper-1024x576.jpg)
5.二次开发
项目核心原理是将ros订阅节点通过rosbridge暴露出来，将对需要的api使用fastmcp库封装成MCP格式，通过小智MCP接入点供土豆子调用

本项目暂时采用模拟手柄输入来控制小派，实际上可以直接向ros相关节点输入数据来更灵活地控制小派而不只是使用joy节点

除此以外mcp tool可以增加诸如距离，角度等参数提高动作自由度，并获取ros相关节点的数据并返回作为工具执行结果

不过以上还需要硬件厂商做好相关适配

