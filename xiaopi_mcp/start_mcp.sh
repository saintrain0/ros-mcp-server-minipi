#!/bin/bash
roslaunch rosbridge_server rosbridge_websocket.launch port:=9091 
#默认端口是9090，但实践中9090有时会被占用，因此指定了9091，server.py里也需要同步修改为9091.
#如果9090端口占用了：
sudo kill $(sudo lsof -t -i :9090) 2>/dev/null



cd ~/ros-mcp-server-minipi
uv run mcp_pipe.py server.py