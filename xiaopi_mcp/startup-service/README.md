# 开机启动MCP服务
## 添加并启动服务，自动运行ROS BRIDGE
```
    sudo nano /etc/systemd/system/rosbridge.service     #编辑服务文件，写入文件内容
    sudo systemctl daemon-reload                        #重新载入
    sudo systemctl enable rosbridge.service             #使能服务
```
## 添加并启动服务，自动运行UV MCP
```
    sudo nano /etc/systemd/system/uv-server.service     #编辑服务文件，写入文件内容
    sudo systemctl daemon-reload                        #重新载入
    sudo systemctl enable uv-server.service

```

## 由于重名问题报错，添加rosbridge_websocket_auto.launch
```
mkdir -p ~/ros-mcp-server-minipi/launch
nano ~/ros-mcp-server-minipi/launch/rosbridge_websocket_auto.launch     #编辑文件，写入文件内容
```

## 重启后测试：
```
    sudo systemctl restart rosbridge.service            #重启服务
    sudo systemctl restart uv-server.service

    sudo reboot                                         #或者也可以直接重启系统

    systemctl status rosbridge.service                  #重启后查看服务状态
    systemctl status uv-server.service                  #重启后查看服务状态
```
如果查看两个服务的状态都正常，就可以尝试实测语音控制了。
250917晚，测试成功。