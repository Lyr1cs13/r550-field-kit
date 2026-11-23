# 一号车视觉开发 Agent 交接

你的工作目录是 RDK 上的 `~/r550-vision`。日常可以和巡航组同时改代码、跑离线测试。需要使用实车时，通过 `./kit profile` 交接控制权。

| 工程 | RDK 目录 | ROS 通信域 | 实车面板 | 建图面板 |
| --- | --- | ---: | ---: | ---: |
| 视觉开发 | `~/r550-vision` | 42 | 8990 | 8996 |
| 纯净巡航 | `~/r550-patrol` | 41 | 8890 | 8896 |

## 开工

用 VS Code Remote SSH 连接一号车 `wheeltec@192.168.0.100`，打开 `/home/wheeltec/r550-vision`。在每个需要运行 ROS 命令的新 Bash 终端加载视觉工程环境。

```bash
cd ~/r550-vision && source tools/ros_environment.sh
printf '%s | %s | ROS 域 %s\n' "$(pwd -P)" "$R550_WORKSPACE_NAME" "$ROS_DOMAIN_ID"
./kit profile status
```

预期环境为 `/home/wheeltec/lab_cruise_clean | vision | ROS 域 42`。`r550-vision` 是已有视觉工程的别名。控制权状态显示 `lab_cruise_clean` 也代表视觉工程，显示 `r550-patrol` 则代表巡航组占车。**车停在桩上、正在充电，也可能仍由巡航组占用。**

## 日常开发与离线预览

视觉节点在 `demo/vision_node.py`，几何处理和传感器状态分别在 `demo/vision_geometry.py`、`demo/vision_health.py`。相机标定在 `demo/vision_config.json`。用已录制的图像或相机话题调试，可以和巡航同时进行。

```bash
python3 -m unittest discover -s demo/tests -p 'test_vision_mode.py'
./kit preview
```

在 VS Code 的「端口」中转发车端 `8991`，打开显示的本地地址。通常是 `http://127.0.0.1:8991/`。已有预览时直接转发端口即可。改动完成后用 `./kit test` 和 `python3 tools/check_repository.py` 做完整软件检查。

## 实车联调与交还

1. 收到实车联调任务后，先执行 `./kit profile status`。若巡航占用，由**巡航操作者**结束任务、停稳，并在其工程执行 `./kit profile release`。等待期间可继续离线开发。
2. 状态为「空闲」时，在视觉工程执行下面的命令。如果视觉已持有实车且面板在线，直接复用面板即可。

   ```bash
   cd ~/r550-vision && ./kit profile select && ./kit site && ./kit nav
   ```

3. 打开视觉实车面板 `http://192.168.0.100:8990/`。上述命令启动服务，发车或回充按当次测试任务执行。
4. 结束时在视觉面板停止任务。等车辆停稳、建图或传感器切换结束，再执行交还命令。

   ```bash
   cd ~/r550-vision && ./kit profile release && ./kit profile status
   ```

看到「空闲」即已交还。若提示仍有任务或串口占用，查看视觉工程的 `state/logs/` 和 `state/ros/log/`，从视觉面板结束对应操作，再重试释放。

## 改动与部署范围

- 源码、配置、地图、桩位和点位只写视觉工程自己的副本。两组共用 Linux 账号、系统 ROS 和厂家驱动，系统级变更由双方协调。
- 实车控制统一走视觉工程的 `./kit` 和 `8990` 面板。保留工程配置的域和控制权锁；避免跨域发布 `/cmd_vel`、直接写底盘串口或批量结束全车 ROS 进程。
- 从开发电脑部署时，在包含 `demo/vision_node.py` 的视觉源码目录执行 `python3 tools/deploy_robot.py --host wheeltec@192.168.0.100 --target /home/wheeltec/r550-vision`。如果视觉工程正持有实车，先按上节交还。直接用 Remote SSH 修改车端视觉源码时，可省去这次传输。
