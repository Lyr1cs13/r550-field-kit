# 开发与维护

## 模块入口

| 模块 | 文件 |
|---|---|
| HTTP / 上位机 | `demo/server.py`、`demo/web/` |
| 多点、停留、暂停、回充编排 | `demo/mission.py` |
| ROS / Nav2 / 原生红外接口 | `demo/ros_backend.py` |
| 到点、朝向策略 | `demo/arrival_policy.py` |
| 持续充电判断、轮程保护 | `demo/charge_confirmation.py` |
| 回桩后自动定位校准 | `demo/dock_localization.py` |
| 连续采集线路与避障接回 | `demo/survey_ros.py`、`survey_path.py` |
| WS63 串口、校时与位置关联 | `demo/broadcast_survey.py`、`device_clock.py` |
| SERVO_CAR_2 定点协议与结果存储 | `demo/servo_car.py`、`servo_storage.py` |
| 新场景导入与标定 | `tools/scene.py` |
| 手动/自动 SLAM 组合 | `rrt_mapping/mapping.launch.py` |
| RRT 任务调度 | `rrt_mapping/frontier_tasks.py` |

## 开发流程

1. VS Code Remote SSH 连接小车，修改独立分支。
2. `./kit test` 运行软件检查；无车可用 `./kit preview`。
3. 有 ROS 的电脑检查启动参数与依赖；`./kit doctor`。
4. 停车后重启对应程序，再做短距离现场检查。

导航读取固定地图，建图单独运行。两种模式用 `./kit stop` 切换；不同时运行两套底盘和控制器。

## 参数位置

- 导航速度、加减速、足迹、膨胀、目标检查：`demo/navigation.yaml`。
- 过滤后的 IMU 融合：`demo/assets/demo_ekf_nav.yaml`。
- 自动探索导航参数：`rrt_mapping/navigation.yaml`。
- 自车天线回波过滤开关、实测位置：`config/vehicle.yaml`。
- 地图、桩位、禁区和采集范围：`scenes/场景名/`。

同型号新车先沿用运动参数。出现过冲和摆头时，先记录 `/cmd_vel`、`/odom`、定位和 TF，分清路径几何、反馈延迟、制动及定位问题，再改控制参数。

## 厂家功能与本工程增加的内容

厂家提供底盘串口、轮速里程计、IMU、雷达驱动、SLAM、RRT、Nav2 和 STM32 红外回充。本工程没有替换这些硬件驱动。

增加了网页点位与任务编排、位置/朝向分阶段停靠、连续路径采集、回桩定位锚点、回充确认与失败复核、检查点管理、门槛禁区、WS63 采集及数据后处理。

开发中处理过的典型问题：原始 IMU 静止漂移、启动参数覆盖导致未使用过滤 IMU、末端反复修正、雷达扫到自身天线、低障碍空转导致里程错位、过早截断厂家回充、无线桥接 ARP、串口时钟漂移误丢后段数据。对应恢复步骤见故障速查。

## 版本与回退

```bash
git status
git switch -c 调整名称
./kit test
git add demo tools config docs
git commit -m '说明改动'
```

部署前 `./kit stop`。回退软件用已知好 tag 或 commit，新场景的地图、桩位、导航点和采集数据另行备份。场景切换前的资产在 `state/backups/`；SLAM 好检查点在 `maps/`。

所有命令使用当前仓库目录，不依赖原电脑的盘符、私钥、实验室坐标或固定网卡 MAC。
