# R550 Field Kit

R550 Pro 四驱底盘 / RDK X5 / ROS 2 Humble 的部署与使用工程。

包含远程开发、新场地建图、RRT 自动探索、地图导航、多点巡航、原生红外回充与充电续作、WS63 RSSI 采集与热力图、ESP32-C5 CSI 定点采集，以及双车双频 FAS 实验。

## 首次部署

使用厂家 Humble 镜像，保留 `~/wheeltec_ros2` 驱动工作区。电脑安装配套 Ubuntu 22.04 虚拟机。

小车终端：

```bash
git clone https://github.com/Lyr1cs13/r550-field-kit.git ~/lab_cruise_ws
cd ~/lab_cruise_ws
bash kit setup robot
./kit network --interface wlan0 --peer 192.168.0.250
./kit doctor
```

电脑 Ubuntu 终端：

```bash
git clone https://github.com/Lyr1cs13/r550-field-kit.git ~/lab_cruise_ws
cd ~/lab_cruise_ws
bash kit setup viewer
./kit network --interface ens33 --peer 192.168.0.100
```

网卡名称用 `ip -br addr` 核对；同一辆车与电脑使用相同 `ROS_DOMAIN_ID`。依赖已装好时可用 `setup robot --skip-apt`。

## 新场地：建图到巡航

在统一面板选车并打开「新场地建图」。把车尾贴合新桩，标定当前位置，选择遥控建图或框选自动探索。随时可用当前地图回桩，每次保存生成一个可恢复的检查点。[新场地页面用法](docs/02-mapping.md#上位机建图)

### 命令行入口

1. 摆好充电桩，车尾贴桩，车头朝场内。小车运行 `./kit mapping new --at-dock`。
2. 电脑运行 `./kit view`；用手柄遥控，或在小车 SSH 终端运行 `./kit teleop`。
3. 扫完一段，停稳并核对红点与墙线，执行 `./kit mapping save`；记下输出的检查点目录。
4. 完成后 `./kit stop`，导入地图并标桩：

```bash
./kit scene new room1 --map maps/检查点目录/WHEELTEC.yaml
./kit scene dock room1 --pose 0 0 0
./kit scene use room1
./kit nav
```

`--pose` 是贴桩时的实际地图坐标 `x y 朝向角度`。按上述贴桩起图时从 `0 0 0` 开始核对。

5. 打开[巡航面板](http://192.168.0.100:8890/)，核对定位，地图加点，设置朝向和停留时间，开始巡航。主动回充按钮使用厂家红外贴合流程。

自动探索：将第 1 步改成 `./kit mapping new --at-dock --auto`，在 RViz 用 Publish Point 选四角、第五点点车旁空地，再运行 `./kit explore resume`。

## 操作手册

| 要做什么 | 文档 |
|---|---|
| 接网卡、SSH、电脑与小车同时联网 | [远程开发与网络](docs/01-network.md) |
| 手柄/键盘遥控、保存地图 | [手动建图](docs/02-mapping.md) |
| 自动探索、暂停、检查点续建、门槛禁区 | [自动建图与续建](docs/03-exploration.md) |
| 点位、朝向、停留、校准、回充 | [导航与回充](docs/04-navigation.md) |
| 开机、虚拟机部署到车端、逐项实车验收 | [开测操作顺序](docs/11-start-and-verify.md) |
| 固定朝向、倒车、受阻等待、定位保护、充电续作与无车测试 | [鲁棒导航说明](docs/10-robust-navigation.md) |
| 碰低椅脚、里程计漂移、坏图、摆头、贴桩不充电 | [故障恢复速查](docs/05-recovery.md) |
| WS63 固件、采集、位置关联、热力图 | [WS63 与 RSSI](docs/06-ws63.md) |
| 旋转基站定点采集、串口探测与固件联调 | [SERVO_CAR_2 对接手册](docs/08-servo-car-v2.md) |
| ESP32-C5 串口试采、面板联动与位置对齐 | [C5 定点采集](docs/09-esp32-c5.md) |
| 两台 R550 共用地图、分车管理和运行区 | [双车巡航](docs/12-dual-r550.md) |
| 双车双频与固定相控阵、状态搜索和续采 | [FAS 双车联调](docs/13-fas-dual-radio.md) |
| 开发入口、修改与测试、版本回退 | [开发与维护](docs/07-development.md) |

常用命令：`./kit hold` 停车并暂停地图写入；`./kit stop` 停车并关闭本工程；`./kit api status` 查看导航状态。

## Windows巡航面板

双击[打开巡航面板.cmd](打开巡航面板.cmd)，进入本机的实车巡航页面。页面连接车端服务时使用车上的地图和状态，断线后继续显示已保存的场地地图。新电脑可先连车同步，或将现场地图文件放在本机的 `demo/data/desktop-panel/scene`。Ubuntu 的命令行演练入口为 `./kit preview`。

## 场景与目录

```text
demo/             上位机、导航任务、定位校准、回充、采集
rrt_mapping/      手动/自动建图组合、探索调度、检查点
scenes/           每个场地的地图、桩位、禁区、采集路线
config/           车辆与网络模板
ws63/             固定发射端和车载接收端源码
tools/            安装、诊断、场景管理、热力图处理
vendor/           控制器扩展源码及原始版权声明
```

`scenes/demo-room` 是生成的示例地图，可用 `./kit preview` 在无车情况下熟悉面板。新场地不使用示例桩位或路线。

旋转定点采集使用配套的 `sle_servo_p2p` Server 和 Client 固件。可在小车终端运行 `./kit servo-probe` 核对串口及 SLE 链路，然后在面板选择“旋转定点采集（SERVO_CAR_2）”。沿途广播采集使用仓库里的 `ws63/beacon_survey`。

本工程以 2026-10-30 已运行的导航和采集版本为基础，另加入场景迁移工具。首次上新车按手册完成直行、转弯、两点巡航和回充检查，再运行长任务。
