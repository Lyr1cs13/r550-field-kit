# 固定朝向巡航与异常恢复

本页记录ROS 2 Humble巡航的控制逻辑及软件与实车的验收方法。

## 行驶和到点

普通停靠使用 4 cm、4°。固定基站 RSSI 定点采集使用 12 cm、3°，每条样本绑定实际地图位姿。位置达标后停车，再修正车头。小角度原地转向提高起转指令，依据轮速反馈提前收转，停稳后复核。到点日志记录耗时、控制器和修正次数。

直线段通过 Nav2 实时路径检查后，使用前视距离 0.80 m 的 FixedHeading。倒车沿原朝向输出负线速度。进入位置门限即停车复核，再处理朝向。需要绕障或车头与路段不一致时，由 Nav2 规划绕行。

双车定点采集使用 5.5 cm、3°。近点直线候选先检查整段净空，再决定前进或倒车。位置达标后，末端只执行零线速度的原地转向。调度重试使用同一到点门限，先复用已停稳且位姿达标的车辆，只为仍未到点的一车重新规划。

四驱差速车不能横移。环线上与统一车头方向垂直的一边，车辆仍需改变行驶朝向，然后在采集点恢复统一朝向。软件不能保证这一边也像平行边一样无转向连续通过。如果采集布局允许，优先把多数点位排列在与测量朝向平行的长直线上，以前进和倒车连接，减少横向换边次数。

网页点位区增加“统一朝向”和“载入未完成点位”。应用朝向只保存点位，不发车。生成等间距点位时，可以使用明确画出的通道折线：

```bash
python3 tools/fixed_route.py --scene scenes/现场场景 \
  --vertices 通道拐点.json --count 50 --heading-deg 0 --wait 3 \
  --output 统一朝向50点.json
```

通道拐点文件为`[[x1,y1],[x2,y2],...]`。只有末尾明确重复第一个拐点才闭环，避免自动跨越门槛。工具检查整条折线的静态通行空间，并写入map_id。导入面板后仍由实时导航处理障碍。这里不预填未经现场确认的北区坐标。

## 遇到人、受阻和定位异常

可绕行的通路交给Nav2规划。普通点位、离桩和回桩途中遇到临时阻塞时保留原目标，停车后逐步延长重试间隔，最长10秒。不会因为一次导航失败直接跳到下一采集点。永久堵路或低于雷达扫描面的物体不能承诺自动通过。

速度链为控制器→速度平滑→碰撞监视→最终速度门控→底盘。门控只有在导航任务、定位和传感器就绪时接受运动许可。控制指令或许可心跳超过0.3秒、有效雷达超过0.4秒未更新时停止输出运动速度。Nav2 Humble 1.1.15的断流行为经原生测试发现不足，因此雷达新鲜度检查独立放在最终门控中。门控节点自身也有心跳，缺失时任务暂停。厂家STM32红外贴合仍由原生硬件模式执行。独立门控在回充监督心跳丢失超过0.5秒时只发送停止回充指令，不自行启用或重启红外模式；硬件实际制动行为不能用ROS指令测试代替。

局部与全局软膨胀半径分别为 0.36 m 和 0.42 m。普通导航线速上限为 0.12 m/s，加速上限为 0.20 m/s²。雷达发现障碍进入当前行驶弧线前方 0.48 m 范围时，逐步压低速度。旁侧墙线仍参与规划代价。近身障碍进入车身保护区时停车。

近障停车后可执行一次约 20 cm 的低速直退。短退要求定位有效、车尾和退出通道净空，速度上限 0.05 m/s。净空消失、位置偏离、里程无有效进展或许可中断时停车。停稳复核后可指定走廊中线作为返程过渡点。`config/vehicle.yaml` 可记录该车确认过的天线回波坐标，原始雷达话题保留。

轮式里程计报告前进而连续雷达帧仍匹配在原位置时，控制链停车并保留当前点。停稳后请求 AMCL 无运动更新。恢复要求匹配到空转前的雷达定位位置，并通过墙线、协方差和连续稳定检查。短暂遮挡或缺少能辨别运动方向的墙线时，系统继续用避障与导航进度检查。首次复核通过可以重规划。同一处再次空转时保持停车，处理现场后重新定位。

重复走廊可能出现错误但自洽的定位，二维雷达也可能看不到低障碍。这套一致性检查不能证明全局位置绝对正确。实车若确有空转或低障碍碰撞，需要检查雷达覆盖、安装外形、轮速和IMU；必要的近地测距或接触传感器无法靠软件模拟补足。

## 回充、续作和重启

低电量续采由[config/endurance.yaml](../config/endurance.yaml)配置。按车设置低电量阈值、恢复电压、最短充电时间及电压保持时间。设置 `repeat_only: true` 时，勾选「循环巡航」启用续采。普通任务在低电量时结束并回充。

启用后，保留已写盘数据和下一待测点。充电、停稳、桩位校准和电压均满足条件后，从保留的点位继续。固定基站定点模式还会在桩上等待接收恢复。手动暂停或停止会撤销自动续作。先在现场跑通一次低电回桩、充电和续采，再延长试运行时长。

每次前往点位前原子保存`mission_checkpoint.json`。程序重启后不自动发车；可在面板载入剩余点位，重新核对采集模式和定位再开始。未确认完成的当前站可能重新采集，不能将中断记录当成已完成。主动停止明确取消任务并清除续作记录。

## 可复现测试

```bash
python3 -m unittest discover -s demo/tests
python3 tools/check_repository.py
```

原生仿真需在专用测试目录构建仓库里的两个Nav2扩展，加载其overlay后运行：

```bash
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=187
unset CYCLONEDDS_URI
source /opt/ros/humble/setup.bash
source overlay/install/setup.bash
python3 tools/native_navigation_check.py --repeat 12
```

该程序使用真实 Nav2 控制器、代价地图、速度平滑、碰撞监视和门控节点。二维运动模型带有限加减速度。测试覆盖前进、倒车、临时挡路、反复停靠和断流，按当前停靠门限验收，并记录是否需要末端修正。仿真使用理想地图变换，结果保存在 `state/robustness/native/`。

50点位流程测试另用可控后端注入三次传感器中断和一次回充，核对同一点恢复、点位顺序以及最后完成状态。这是任务状态机测试，不是50个实体点位的运动验收。

### 双车定点到达与原地调向

统一面板的「50 个采集点」使用这套控制。两车先检查各自的直线路段、实时代价地图和对车预约。车头顺路就前进，倒车可省下明显的转角时就倒车。直线被挡时重新规划。预计到点后还要转 45° 以上时，直行阶段争取靠到距目标 2.2 厘米。进入通常的停车半径后，最多多用 1 秒争取这段余量。

车停稳后，原生 Spin 检查旋转扫过的车身。陀螺仪测量实际起转和转速，程序在同一转向内限速、预估余转并刹停。转向指令的纵向和横向速度都为零。车端连续核对不同时间戳的定位与雷达，实测位置误差 ≤5.5 厘米、朝向误差 ≤3° 才完成。只差朝向时保留位置，连续观测证实位置超界才重新移动。操作员停车优先，恢复通行沿用原轮次。

现场记录查看 `health.arrival.last_turn`。其中 `requested_deg` 是本次请求的有符号转角，`gyro_at_release_deg` 是主动刹停时陀螺仪累计的转角，`map_turn_deg` 和 `centre_shift_m` 是静止复核时的地图转角和中心位移。独立地面标记可以把轮胎擦滑和定位抖动区分开。

隔离 ROS 仿真使用上面的本机 ROS 环境。下列第一条命令使用合成底盘运行真实的控制器、Spin、速度平滑和碰撞保护。第二条用已有实车 JSONL 拟合起转延迟、制动和剩余转量，在留出的片段上比较三种控制。第三条在保存的地图上检查全部点对的直线净空，再改变假定的旋转中心，计算停稳后需要补位置的比例。

```bash
python3 tools/native_navigation_check.py --safety-only
python3 tools/fas_spin_benchmark.py state/logs/turn-a.jsonl state/logs/turn-b.jsonl \
  --output state/robustness/spin.json --repetitions 64
python3 tools/fas_station_replay.py --scene scenes/现场场景 --route state/north-u-50.json \
  --output state/robustness/stations.json state/logs/turn-a.jsonl state/logs/turn-b.jsonl
```

直线段使用 RPP，按距离、曲率和净空调整速度。原地段沿用 Humble Spin 的旋转碰撞检查。四个固定轮与地面有相对滑动，实际瞬时旋转中心会随地面和轮胎变化。[RPP 论文](https://arxiv.org/abs/2305.20026) · [Humble Spin 源码](https://api.nav2.org/nav2-humble/html/spin_8cpp_source.html) · [四驱滑移转向实验](https://babel.isa.uma.es/_utils/downloads/logdownloads.php?f=downloads%2Fjafma%2Fpapers%2Fmandow2007ekw.pdf&t=pdf&u=jafma)

## 实车验收与摆头诊断

当前足迹以 R550 Pro 四驱的车型尺寸为基准。换车或调整附件后，先核对轮子、支架和天线的完整外形，再测制动距离和扫描盲区。

短距离测试记录命令角速度、轮速角速度和AMCL朝向。命令自身左右变化时，检查路线曲率、前视距离和定位跳变。命令基本直行而轮速或实体车偏转时，检查左右轮增益、轮径、轮距、地面打滑和电机响应。

```bash
bash tools/record_navigation.sh
```

该命令只录制数据，不发车。到点另记录`arrival_measurements.jsonl`，包含目标、实测地图位姿、协方差和扫描一致性。用独立地面标记或测量设备验证实体误差，不能只看网页箭头。

依次验证直线往返、90°换边、前后近障碍、人员短时挡路、短暂雷达断流、重复点位、真实低电量回充与续作。每个点位记录外部实测误差与完成耗时。长时间和有人环境的可靠性以实车测试为准。

原生接口依据：[RPP反向行驶参数](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/)、[Humble碰撞监视参数](https://github.com/ros-navigation/navigation2/blob/humble/nav2_collision_monitor/params/collision_monitor_params.yaml)、[1.1.15碰撞时间判断源码](https://github.com/ros-navigation/navigation2/blob/1.1.15/nav2_collision_monitor/src/polygon.cpp)。Humble使用max_points；approach的静态判断包含等号，不能设置为0，否则无障碍也会停住。
