# 手动建图

## 开始

1. 充电桩固定在墙边，接好电源。小车车尾贴合、车头朝场内。
2. 小车运行 `./kit mapping new --at-dock`。
3. 电脑 Ubuntu 运行 `./kit view`。Fixed Frame 为 `map`，打开 Map、LaserScan、TF 和车体显示。
4. 原地停几秒，红点与附近墙体基本重合后发车。

启动器显式启动厂家 `ImuProcessor`，EKF 使用 `/imu/data_filtered`。无需再改厂家文件中的两个 `robot_nav=False`，也不要额外重复启动 IMU 处理节点。

## 遥控

厂家手柄可以直接使用。键盘控制在小车终端运行：

```bash
./kit teleop
```

`i` 前进，`j/l` 转向，`k` 停止。初始线速度 0.10 m/s，角速度 0.25 rad/s；键盘终端保持焦点。手机或手柄具体键位沿用厂家遥控器说明。

先走主要走廊，再补支路；转弯放慢。长直线沿通道中部走；经过已扫区域形成回环。门口只扫到目标场地边界。

## 保存

停车，核对扫描与墙线，再运行：

```bash
./kit mapping save
```

输出目录在 `maps/checkpoint_日期时间/`，包含：

| 文件 | 用途 |
|---|---|
| `WHEELTEC.pgm` + `WHEELTEC.yaml` | 地图导航、地图交接 |
| `slam_graph.posegraph` + `slam_graph.data` | 下次继续建图 |
| `native_slam_params.yaml` | 同步恢复 SLAM 参数 |
| `dock_start.json` | 本图的桩位参考 |
| `threshold_keepout.*` | 导航禁区 |
| `manifest.json` | 文件校验及恢复入口 |

保存会暂停地图写入。继续手动补扫时用 `./kit mapping continue`；结束时 `./kit stop`。

每完成一段走廊存一个检查点。碰撞后先暂停，不把明显错位的图覆盖为好图。

## 转为导航场景

```bash
./kit stop
./kit scene new room1 --map maps/checkpoint_日期时间/WHEELTEC.yaml
./kit scene dock room1 --pose 0 0 0
./kit scene use room1
./kit nav
```

场景名由自己指定。已有其他来源地图时，贴桩坐标要从该地图实际定位读取，不能默认使用零点。
