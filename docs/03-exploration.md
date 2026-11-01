# 自动建图与检查点续建

## 首次自动探索

```bash
# 小车：贴合新桩，从这里建立地图原点
./kit mapping new --at-dock --auto
# 电脑 Ubuntu
./kit view
```

RViz 顶部选 **Publish Point**：

1. 沿同一方向依次点目标区域四角，第 1、3 点为对角；可包含灰色未扫描区。
2. 第五点点车旁已知空地。工具若恢复成相机，每点一次重新选择 Publish Point。
3. 小车执行 `./kit explore resume` 开始。

厂家 local/global RRT 和 filter 产生边界候选，本工程调度 Nav2 目标。默认低速探索；不跨已标记门槛，也不靠加速硬冲小坎。

## 暂停、继续、回桩

```bash
./kit explore pause      # 停车并暂停地图写入
./kit explore resume     # 恢复地图写入和探索
./kit explore return     # 返回本轮记录的桩前并交给厂家红外
./kit mapping save      # 停稳后保存地图、原生图和桩位
```

任务状态：`ros2 topic echo /rrt_task_status --once`。日志：`state/logs/mapping.log`。

灰色未探索区域不一定都可走，卡座内部和封闭区域不会为了填满颜色强行进入。

## 门槛与出口禁区

导航场景用：

```bash
./kit scene keepout room1 --rect XMIN XMAX YMIN YMAX --rect XMIN2 XMAX2 YMIN2 YMAX2
./kit scene use room1
```

建图中的禁区用 `./kit mapping avoid --rect ...`，参数顺序相同。保留门槛的实际宽度和车身余量；禁区单独保存在 mask，不画成 SLAM 地图里的墙。

RRT 四角限制探索目标；门口不想驶出时，用禁区封住出口，不能只靠四角框。

## 下次开机继续

保留整个检查点目录。小车放回原桩，方向与起图时一致：

```bash
./kit mapping resume maps/checkpoint_日期时间
```

恢复后默认暂停。先看扫描与墙线，再选四角、第五点，最后 `./kit explore resume`。

续建会更新已有区域并扩展新区域。只开地图导航时不会改保存的底图，实时障碍只更新代价地图。

若新扫部分已经错位：`./kit hold`，回原桩，`./kit stop`，恢复最近一个好检查点，再补扫受影响路段。PGM/YAML 本身不能恢复 SLAM 图优化历史。
