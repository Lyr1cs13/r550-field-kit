# WS63 旋转基站定点采集

配套固件为 `sle_servo_p2p`，车载 Client 的 AT 串口使用 `SERVO_CAR_2` v2。每个导航点停车后测一轮，包含水平 0° 和 180° 两个节点。完成时间由测量流程决定。

## 给固件开发者的交接

| 对象 | 交接内容 |
|---|---|
| 旋转 Server | 响应 Client 的整轮触发，控制舵机，按顺序提供角度标签和无线测试数据，确认两节点完成 |
| 车载 Client | 与 Server 使用同一套 SLE 服务和属性，测量 RSSI 与吞吐量。接收冻结的点位及坐标，回传两个节点、结束消息和移动许可 |
| RDK 小车 | 导航、停稳判断、冻结实际位姿、分配轮次、可靠保存结果，以及依据移动许可继续路线 |

请固件方交付同一版本的 Server 与 Client 烧录包，或提供对应源码和编译选项。联调时核对 SLE 设备匹配、实际 AT UART、`CARHELLO` 和 `CARERROR` 的实机回执，以及整轮超时。仓库的 `ws63/beacon_survey` 是固定基站广播固件，旋转工程的烧录包由固件方提供。

两个角度之间的节点确认走 Client 与 Server 的无线接口。RDK 在整轮结果写盘后发一次车端确认。串口重发使用原轮次与原坐标。

## 在同款车上部署

先按 [首次部署](../README.md#首次部署) 完成导航环境，再按 [导航与回充](04-navigation.md) 导入本站地图、标定桩位并核对定位。已部署的同款车可在停稳后更新源码。

```bash
cd ~/lab_cruise_ws
./kit stop
git pull --ff-only
```

1. 给旋转板和车载板分别烧录配套 Server 与 Client。确认 Client 的实际 AT UART 为 115200、8N1，命令行结尾用 CRLF。清单所述的默认口是 UART0，使用板卡配置核对 NV 覆盖。裸 UART 接线要共地并核对两端逻辑电平。
2. 同款 HiHope Client 可用板载数据口接 RDK。运行 `ls -l /dev/serial/by-id/`，核对新出现的 WS63 串口。本工程自动选择唯一的 CH340K（`1a86:7522`），多块板时用完整串口路径。权限不足时将运行账户加入 `dialout`，重新登录。
3. 在巡航服务未占用此串口时运行探测。命令查询版本与状态，可直接把回执交给固件开发者。

```bash
./kit servo-probe
# 多个设备时，替换下面的路径
./kit servo-probe --device /dev/serial/by-id/你的WS63
```

探测需要 `+CARHELLO:` 明确声明 `SERVO_CAR_2` 或以 `2`／`v2` 声明版本。随后显示 `CARSTATUS` 的轮次、点位、状态、SLE 就绪标记及节点掩码。待命且 `link_ready=1` 时可以开始首轮联调。

4. 运行 `./kit nav`。打开巡航面板，确认定位，添加两个本站的普通导航点。选择“旋转定点采集（SERVO_CAR_2）”，应用串口并等到串口与 SLE 就绪。首次联调可取消结束后回充，再开始两点巡航。
5. 观察第一个点停稳后才开始转动，收齐两个角度后才离开。第二个点完成后任务结束。需要回充时勾选结束后回充或在采集结束后使用主动回充。

定时停留适用于普通巡航。固定基站路径采集继续使用 [广播模式](06-ws63.md) 和原配套固件。旋转定点路线在面板使用普通点位。已有快速回环引导点可用面板的“恢复原巡航点位”或重新设置。

## 串口对齐

```text
RDK → Client  AT+CARHELLO
RDK → Client  AT+CARSTATUS
Client → RDK  +CARSTATUS:round,position,state,link_ready,result_mask
RDK → Client  AT+CARREADY=7,3,1200,-50
Client → RDK  OK
Client → RDK  +CARRESULT:7,3,1200,-50,0,0,0,1000000,1500000,500000,80000,100,-81,10,1280000,1200000
Client → RDK  +CARRESULT:7,3,1200,-50,1,180,0,2000000,2500000,500000,0,0,NA,0,0,0
Client → RDK  +CARROUNDEND:7,3,2
RDK → Client  AT+CARACK=7
Client → RDK  +CARMOVE:7,3
```

- `position_id` 是本次路线的停靠序号。同一站重测时保持原序号及实际位置快照。`round_id` 取本机已分配最大编号与 Client 当前编号中的较大值，再加一。
- `CARREADY` 的顺序为轮次、位置编号、实际地图 x 毫米、实际地图 y 毫米。坐标用当前场景的 ROS `map` 原点与坐标轴。完整浮点位姿和朝向另存于 RDK。
- `CARRESULT` 的字段为 `round,position,x_mm,y_mm,node,horizontal_deg,vertical_deg,start_us,end_us,duration_us,bytes,packets,rssi_dbm,rssi_count,bps,payload_bps`。节点 0 对应水平 0°，节点 1 对应水平 180°。保持相同编号和坐标的结果可重发。结果中的时间使用 Client 自己的时钟，RDK 另存接收 UTC。
- `CARSTATUS` 状态 0 为待命，1 为测量中，2 为等待确认，3 为允许移动，4 为失败。`CARBUSY` 是测量进度。`OK` 是命令回执。两者均可出现在停车期间。
- RDK 按轮次与节点去重。冲突、坐标不符或写盘失败时任务停在当前点。`RSSI=NA`、零吞吐量及缺测计数保留在数据里。
- 两个节点与匹配的 `CARROUNDEND` 都已写盘时，RDK 发 `CARACK`。收到同轮同站的 `CARMOVE` 后，巡航只处理一次放行。暂停期间仍可保存本轮结果，继续巡航时才处理放行。

## 中断与数据

开始回执丢失时重发相同的 `CARREADY`。确认后的放行丢失时重发相同的 `CARACK`。按清单的 90 秒 Client 整轮上限，车端每轮等待上限为 105 秒。Client 确认本轮失败且 SLE 在 12 秒内恢复时，车端在原站最多新开一轮重测。

USB 断开、轮次变化、超时或停车条件失效时，任务停车，已有节点和轮次状态留在本机。未放行的轮次保存在数据库中。恢复时保持小车停稳，按固件方步骤结束 Server 上一轮并让 Client 回到待命，确认 `state=0`、`link_ready=1`。再在面板选择旋转定点模式并应用连接，核对实际位置后重新选点开始。重启面板或切换定时模式不能解除未放行轮次。车端停止键只停止本地任务，旋转端中断由固件方的操作流程处理。

面板的“导出采集与位置记录”在旋转模式下下载结果 ZIP。“导出历史旋转定点记录”可在其他模式下读取已有结果。

| 文件 | 内容 |
|---|---|
| `servo_results.csv` | 每角度一行，完整串口字段、接收 UTC、会话、点位、位姿来源、朝向、地图版本与轮次状态 |
| `rounds.json` | 实际与目标位姿、失败详情、结束标记及移动许可处理标记 |
| `maps/<map_id>/` | 对应版本的地图与场景文件 |

本机数据库位于 `demo/data/ros/collection/servo_car_v2.sqlite3`，按提交保存结果与轮次上限。更新软件时保留此文件。

## 首次联调检查

1. 探测串口版本、SLE 链路及待命状态。确认未发 `CARREADY` 时旋转端处于待命。
2. 跑两个站，确认小车停稳后才触发，0° 和 180° 两行绑定同一位置，保存后收到放行才离站。
3. 重发节点和放行消息，检查没有重复行或重复出发。测试丢失开始回执、放行以及串口断开后的停留和恢复。
4. 导出 ZIP，核对坐标、角度、RSSI、吞吐量、地图版本与两个轮次状态。

无需接车即可运行 `python3 -m unittest discover -s demo/tests -p test_servo_car.py`，覆盖串口分包、双点放行、写盘失败、重试、断线与重启。真实固件的接线、版本回执和两站过程按以上步骤核对。
