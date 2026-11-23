# 双车固定基站单次采样

## 烧录和接线

| 基站板 | 文件 | 烧录方式 |
| --- | --- | --- |
| WS63 接收板 | `WS63-BASE-ONCE.fwpkg` | BurnTool |
| ESP32-C5 接收主控 | `C5-BASE-main.bin` | ESP32-C5，偏移 `0x0` |
| ESP32-C5 阵列辅控 | `C5-BASE-sub.bin` | ESP32-C5，偏移 `0x0` |

两份 C5 镜像来自 11 月 22 日交接源码，用本机 ESP-IDF 5.5.4 构建。若基站 C5 已运行该交接版的主辅控固件，可以沿用。WS63 新增 `WSONCE`。原网页的连续扫描入口仍可单独使用。基站 USB 接电脑的是 **WS63 接收板和 C5 主控的原生 USB**，C5 辅控与主控按原接线通过 UART1 确认移相状态。车上的四块发送板沿用已经烧录的固件。

## 启动基站

在 Windows 基站电脑解压交付包，用 Python 安装 `pyserial`，然后运行 `start-base.ps1`。首次输入两块接收板的 COM 号，脚本会保存端口并创建 `base.key`。换串口时加 `-SetPorts`。也可以用下面的命令直接启动。

```powershell
python -m pip install pyserial
python -c "from pathlib import Path; import secrets; p=Path('base.key'); p.exists() or p.write_text(secrets.token_urlsafe(32))"
python fas_base.py --ws63 COM10 --c5 COM14 --mac-a D0:CF:13:E4:B6:54 --mac-b D0:CF:13:E4:BE:10 --key-file base.key --data captures --bind 192.168.169.1
```

将 COM 号替换成基站实际端口，同一串口由一个采集程序占用。这里的 `192.168.169.1` 是当前 Windows 与虚拟机之间的 VMware 网卡地址。基站与调度在同一台电脑时可用默认的 `127.0.0.1`。基站在另一台电脑时改用调度可达的基站网卡地址，并把同一个 `base.key` 放在调度配置文件旁。基站启动后等待调度请求。

每次请求携带一个状态和两车实测位置。WS63 执行一次 `WSONCE`，C5 用固定 GPIO 状态并核对辅控 ACK。两种无线在同一次停车期间分别采窗口，按各自窗口统计 A/B 的算术均值。主机保存原始串口日志及回执，再返回给调度。状态映射包含在包里的 `c5_states.json`。

## 从双车上位机启动

用现有巡航入口确认两车定位就绪、停稳。在虚拟机打开已准备好的 `~/fas-auto-20261122`，把基站的 `base.key` 放进该目录。另行部署时，从交付包解压 `coordinator.zip` 即可得到同一目录结构。示例配置使用虚拟机本机的双车转发端口和 Windows 基站地址。首次用现场 50 点路线做静止试采。

```bash
cd ~/fas-auto-20261122
bash run-bench.sh
```

静止试采通过后，正式运行用独立数据库。两车先停稳，从状态 0 开始采。某个状态同时满足 WS63 和 C5 的 `A 的 RSSI 均值 − B 的 RSSI 均值 ≥ 6 dB` 时，保存第一个共同命中的状态并换下一对。只有 0 至 4095 全部有效采过且无共同命中，才给这个位置对记为 `no_hit`。两点必须不同，交换 A/B 也算同一个位置组合，实际分配的 A/B 身份随数据保存。50 点最多有 1225 个组合，候选还需满足 0.95 m 最小间距及现场通行条件。

```bash
bash run-auto.sh
```

窗口和最少样本数是初始联调值，现场可按包率调整，正式实验开始后保留同一组参数以便续作。

## 暂停、断电和续作

- 暂停时在**双车调度终端**按 Ctrl+C。程序关窗、请求停车并保存 SQLite 进度。需要同时关闭基站时，再在基站终端按 Ctrl+C。
- 恢复时先启动基站，再用**同一条正式命令及同一个 SQLite 文件**。调度回到未完成的位置对，跳过该对已有效测过的状态。低电回充后也按这个顺序继续。未完成的窗口会重测，旧日志保留。
- 停止后在双车调度电脑查看 SQLite 旁的同名 JSON，每个位置对列出已测状态数和首个共同达标的状态编号。完整差值、实际位姿和回执保存在 SQLite。基站电脑的 `captures` 下按采样 ID 保存两路原始日志和 `receipt.json`。
