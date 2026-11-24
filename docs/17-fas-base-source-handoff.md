# 固定基站源码交接

本版以 `ESP32_WS63_source_handoff_20261122_131132.zip` 为底稿，接入双车调度的 `FAS_PAIR_1` 接口。两车停稳后，调度发来采样编号、位置对和指定状态。基站测完该状态，保存原始数据并回传结果，然后等待下一条指令。

## 改动位置

| 位置 | 改动 |
| --- | --- |
| `ws63/sle_rssi_dual_server/client/dual_client.c` | 增加 `WSONCE,状态编号,总时长毫秒,稳定等待毫秒`。验证参数及串口状态，按指定编号设置两片移相器，启动一次采样并输出 A/B 样本数和 RSSI 和值。窗口结束后输出 `source=one_shot_done` 并停住。链路等待也有截止时间。`WSGET` 增加 `oneshot_api=1`。现有 `WSSTOP` 可中断这次采样。 |
| `tools/fas_base.py` | 新增基站主机程序，独占 WS63 接收板和 C5 主控串口。接收采样请求、分发硬件命令、封装结果、保存原始日志，支持查询和取消。 |
| `tools/start_fas_base.ps1` | Windows 启动入口。首次选择两个基站 COM 口并生成本地密钥，后续沿用。默认配置对应现有双车及 Windows 与虚拟机之间的网络。 |
| `esp32/firmware/main`、`sub`、`common` | 沿用交接版 C5 源码。主机通过 `GPIO24CFG` 固定状态并核对辅控 ACK，再用 `SESSIONARM/SESSIONSTART/SESSIONSTOP` 管理窗口。 |

主控输出的 `H` 行核对两侧配置，UART1 的 `A` 行只确认辅控 12 位值。采样数据的 `C` 行才携带合并后的 24 位状态。实板已验证状态 0、200、201，以及取消状态 200 后重新采样。

`patches/ws63-one-shot.diff` 是相对于上述交接包的 WS63 采样代码差异。源码包还包含可放进 SDK 的 WS63 工程目录及主机测试 `demo/tests/test_fas_base.py`。随包的 `tools/build_roles.py` 额外适配了本机 SDK 的 `custom` 示例接入路径。

## 与移动端对接

| 请求 | 行为 |
| --- | --- |
| `GET /fas/health` | 返回协议版本、配置的 A/B 射频身份、接收链和基站状态 |
| `POST /fas/capture` | 按 `trial_id` 受理一次采样。传入运行编号、指定状态、A/B 身份、地图 ID、点位编号、实际位姿、窗口长度和最少样本数 |
| `GET /fas/capture/<trial_id>` | 查询 `running`、`complete`、`incomplete` 或 `canceled`。有效结果附实际状态核验、A/B 样本数、平均 RSSI 和原始数据目录 |
| `POST /fas/cancel` | 取消指定运行中的窗口。关闭确认前返回 `canceling`，关闭后返回 `idle` |

请求通过 `X-FAS-Key` 验证。重复提交同一采样编号和相同内容会返回原记录。位姿使用同一张地图，`x、y` 用米，`yaw` 用弧度。WS63 和 C5 在同一次停车期间分别采窗口，按各自接收链计算 A、B 的算术均值。接收状态和车辆身份经过核对后才输出有效结果。

移动端负责选择位置对、派车、处理回充和判定 6 dB。每一对位置从状态 0 开始。两种无线在同一状态各自达到 `A − B ≥ 6 dB` 时记录命中，换下一对。扫完 0 至 4095 均未命中时记录未找到，也继续下一对。调度的 SQLite 保存有效状态和下一步进度，基站按 `trial_id` 保存两路串口日志与回执。重启基站时先关闭遗留窗口，再接收调度发来的续测请求。

## WS63 重编译

在带有 `application/samples/custom` 的 WS63 SDK 中，把 `sle_rssi_dual_server` 复制到该目录。SDK 接入片段在 `ws63/sdk-integration.md`。如果已用原交接版接入 SDK，可以直接替换 `client/dual_client.c`，沿用原有的顶层接入与工具链。用对应环境编译 Client 角色。

```bash
cd <SDK>/src
python3 application/samples/custom/sle_rssi_dual_server/tools/build_roles.py CLIENT
```

主机需要 Python 和 `pyserial`。在源码包根目录运行 `python -m unittest discover -s demo/tests -p test_fas_base.py`，可离线核对单窗、取消、重复请求及重启恢复。实际板卡验收先让两车停在原地，确认基站能分辨 A/B，再测状态 200、201 和取消后重采。

基站关窗回执超时会保留原始日志并标记该窗无效。基站自动重开两路串口，重新确认 WS63 停止和 C5 会话关闭后开放采样。主控在此期间保留断点，等待旧窗口关闭后重采。
