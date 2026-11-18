# 双车双频固定基站联调

## 设备接线

| 位置 | WS63 | ESP32-C5 |
|---|---|---|
| A、B 两车各一套 | 对应车号的 FAS 发端接本车 RDK USB。停稳开窗才广播 | 两车都用拓扑包中的 `0xE905` 发端。接 RDK USB 供电，按各自完整 Wi-Fi STA MAC 注册 |
| 固定基站 | 收 A、B 的星闪包，按相位、接收链和发端分流。原始数据写基站主机 | 主控、辅控按阵列包接线。主控的原生 USB 出数据。辅控 USB 供电，板间 UART1 同步 |

现有 C5 离线包的 `03_Car1_Client.cmd` 和 `04_Car2_Client.cmd` 烧入同一镜像，窗口登记各自的真实 MAC。车载固件默认静默，收到基站的发现和白名单配置后才参加发包。基站主控以完整 MAC 分流，并核对实际工作信道。

现有 WS63 相控阵包在接收端读取一条已建立 SLE 链路的 RSSI，自动扫描两片移相器的 `64 × 64` 组设置。新车载 A、B 固件采用可区分的无连接广播，地址分别为 `52:35:35:30:00:a1` 与 `52:35:35:30:00:b1`。基站接收端需要被动扫描两个地址，按指令选相位，并分别保存 A、B 的逐包 RSSI。

PE44820 本身是 8 bit。WS63 示例每片只用高 6 bit，因此两片合成 4096 个数字状态。芯片标称频段为 1.7–2.2 GHz，厂家列出 1.1–3.0 GHz 的窄带扩展应用。2.4 GHz 的相位和插损需按实物校验，5.8 GHz 侧需确认所用硬件的对应频段。[pSemi PE44820](https://www.psemi.com/products/rf-phase-amplitude-control/phase-shifters/pe44820/)

## 基站主机接口

HTTP 请求使用 `X-FAS-Key`。采集窗口及原始文件归基站主机管理。WS63、C5 的原始包均保留各自的接收时间、完整发端地址、接收链编号、轮次和实际状态字。

| 请求 | 用途 |
|---|---|
| `GET /fas/health` | 返回 `protocol: "FAS_PAIR_1"`、`ready: true`、`tx_configured` 和已启用的 `receivers` |
| `POST /fas/capture` | 接收一个采样请求。按 `trial_id` 幂等。两频段设置成功并稳定后开始记录，立即返回受理状态 |
| `GET /fas/capture/<trial_id>` | 返回该窗口状态、完整回执或 404。结束后持久保存同一回执 |
| `POST /fas/cancel` | 按 `run_id`、`trial_id` 关闭该窗口，释放 C5 发包，保留原始数据 |

`tx_configured` 使用 `{"sle":{"A":"…","B":"…"},"c5":{"A":"…","B":"…"}}`。接收链用 `sle/main`、`sle/aux`、`c5/main`、`c5/sub` 等实际可独立计算的名称。主辅板透传同一链数据时只登记一个可用接收链。

采样请求示例：

```json
{
  "protocol": "FAS_PAIR_1",
  "run_id": "44e76acffc3a42f39cc1bcb23334eb01",
  "trial_id": "afd543c846a74808aaecd64a20058d01",
  "state_id": 65,
  "radios": {"sle": [1, 1], "c5": 4097},
  "tx": {
    "sle": {"A": "52:35:35:30:00:a1", "B": "52:35:35:30:00:b1"},
    "c5": {"A": "02:00:00:00:00:c1", "B": "02:00:00:00:00:c2"}
  },
  "cars": {
    "A": {"point_id": "fixed-003", "pose": [0.12,-1.45,-1.57], "frame": "map", "map_id": "当前场地图ID"},
    "B": {"point_id": "fixed-028", "pose": [2.46,-7.02,-1.57], "frame": "map", "map_id": "当前场地图ID"}
  },
  "min_samples": 30,
  "window_s": 3.0
}
```

基站确认实际应用的状态后，按同一接收链和同一窗口计算两车的 RSSI 中位数。完成回执原样带回 `protocol`、`run_id`、`trial_id`、`state_id`、`radios`、`tx`，另加以下字段。

```json
{
  "status": "complete",
  "raw_ref": "run/afd543c846a74808aaecd64a20058d01",
  "data": {
    "sle/main": {"epoch": 32, "rssi_dbm": {"A": -62, "B": -69}, "samples": {"A": 45, "B": 43}},
    "c5/main":  {"epoch": 97, "rssi_dbm": {"A": -65, "B": -72}, "samples": {"A": 41, "B": 41}}
  }
}
```

使用 `status: "incomplete"` 报告丢线、切态未确认或样本不足。每次 `POST` 应在 2 秒内返回。采样后及时关闭发包，超时或主机失联后由基站自行结束窗口。早于应用回执或落在前一轮 epoch 的数据留在原始记录，排除出本轮统计。

SLE 的例程按 `state_id = 64 * N1 + N2` 编号。C5 包回传 24 bit 的 `gpio_state`，低 12 bit 为主控，高 12 bit 为辅控。基站提供长度 4096 的 `c5_states.json` 整数数组，按顺序将实验编号映射到原始状态字。聚合代码见 [C5 采集行与窗口解析](../demo/fas_experiment.py)。`purpose: "preflight"` 是发车前静止联调窗口，允许两车停在当前已知位置，保留数据并单独标记。

## 车端配置与启用

1. 两车用同一 `map_id` 的场地图，各自标定实际充电桩。保留原项目，通过既有巡航入口启动已安装本功能的版本。
2. 两车分别烧 WS63 A 或 B 固件。新样例的 CMake、Kconfig 入口位于 [WS63 SDK 片段](../ws63/sdk_integration/)。USB 串口 115200、8N1，`AT+FASHELLO` 回应角色、星闪地址和就绪/发包状态。上位机每个采样窗口用 `AT+FASON` 续发，窗口结束用 `AT+FASOFF`，失联 5 秒自动停发。
3. C5 使用离线包的车载镜像。烧录完成后贴好车号和真实 STA MAC。两车 RDK 分别查询 `ls -l /dev/serial/by-id /dev/serial/by-path`，确认板与设备路径。RDK 读取 WS63 串口，C5 的发端身份由固定基站收包验证。
4. 在各车工程的 `state/fas_robot.json` 保存 `car`、`map_id`、`ws63_device`、`c5_device`、`c5_mac`。其中 `car` 是 `A` 或 `B`，两块板各填对应的 `/dev/serial/by-id/…` 或 `/dev/serial/by-path/…`。重启车端巡航服务，面板保留普通定时模式。为各车的 `config/endurance.yaml` 设置启用、低电判定、实测续作电压与稳定时长。
5. 在基站电脑配置两条分别可达的车端 URL，使用现有 VM 分车入口或转发。配置文件示例如下。

```json
{
  "cars": {"A": "http://127.0.0.1:8891", "B": "http://127.0.0.1:8892"},
  "base": {"address": "http://127.0.0.1:8795", "key_file": "base.key"},
  "tx": {
    "sle": {"A": "52:35:35:30:00:a1", "B": "52:35:35:30:00:b1"},
    "c5": {"A": "02:00:00:00:00:c1", "B": "02:00:00:00:00:c2"}
  },
  "criteria": ["sle/main", "c5/main"],
  "c5_states": "c5_states.json"
}
```

将示例 C5 地址替换为实际值。默认示例判据为同一状态、同一位置对两频段分别满足 `median(A) - median(B) >= 6 dB`。如按频段各自寻找位置对，分别设置判定接收链并使用独立实验库。

## 开测和续作

用仓库自带的示例地图生成 50 点并做离线演练。

```bash
mkdir -p state
python3 tools/fixed_route.py --scene scenes/demo-room --vertices config/fas-u.example.json \
  --count 50 --heading-deg -90 --clearance .27 --output state/fas-demo-u-50.json
python3 tools/fas_experiment.py --route state/fas-demo-u-50.json --journal state/fas-rehearsal.sqlite
```

到现场后按实际通道填写折线拐点，用同一命令从两车共用的场景生成 `state/north-u-50.json`。先做静止联调，核对基站原始数据和两车身份，再运行少量状态的实车试采。

```bash
python3 tools/fas_experiment.py --route state/north-u-50.json --journal state/fas-bench.sqlite --live-config fas-config.json --bench --window-s 3
python3 tools/fas_experiment.py --route state/north-u-50.json --journal state/fas-run.sqlite --live-config fas-config.json --count 16 --window-s 3
```

正式轮次使用 `--count 4096`。沿用同一数据库和参数即可跳过已判定的状态。手动按面板「停止任务」后，核对现场再加 `--resume-operator`。要切回普通巡航，沿用同一命令加 `--release`，释放已停稳的在线车辆。中断窗口先关闭，再复测当前状态。SQLite 保存试验结果、未完成窗口和回充状态，同目录 JSON 给出逐状态结论。

- 每个状态优先复测上一个有效位置对。未通过时依次尝试新对，优先历史上成功且调度距离较短的候选。达到搜索预算时记 `budget_exhausted`，所有候选都有效测过仍未找到时记 `no_match`。
- 车停稳后，以实际地图 `x、y、yaw` 开窗。结束后复核位姿。位置对可以在不同状态重复使用，每个状态分别实测确认。
- 同一走廊内，车身与直线对齐且实时净空允许时，按目标在车头或车尾一侧决定直行或倒车。倒车上限为 0.08 m/s。拐角和横向目标使用原生规划。停稳后按 3° 朝向门限验收，超差时结合轮速回传提前制动，再复核朝向。到点记录包含行驶方向、调向次数和从首次调向到停稳的时间。
- 两条走廊预约分开时双车并行，相交时依序通行。无法避免会车时换候选。回充独占路线，满足电压、充电稳定和桩位校准条件后续采。通道始终被挡、定位失效或充电接触失败时保留进度并停车。
- 每态若至少采 1 秒，4096 态仅采样就需约 68 分钟。每态若另走 20 秒，单是行驶就约 22.8 小时。优先复用两车停位，实际用时按现场预检测到的有效速率估算。
