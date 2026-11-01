# 远程开发与网络

## 接线与网络分工

- 内置 Wi-Fi 连校园网/办公室网络，供电脑上网。
- 外接 USB Wi-Fi 连小车热点，供 SSH 和 ROS。
- VMware 的桥接网卡绑定外接 Wi-Fi；另加 NAT 网卡给 Ubuntu 上网。
- 不把 USB Wi-Fi 直通给虚拟机；由 Windows 持有，再通过 VMware 桥接。

建议地址：小车 `192.168.0.100`，Ubuntu 车网 `192.168.0.250/24`。车网网卡不配置默认网关；互联网默认路由留给 NAT。

Windows 查看实际网卡：

```powershell
Get-NetAdapter
Set-NetIPInterface -InterfaceAlias 'WLAN 2' -AddressFamily IPv4 -IgnoreDefaultRoutes Enabled -InterfaceMetric 500
powershell -ExecutionPolicy Bypass -File tools/remote.ps1
ssh r550
```

`WLAN 2` 换成外接网卡实际名称。代理软件将车网 `192.168.0.0/24` 设为直连。

Ubuntu：

```bash
ip -br addr
sudo nmcli con add type ethernet ifname ens33 con-name robot-lan ipv4.method manual ipv4.addresses 192.168.0.250/24 ipv4.never-default yes
sudo nmcli con up robot-lan
```

网卡名称按实际替换；已有车网连接时直接修改原连接，不重复新建。

首次安装依赖时让 RDK 通过网线或 USB 共享联网；安装完成后可离线运行。

## SSH 与文件同步

```bash
ssh wheeltec@192.168.0.100
scp 文件 wheeltec@192.168.0.100:~/lab_cruise_ws/
```

首次确认主机指纹。需要免密时，在电脑生成独立 SSH 密钥并使用 `ssh-copy-id` 安装公钥；不复制别人的私钥。

开发时用 VS Code Remote SSH 连接小车。ROS、驱动、导航运行在小车；RViz 在电脑 Ubuntu 本地运行。

## ROS 网络

小车：`./kit network --interface wlan0 --peer 192.168.0.250 --domain 20`。

电脑：`./kit network --interface ens33 --peer 192.168.0.100 --domain 20`。

新终端通过 `./kit` 自动载入网络环境。手动运行 ROS 时先执行：

```bash
source /opt/ros/humble/setup.bash
source ~/wheeltec_ros2/install/setup.bash
source ~/.config/wheeltec_ros/network.sh
ros2 topic hz /scan
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_footprint
```

SSH 通、ROS 不通时，先核对两端 Domain、网卡和时间，不先改导航参数。

## 无线桥接的 ARP 问题

若 Windows 能连车，而桥接 Ubuntu 时通时不通，小车的 `ip neigh` 对 Ubuntu 地址显示 FAILED：

1. Windows `Get-NetAdapter` 获取外接 Wi-Fi 的真实 MAC。
2. 小车运行 `sudo ip neigh replace 192.168.0.250 lladdr 外接网卡MAC nud permanent dev wlan0`。
3. `sudo iw dev wlan0 set power_save off`。

这是特定无线桥接组合的处理方法。需要开机保留时用 `tools/wifi_bridge.sh --install`。换网卡后同步更新 MAC。

## 时钟

小车和电脑分别 `date -Ins`。小车离线开机回到错误年份时，先停止 ROS，再从电脑 Ubuntu 执行：

```bash
ssh -t wheeltec@192.168.0.100 sudo date -u -s "@$(date +%s)"
```

已经启动建图时不跳变系统时间。远程外网开发可用校园 VPN 或现有组网工具，SSH 仍走其提供的私网地址。
