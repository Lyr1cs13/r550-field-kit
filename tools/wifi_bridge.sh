#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
if [[ "${1:-}" == --install ]]; then
  iface=${2:?网卡名};peer=${3:?电脑Ubuntu车网IP};mac=${4:?外接WiFi真实MAC}
  [[ "$iface" =~ ^[a-zA-Z0-9_.:-]+$ && "$peer" =~ ^[0-9.]+$ && "$mac" =~ ^([a-fA-F0-9]{2}:){5}[a-fA-F0-9]{2}$ ]] || exit 2
  printf 'ROBOT_IFACE=%s\nVIEWER_IP=%s\nBRIDGE_MAC=%s\n' "$iface" "$peer" "$mac" > /etc/default/r550-field-network
  cat > /etc/systemd/system/r550-field-network.service <<EOF
[Unit]
Description=R550 wireless bridge peer and power management
After=NetworkManager.service
[Service]
Type=oneshot
ExecStart=/bin/bash $root/tools/wifi_bridge.sh --apply
RemainAfterExit=yes
[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload;systemctl enable --now r550-field-network.service
elif [[ "${1:-}" == --apply ]]; then
  source /etc/default/r550-field-network
  for attempt in $(seq 1 30); do
    if ip -4 addr show "$ROBOT_IFACE" | grep -q 'inet '; then
      iw dev "$ROBOT_IFACE" set power_save off
      ip neigh replace "$VIEWER_IP" lladdr "$BRIDGE_MAC" nud permanent dev "$ROBOT_IFACE"
      exit 0
    fi
    sleep 2
  done
  exit 1
else echo 'sudo bash tools/wifi_bridge.sh --install wlan0 192.168.0.250 外接网卡MAC';exit 2;fi
