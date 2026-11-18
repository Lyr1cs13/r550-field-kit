#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
local_panel=http://127.0.0.1:8891
robot_panel=http://192.168.0.100:8890
robot_site=http://192.168.0.100:8896
data="$root/demo/data/desktop-panel"
fleet_config="$root/state/fleet/panel.json"
mkdir -p "$root/state/logs"
exec 9>"$root/state/open-vm-panel.lock"
flock -n 9 || exit 0
exec >>"$root/state/logs/vm-demo-launch.log" 2>&1

panel_available() {
  python3 - "$1" <<'PY'
import json
import sys
from urllib.request import ProxyHandler, build_opener

try:
    opener = build_opener(ProxyHandler({}))
    with opener.open(sys.argv[1] + '/api/state', timeout=1) as response:
        state = json.load(response)
    available = state['health']['mode'] == 'robot'
except (OSError, ValueError, KeyError, TypeError):
    available = False
sys.exit(0 if available else 1)
PY
}

open_panel() {
  if ! panel_available "$local_panel"; then
    nohup python3 "$root/demo/desktop_panel.py" --robot-panel "$robot_panel" --data "$data" \
      >"$root/state/logs/desktop-panel.log" 2>&1 </dev/null 9>&- &
    for attempt in {1..30}; do
      if panel_available "$local_panel"; then break; fi
      sleep .2
    done
  fi
  panel_available "$local_panel" || return 1
  xdg-open "$local_panel/" 9>&- &
}

open_fleet_panel() {
  local address
  address=$(python3 -c 'import json, sys; print("http://127.0.0.1:{}".format(json.load(open(sys.argv[1]))["port"]))' "$fleet_config")
  if ! curl --noproxy '*' -fsS --max-time 1 "$address/api/fleet" >/dev/null; then
    nohup python3 "$root/demo/fleet_panel.py" --config "$fleet_config" \
      >"$root/state/logs/fleet-panel.log" 2>&1 </dev/null 9>&- &
    for attempt in {1..30}; do
      if curl --noproxy '*' -fsS --max-time 1 "$address/api/fleet" >/dev/null; then break; fi
      sleep .2
    done
  fi
  curl --noproxy '*' -fsS --max-time 1 "$address/api/fleet" >/dev/null || return 1
  xdg-open "$address/" 9>&- &
}

start_robot() {
  if panel_available "$robot_panel" && site_available; then return 0; fi
  local deadline=$((SECONDS+90)) status
  while (( SECONDS < deadline )); do
    if ssh -n -o BatchMode=yes -o ConnectTimeout=3 -o ConnectionAttempts=1 \
        -o ServerAliveInterval=3 -o ServerAliveCountMax=2 r550 \
        'cd "$HOME/lab_cruise_ws" && ./kit site && if [[ -f state/active_scene.json && ! -f state/site-mapping/owns-robot ]] && ! curl --noproxy "*" -fsS --max-time 2 http://127.0.0.1:8890/api/state >/dev/null; then ./kit nav; fi'; then
      return 0
    else
      status=$?
    fi
    [[ "$status" = 255 ]] || return "$status"
    sleep 2
  done
  return 1
}

site_available() {
  curl --noproxy '*' -fsS --max-time 1 "$robot_site/api/site/state" >/dev/null
}

notice() {
  printf '%s\n' "$1"
  if command -v notify-send >/dev/null; then notify-send '巡航演示' "$1" || true; fi
}

if [[ -f "$fleet_config" ]]; then
  if ! open_fleet_panel; then
    notice '巡航管理页启动失败，请查看 fleet-panel.log。'
    exit 1
  fi
  if ! start_robot; then
    printf '%s\n' '一号车未响应；管理页继续使用当前在线的车辆。'
  fi
  exit 0
fi

opened=false
if panel_available "$local_panel" || [[ -f "$data/map_snapshot.json" || -f "$data/scene/map.yaml" ]]; then
  if open_panel; then opened=true; fi
fi
if ! start_robot; then
  notice '未能启动小车导航。请检查车网连接或查看 vm-demo-launch.log。'
  exit 1
fi
if ! panel_available "$robot_panel" && site_available; then
  xdg-open "$robot_site/" 9>&- &
  exit 0
fi
for attempt in {1..45}; do
  if panel_available "$robot_panel"; then
    if [[ $opened == false ]] && ! open_panel; then
      notice '虚拟机面板启动失败，请查看 desktop-panel.log。'
      exit 1
    fi
    exit 0
  fi
  sleep 1
done
notice '小车已接收导航启动请求，面板还未就绪。请查看车端 state/logs。'
exit 1
