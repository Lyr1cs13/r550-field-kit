#!/usr/bin/env bash
set -eo pipefail
root=${R550_KIT_ROOT:?Run through ./kit nav}
session_prefix=${R550_SCREEN_PREFIX:-r550}
port=${R550_PANEL_PORT:-8890}
bind=${R550_PANEL_BIND:-0.0.0.0}
[[ ! -f "$root/state/site-mapping/owns-robot" ]] || { echo '车辆正在使用新场地页面，请在该页面切换巡航。'; exit 1; }
[[ "$port" =~ ^[0-9]{4,5}$ ]] && (( port <= 65535 )) || { echo '无效的面板端口'; exit 2; }
ros2 pkg prefix nav2_collision_monitor >/dev/null 2>&1 || { echo '缺少速度碰撞保护组件，请先运行 ./kit setup robot。'; exit 1; }
if pgrep -f 'wheeltec_robot_node|controller_server|sync_slam_toolbox_node|async_slam_toolbox_node|frontier_tasks.py|demo/server[.]py.*--backend[ =]+ros([[:space:]]|$)' >/dev/null; then
  echo '已有底盘、建图或导航运行；先执行 ./kit stop。';exit 1
fi
if command -v fuser >/dev/null; then
  for device in /dev/wheeltec_controller /dev/wheeltec_lidar; do
    if [[ -e "$device" ]] && fuser -s "$device"; then
      printf '设备 %s 已由另一程序使用\n' "$device"
      exit 1
    fi
  done
fi
if [[ -x /usr/local/sbin/r550-host-clock ]] && (( $(date +%s) < 1704067200 )); then
  echo '车端时钟未初始化，请从上位机对时并启动巡航。' >&2
  exit 1
fi
mkdir -p "$root/state/logs"
data_dir=$(python3 - "$root" <<'PY'
import json, sys
from pathlib import Path
root=Path(sys.argv[1])
scene=json.loads((root/'state/active_scene.json').read_text())['scene']
if (root/'scenes'/scene/'site.json').is_file():
    print(root/'demo/data/scenes'/scene)
else:
    print(root/'demo/data/ros')
PY
)
screen -dmS "${session_prefix}_nav" bash -c 'exec ros2 launch "$1/demo/demo.launch.py" > "$1/state/logs/navigation.log" 2>&1' _ "$root"
screen -dmS "${session_prefix}_ui" bash -c 'exec python3 -u "$1/demo/server.py" --backend ros --host "$2" --port "$3" --data "$4" > "$1/state/logs/panel.log" 2>&1' _ "$root" "$bind" "$port" "$data_dir"
printf '车端面板启动于端口 %s\n' "$port"
