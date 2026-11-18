#!/usr/bin/env bash
set -euo pipefail
root=${R550_KIT_ROOT:?Run through ./kit site}
port=${R550_SITE_PORT:-8896}
nav_port=${R550_PANEL_PORT:-8890}
bind=${R550_PANEL_BIND:-0.0.0.0}
prefix=${R550_SCREEN_PREFIX:-r550}
[[ "$port" =~ ^[0-9]{4,5}$ && "$nav_port" =~ ^[0-9]{4,5}$ ]] &&
  (( port < 65536 && port != nav_port )) || { echo '场地面板端口无效'; exit 2; }
mkdir -p "$root/state/logs"
exec 9>"$root/state/site-start.lock"
flock 9
local_url="http://127.0.0.1:$port/api/site/state"
ready() { curl -fs --noproxy '*' --max-time 1 "$local_url" >/dev/null; }
if ! ready; then
  screen -dmS "${prefix}_site" bash -c 'exec python3 -u "$1/rrt_mapping/site_service.py" --bind "$2" --port "$3" --nav-port "$4" >>"$1/state/logs/site-panel.log" 2>&1' _ "$root" "$bind" "$port" "$nav_port" 9>&-
  for i in {1..20}; do
    if ready; then break; fi
    sleep .2
  done
fi
ready || { echo '场地面板未启动，请查看 state/logs/site-panel.log'; exit 1; }
printf '新场地面板已启动，端口 %s\n' "$port"
