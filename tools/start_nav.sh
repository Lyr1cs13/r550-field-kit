#!/usr/bin/env bash
set -eo pipefail
root=${R550_KIT_ROOT:?Run through ./kit nav}
if pgrep -f 'wheeltec_robot_node|controller_server|sync_slam_toolbox_node|async_slam_toolbox_node|frontier_tasks.py|demo/server.py' >/dev/null; then
  echo '已有底盘、建图或导航运行；先执行 ./kit stop。';exit 1
fi
mkdir -p "$root/state/logs"
screen -dmS r550_nav bash -c 'exec ros2 launch "$1/demo/demo.launch.py" > "$1/state/logs/navigation.log" 2>&1' _ "$root"
screen -dmS r550_ui bash -c 'exec python3 -u "$1/demo/server.py" --backend ros --host 0.0.0.0 --port 8890 > "$1/state/logs/panel.log" 2>&1' _ "$root"
echo '面板： http://小车IP:8890/ ，启动后不发车。'
