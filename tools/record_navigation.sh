#!/usr/bin/env bash
# Read-only capture. Does not start navigation or change velocity owners.
set -eo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$HOME/wheeltec_ros2/install/setup.bash"
[[ ! -f "$HOME/.config/wheeltec_ros/network.sh" ]] || source "$HOME/.config/wheeltec_ros/network.sh"
set -u
mkdir -p "$root/state/navigation-recordings"
exec ros2 bag record --storage-preset-profile resilient \
  -o "$root/state/navigation-recordings/$(date +%Y%m%d-%H%M%S)" \
  /scan /odom /odom_combined /amcl_pose /tf /tf_static /plan /plan_smoothed \
  /nav_cmd_vel /collision_cmd_vel /guard_cmd_vel /cmd_vel /navigation_motion_permit \
  /PowerVoltage /robot_charging_flag /robot_charging_current /robot_red_flag
