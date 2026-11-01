#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)";role=${1:-robot}
[[ "$role" == robot || "$role" == viewer ]] || { echo './kit setup robot|viewer [--skip-apt]'; exit 2; }
if [[ "${2:-}" != --skip-apt ]]; then
  sudo apt-get update
  sudo apt-get install -y git screen python3-yaml python3-serial python3-numpy python3-scipy python3-pil fonts-noto-cjk \
    python3-colcon-common-extensions python3-rosdep ros-humble-rmw-cyclonedds-cpp ros-humble-teleop-twist-keyboard \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-rviz2
fi
mkdir -p "$root/state" "$root/maps"
if [[ "$role" == robot ]]; then
  source /opt/ros/humble/setup.bash
  source "$HOME/wheeltec_ros2/install/setup.bash"
  overlay="$root/vendor_nav2_overlay";mkdir -p "$overlay/src"
  cp -a "$root/vendor/nav2_controller" "$root/vendor/nav2_rotation_shim_controller" "$overlay/src/"
  cd "$overlay"
  MAKEFLAGS=-j2 colcon build --packages-select nav2_controller nav2_rotation_shim_controller \
    --allow-overriding nav2_controller nav2_rotation_shim_controller --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  source install/setup.bash
  python3 "$root/tools/verify_overlay.py" "$overlay"
fi
chmod +x "$root/kit"
echo '安装完成；下一步配置 DDS 网络，再运行 ./kit doctor。'
