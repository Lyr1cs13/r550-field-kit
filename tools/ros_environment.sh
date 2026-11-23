# Source from a new Bash terminal to use this project's ROS environment.
export R550_KIT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
source "$HOME/wheeltec_ros2/install/setup.bash"
if [[ -f "$R550_KIT_ROOT/vendor_nav2_overlay/install/setup.bash" ]]; then
  source "$R550_KIT_ROOT/vendor_nav2_overlay/install/setup.bash"
fi
if [[ -f "$R550_KIT_ROOT/state/network.sh" ]]; then
  source "$R550_KIT_ROOT/state/network.sh"
elif [[ -f "$HOME/.config/wheeltec_ros/network.sh" ]]; then
  source "$HOME/.config/wheeltec_ros/network.sh"
fi
[[ ! -f "$R550_KIT_ROOT/state/runtime.env" ]] || source "$R550_KIT_ROOT/state/runtime.env"
[[ ! -f "$R550_KIT_ROOT/state/workspace.env" ]] || source "$R550_KIT_ROOT/state/workspace.env"
