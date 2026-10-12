#!/usr/bin/env bash
# Select the local startup-crash fix without changing the system ROS install.
set -eo pipefail
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
task_overlay="$task_root/output/gz_control_safe_overlay"
if [[ ! -f "$task_overlay/setup.bash" ]]; then
  echo "Build first: bash $task_root/scripts/build_gz_control_safe_overlay.sh" >&2
  exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_overlay/setup.bash"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$task_overlay/gz_ros2_control/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
if [[ $# -eq 0 ]]; then
  echo "Usage: bash $0 command [arguments...]" >&2
  exit 2
fi
exec "$@"
