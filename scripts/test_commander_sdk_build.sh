#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy was not found at /opt/ros/jazzy/setup.bash" >&2
  exit 2
fi

cd "${WORKSPACE_DIR}"
set +u
source /opt/ros/jazzy/setup.bash
set -u

colcon build \
  --symlink-install \
  --packages-up-to zhishu_robot_commander_cpp

set +u
source install/setup.bash
set -u
colcon test \
  --packages-select zhishu_robot_commander_cpp \
  --event-handlers console_direct+
colcon test-result --verbose

echo "Commander SDK build and unit tests passed."
