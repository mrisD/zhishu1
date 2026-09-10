#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy was not found at /opt/ros/jazzy/setup.bash" >&2
  exit 2
fi

cd "${WORKSPACE_DIR}"
source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-up-to zhishu_robot_commander_py

source install/setup.bash
colcon test \
  --packages-select zhishu_robot_commander_py \
  --event-handlers console_direct+
colcon test-result --verbose

echo "Python Commander build and unit tests passed."
