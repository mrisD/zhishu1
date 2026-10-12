#!/usr/bin/env bash
# Rebuild the installed 1.2.20 plugin with startup empty-state guards.
# Isolated workspace overlay only: /opt/ros is never modified.
set -euo pipefail
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
task_source=$(mktemp -d /tmp/guohua-gz-control-source.XXXXXX)
git clone --depth 1 --branch 1.2.20 https://github.com/ros-controls/gz_ros2_control.git "$task_source/upstream"
test "$(git -C "$task_source/upstream" rev-parse HEAD)" = 89ca76ed20842377f879c711c756d444c5b7c20b
git -C "$task_source/upstream" apply "$task_root/scripts/patches/gz_ros2_control_1.2.20_empty_joint_state.patch"
set +u
source /opt/ros/jazzy/setup.bash
set -u
colcon --log-base "$task_source/log" build \
  --base-paths "$task_source/upstream/gz_ros2_control" \
  --build-base "$task_source/build" \
  --install-base "$task_root/output/gz_control_safe_overlay" \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo
echo "Built isolated overlay: $task_root/output/gz_control_safe_overlay"
echo "Source its setup.bash after the workspace setup.bash before testing."
