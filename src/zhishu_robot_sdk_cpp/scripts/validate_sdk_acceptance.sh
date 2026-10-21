#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [workspace-root]" >&2
  exit 2
fi
WS_ROOT="$(realpath "${1:-$PWD}")"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS 2 Jazzy setup not found at $ROS_SETUP" >&2
  exit 2
fi
if [[ ! -d "$WS_ROOT/src/zhishu_robot_sdk_cpp" ]]; then
  echo "ERROR: $WS_ROOT/src/zhishu_robot_sdk_cpp not found" >&2
  exit 2
fi

# shellcheck disable=SC1091
set +u  # ROS setup scripts read optional environment variables without defaults.
source "$ROS_SETUP"
set -u
cd "$WS_ROOT"

colcon build --packages-up-to zhishu_robot_sdk_cpp --cmake-args -DBUILD_TESTING=ON
# shellcheck disable=SC1091
set +u
source "$WS_ROOT/install/setup.bash"
set -u
colcon test --packages-select zhishu_robot_sdk_cpp --event-handlers console_direct+
colcon test-result --verbose

LIB=""
for candidate in \
  "$WS_ROOT/install/zhishu_robot_sdk_cpp/lib/libzhishu_robot_sdk.so" \
  "$WS_ROOT/install/lib/libzhishu_robot_sdk.so"; do
  if [[ -f "$candidate" ]]; then LIB="$candidate"; break; fi
done
if [[ -z "$LIB" ]]; then
  echo "ERROR: libzhishu_robot_sdk.so was not found in install/" >&2
  exit 1
fi

echo "Checking shared-library dependencies: $LIB"
LDD_OUTPUT="$(ldd "$LIB")"
echo "$LDD_OUTPUT"
if grep -q "not found" <<< "$LDD_OUTPUT"; then
  echo "ERROR: unresolved shared-library dependency detected" >&2
  exit 1
fi

SYMBOLS_FILE="$(mktemp)"
trap 'rm -f "$SYMBOLS_FILE"' EXIT
nm -D -C "$LIB" > "$SYMBOLS_FILE"
if ! grep -Fq 'zhishu_robot_sdk::RobotSdk::moveJoints(' "$SYMBOLS_FILE"; then
  echo "ERROR: expected RobotSdk::moveJoints symbol is not exported" >&2
  exit 1
fi

echo "PASS: build, registered tests, shared-library dependencies and public symbol checks completed."
echo "NOTE: this script does not execute robot motion or claim hardware validation."
