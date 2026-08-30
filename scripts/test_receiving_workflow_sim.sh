#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
HEADLESS="true"
SIMULATE_ROBOT="true"
REQUIRE_COMMANDER="true"
MISMATCH_ATTEMPTS="1"
EXPECT_RETRY="true"

usage() {
  echo "Usage: bash scripts/test_receiving_workflow_sim.sh [options]"
  echo "  --gui         Show the Gazebo GUI"
  echo "  --skip-robot  Test LES, scanner, AGV and box flow without MoveIt commands"
  echo "  --no-retry    Make the first barcode scan succeed"
  echo "  --help        Show this help"
}

while (($# > 0)); do
  case "$1" in
    --gui)
      HEADLESS="false"
      ;;
    --skip-robot)
      SIMULATE_ROBOT="false"
      REQUIRE_COMMANDER="false"
      ;;
    --no-retry)
      MISMATCH_ATTEMPTS="0"
      EXPECT_RETRY="false"
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy was not found at /opt/ros/jazzy/setup.bash" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Workspace is not built; run colcon build --packages-up-to zhishu_robot_receiving_sim" >&2
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

LOG_DIR="${WORKSPACE_DIR}/log/receiving_sim_acceptance"
mkdir -p "${LOG_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LAUNCH_LOG="${LOG_DIR}/receiving_${STAMP}.log"

setsid ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py \
  auto_start:=false \
  headless:="${HEADLESS}" \
  use_rviz:=false \
  simulate_robot_motion:="${SIMULATE_ROBOT}" \
  part_mismatch_attempts:="${MISMATCH_ATTEMPTS}" \
  >"${LAUNCH_LOG}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  group_has_live_processes() {
    ps -eo pgid=,stat= | awk -v pgid="${LAUNCH_PID}" \
      '$1 == pgid && $2 !~ /^Z/ {found = 1} END {exit(found ? 0 : 1)}'
  }

  if group_has_live_processes; then
    kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      if ! group_has_live_processes; then
        break
      fi
      sleep 0.25
    done
    if group_has_live_processes; then
      kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
      for _ in $(seq 1 20); do
        if ! group_has_live_processes; then
          break
        fi
        sleep 0.25
      done
    fi
    if group_has_live_processes; then
      kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
    fi
  fi
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Waiting for receiving simulation services (log: ${LAUNCH_LOG})"
SERVICE_READY="false"
for _ in $(seq 1 120); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "Simulation launch exited before becoming ready" >&2
    tail -n 80 "${LAUNCH_LOG}" >&2
    exit 1
  fi
  if ros2 service type /receiving_sim/start 2>/dev/null | \
    grep -q "std_srvs/srv/Trigger"; then
    SERVICE_READY="true"
    break
  fi
  sleep 1
done

if [[ "${SERVICE_READY}" != "true" ]]; then
  echo "Timed out waiting for /receiving_sim/start" >&2
  tail -n 80 "${LAUNCH_LOG}" >&2
  exit 1
fi

# The receiving services start before Gazebo finishes spawning ros2_control and
# before the delayed Commander launch.  Starting the AGV during that phase can
# overlap a simulation-clock reset and invalidate odometry.  The Commander
# subscription is the final component brought up by this launch file, so use it
# as the complete-stack readiness signal even when --skip-robot is selected.
STACK_READY="false"
for _ in $(seq 1 120); do
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "Simulation launch exited while waiting for the control stack" >&2
    tail -n 80 "${LAUNCH_LOG}" >&2
    exit 1
  fi
  if ros2 topic info /zhishu_robot_commander/both_arms/named_target 2>/dev/null | \
    grep -Eq "Subscription count: [1-9][0-9]*"; then
    STACK_READY="true"
    break
  fi
  sleep 1
done

if [[ "${STACK_READY}" != "true" ]]; then
  echo "Timed out waiting for the Commander control stack" >&2
  tail -n 80 "${LAUNCH_LOG}" >&2
  exit 1
fi
sleep 1

set +e
ros2 run zhishu_robot_receiving_sim receiving_acceptance_test --ros-args \
  -p expect_barcode_retry:="${EXPECT_RETRY}" \
  -p require_commander_result:="${REQUIRE_COMMANDER}"
TEST_STATUS=$?
set -e

if ((TEST_STATUS != 0)); then
  echo "Receiving simulation acceptance FAILED" >&2
  tail -n 120 "${LAUNCH_LOG}" >&2
  exit "${TEST_STATUS}"
fi

echo "Receiving simulation acceptance PASSED"
echo "Launch log: ${LAUNCH_LOG}"
