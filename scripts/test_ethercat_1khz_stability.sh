#!/usr/bin/env bash
set -o pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="${1:-$(cd "${script_dir}/.." && pwd)}"
duration_s="${2:-120}"
log_dir="${workspace}/log/ethercat_stability"
timestamp="$(date +%Y%m%d_%H%M%S)"
run_log="${log_dir}/run_${timestamp}.log"
summary_log="${log_dir}/summary_${timestamp}.txt"

if ! [[ "${duration_s}" =~ ^[0-9]+$ ]] || (( duration_s < 15 )); then
  echo "duration must be an integer >= 15 seconds" >&2
  exit 2
fi

if pgrep -af 'ros2_control_node|zhishu_robot.moveit.launch.py' >/dev/null; then
  echo "A real-robot control process is already running; stop it before this test." >&2
  pgrep -af 'ros2_control_node|zhishu_robot.moveit.launch.py' >&2
  exit 2
fi

mkdir -p "${log_dir}"
cd "${workspace}" || exit 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

{
  echo "EtherCAT 1 kHz communication-only stability test"
  echo "Started: $(date --iso-8601=seconds)"
  echo "Duration: ${duration_s} s"
  echo "Kernel: $(uname -a)"
  echo "Realtime limit: $(ulimit -r)"
  echo "Memory-lock limit (KiB): $(ulimit -l)"
  echo
  ethercat master 2>&1
  ethercat slaves 2>&1
} | tee "${summary_log}"

echo "Starting communication-only mode; motors remain Operation Disabled and no trajectory is sent."
set +e
timeout --signal=INT --kill-after=10 "${duration_s}s" \
  ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  left_arm_use_fake_hardware:=false right_arm_use_fake_hardware:=false \
  torso_head_use_fake_hardware:=false use_rviz:=false \
  enable_motors_on_activate:=false \
  2>&1 | tee "${run_log}"
launch_rc=${PIPESTATUS[0]}
set -e

timing_lines=$(grep -c 'ethercat_driver: timing rate=' "${run_log}" || true)
deadline_misses=$(awk '
  /deadline_misses=/ {
    split($0, a, "deadline_misses="); split(a[2], b, " "); total += b[1]
  }
  END { print total + 0 }
' "${run_log}")
bad_rates=$(awk '
  /timing rate=/ {
    split($0, a, "timing rate="); split(a[2], b, " ");
    if (b[1] < 990.0 || b[1] > 1010.0) bad++
  }
  END { print bad + 0 }
' "${run_log}")
rt_failures=$(grep -cE 'mlockall failed|pthread_setschedparam.*failed' "${run_log}" || true)
hard_faults=$(grep -ciE 'link.*down|TIMED OUT|UNMATCHED|SKIPPED|not ready' "${run_log}" || true)
wc_regressions=$(awk '
  /ethercat_driver: master[0-9]+ slaves=/ {
    split($0, a, "master"); split(a[2], b, " "); master = b[1]
    if ($0 ~ /wc_state=2([^0-9]|$)/) complete[master] = 1
    else if (complete[master] && $0 ~ /wc_state=[01]([^0-9]|$)/) regressions++
  }
  END { print regressions + 0 }
' "${run_log}")
ethercat_faults=$(( hard_faults + wc_regressions ))
cm_overruns_total=$(grep -cE 'Overrun detected|Overrun might occur' "${run_log}" || true)
cm_overruns_steady=$(awk '
  /You can start planning now!/ { ready = 1; next }
  ready && /Overrun detected|Overrun might occur/ { overruns++ }
  END { print overruns + 0 }
' "${run_log}")
expected_timing_lines=$(( duration_s / 60 ))

result=PASS
if (( launch_rc != 124 && launch_rc != 130 && launch_rc != 0 )); then result=FAIL; fi
if (( timing_lines < expected_timing_lines )); then result=FAIL; fi
if (( deadline_misses != 0 || bad_rates != 0 || rt_failures != 0 || ethercat_faults != 0 || cm_overruns_steady != 0 )); then result=FAIL; fi

{
  echo
  echo "Finished: $(date --iso-8601=seconds)"
  echo "Result: ${result}"
  echo "Launch exit: ${launch_rc} (124/130 are expected timeout exits)"
  echo "Timing samples: ${timing_lines} (minimum expected ${expected_timing_lines})"
  echo "Deadline misses: ${deadline_misses}"
  echo "Out-of-range rates (<990 or >1010 Hz): ${bad_rates}"
  echo "Realtime setup failures: ${rt_failures}"
  echo "EtherCAT/WC fault indicators: ${ethercat_faults}"
  echo "Controller-manager overruns: ${cm_overruns_total} total, ${cm_overruns_steady} after MoveIt ready"
  echo "Run log: ${run_log}"
  echo
  ethercat master 2>&1
  ethercat slaves 2>&1
} | tee -a "${summary_log}"

[[ "${result}" == PASS ]]
