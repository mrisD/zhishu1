#!/usr/bin/env bash
set -Eeuo pipefail

readonly CONTROLLER_MANAGER="/controller_manager"
readonly CONTROLLER="dual_arm_controller"
readonly CONTROLLER_NODE="/dual_arm_controller"
readonly HARDWARE_COMPONENT="RightArmSystem"
readonly HOLD_PARAMETER="set_last_command_interface_value_as_state_on_activation"

fail()
{
  echo "错误：$*" >&2
  exit 1
}

call_success_service()
{
  local service_name="$1"
  local service_type="$2"
  local request="$3"
  local output

  if ! output="$(ros2 service call "${service_name}" "${service_type}" "${request}" 2>&1)"; then
    printf '%s\n' "${output}" >&2
    return 1
  fi
  printf '%s\n' "${output}"
  grep -Eiq 'success[=:][[:space:]]*(true|True)' <<<"${output}"
}

command -v ros2 >/dev/null 2>&1 || fail "未找到 ros2，请先 source ROS 2 和工作区环境。"

controller_list="$(ros2 control list_controllers -c "${CONTROLLER_MANAGER}")" || \
  fail "无法连接 ${CONTROLLER_MANAGER}。"
controller_state="$(awk -v name="${CONTROLLER}" '$1 == name {print $NF; exit}' <<<"${controller_list}")"
[[ -n "${controller_state}" ]] || fail "未找到 ${CONTROLLER}。"

hold_value="$(ros2 param get "${CONTROLLER_NODE}" "${HOLD_PARAMETER}" 2>&1)" || \
  fail "无法读取 ${CONTROLLER_NODE} 的安全保持参数。"
grep -Eiq 'false[[:space:]]*$' <<<"${hold_value}" || \
  fail "${HOLD_PARAMETER} 不是 false，拒绝恢复。"

if [[ "${controller_state}" == "active" ]]; then
  echo "[1/5] 停止 ${CONTROLLER}..."
  ros2 control switch_controllers --deactivate "${CONTROLLER}" --strict \
    -c "${CONTROLLER_MANAGER}" || fail "停止 ${CONTROLLER} 失败。"
elif [[ "${controller_state}" == "inactive" ]]; then
  echo "[1/5] ${CONTROLLER} 已停止。"
else
  fail "${CONTROLLER} 当前状态为 ${controller_state}，只允许从 active 或 inactive 恢复。"
fi

echo "[2/5] 清除右臂 Fault / FaultReaction / QuickStop..."
call_success_service /right_arm/reset_fault std_srvs/srv/Trigger '{}' || \
  fail "右臂故障复位失败；控制器保持停止，电机保持禁用。"

echo "[3/5] 将 ${HARDWARE_COMPONENT} 恢复为 active（电机仍保持禁用）..."
ros2 control set_hardware_component_state "${HARDWARE_COMPONENT}" active \
  -c "${CONTROLLER_MANAGER}" || \
  fail "恢复 ${HARDWARE_COMPONENT} 失败；控制器和右臂电机保持禁用。"

echo "[4/5] 重新激活 ${CONTROLLER}，从实际关节位置建立保持命令..."
ros2 control switch_controllers --activate "${CONTROLLER}" --strict \
  -c "${CONTROLLER_MANAGER}" || \
  fail "激活 ${CONTROLLER} 失败；右臂电机保持禁用。"

echo "[5/5] 右臂仍未使能。确认急停已释放、人员已离开机械臂、旧轨迹无需继续。"
[[ -t 0 ]] || fail "当前不是交互终端，拒绝自动使能右臂。"
read -r -p "输入 ENABLE RIGHT 确认使能右臂（其他输入取消）：" confirmation
if [[ "${confirmation}" != "ENABLE RIGHT" ]]; then
  echo "已取消使能；控制器保持当前位置命令，右臂电机保持禁用。"
  exit 0
fi

call_success_service /right_arm/enable std_srvs/srv/SetBool '{data: true}' || \
  fail "右臂使能失败。"
echo "右臂恢复完成：旧轨迹已丢弃，当前保持实际关节位置。"
