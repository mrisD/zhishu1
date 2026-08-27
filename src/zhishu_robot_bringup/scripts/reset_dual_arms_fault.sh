#!/usr/bin/env bash
set -Eeuo pipefail

readonly CONTROLLER_MANAGER="/controller_manager"
readonly CONTROLLER="dual_arm_controller"
readonly CONTROLLER_NODE="/dual_arm_controller"
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

echo "[2/5] 依次清除左右臂 Fault / FaultReaction / QuickStop..."
call_success_service /left_arm/reset_fault std_srvs/srv/Trigger '{}' || \
  fail "左臂故障复位失败；控制器保持停止，两臂电机保持禁用。"
call_success_service /right_arm/reset_fault std_srvs/srv/Trigger '{}' || \
  fail "右臂故障复位失败；控制器保持停止，两臂电机保持禁用。"

echo "[3/5] 将左右臂硬件组件恢复为 active（电机仍保持禁用）..."
ros2 control set_hardware_component_state LeftArmSystem active \
  -c "${CONTROLLER_MANAGER}" || \
  fail "恢复 LeftArmSystem 失败；控制器和两臂电机保持禁用。"
ros2 control set_hardware_component_state RightArmSystem active \
  -c "${CONTROLLER_MANAGER}" || \
  fail "恢复 RightArmSystem 失败；控制器和两臂电机保持禁用。"

echo "[4/5] 重新激活 ${CONTROLLER}，从两臂实际关节位置建立保持命令..."
ros2 control switch_controllers --activate "${CONTROLLER}" --strict \
  -c "${CONTROLLER_MANAGER}" || \
  fail "激活 ${CONTROLLER} 失败；两臂电机保持禁用。"

echo "[5/5] 两臂仍未使能。确认急停已释放、人员已离开机械臂、旧轨迹无需继续。"
[[ -t 0 ]] || fail "当前不是交互终端，拒绝自动使能双臂。"
read -r -p "输入 ENABLE BOTH 确认依次使能双臂（其他输入取消）：" confirmation
if [[ "${confirmation}" != "ENABLE BOTH" ]]; then
  echo "已取消使能；控制器保持当前位置命令，两臂电机保持禁用。"
  exit 0
fi

call_success_service /left_arm/enable std_srvs/srv/SetBool '{data: true}' || \
  fail "左臂使能失败；两臂保持禁用。"
if ! call_success_service /right_arm/enable std_srvs/srv/SetBool '{data: true}'; then
  echo "右臂使能失败，正在回退并主动禁止左臂..." >&2
  call_success_service /left_arm/enable std_srvs/srv/SetBool '{data: false}' || \
    echo "严重警告：无法确认左臂已经禁用，请立即急停并现场检查。" >&2
  fail "双臂使能未完成。"
fi
echo "双臂恢复完成：旧轨迹已丢弃，当前保持两臂实际关节位置。"
