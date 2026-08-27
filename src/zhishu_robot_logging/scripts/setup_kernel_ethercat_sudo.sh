#!/usr/bin/env bash
set -euo pipefail

readonly SUDO_BIN="/usr/bin/sudo"
readonly DMESG_BIN="/usr/bin/dmesg"
readonly SUDOERS_FILE="/etc/sudoers.d/zhishu_robot_dmesg"

if [[ ! -x "${SUDO_BIN}" ]]; then
  echo "错误：找不到 ${SUDO_BIN}" >&2
  exit 1
fi

if [[ ! -x "${DMESG_BIN}" ]]; then
  echo "错误：找不到 ${DMESG_BIN}" >&2
  exit 1
fi

VISUDO_BIN="$(command -v visudo || true)"
TIMEOUT_BIN="$(command -v timeout || true)"
if [[ -z "${VISUDO_BIN}" ]]; then
  echo "错误：找不到 visudo" >&2
  exit 1
fi
if [[ -z "${TIMEOUT_BIN}" ]]; then
  echo "错误：找不到 timeout" >&2
  exit 1
fi

if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
  TARGET_USER="${SUDO_USER}"
else
  TARGET_USER="$(id -un)"
fi

if [[ "${TARGET_USER}" == "root" ]]; then
  echo "错误：请使用普通机器人账户运行本脚本，不要直接以 root 用户运行。" >&2
  exit 1
fi

if [[ ! "${TARGET_USER}" =~ ^[A-Za-z0-9._-]+$ ]]; then
  echo "错误：用户名包含不支持的字符：${TARGET_USER}" >&2
  exit 1
fi

TEMP_RULE="$(mktemp /tmp/zhishu_robot_dmesg.XXXXXX)"
cleanup() {
  if [[ -f "${TEMP_RULE}" ]]; then
    /usr/bin/rm -f -- "${TEMP_RULE}"
  fi
}
trap cleanup EXIT

printf '%s ALL=(root) NOPASSWD: %s --follow-new --ctime\n' \
  "${TARGET_USER}" "${DMESG_BIN}" > "${TEMP_RULE}"
chmod 0440 "${TEMP_RULE}"

echo "将为用户 ${TARGET_USER} 配置 EtherCAT 内核日志读取权限。"
echo "接下来只需要输入这一次 sudo 密码；密码不会被脚本保存。"

"${SUDO_BIN}" -v
"${SUDO_BIN}" "${VISUDO_BIN}" -cf "${TEMP_RULE}"
"${SUDO_BIN}" /usr/bin/install \
  -o root -g root -m 0440 "${TEMP_RULE}" "${SUDOERS_FILE}"
"${SUDO_BIN}" "${VISUDO_BIN}" -cf "${SUDOERS_FILE}"

"${SUDO_BIN}" -k
set +e
"${TIMEOUT_BIN}" 2s "${SUDO_BIN}" -n \
  "${DMESG_BIN}" --follow-new --ctime >/dev/null 2>&1
TEST_STATUS=$?
set -e

if [[ ${TEST_STATUS} -ne 0 && ${TEST_STATUS} -ne 124 ]]; then
  echo "错误：免密码规则测试失败，返回码 ${TEST_STATUS}。" >&2
  echo "请执行：sudo visudo -cf ${SUDOERS_FILE}" >&2
  exit 1
fi

echo
echo "配置成功。"
echo "sudoers 文件：${SUDOERS_FILE}"
echo "日志程序现在可以自动执行："
echo "  sudo -n ${DMESG_BIN} --follow-new --ctime"
echo
echo "请重新启动 bringup；新的 ethercat_kernel.log 将不再出现密码错误。"
