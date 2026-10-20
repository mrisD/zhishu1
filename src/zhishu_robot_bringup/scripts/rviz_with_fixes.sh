#!/usr/bin/env bash
set -eo pipefail
# Only RViz inherits the patched visualization package; robot nodes keep their environment.
if [[ -n "${ZHISHU_RVIZ_FIX_PREFIX:-}" ]]; then
  if [[ ! -f "$ZHISHU_RVIZ_FIX_PREFIX/local_setup.bash" ]]; then
    echo "RViz fix prefix not found: $ZHISHU_RVIZ_FIX_PREFIX" >&2
    exit 1
  fi
  source "$ZHISHU_RVIZ_FIX_PREFIX/local_setup.bash"
else
  for rviz_fix_prefix in "$HOME/zhul/rviz_fix_ws/install" "$HOME/rviz_fix_ws/install"; do
    if [[ -f "$rviz_fix_prefix/local_setup.bash" ]]; then
      source "$rviz_fix_prefix/local_setup.bash"
      break
    fi
  done
fi
exec "$@"
