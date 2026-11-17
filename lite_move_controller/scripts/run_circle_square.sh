#!/usr/bin/env bash
# 仅运行外部接口示例。先加载已安装工作空间，并由操作者启动 move_controller。
# 不访问底层服务、不终止已有节点、不依赖源码目录或 URDF。
set -eo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
GRIPPER_ARGS=()
if [[ "${ENABLE_GRIPPER:-1}" == "0" ]]; then
  GRIPPER_ARGS+=(--no-gripper)
fi
exec "${PYTHON_BIN:-/usr/bin/python3}" "${SCRIPT_DIR}/run_circle_square.py" \
  --duration "${DURATION:-200}" --speed "${SPEED:-0.01}" \
  --prepose-sec "${PREPOSE:-30}" --plane "${PLANE:-vertical}" \
  --dist-sym "${DIST_SYM:-0.20}" \
  --stats-file "${STATS_FILE:-circle_square_tracking_stats.csv}" \
  "${GRIPPER_ARGS[@]}" "$@"
