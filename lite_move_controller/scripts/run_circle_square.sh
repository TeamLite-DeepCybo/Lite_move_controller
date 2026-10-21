#!/usr/bin/env bash
# 一键：左圆右方（关节预摆位 → 位姿形状），200 s 阻抗 + 动力学前馈。
#
# 前置（必须已起，且 source_names 含 lite_move_controller）：
#   ros2 launch lite_bringup lite_bringup.launch.py mode:=arms controller:=modes \
#       control_mode:=mit source_names:=lite_move_controller
#
# 用法：
#   bash src/lite_move_controller/scripts/run_circle_square.sh
#   DURATION=60 bash ...        # 短跑
#   SPEED=0.05 bash ...         # 更慢
#
# 说明：预摆位走**关节模式**（同一 TRACKING 内），不经过 HOLD —— HOLD 是零力
# 拖动，会漂移且长时间保持会让电机发热。
# ⚠️ ROS 的 setup.bash 会引用未定义变量（AMENT_TRACE_SETUP_FILES 等），
#    在 `set -u` 下 sourcing 会直接报 "未绑定的变量"。所以 source 期间关掉 nounset。
set -eo pipefail
WS=${WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}
set +u
source /opt/ros/jazzy/setup.bash
source "${WS}/install/setup.bash"
set -u

DURATION=${DURATION:-200}
SPEED=${SPEED:-0.05}
PREPOSE=${PREPOSE:-8}
PLANE=${PLANE:-vertical}
DIST_SYM=${DIST_SYM:-0.20}
PARAMS=${PARAMS:-${WS}/src/lite_move_controller/config/impedance_test_200s.yaml}
STATS_FILE=${STATS_FILE:-${WS}/tmp/circle_square_tracking_stats.csv}

echo "[1/4] 检查 bringup / 数据源授权 / 重复实例"
# ⚠️ 两套 bringup 会在同一条 CAN 上互相抢控制器（实测：HOLD↔ZERO_TORQUE 每秒
#    来回切、/joint_states 时有时无）。先确认只有一套。
N_CM=$(pgrep -c -f "[r]os2_control_node" || true)
N_BR=$(pgrep -c -f "[b]ridge_node" || true)
if [[ "${N_CM:-0}" != "1" || "${N_BR:-0}" != "1" ]]; then
  echo "✗ 检测到 ${N_CM:-0} 个 controller_manager / ${N_BR:-0} 个 bridge。"
  echo "  同一条 CAN 上只能有 **一套** bringup。请先停掉多余实例："
  echo "  pkill -f 'lite_bringup.launch.py'; pkill -f ros2_control_node; pkill -f bridge_node"
  exit 1
fi
# /joint_states 必须已有 14 个关节（否则控制器起来也动不了）
NJS=$( (timeout 8 ros2 topic echo /joint_states --once --field name 2>/dev/null || true) \
        | grep -o "'[a-z_]*'" | wc -l)
if [[ "${NJS:-0}" -lt 14 ]]; then
  echo "✗ /joint_states 只有 ${NJS:-0} 个关节（需要 14）。检查 joint_state_broadcaster / 电机供电。"
  exit 1
fi
STATUS=$(timeout 10 ros2 service call /lite_bridge/get_status \
  robstride_msgs/srv/GetBridgeStatus "{}" 2>/dev/null || true)
if [[ "${STATUS}" != *"protocol_name='mit'"* ]]; then
  echo "✗ 桥不在 MIT（或未启动）。先起 bringup："
  echo "  ros2 launch lite_bringup lite_bringup.launch.py mode:=arms controller:=modes control_mode:=mit source_names:=lite_move_controller"
  exit 1
fi
if [[ "${STATUS}" != *"lite_move_controller"* ]]; then
  echo "✗ source_names 不含 lite_move_controller —— 命令会被丢弃"
  exit 1
fi

echo "[2/4] 起 move_controller 与 lite_ik_node 微分逆解节点（若未在跑）"
if ! ros2 node list 2>/dev/null | grep -qx '/lite_move_controller' || \
   ! ros2 node list 2>/dev/null | grep -qx '/lite_ik_node'; then
  # 若有孤立旧实例在跑，先杀掉以彻底杜绝单节点旧实例抢占话题或丢失外部 IK
  pkill -f "move_controller_node" 2>/dev/null || true
  pkill -f "lite_ik_node" 2>/dev/null || true
  sleep 0.8
  mkdir -p "${WS}/log"
  ros2 launch lite_move_controller move_controller.launch.py mode:=impedance \
    enable_ik_node:=true params_file:="${PARAMS}" > "${WS}/log/lmc_test_controller.log" 2>&1 &
  for _ in $(seq 1 40); do
    if ros2 node list 2>/dev/null | grep -qx '/lite_move_controller' && \
       ros2 node list 2>/dev/null | grep -qx '/lite_ik_node'; then
      break
    fi
    sleep 0.5
  done
fi

echo "[3/4] enable（内部会经过 HOLD 再切 TRACKING）"
# ⚠️ 这里**不再固定 sleep**：enable 的序列是 ARMING → SWITCHING(HOLD) → TRACKING，
#    在 HOLD 窗口里下发会被静默忽略。驱动会自己等到 TRACKING/授权后才发第一帧。
if echo "${STATUS}" | grep -q "arbitration_state_name='FAULT'"; then
  echo "  桥处于 FAULT → 先 clear_fault"
  timeout 10 ros2 service call /lite_bridge/clear_fault std_srvs/srv/Trigger "{}" >/dev/null || true
fi
timeout 15 ros2 service call /lite_move_controller/enable std_srvs/srv/Trigger "{}" >/dev/null

echo "[4/4] 预摆位 ${PREPOSE}s + 形状 ${DURATION}s（速度 ${SPEED} m/s）"
trap 'timeout 10 ros2 service call /lite_move_controller/disable std_srvs/srv/Trigger "{}" >/dev/null 2>&1 || true' EXIT
/usr/bin/python3 "${WS}/src/lite_move_controller/scripts/run_circle_square.py" \
  --prepose-sec "${PREPOSE}" --duration "${DURATION}" --speed "${SPEED}" \
  --circle-d 0.12 --square-side 0.08 \
  --dist-sym "${DIST_SYM}" --plane "${PLANE}" \
  --urdf "${WS}/tmp/dyid_runs/run_20261019_234605/robot.urdf" \
  --stats-file "${STATS_FILE}"

echo "完成：控制器回到 STANDBY；桥会 SAFE_HOLD 顶住最后目标（HOLD 会漂移，属正常）"
