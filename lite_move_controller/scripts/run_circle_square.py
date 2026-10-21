#!/usr/bin/env python3
"""左圆右方测试驱动（关节预摆位 → 位姿形状，同一 TRACKING 内完成）。

一揽子脚本：只要底层 bringup 已启动（MIT 模式），本脚本自动完成：
  1. launch move_controller_node + lite_ik_node（自动杀残留实例）
  2. 等待节点就绪 → 自动 enable 控制器
  3. 预摆位 → 形状跟踪 → 统计报表
  4. 退出时自动清理子进程

特点：
1. 几何对齐：在垂直于机器人对称面（X=0）的平面内绘图，双臂中点严格对称（距离对称面约 0.2m）。
2. 全链路误差统计：在线统计并解耦 终端期望轨迹 (p_d)、终端规划轨迹 (p_plan)、终端跟踪轨迹 (p_act) 三者关系。
3. 路径安全：纯本地 tmp 目录解析，杜绝 /tmp 根目录越界。
"""
import sys, os

# ---------------------------------------------------------------------------
# 环境自适应引导：
# 1. 自动定位工作区根目录
# 2. 如果在 conda 环境（Python 版本与 ROS 不兼容）或未加载工作区库路径，
#    自动补齐 LD_LIBRARY_PATH、PYTHONPATH 并切到系统 /usr/bin/python3 执行
# ---------------------------------------------------------------------------
def _find_workspace_root():
    cur = os.path.dirname(os.path.abspath(__file__))
    while cur and cur != os.path.dirname(cur):
        if os.path.isdir(os.path.join(cur, 'install')) or os.path.isdir(os.path.join(cur, 'build')):
            return cur
        cur = os.path.dirname(cur)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))

_ws_root = _find_workspace_root()
_install_dir = os.path.join(_ws_root, 'install')

_need_reexec = False
_target_py = sys.executable

if os.path.exists('/usr/bin/python3') and sys.executable != '/usr/bin/python3':
    _target_py = '/usr/bin/python3'
    _need_reexec = True

_lib_dirs = []
_py_dirs = []
if os.path.exists(_install_dir):
    for _d in os.listdir(_install_dir):
        _lp = os.path.join(_install_dir, _d, 'lib')
        if os.path.isdir(_lp):
            _lib_dirs.append(_lp)
        for _sub in ['lib/python3.12/site-packages', 'local/lib/python3.12/dist-packages']:
            _pp = os.path.join(_install_dir, _d, _sub)
            if os.path.isdir(_pp):
                _py_dirs.append(_pp)

_curr_ld = os.environ.get('LD_LIBRARY_PATH', '')
if not all(_lp in _curr_ld for _lp in _lib_dirs if 'robstride_msgs' in _lp or 'move_msgs' in _lp):
    _need_reexec = True

if _need_reexec and '_AUTO_BOOTSTRAP_DONE' not in os.environ:
    _new_ld = ':'.join(_lib_dirs) + (':' + _curr_ld if _curr_ld else '')
    _curr_py = os.environ.get('PYTHONPATH', '')
    _new_py = ':'.join(_py_dirs) + (':' + _curr_py if _curr_py else '')
    _new_env = dict(os.environ, LD_LIBRARY_PATH=_new_ld, PYTHONPATH=_new_py, _AUTO_BOOTSTRAP_DONE='1')
    os.execve(_target_py, [_target_py] + sys.argv, _new_env)

import argparse, math, time, csv, signal, subprocess, shlex, atexit
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from move_msgs.msg import MoveStatus, MoveTarget, MoveTargetArray
from move_msgs.srv import GetMoveStatus
from std_srvs.srv import Trigger
from robstride_msgs.msg import SafetyStatus, MotionCommand
import pinocchio as pin

# ---------------------------------------------------------------------------
# 自动拉起 move_controller launch 并在退出时清理
# ---------------------------------------------------------------------------
_launch_proc = None   # 全局引用，供 atexit / signal 清理
_launch_log_file = None


def _cleanup_launch():
    """终止由本脚本拉起的 launch 子进程树。"""
    global _launch_proc, _launch_log_file
    if _launch_proc is None:
        return
    proc = _launch_proc
    _launch_proc = None
    if proc.poll() is None:
        # 先 SIGINT（让 ros2 launch 优雅关闭子节点），2 s 后 SIGKILL
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except OSError:
            pass
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except OSError:
                pass
    if _launch_log_file is not None:
        try:
            _launch_log_file.close()
        except OSError:
            pass
        _launch_log_file = None


def _launch_move_controller(ws, mode='regular', joint_mode='arms',
                            auto_enable=False, extra_args=None):
    """用 ros2 launch 拉起 move_controller_node + lite_ik_node。

    返回 subprocess.Popen。调用前会自动杀掉残留的 move_controller_node 实例。
    """
    global _launch_proc, _launch_log_file

    # 杀残留实例（与节点的 duplicate_instance_policy=kill 等效，但更可靠）
    for proc_name in ['move_controller_node', 'lite_ik_node']:
        try:
            out = subprocess.check_output(
                ['pgrep', '-f', proc_name], text=True).strip()
            for pid_str in out.splitlines():
                pid = int(pid_str.strip())
                if pid != os.getpid():
                    print(f"  ⚡ 杀掉残留 {proc_name} (PID {pid})")
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except OSError:
                        pass
                    time.sleep(0.1)
        except (subprocess.CalledProcessError, ValueError):
            pass  # 没找到残留进程，正常

    # 构造 launch 命令
    # source install/setup.bash 在 shell 里执行，确保环境正确
    setup_bash = os.path.join(ws, 'install', 'setup.bash')
    launch_cmd = (
        f'source {shlex.quote(setup_bash)} && '
        f'ros2 launch lite_move_controller move_controller.launch.py'
        f' mode:={mode} joint_mode:={joint_mode}'
        f' auto_enable:={"true" if auto_enable else "false"}'
        f' duplicate_instance_policy:=kill'
    )
    if extra_args:
        launch_cmd += ' ' + extra_args

    log_dir = os.path.join(ws, 'tmp')
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, 'move_controller_launch.log')
    _launch_log_file = open(log_path, 'w', buffering=1)

    print(f"🚀 自动拉起 move_controller (日志: {log_path}): mode={mode}, joint_mode={joint_mode}")
    proc = subprocess.Popen(
        launch_cmd, shell=True, executable='/bin/bash',
        stdout=_launch_log_file, stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,  # 新进程组，方便整组杀
    )
    _launch_proc = proc
    atexit.register(_cleanup_launch)
    return proc

LEFT = ['left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
        'left_elbow_pitch', 'left_wrist_yaw', 'left_wrist_roll', 'left_wrist_pitch']
RIGHT = ['right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
         'right_elbow_pitch', 'right_wrist_yaw', 'right_wrist_roll', 'right_wrist_pitch']
ALL = LEFT + RIGHT

# 终端夹爪水平向前姿态：
# 工具系 +Z (手指伸出方向) 对齐世界系 -Y (水平向前)
# 工具系 +X (手指开合方向) 对齐世界系 +X (水平横向)
# 工具系 +Y (手爪法向) 对齐世界系 +Z (垂直向上)
# 对应欧拉角 (90°, 0°, 0°)，对应四元数 [x=0.7071068, y=0.0, z=0.0, w=0.7071068]
HORIZONTAL_FORWARD_QUAT = np.array([0.70710678, 0.0, 0.0, 0.70710678])

SAFETY_FLAG_NAMES = (
    (SafetyStatus.FLAG_BUS_OFF, 'BUS_OFF'),
    (SafetyStatus.FLAG_RX_TIMEOUT, 'RX_TIMEOUT'),
    (SafetyStatus.FLAG_TX_QUEUE_OVERRUN, 'TX_QUEUE_OVERRUN'),
    (SafetyStatus.FLAG_MOTOR_FAULT, 'MOTOR_FAULT'),
    (SafetyStatus.FLAG_TEMPERATURE_LIMIT, 'TEMPERATURE_LIMIT'),
    (SafetyStatus.FLAG_INVALID_FRAME, 'INVALID_FRAME'),
    (SafetyStatus.FLAG_SERVO_DROP, 'SERVO_DROP'),
    (SafetyStatus.FLAG_POSITION_CLAMPED, 'POSITION_CLAMPED'),
    (SafetyStatus.FLAG_ESTOP, 'ESTOP'),
)


def safety_flags_text(flags):
    """把稳定的 SafetyStatus 位掩码变成人能直接排查的文本。"""
    names = [name for bit, name in SAFETY_FLAG_NAMES if flags & bit]
    return '|'.join(names) if names else 'NONE'


class RuntimeSafetyAbort(RuntimeError):
    """运行中安全链或控制权丢失；调用方必须停止下发并以失败码退出。"""

def filleted_square_point(side, r_corner, arc):
    """带圆角平滑过渡的正方形轨迹发生器（保证 C1 速度连续，消除拐角加速度奇点）"""
    h = side / 2.0
    rc = min(r_corner, h * 0.49)
    L_straight = side - 2.0 * rc
    L_arc = 0.5 * math.pi * rc
    L_segment = L_straight + L_arc
    L_total = 4.0 * L_segment

    s = arc % L_total
    seg_idx = int(s // L_segment) % 4
    s_local = s - seg_idx * L_segment

    # 4个圆角弧的中心坐标
    c_x = [h - rc, -h + rc, -h + rc, h - rc]
    c_z = [h - rc, h - rc, -h + rc, -h + rc]

    if s_local < L_straight:
        if seg_idx == 0:   # 右侧边向上
            return h, (-h + rc) + s_local
        elif seg_idx == 1: # 顶侧边向左
            return (h - rc) - s_local, h
        elif seg_idx == 2: # 左侧边向下
            return -h, (h - rc) - s_local
        else:              # 底侧边向右
            return (-h + rc) + s_local, -h
    else:
        arc_s = s_local - L_straight
        theta_rel = arc_s / rc
        if seg_idx == 0:
            theta = 0.0 + theta_rel
        elif seg_idx == 1:
            theta = 0.5 * math.pi + theta_rel
        elif seg_idx == 2:
            theta = math.pi + theta_rel
        else:
            theta = 1.5 * math.pi + theta_rel
        return c_x[seg_idx] + rc * math.cos(theta), c_z[seg_idx] + rc * math.sin(theta)

def solve_ik_6d_bounded(model, data, arm_joints, tip_id, target_pos, target_quat, q_seed_dict):
    """Pinocchio 6D 阻尼最小二乘有界逆解（带零空间避限位拉伸与安全缓冲裕度）"""
    joint_ids = [model.getJointId(n) for n in arm_joints]
    q_min = np.array([model.lowerPositionLimit[model.joints[jid].idx_q] for jid in joint_ids])
    q_max = np.array([model.upperPositionLimit[model.joints[jid].idx_q] for jid in joint_ids])

    R_target = pin.Quaternion(target_quat[3], target_quat[0], target_quat[1], target_quat[2]).toRotationMatrix()

    q = pin.neutral(model)
    for n, v in q_seed_dict.items():
        if n in arm_joints:
            q[model.joints[model.getJointId(n)].idx_q] = v

    for _ in range(400):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        curr_M = data.oMf[tip_id]
        err_p = target_pos - curr_M.translation
        R_err = R_target @ curr_M.rotation.T
        err_r = pin.log3(R_err)
        err = np.concatenate([err_p, err_r])

        if np.linalg.norm(err_p) < 1e-4 and np.linalg.norm(err_r) < 1e-3:
            break

        J = pin.computeFrameJacobian(model, data, q, tip_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        idx_v = [model.joints[jid].idx_v for jid in joint_ids]
        J_arm = J[:, idx_v]

        # 零空间自运动拉向关节中位
        q_arm = np.array([q[model.joints[jid].idx_q] for jid in joint_ids])
        q_mid = (q_min + q_max) / 2.0
        q_range = np.maximum(0.1, q_max - q_min)
        grad_H = 2.0 * (q_arm - q_mid) / (q_range**2)

        JJt = J_arm @ J_arm.T
        J_pinv = J_arm.T @ np.linalg.inv(JJt + 1e-4 * np.eye(6))
        dq = J_pinv @ err + (np.eye(len(arm_joints)) - J_pinv @ J_arm) @ (-0.05 * grad_H)

        max_step = 0.05
        sn = np.max(np.abs(dq))
        if sn > max_step:
            dq = dq * (max_step / sn)

        for i, jid in enumerate(joint_ids):
            q[model.joints[jid].idx_q] += dq[i]
            # 严格保留安全缓冲（15 mrad），绝不贴死极限
            q[model.joints[jid].idx_q] = np.clip(q[model.joints[jid].idx_q], q_min[i] + 0.015, q_max[i] - 0.015)

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    q_out = {n: float(q[model.joints[model.getJointId(n)].idx_q]) for n in arm_joints}
    return q_out, data.oMf[tip_id].translation.copy(), target_quat.copy()

def print_and_save_stats(records, output_csv, plane_name, dist_sym, circle_d, square_side):
    if len(records) < 5:
        print("采样点不足，跳过统计分析。")
        return

    # records: [(t, p_dL, p_planL, p_actL, p_dR, p_planR, p_actR), ...]
    t = np.array([r[0] for r in records], dtype=float)
    p_dL = np.array([r[1] for r in records], dtype=float)
    p_planL = np.array([r[2] for r in records], dtype=float)
    p_actL = np.array([r[3] for r in records], dtype=float)
    p_dR = np.array([r[4] for r in records], dtype=float)
    p_planR = np.array([r[5] for r in records], dtype=float)
    p_actR = np.array([r[6] for r in records], dtype=float)

    def calc_metrics(e_vec):
        norm_mm = np.linalg.norm(e_vec, axis=1) * 1000.0
        rmse = float(np.sqrt(np.mean(norm_mm**2)))
        max_err = float(np.max(norm_mm))
        mean_err = float(np.mean(norm_mm))
        std_err = float(np.std(norm_mm))
        return rmse, max_err, mean_err, std_err

    # 左臂 (Circle)
    e_plan_L = p_planL - p_dL
    e_ctrl_L = p_actL - p_planL
    e_total_L = p_actL - p_dL

    rmse_pL, max_pL, mean_pL, std_pL = calc_metrics(e_plan_L)
    rmse_cL, max_cL, mean_cL, std_cL = calc_metrics(e_ctrl_L)
    rmse_tL, max_tL, mean_tL, std_tL = calc_metrics(e_total_L)

    # 右臂 (Square)
    e_plan_R = p_planR - p_dR
    e_ctrl_R = p_actR - p_planR
    e_total_R = p_actR - p_dR

    rmse_pR, max_pR, mean_pR, std_pR = calc_metrics(e_plan_R)
    rmse_cR, max_cR, mean_cR, std_cR = calc_metrics(e_ctrl_R)
    rmse_tR, max_tR, mean_tR, std_tR = calc_metrics(e_total_R)

    print("\n" + "=" * 88)
    print("📊 终端轨迹全链路误差统计报告 (End-Effector Trajectory Error Analysis)")
    print("=" * 88)
    print(f"  • 平面构型: 垂直于对称面 (X=0) 的 {plane_name}")
    print(f"  • 双臂对称距: 左中点 X = +{dist_sym:.3f} m, 右中点 X = -{dist_sym:.3f} m (间距 = {2*dist_sym:.3f} m)")
    print(f"  • 采样规模: {len(records)} 帧，持续时长 {t[-1] - t[0]:.2f} s")
    print("-" * 88)
    print("  【数学解耦关系验证】")
    print("      e_total (端到端总误差) = e_plan (规划滤波滞后) + e_ctrl (控制执行误差)")
    print("      - e_plan = p_plan - p_d   : 反映滤波低通截止延迟、插值平滑与运动学可行性")
    print("      - e_ctrl = p_act - p_plan : 反映阻抗刚度/阻尼响应、重力摩擦补偿残差与伺服带宽")
    print("      - 空间分量上: e_total 严格等于 e_plan + e_ctrl; 模长满足三角不等式: ||e_total|| <= ||e_plan|| + ||e_ctrl||")
    print("-" * 88)
    print(f"【左臂 —— 圆形轨迹 (Circle d={circle_d*100:.0f}cm)】")
    print(f"  1. 规划滞后误差 (e_plan) : RMSE = {rmse_pL:6.2f} mm | Max = {max_pL:6.2f} mm | Mean = {mean_pL:6.2f} mm (±{std_pL:5.2f})")
    print(f"  2. 运控执行误差 (e_ctrl) : RMSE = {rmse_cL:6.2f} mm | Max = {max_cL:6.2f} mm | Mean = {mean_cL:6.2f} mm (±{std_cL:5.2f})")
    print(f"  3. 终端总跟踪差 (e_total): RMSE = {rmse_tL:6.2f} mm | Max = {max_tL:6.2f} mm | Mean = {mean_tL:6.2f} mm (±{std_tL:5.2f})")
    print("-" * 88)
    print(f"【右臂 —— 正方形轨迹 (Square a={square_side*100:.0f}cm)】")
    print(f"  1. 规划滞后误差 (e_plan) : RMSE = {rmse_pR:6.2f} mm | Max = {max_pR:6.2f} mm | Mean = {mean_pR:6.2f} mm (±{std_pR:5.2f})")
    print(f"  2. 运控执行误差 (e_ctrl) : RMSE = {rmse_cR:6.2f} mm | Max = {max_cR:6.2f} mm | Mean = {mean_cR:6.2f} mm (±{std_cR:5.2f})")
    print(f"  3. 终端总跟踪差 (e_total): RMSE = {rmse_tR:6.2f} mm | Max = {max_tR:6.2f} mm | Mean = {mean_tR:6.2f} mm (±{std_tR:5.2f})")
    print("=" * 88)

    # 导出 CSV
    try:
        os.makedirs(os.path.dirname(os.path.abspath(output_csv)), exist_ok=True)
        with open(output_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'time_s',
                'L_des_x', 'L_des_y', 'L_des_z',
                'L_plan_x', 'L_plan_y', 'L_plan_z',
                'L_act_x', 'L_act_y', 'L_act_z',
                'R_des_x', 'R_des_y', 'R_des_z',
                'R_plan_x', 'R_plan_y', 'R_plan_z',
                'R_act_x', 'R_act_y', 'R_act_z'
            ])
            for i in range(len(records)):
                row = [f"{t[i]:.4f}"]
                for p in [p_dL[i], p_planL[i], p_actL[i], p_dR[i], p_planR[i], p_actR[i]]:
                    row.extend([f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}"])
                writer.writerow(row)
        print(f"💾 详细三级轨迹点云已保存至: {output_csv}\n")
    except Exception as ex:
        print(f"⚠️ 保存 CSV 统计数据失败: {ex}")

def main():
    ws_default = _ws_root
    ws = os.environ.get('WS', ws_default)

    # 本地 tmp 路径安全回退
    candidate_urdfs = [
        os.path.join(ws, 'src/tmp/dyid_runs/run_20261019_234605/robot.urdf'),
        os.path.join(ws, 'tmp/dyid_runs/run_20261019_234605/robot.urdf'),
        os.path.join(ws, 'src/Lite_IK_node/config/model/lite_000_asm.urdf'),
        os.path.join(ws, 'install/lite_ik_node/share/lite_ik_node/config/model/lite_000_asm.urdf'),
        os.path.join(ws, 'src/lite_ik_node/config/model/lite_000_asm.urdf'),
        os.path.join(ws, 'src/lite_motor_controller/lite_description/urdf/lite_000_asm.urdf'),
    ]
    default_urdf = next((u for u in candidate_urdfs if os.path.exists(u)), candidate_urdfs[0])

    ap = argparse.ArgumentParser(description="左圆右方高精度轨迹测试与误差解耦统计一揽子驱动")
    ap.add_argument('--mode', type=str, default='regular', choices=['regular', 'admittance', 'impedance'],
                    help='运控层控制律模式：regular / admittance / impedance (默认 regular)')
    ap.add_argument('--joint-mode', type=str, default='arms', choices=['arms', 'arms_grippers'],
                    help='关节模式：arms (14) 或 arms_grippers (16) (默认 arms)')
    ap.add_argument('--no-launch', action='store_true',
                    help='不自动拉起 move_controller launch（若已在另一终端手动启动）')
    ap.add_argument('--auto-enable', type=lambda s: s.lower() in ('1', 'true', 'yes', 'on'), default=True,
                    help='自动调用 enable 服务激活控制器 (默认 True)')
    ap.add_argument('--prepose-sec', type=float, default=8.0, help='关节预摆位时间 (s)')
    ap.add_argument('--duration', type=float, default=200.0, help='形状测试运行时间 (s)')
    ap.add_argument('--speed', type=float, default=0.05, help='末端运行线速度 (m/s)')
    ap.add_argument('--circle-d', type=float, default=0.12, help='左臂圆直径 (m)')
    ap.add_argument('--square-side', type=float, default=0.08, help='右臂正方形边长 (m)')
    ap.add_argument('--corner-r', type=float, default=0.015, help='右臂正方形圆角半径 (m)')
    ap.add_argument('--dist-sym', type=float, default=0.20, help='图形中点距离机器人对称面 (X=0) 的距离 (m)')
    ap.add_argument('--center-y', type=float, default=-0.30, help='图形中点 Y 坐标 (m)')
    ap.add_argument('--center-z', type=float, default=0.60, help='图形中点 Z 坐标 (m)')
    ap.add_argument('--plane', type=str, default='vertical', choices=['vertical', 'horizontal'],
                    help='绘图平面：vertical (垂直对称面的竖直面 X-Z) 或 horizontal (垂直对称面的水平面 X-Y)')
    ap.add_argument('--blend-sec', type=float, default=2.0, help='预摆位末端到形状起点过渡时间 (s)')
    ap.add_argument('--rate', type=float, default=100.0, help='下发控制频率 (Hz)')
    ap.add_argument('--urdf', type=str, default=default_urdf, help='用于运动学计算的本地 URDF 路径')
    ap.add_argument('--target-topic', type=str, default='/lite_move_controller/task_target',
                    help='目标发布话题 (直接给运控层 或 给 lite_ik_node)')
    ap.add_argument('--stats-file', type=str,
                    default=os.path.join(ws, 'tmp/circle_square_tracking_stats.csv'),
                    help='误差统计输出 CSV 文件路径')
    a = ap.parse_args()

    print(f"加载机器人 URDF: {a.urdf}")
    if not os.path.exists(a.urdf):
        print(f"❌ 错误: URDF 文件不存在: {a.urdf}")
        sys.exit(1)

    model = pin.buildModelFromUrdf(a.urdf)
    data = model.createData()
    tip_l = model.getFrameId('left_gripper_tip_middle_link')
    tip_r = model.getFrameId('right_gripper_tip_middle_link')

    def calc_fk(vals, tip_id):
        q = pin.neutral(model)
        for n, v in vals.items():
            if n in vals and model.existJointName(n):
                q[model.joints[model.getJointId(n)].idx_v] = v
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        M = data.oMf[tip_id]
        return np.array(M.translation), np.array(pin.Quaternion(M.rotation).coeffs())

    rclpy.init()
    node = Node('circle_square_driver')

    st_meas = {}
    st_cmd = {}

    node.create_subscription(JointState, '/joint_states',
                             lambda m: st_meas.update({n: p for n, p in zip(m.name, m.position)}), 10)
    node.create_subscription(MotionCommand, '/lite_bridge/motion_command',
                             lambda m: st_cmd.update({n: p for n, p in zip(m.joint_names, m.position)}), 10)
    pub = node.create_publisher(MoveTargetArray, a.target_topic, 10)

    print("等待 /joint_states 14 个关节就绪 (确保 lite_bringup 已启动)...")
    t0 = time.time()
    while time.time() - t0 < 15.0 and len(st_meas) < 14:
        rclpy.spin_once(node, timeout_sec=0.1)
    if len(st_meas) < 14:
        print("❌ 错误: 未能在 15 秒内收到 /joint_states 14 关节反馈。请确认 lite_bringup 是否正常运行并进入 MIT 模式！")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    print(f"✓ /joint_states 已就绪 ({len(st_meas)} 关节)")

    # 客户端建立
    status_cli = node.create_client(GetMoveStatus, '/lite_move_controller/get_status')
    enable_cli = node.create_client(Trigger, '/lite_move_controller/enable')
    disable_cli = node.create_client(Trigger, '/lite_move_controller/disable')
    clear_fault_cli = node.create_client(Trigger, '/lite_move_controller/clear_fault')

    # 一揽子自动拉起：如果尚未启动 move_controller，由脚本直接拉起
    if not a.no_launch:
        if not status_cli.service_is_ready():
            _launch_move_controller(ws, mode=a.mode, joint_mode=a.joint_mode)
        else:
            print("✓ 检测到已在运行的 lite_move_controller 服务，复用该实例。")

    # 安全与控制器状态监视。
    # /safety_status 有 CAN0/CAN1 等多个发布者，绝不能用单个 level 保存最后一条
    # 消息：健康总线的 OK 会覆盖故障总线的 FAULT。按 source 分开记账，并把运行期
    # 首个 FAULT 锁存到退出；桥同样会锁住 FAULT，禁止轨迹脚本自行清故障后续跑。
    safety_by_source = {}
    fatal_safety = {'event': None}
    latest_move_status = {'msg': None, 'recv_at': 0.0}

    def on_safety(m):
        source = m.source or '<unknown>'
        event = (source, int(m.level), int(m.flags), m.message)
        safety_by_source[source] = event
        if m.level >= SafetyStatus.FAULT and fatal_safety['event'] is None:
            fatal_safety['event'] = event

    def on_move_status(m):
        latest_move_status['msg'] = m
        latest_move_status['recv_at'] = time.monotonic()

    safety_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(SafetyStatus, '/safety_status', on_safety, safety_qos)
    node.create_subscription(MoveStatus, '/lite_move_controller/status', on_move_status, 10)

    def max_safety_level():
        return max((event[1] for event in safety_by_source.values()), default=0)

    def format_safety_event(event):
        source, level, flags, message = event
        detail = (f"source={source}, level={level}, flags=0x{flags:X} "
                  f"[{safety_flags_text(flags)}]")
        if message:
            detail += f", message={message}"
        if flags & (SafetyStatus.FLAG_RX_TIMEOUT | SafetyStatus.FLAG_MOTOR_FAULT):
            detail += '；具体关节见 lite_driver 同时刻日志'
        return detail

    def assert_runtime_healthy(stage):
        event = fatal_safety['event']
        if event is not None:
            raise RuntimeSafetyAbort(f'{stage}检测到硬件安全故障：{format_safety_event(event)}')

        s = latest_move_status['msg']
        if s is None:
            return
        status_age = time.monotonic() - latest_move_status['recv_at']
        if status_age > 1.0:
            raise RuntimeSafetyAbort(
                f'{stage}的运控状态已 {status_age:.2f} s 未更新，控制器可能已退出')
        healthy = (s.state_name in ('RUNNING', 'DWELL') and s.bridge_authorized
                   and s.bridge_mode_name == 'TRACKING'
                   and s.bridge_arbitration_state != 'FAULT')
        if not healthy:
            raise RuntimeSafetyAbort(
                f'{stage}失去控制权：state={s.state_name} ({s.state_detail}), '
                f'bridge={s.bridge_mode_name}, auth={s.bridge_authorized}, '
                f'arb={s.bridge_arbitration_state}')

    # 等待服务就绪
    print("等待 lite_move_controller 节点与服务上线...")
    t0 = time.time()
    while time.time() - t0 < 30.0:
        if status_cli.service_is_ready() and enable_cli.service_is_ready():
            break
        if _launch_proc is not None and _launch_proc.poll() is not None:
            print(f"❌ lite_move_controller launch 异常退出 (exit code: {_launch_proc.poll()})")
            log_path = os.path.join(ws, 'tmp/move_controller_launch.log')
            if os.path.exists(log_path):
                print("--- 启动日志末尾 ---")
                with open(log_path, 'r') as lf:
                    print(''.join(lf.readlines()[-30:]))
            node.destroy_node(); rclpy.shutdown(); sys.exit(1)
        time.sleep(0.5)
    else:
        print("❌ 30 s 内 lite_move_controller 服务未上线，退出！")
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    def get_status_msg():
        if not status_cli.service_is_ready():
            return None
        fut = status_cli.call_async(GetMoveStatus.Request())
        rclpy.spin_until_future_complete(node, fut, timeout_sec=1.5)
        res = fut.result()
        if res is not None:
            latest_move_status['msg'] = res.status
            latest_move_status['recv_at'] = time.monotonic()
            return res.status
        return None

    def ready_state():
        s = get_status_msg()
        if s is None:
            return False, 'get_status 服务未就绪或超时'
        ok = (s.state_name in ('RUNNING', 'DWELL') and s.bridge_authorized
              and s.bridge_mode_name == 'TRACKING'
              and s.bridge_arbitration_state != 'FAULT'
              and fatal_safety['event'] is None
              and max_safety_level() < SafetyStatus.FAULT)
        return ok, ('%s/%s auth=%s arb=%s safety=%d' %
                    (s.state_name, s.bridge_mode_name, s.bridge_authorized,
                     s.bridge_arbitration_state, max_safety_level()))

    print("等待 lite_move_controller 完成初始化并进入 STANDBY...")
    t0 = time.time()
    while time.time() - t0 < 30.0:
        st = get_status_msg()
        if st is not None and st.state_name in ('STANDBY', 'RUNNING', 'DWELL', 'FAULT'):
            print(f"✓ 运控节点已就绪: state={st.state_name}")
            break
        if _launch_proc is not None and _launch_proc.poll() is not None:
            print(f"❌ lite_move_controller launch 异常退出 (exit code: {_launch_proc.poll()})")
            sys.exit(1)
        time.sleep(0.5)
    else:
        print("❌ 30 s 内 lite_move_controller 未进入就绪状态，退出！")
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    print("激活并等待控制器进入 TRACKING (授权就绪)...")
    t0 = time.time()
    last_enable_call = 0.0
    while time.time() - t0 < 35.0:
        ok, why = ready_state()
        if ok:
            print(f"✓ 控制器就绪: {why}")
            break
        if fatal_safety['event'] is not None:
            print(f"❌ 启动阶段检测到硬件安全故障："
                  f"{format_safety_event(fatal_safety['event'])}")
            node.destroy_node(); rclpy.shutdown(); sys.exit(3)

        st = latest_move_status['msg']
        curr_state = st.state_name if st else ''
        if curr_state == 'FAULT':
            print(f"控制器处于 FAULT ({st.state_detail})，调用 /lite_move_controller/clear_fault 清除故障...")
            cfut = clear_fault_cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, cfut, timeout_sec=2.0)
            time.sleep(0.5)
        elif curr_state == 'STANDBY' and a.auto_enable:
            now_sec = time.time()
            if now_sec - last_enable_call >= 2.0:
                last_enable_call = now_sec
                print("控制器处于 STANDBY，正在调用 /lite_move_controller/enable 激活...")
                efut = enable_cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, efut, timeout_sec=2.0)
                eres = efut.result()
                if eres is not None:
                    print(f"  ✓ enable 请求结果: {eres.message}")

        print(f"等待控制器进入 TRACKING: {why}")
        time.sleep(1.0)
    else:
        print("❌ 35 s 内未进入 TRACKING/授权，拒绝下发！")
        node.destroy_node(); rclpy.shutdown(); sys.exit(2)

    # ---------------- 轨迹中心点与起始位姿解算 ----------------
    r = 0.5 * a.circle_d
    h = 0.5 * a.square_side

    # 左侧中心 (X = +dist_sym), 右侧中心 (X = -dist_sym)
    cL = np.array([a.dist_sym, a.center_y, a.center_z])
    cR = np.array([-a.dist_sym, a.center_y, a.center_z])

    # 根据选定平面计算形状轨迹起点
    # 正方形起点为圆角起点 (h, -h + corner_r)
    sx0, sz0 = filleted_square_point(a.square_side, a.corner_r, 0.0)
    if a.plane == 'vertical':
        # 竖直前向平面 (X-Z 平面，Y 保持常数)
        # 形状在 X 和 Z 方向变化
        left_start = cL + np.array([r, 0.0, 0.0])
        right_start = cR + np.array([sx0, 0.0, sz0])
    else:
        # 水平桌面平面 (X-Y 平面，Z 保持常数)
        # 形状在 X 和 Y 方向变化
        left_start = cL + np.array([r, 0.0, 0.0])
        right_start = cR + np.array([sx0, sz0, 0.0])

    print("\n--- 轨迹几何配置 ---")
    print(f"平面类型: {'垂直于对称面的竖直面 (X-Z)' if a.plane=='vertical' else '垂直于对称面的水平面 (X-Y)'}")
    print(f"终端姿态: 严格水平向前 (Quaternion: {HORIZONTAL_FORWARD_QUAT})")
    print(f"左圆中心: {np.round(cL, 3)} (直径 {a.circle_d:.2f} m)")
    print(f"右方中心: {np.round(cR, 3)} (边长 {a.square_side:.2f} m, 圆角 {a.corner_r:.3f} m)")
    print(f"形状起点: L={np.round(left_start, 3)}, R={np.round(right_start, 3)}")

    # 预摆位关节角求解（自动对准形状起点与严格水平向前姿态）
    qL_seed = {n: st_meas.get(n, 0.0) for n in LEFT}
    qR_seed = {n: st_meas.get(n, 0.0) for n in RIGHT}
    qL_seed.update({'left_shoulder_pitch': 0.4, 'left_shoulder_roll': -0.7, 'left_shoulder_yaw': 0.0, 'left_elbow_pitch': -1.6})
    qR_seed.update({'right_shoulder_pitch': 0.4, 'right_shoulder_roll': 0.8, 'right_shoulder_yaw': 0.0, 'right_elbow_pitch': 1.6})

    qL_pre, pL_real, _ = solve_ik_6d_bounded(model, data, LEFT, tip_l, left_start, HORIZONTAL_FORWARD_QUAT, qL_seed)
    qR_pre, pR_real, _ = solve_ik_6d_bounded(model, data, RIGHT, tip_r, right_start, HORIZONTAL_FORWARD_QUAT, qR_seed)

    print(f"预摆位解算达成: L_err={np.linalg.norm(left_start-pL_real)*1000:.2f} mm, R_err={np.linalg.norm(right_start-pR_real)*1000:.2f} mm")

    seq = 0
    def joint_targets_goal():
        nonlocal seq
        arr = MoveTargetArray(); arr.targets = []
        for arm, q_target in (('left', qL_pre), ('right', qR_pre)):
            m = MoveTarget()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = 'world_root'
            m.arm = arm
            m.frame_id = 'world_root'
            m.target_type = MoveTarget.TARGET_JOINTS
            joints = LEFT if arm == 'left' else RIGHT
            m.joint_positions = [float(q_target[n]) for n in joints]
            m.params_valid = False
            seq += 1
            m.sequence = seq
            arr.targets.append(m)
        return arr

    def pose_targets(t, blend=0.0):
        nonlocal seq
        arr = MoveTargetArray(); arr.targets = []
        ang = a.speed * t / r
        sx, sz = filleted_square_point(a.square_side, a.corner_r, a.speed * t)

        if a.plane == 'vertical':
            # X-Z 竖直平面
            pl = cL + np.array([r * math.cos(ang), 0.0, r * math.sin(ang)])
            pr = cR + np.array([sx, 0.0, sz])
        else:
            # X-Y 水平面
            pl = cL + np.array([r * math.cos(ang), r * math.sin(ang), 0.0])
            pr = cR + np.array([sx, sz, 0.0])

        if blend > 0.0:
            alpha = min(1.0, max(0.0, 1.0 - blend / a.blend_sec))
            s_smooth = 10.0 * (alpha**3) - 15.0 * (alpha**4) + 6.0 * (alpha**5)
            pl = left_start + s_smooth * (pl - left_start)
            pr = right_start + s_smooth * (pr - right_start)

        for arm, p in (('left', pl), ('right', pr)):
            m = MoveTarget()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = 'world_root'
            m.arm = arm
            m.frame_id = 'world_root'
            m.target_type = MoveTarget.TARGET_POSE
            m.position.x = float(p[0])
            m.position.y = float(p[1])
            m.position.z = float(p[2])
            m.orientation.x = float(HORIZONTAL_FORWARD_QUAT[0])
            m.orientation.y = float(HORIZONTAL_FORWARD_QUAT[1])
            m.orientation.z = float(HORIZONTAL_FORWARD_QUAT[2])
            m.orientation.w = float(HORIZONTAL_FORWARD_QUAT[3])
            m.params_valid = False
            seq += 1
            m.sequence = seq
            arr.targets.append(m)
        return arr, pl, pr

    period = 1.0 / a.rate
    records = []
    exit_code = 0

    try:
        # [阶段 1/2] 关节预摆位
        print(f"\n[1/2] 预摆位 {a.prepose_sec:.1f} s（运控层平滑规划就位）...", flush=True)
        start_time = time.monotonic()
        last_print = -1.0
        while True:
            t = time.monotonic() - start_time
            if t >= a.prepose_sec:
                break
            pub.publish(joint_targets_goal())
            rclpy.spin_once(node, timeout_sec=0.0)
            assert_runtime_healthy('预摆位阶段')
            if t - last_print >= 1.0:
                last_print = t
                err_L = max(abs(st_meas.get(n, 0.0) - qL_pre[n]) for n in LEFT)
                err_R = max(abs(st_meas.get(n, 0.0) - qR_pre[n]) for n in RIGHT)
                print(f"  预摆位进度: t={t:4.1f}/{a.prepose_sec:.1f}s | 剩余最大关节差: 左 {err_L:.3f} rad, 右 {err_R:.3f} rad", flush=True)
            time.sleep(0.05)

        # [阶段 2/2] 位姿形状跟踪与三级轨迹采集
        print(f"\n[2/2] 绘制形状 {a.duration:.0f} s（位姿模式，速度 {a.speed} m/s）...", flush=True)
        start_time = time.monotonic()
        while True:
            t = time.monotonic() - start_time
            blend_left = max(0.0, a.blend_sec - t)
            tt = max(0.0, t - a.blend_sec)
            tt = min(tt, a.duration) if a.duration > 0 else tt

            msg, pl_des, pr_des = pose_targets(tt, blend=blend_left)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            assert_runtime_healthy('形状跟踪阶段')

            # 计算三级末端轨迹（期望、规划、实测）
            # 1. 期望: pl_des, pr_des
            # 2. 规划: 来自 /lite_bridge/motion_command 的 FK (若未收到则暂用实测/期望填充)
            q_plan_L = {n: st_cmd.get(n, st_meas.get(n, 0.0)) for n in LEFT}
            q_plan_R = {n: st_cmd.get(n, st_meas.get(n, 0.0)) for n in RIGHT}
            pl_plan, _ = calc_fk(q_plan_L, tip_l)
            pr_plan, _ = calc_fk(q_plan_R, tip_r)

            # 3. 跟踪实测: 来自 /joint_states 的 FK
            q_act_L = {n: st_meas.get(n, 0.0) for n in LEFT}
            q_act_R = {n: st_meas.get(n, 0.0) for n in RIGHT}
            pl_act, _ = calc_fk(q_act_L, tip_l)
            pr_act, _ = calc_fk(q_act_R, tip_r)

            records.append((t, pl_des, pl_plan, pl_act, pr_des, pr_plan, pr_act))

            if int(t) % 10 == 0 and abs(t - round(t)) < period / 2:
                err_L = np.linalg.norm(pl_act - pl_des) * 1000.0
                err_R = np.linalg.norm(pr_act - pr_des) * 1000.0
                print(f"  t={t:5.1f} s | 跟踪总差: 左(圆) {err_L:4.1f} mm, 右(方) {err_R:4.1f} mm", flush=True)

            if a.duration > 0 and t >= a.duration + 1.0:
                break
            time.sleep(period)

    except RuntimeSafetyAbort as ex:
        exit_code = 3
        print(f"\n❌ {ex}", flush=True)
        print("已停止下发；不会自动清除 FAULT 或重新 enable。请先排除硬件/CAN 故障，再重新运行。",
              flush=True)
    except KeyboardInterrupt:
        exit_code = 130
        print("\n捕获中断信号，正在收尾退出...")
    finally:
        # 退出收尾：调用 disable 释放控制权并清理后台进程
        try:
            if 'disable_cli' in locals() and disable_cli.service_is_ready():
                print("\n🛑 退出收尾：调用 /lite_move_controller/disable 释放控制权...")
                dis_fut = disable_cli.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, dis_fut, timeout_sec=2.0)
        except Exception:
            pass

        # 输出统计结果与报表
        plane_str = "竖直面 (X-Z)" if a.plane == 'vertical' else "水平面 (X-Y)"
        print_and_save_stats(
            records, a.stats_file, plane_str, a.dist_sym, a.circle_d, a.square_side)
        node.destroy_node()
        rclpy.shutdown()
        _cleanup_launch()
    return exit_code

if __name__ == '__main__':
    sys.exit(main())
