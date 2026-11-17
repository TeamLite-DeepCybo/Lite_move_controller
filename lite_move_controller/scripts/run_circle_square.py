#!/usr/bin/python3
"""左圆右方：仅通过 lite_move_controller 的公开 ROS 接口操作。

不启动/停止其它节点，不访问电机层，不加载 URDF 或内部运动学库。
先用 ik_query 预查轨迹，再 enable，发送关节预摆位及末端位姿流，最后 disable。
反馈仅使用公开 MoveStatus；CSV 保留原列名，接口未提供的末端反馈留空。
"""
import argparse
import csv
import math
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from move_msgs.msg import MoveStatus, MoveTarget, MoveTargetArray
from move_msgs.srv import GetMoveStatus, QueryIk
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

LEFT = ['left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
        'left_elbow_pitch', 'left_wrist_yaw', 'left_wrist_roll', 'left_wrist_pitch']
RIGHT = [name.replace('left_', 'right_') for name in LEFT]
QUAT = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
# 2026-10-29 垂直平面示例记录的起点构型，仅作公开 ik_query 的种子。
PREPOSE_SEEDS = (
    (0.409429574, -0.948332888, 0.231273395, -1.610984789, 0.763355715, -0.210960238, -0.255422942),
    (0.410166818, 1.186376376, 0.032087510, 1.770029188, -0.290830379, -0.227434094, -0.050001317),
)


def filleted_square_point(side, r_corner, arc):
    h = side / 2.0
    rc = min(r_corner, h * 0.49)
    straight = side - 2.0 * rc
    segment = straight + 0.5 * math.pi * rc
    s = arc % (4.0 * segment)
    i = int(s // segment) % 4
    u = s - i * segment
    if u < straight:
        return ((h, -h + rc + u), (h - rc - u, h),
                (-h, h - rc - u), (-h + rc + u, -h))[i]
    angle = i * math.pi / 2.0 + (u - straight) / rc
    cx, cz = ((h-rc, h-rc), (-h+rc, h-rc), (-h+rc, -h+rc), (h-rc, -h+rc))[i]
    return cx + rc * math.cos(angle), cz + rc * math.sin(angle)


def shape_points(a, elapsed):
    radius = a.circle_d / 2.0
    angle = elapsed * a.speed / radius
    sx, sz = filleted_square_point(a.square_side, a.corner_r, elapsed * a.speed)
    offsets = ((radius * math.cos(angle), radius * math.sin(angle)), (sx, sz))
    points = []
    for center_x, (x, z) in zip((a.dist_sym, -a.dist_sym), offsets):
        points.append((center_x + x, a.center_y, a.center_z + z)
                      if a.plane == 'vertical' else (center_x + x, a.center_y + z, a.center_z))
    return points


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--prepose-sec', type=float, default=30.0)
    ap.add_argument('--prepose-speed', type=float, default=0.10, help='预摆位最大关节速度 rad/s')
    ap.add_argument('--prepose-accel', type=float, default=0.15, help='预摆位最大关节加速度 rad/s²')
    ap.add_argument('--duration', type=float, default=200.0, help='0 表示持续运行，Ctrl+C 停止')
    ap.add_argument('--speed', type=float, default=0.01)
    ap.add_argument('--circle-d', type=float, default=0.12)
    ap.add_argument('--square-side', type=float, default=0.08)
    ap.add_argument('--corner-r', type=float, default=0.015)
    ap.add_argument('--dist-sym', type=float, default=0.20)
    ap.add_argument('--center-y', type=float, default=-0.30)
    ap.add_argument('--center-z', type=float, default=0.60)
    ap.add_argument('--plane', choices=['vertical', 'horizontal'], default='vertical')
    ap.add_argument('--blend-sec', type=float, default=2.0)
    ap.add_argument('--rate', type=float, default=100.0, help='上游目标发送频率；IK/控制频率由控制器配置')
    ap.add_argument('--target-topic', default='/lite_move_controller/task_target')
    ap.add_argument('--stats-file', default='circle_square_tracking_stats.csv')
    ap.add_argument('--urdf', help='兼容旧命令行；外部示例不再读取模型文件')
    ap.add_argument('--enable-gripper', dest='enable_gripper', action='store_true', default=True)
    ap.add_argument('--no-gripper', '--disable-gripper', dest='enable_gripper', action='store_false')
    a = ap.parse_args()
    for key in ('speed', 'circle_d', 'square_side', 'corner_r', 'prepose_sec', 'prepose_speed',
                'prepose_accel', 'rate', 'blend_sec'):
        if not math.isfinite(getattr(a, key)) or getattr(a, key) <= 0:
            ap.error(key + ' 必须为正且有限')
    for key in ('duration', 'dist_sym', 'center_y', 'center_z'):
        if not math.isfinite(getattr(a, key)):
            ap.error(key + ' 必须有限')
    if a.duration < 0:
        ap.error('duration 必须非负')
    if a.urdf:
        print('--urdf 已不需要：示例仅调用公开接口，不读取该文件。', flush=True)

    rclpy.init()
    node = Node('circle_square_driver')
    latest = {'status': None, 'recv': 0.0}

    def on_status(msg):
        latest.update(status=msg, recv=time.monotonic())

    node.create_subscription(MoveStatus, '/lite_move_controller/status', on_status, 10)
    pub = node.create_publisher(MoveTargetArray, a.target_topic, 10)
    grip = node.create_publisher(JointState, '/gripper/command', 10)
    status_cli = node.create_client(GetMoveStatus, '/lite_move_controller/get_status')
    ik_cli = node.create_client(QueryIk, '/lite_move_controller/ik_query')
    enable_cli = node.create_client(Trigger, '/lite_move_controller/enable')
    disable_cli = node.create_client(Trigger, '/lite_move_controller/disable')
    enabled = False
    output = None
    exit_code = 0

    def call(client, request, timeout=10.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(client.srv_name + ' 不可用')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(client.srv_name + ' 超时')
        return future.result()

    def read_status():
        response = call(status_cli, GetMoveStatus.Request())
        if not response.ok:
            raise RuntimeError(response.message)
        on_status(response.status)
        return response.status

    def healthy():
        s = latest['status']
        if s is None or time.monotonic() - latest['recv'] > 1.0:
            raise RuntimeError('move_controller 状态失鲜')
        if (s.state not in (MoveStatus.STATE_RUNNING, MoveStatus.STATE_DWELL)
                or not s.bridge_authorized or s.bridge_mode != 2
                or s.bridge_arbitration_state == 'FAULT'):
            raise RuntimeError(f'停止发目标：{s.state_name}，{s.state_detail}，桥={s.bridge_mode_name}')
        return s

    def target(arm, position=None, joints=None):
        m = MoveTarget()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = m.frame_id = 'world_root'
        m.arm = arm
        m.timeout_ms = 200.0
        if joints is not None:
            m.target_type = MoveTarget.TARGET_JOINTS
            m.joint_positions = list(joints)
        else:
            m.target_type = MoveTarget.TARGET_POSE
            m.position.x, m.position.y, m.position.z = position
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = QUAT
        return m

    def query(arm, point, seed):
        # 预查无需抢占控制节拍；为每次独立查询留出求解器预算恢复时间。
        time.sleep(0.02)
        req = QueryIk.Request()
        req.arm, req.frame_id, req.want_joints = arm, 'world_root', True
        req.position.x, req.position.y, req.position.z = point
        req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = QUAT
        req.seed_joints = list(seed)
        result = call(ik_cli, req)
        if (not result.ok or result.ik_status_name not in ('SOLVED', 'LIMIT_CLAMPED') or len(result.joints) != 7
                or not all(math.isfinite(x) for x in result.joints)
                or not math.isfinite(result.position_residual_norm)
                or not math.isfinite(result.orientation_residual_norm)
                or result.position_residual_norm > 0.01 or result.orientation_residual_norm > 0.10):
            raise RuntimeError(f'{arm} 轨迹预查失败：{result.message}; '
                               f'位置残差={result.position_residual_norm:.4f}m')
        return list(result.joints)

    def interrupted(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        s = read_status()
        if s.control_mode == 'gravity_compensation':
            raise RuntimeError('当前为纯重力补偿，拒绝运动目标；请用跟踪模式启动运控')
        if (s.state != MoveStatus.STATE_STANDBY or not s.bridge_authorized or s.bridge_mode != 2):
            raise RuntimeError('请从 STANDBY 启动示例；不接管正在运行的其它任务')
        measured = dict(zip(s.joint_names, s.q_measured))
        if not all(n in measured and math.isfinite(measured[n]) for n in LEFT + RIGHT):
            raise RuntimeError('公开状态尚未包含完整双臂实测关节')
        starts = shape_points(a, 0.0)
        # 沿用已有垂直示例的完整构型，避免从收纳姿态选到另一逆解分支。
        seeds = PREPOSE_SEEDS
        prepose = [query(arm, point, seed)
                   for arm, point, seed in zip(('left', 'right'), starts, seeds)]
        # 采样预查只在 enable 前进行，避免查询求解阻塞实时运动。
        for i, arm in enumerate(('left', 'right')):
            seed = prepose[i]
            period = (math.pi*a.circle_d if i == 0 else
                      4*(a.square_side-2*min(a.corner_r, a.square_side*0.245)
                         + math.pi/2*min(a.corner_r, a.square_side*0.245))) / a.speed
            for k in range(1, 65):
                result = query(arm, shape_points(a, k*period/64)[i], seed)
                jump = max(abs(x-y) for x,y in zip(result, seed))
                if jump > 0.35:
                    raise RuntimeError(f'{arm} 第 {k}/64 个采样逆解跳变 {jump:.3f}rad，拒绝启动')
                seed = result
        print('公开 ik_query 轨迹采样预查通过，开始 enable。', flush=True)
        enabled = True  # 请求即可能生效；异常退出也必须请求 disable。
        response = call(enable_cli, Trigger.Request())
        if not response.success:
            raise RuntimeError(response.message)
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            s = read_status()
            if s.state in (MoveStatus.STATE_RUNNING, MoveStatus.STATE_DWELL):
                healthy()
                break
            if s.state in (MoveStatus.STATE_FAULT, MoveStatus.STATE_ERROR):
                raise RuntimeError(s.state_detail)
            time.sleep(0.1)
        else:
            raise RuntimeError('运控 30s 内未完成接管')

        initial = [[dict(zip(s.joint_names, s.q_measured))[n] for n in names] for names in (LEFT, RIGHT)]
        delta = max(abs(y-x) for qs,qg in zip(initial,prepose) for x,y in zip(qs,qg))
        # 五次时间曲线：最大 ds/du=1.875，最大 |d²s/du²|=10/sqrt(3)。
        prepose_duration = max(a.prepose_sec, 1.875*delta/a.prepose_speed,
                               math.sqrt((10/math.sqrt(3))*delta/a.prepose_accel))
        print(f'[1/2] 低速预摆位 {prepose_duration:.1f}s，限速 {a.prepose_speed:g}rad/s；'
              f'平面={a.plane}，Y={a.center_y:g}m', flush=True)
        started = time.monotonic()
        last_print = -1
        tracking_error_since = None
        while True:
            elapsed = time.monotonic() - started
            u = min(1.0, elapsed / prepose_duration)
            blend = 10*u**3 - 15*u**4 + 6*u**5
            goals = [[x + blend*(y-x) for x,y in zip(qs,qg)] for qs,qg in zip(initial,prepose)]
            arr = MoveTargetArray()
            arr.targets = [target(arm, joints=q) for arm, q in zip(('left', 'right'), goals)]
            pub.publish(arr)
            rclpy.spin_once(node, timeout_sec=0.01)
            s = healthy()
            measured = dict(zip(s.joint_names, s.q_measured))
            error = max(abs(measured[n] - q) for names, qs in zip((LEFT, RIGHT), prepose)
                        for n, q in zip(names, qs))
            if int(elapsed) != last_print:
                print(f'  预摆位 {elapsed:.1f}s，最大关节误差 {error:.3f}rad', flush=True)
                last_print = int(elapsed)
            following_error = max(abs(measured[n]-q) for names,qs in zip((LEFT,RIGHT),goals)
                                  for n,q in zip(names,qs))
            if following_error > 0.20:
                tracking_error_since = tracking_error_since or time.monotonic()
                if time.monotonic()-tracking_error_since > 0.5:
                    raise RuntimeError('预摆位持续跟随误差超过 0.20rad，停止推进并回待命保持')
            else:
                tracking_error_since = None
            if elapsed >= prepose_duration and error < 0.08:
                break
            if elapsed > prepose_duration + 20.0:
                raise RuntimeError('预摆位未收敛，不进入形状阶段')
            time.sleep(1.0 / a.rate)

        path = Path(a.stats_file).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        output = path.open('w', newline='', encoding='utf-8')
        writer = csv.writer(output)
        writer.writerow(['time_s'] + [f'{arm}_{kind}_{axis}' for arm in ('L', 'R')
                                     for kind in ('des', 'plan', 'act') for axis in ('x', 'y', 'z')])
        print('[2/2] 左圆右方已运行；反馈仅来自 move_controller/status。', flush=True)
        started = time.monotonic()
        last_print = -1
        last_stamp = None
        while a.duration == 0 or time.monotonic() - started < a.duration:
            elapsed = time.monotonic() - started
            # 前 blend_sec 秒平滑增加轨迹相位速度，起点位置连续。
            t = elapsed
            b = a.blend_sec
            phase_time = b*((t/b)**3 - 0.5*(t/b)**4) if t < b else t - 0.5*b
            arr = MoveTargetArray()
            points = shape_points(a, phase_time)
            arr.targets = [target(arm, position=p) for arm, p in zip(('left', 'right'), points)]
            pub.publish(arr)
            rclpy.spin_once(node, timeout_sec=0.0)
            s = healthy()
            if a.enable_gripper:
                value = (1.0, 0.5, 0.0)[int(elapsed // 10) % 3]
                names = [n for n in ('left_gripper', 'right_gripper') if n in s.joint_names]
                if names:
                    g = JointState()
                    g.name, g.position = names, [value]*len(names)
                    grip.publish(g)
            stamp = (s.header.stamp.sec, s.header.stamp.nanosec)
            if stamp != last_stamp:
                last_stamp = stamp
                # 现有公开接口没有末端规划/实测坐标；保留 CSV 列，留空表示不可用。
                row = [f'{elapsed:.4f}']
                for point in points:
                    row.extend(f'{value:.6f}' for value in point)
                    row.extend([''] * 6)
                writer.writerow(row)
            if int(elapsed / 5) != last_print:
                last_print = int(elapsed / 5)
                print(f'  形状运行 {elapsed:.1f}s，state={s.state_name}，累计下发={s.frames_published}', flush=True)
                output.flush()
            time.sleep(1.0 / a.rate)
    except KeyboardInterrupt:
        print('收到停止请求。', flush=True)
    except Exception as exc:
        print(f'示例停止：{exc}', flush=True)
        exit_code = 1
    finally:
        if output:
            output.close()
        if enabled and rclpy.ok():
            try:
                result = call(disable_cli, Trigger.Request())
                print('disable: ' + result.message, flush=True)
            except Exception as exc:
                print(f'disable 未确认：{exc}；目标已停止发送。', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
