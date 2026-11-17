#!/usr/bin/env python3
"""离线 ROS 集成检查：模拟桥，不连接 CAN。需已构建工作空间。
FASTDDS_BUILTIN_TRANSPORTS=UDPv4 ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST python3 test_runtime_contract.py
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from move_msgs.msg import MoveStatus, MoveTarget, MoveTargetArray
from move_msgs.srv import QueryIk
from robstride_msgs.msg import MotionCommand
from robstride_msgs.srv import SetMode, SetSource, GetBridgeStatus
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

LEFT = ['left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
        'left_elbow_pitch', 'left_wrist_yaw', 'left_wrist_roll', 'left_wrist_pitch']
RIGHT = [n.replace('left_', 'right_') for n in LEFT]
NAMES = LEFT + ['left_gripper'] + RIGHT + ['right_gripper']


def main():
    assert 187 <= int(os.environ.get('ROS_DOMAIN_ID', '-1')) <= 199, '必须使用隔离测试域 187–199'
    assert os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') == 'LOCALHOST'
    rclpy.init()
    node = Node('lite_bridge')
    urdf = Path(get_package_share_directory('lite_ik_node')) / 'config/model/lite_000_asm.urdf'
    limits = {j.attrib['name']: j.find('limit') for j in ET.parse(urdf).getroot().findall('joint')}
    q = [(float(limits[n].attrib['lower']) + float(limits[n].attrib['upper']))/2 for n in NAMES]
    mode = [0]
    requested_modes = []
    locked = ['']
    send_state = [True]
    commands = []
    status = [None]
    grippers = []
    pub_state = node.create_publisher(JointState, '/joint_states', 10)

    def tick():
        if send_state[0]:
            m = JointState()
            m.name, m.position = NAMES, q
            m.velocity, m.effort = [0.0]*16, [0.0]*16
            pub_state.publish(m)

    timer = node.create_timer(0.01, tick)
    def get_status(req, res):
        res.ok = True
        res.protocol = 0
        res.protocol_name = 'mit'
        res.mode = mode[0]
        res.mode_name = {0:'ZERO_TORQUE',1:'HOLD',2:'TRACKING'}[mode[0]]
        res.controller_active = True
        res.active_source = locked[0]
        res.manual_override = locked[0]
        res.source_names = ['lite_move_controller']
        res.arbitration_state_name = 'ACTIVE'
        res.joint_names = NAMES
        return res
    def set_mode(req, res):
        requested_modes.append(req.mode)
        mode[0] = req.mode
        res.ok = res.controller_active = True
        res.actual_mode = req.mode
        res.actual_protocol = 0
        return res
    def set_source(req, res):
        locked[0] = req.source if req.mode else ''
        res.ok = True
        res.manual_override = locked[0]
        res.active_source = locked[0]
        res.available_sources = ['lite_move_controller']
        return res
    services = [node.create_service(GetBridgeStatus, '~/get_status', get_status),
                node.create_service(SetMode, '~/set_mode', set_mode),
                node.create_service(SetSource, '~/set_source', set_source)]
    subscriptions = [node.create_subscription(MotionCommand, '~/motion_command',
                      lambda m: commands.append((time.monotonic(), m)), 100),
                     node.create_subscription(MoveStatus, '/lite_move_controller/status',
                      lambda m: status.__setitem__(0, m), 10),
                     node.create_subscription(JointState, '/lite_move_controller/gripper_command',
                      lambda m: grippers.append(m), 10)]
    target_pub = node.create_publisher(MoveTargetArray, '/lite_move_controller/task_target', 10)
    grip_pub = node.create_publisher(JointState, '/lite_move_controller/gripper_command', 10)
    enable = node.create_client(Trigger, '/lite_move_controller/enable')
    disable = node.create_client(Trigger, '/lite_move_controller/disable')
    query = node.create_client(QueryIk, '/lite_move_controller/ik_query')

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
    def until(predicate, timeout=30):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)
        assert predicate(), '等待条件超时'
    def call(client, request):
        until(client.service_is_ready)
        f = client.call_async(request)
        until(f.done)
        return f.result()
    def stop(p):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
            try: p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
                p.wait(timeout=5)
    def launch(mode_name, expected_state=MoveStatus.STATE_STANDBY, enable_ik=False):
        status[0] = None
        commands.clear()
        log = tempfile.NamedTemporaryFile(prefix='lmc-contract-', suffix='.log', delete=False)
        p = subprocess.Popen(['ros2','launch','lite_move_controller','move_controller.launch.py',
                              'joint_mode:=arms_grippers', 'mode:='+mode_name,
                              'enable_ik_node:='+str(enable_ik).lower(),
                              'urdf_path:='+str(urdf)], stdout=log, stderr=subprocess.STDOUT,
                              start_new_session=True)
        print('runtime log:', log.name, flush=True)
        try:
            until(lambda: status[0] is not None and status[0].state == expected_state
                  and (expected_state != MoveStatus.STATE_STANDBY or
                       (status[0].bridge_authorized and status[0].bridge_mode == 2)))
        except Exception:
            stop(p)
            print(Path(log.name).read_text()[-6000:])
            raise
        return p

    process = None
    try:
        # 夹爪在收到状态/指令前不发零目标；归一化负数和超 1 值均裁剪。
        send_state[0] = False
        process = subprocess.Popen(['ros2','run','lite_move_controller','gripper_node'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   start_new_session=True)
        spin(2)
        assert not grippers, '未初始化夹爪不应发零目标'
        normalized = node.create_publisher(JointState, '/gripper/command', 10)
        spin(0.5)
        m = JointState(); m.name = ['left_gripper','right_gripper']; m.position = [-1.0, 2.0]
        for _ in range(10): normalized.publish(m); spin(0.05)
        assert grippers and list(grippers[-1].position) == [0.0, 0.047], grippers[-1:]
        stop(process); process = None
        send_state[0] = True

        process = launch('gravity_compensation')
        spin(1.2)
        count = len(commands); spin(1.0)
        assert 80 <= len(commands)-count <= 120, 'STANDBY 应持续 100Hz 保持'
        assert 1 not in requested_modes, '健康启动不得经过 HOLD'
        r = call(query, QueryIk.Request())
        assert not r.ok and '关闭' in r.message
        assert call(enable, Trigger.Request()).success
        until(lambda: status[0].state == MoveStatus.STATE_RUNNING)
        spin(1.3)
        assert status[0].control_mode == 'gravity_compensation'
        assert '重力补偿' in status[0].state_detail
        assert commands
        last = commands[-1][1]
        arm_indices = list(range(7)) + list(range(8,15))
        assert all(last.kp[k] == 0 and last.kd[k] == 1 and last.velocity[k] == 0 for k in arm_indices)
        assert any(abs(last.effort[k]) > 0.01 for k in arm_indices), '重力前馈不可全零'
        baseline = list(last.position)
        arr = MoveTargetArray()
        m = MoveTarget(); m.arm = 'left'; m.target_type = MoveTarget.TARGET_JOINTS
        m.joint_positions = [0.0]*7; arr.targets = [m]
        for _ in range(30): target_pub.publish(arr); spin(0.02)
        assert max(abs(a-b) for a,b in zip(commands[-1][1].position,baseline)) < 1e-8
        send_state[0] = False
        spin(0.5)
        assert status[0].state == MoveStatus.STATE_FAULT
        count = len(commands); spin(0.3)
        assert len(commands) == count, '失鲜后不能继续出力'
        stop(process); process = None
        send_state[0] = True
        spin(0.3)
        process = launch('regular')
        assert call(enable, Trigger.Request()).success
        until(lambda: status[0].state in (MoveStatus.STATE_RUNNING, MoveStatus.STATE_DWELL))
        spin(0.8)
        # IK 关闭仍能跟踪关节目标；同一公开入口的位姿目标不能触发内部逆解。
        desired = list(q[:7]); desired[0] += 0.10
        m = MoveTarget(); m.arm='left'; m.target_type=MoveTarget.TARGET_JOINTS
        m.joint_positions=desired; arr.targets=[m]
        for _ in range(60): target_pub.publish(arr); spin(0.02)
        assert abs(commands[-1][1].position[0]-desired[0]) < 0.02
        pose = MoveTarget(); pose.arm='left'; pose.target_type=MoveTarget.TARGET_POSE
        pose.position.x=0.2; pose.position.y=-0.3; pose.position.z=0.6; pose.orientation.w=1.0
        arr.targets=[pose]
        for _ in range(25): target_pub.publish(arr); spin(0.02)
        assert status[0].state == MoveStatus.STATE_DWELL
        # 直接内部物理开度入口也不能绕过 URDF 限位。
        m = JointState(); m.name=['left_gripper','right_gripper']; m.position=[-1.0,2.0]
        for _ in range(10): normalized.publish(m); spin(0.02)
        m = JointState(); m.name=['left_gripper','right_gripper']; m.position=[-10.0,10.0]
        for _ in range(10): grip_pub.publish(m); spin(0.02)
        last=commands[-1][1]
        assert last.position[7] == 0.0 and abs(last.position[15]-0.047)<1e-9
        assert call(disable, Trigger.Request()).success
        until(lambda: status[0].state==MoveStatus.STATE_STANDBY)
        assert mode[0] == 2 and locked[0] == 'lite_move_controller'
        count=len(commands)
        arr.targets=[pose]
        for _ in range(25): target_pub.publish(arr); spin(0.02)
        assert len(commands)>count+30 and status[0].state==MoveStatus.STATE_STANDBY
        assert max(abs(x-y) for x,y in zip(commands[-1][1].position,q)) < 1e-7
        assert 1 not in requested_modes, 'disable 也不得切 HOLD'
        duplicate=node.create_publisher(MotionCommand, '/lite_bridge/motion_command', 10)
        until(lambda: status[0].state==MoveStatus.STATE_FAULT)
        node.destroy_publisher(duplicate)
        stop(process); process = None
        # 上电姿态越界时不能先切 TRACKING，再把实测裁到边界形成位置跳变。
        original = q[0]
        q[0] = float(limits[NAMES[0]].attrib['upper']) + 0.1
        mode[0] = 0; locked[0] = ''; requested_modes.clear()
        process = launch('regular', MoveStatus.STATE_FAULT)
        spin(0.5)
        assert not commands and not requested_modes and not locked[0]
        stop(process); process = None
        q[0] = original
        print('PASS: out-of-range startup refuses authorization and sends no motion', flush=True)
        # 外部 IK 实际节拍：使用实测关节 FK 得到原地目标，不测试机器人运动。
        import pinocchio as pin
        model = pin.buildModelFromUrdf(str(urdf))
        data = model.createData()
        # 显式查询种子与当前待命姿态不同，仍须保持调用方选择的逆解分支。
        process = launch('regular', enable_ik=True)
        for arm, names in (('left', LEFT), ('right', RIGHT)):
            seed = [q[NAMES.index(n)] + (0.3 if j % 2 == 0 else -0.3)
                    for j, n in enumerate(names)]
            seed_model = pin.neutral(model)
            for name, value in zip(names, seed):
                seed_model[model.joints[model.getJointId(name)].idx_q] = value
            pin.forwardKinematics(model, data, seed_model)
            pin.updateFramePlacements(model, data)
            tf = data.oMf[model.getFrameId(arm+'_gripper_tip_middle_link')]
            req = QueryIk.Request(); req.arm = arm; req.frame_id = 'world_root'
            req.want_joints = True; req.seed_joints = seed
            req.position.x, req.position.y, req.position.z = map(float, tf.translation)
            quat = pin.Quaternion(tf.rotation).coeffs()
            req.orientation.x, req.orientation.y, req.orientation.z, req.orientation.w = map(float, quat)
            result = call(query, req)
            assert result.ok and len(result.joints) == 7, result
            assert max(abs(x-y) for x,y in zip(seed,result.joints)) < 0.05, (seed,result)
        stop(process); process = None
        print('PASS: public IK query preserves explicit seed branch', flush=True)
        qm = pin.neutral(model)
        for n, value in zip(NAMES, q):
            jid = model.getJointId(n)
            qm[model.joints[jid].idx_q] = value
        pin.forwardKinematics(model, data, qm)
        pin.updateFramePlacements(model, data)
        pose = MoveTarget(); pose.arm = 'left'; pose.frame_id = 'world_root'
        pose.target_type = MoveTarget.TARGET_POSE
        transform = data.oMf[model.getFrameId('left_gripper_tip_middle_link')]
        pose.position.x, pose.position.y, pose.position.z = map(float, transform.translation)
        quat = pin.Quaternion(transform.rotation).coeffs()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, quat)
        pose.timeout_ms = 200.0
        arr.targets = [pose]
        solved = []
        sub = node.create_subscription(MoveTargetArray, '/lite_move_controller/joint_target',
                                      lambda m: solved.append((time.monotonic(), m)), 100)
        for hz in (12.5, 25.0):
            log = tempfile.NamedTemporaryFile(prefix='ik-rate-', suffix='.log', delete=False)
            process = subprocess.Popen(['ros2','run','lite_ik_node','lite_ik_node','--ros-args',
                                        '-p', 'ik_rate_hz:='+str(hz), '-p', 'urdf_path:='+str(urdf)],
                                        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            until(lambda: target_pub.get_subscription_count() > 0)
            spin(2)
            solved.clear()
            deadline = time.monotonic() + 10.0
            while not solved and time.monotonic() < deadline:
                target_pub.publish(arr); spin(0.005)
            assert solved, '等待第一份 IK 输出超时'
            solved.clear()
            started = time.monotonic()
            while time.monotonic() - started < 6.0:
                target_pub.publish(arr); spin(0.005)
            print('rate samples:', hz, len(solved), [round(t-started, 3) for t, _ in solved[:10]], flush=True)
            assert hz*5.0 <= len(solved) <= hz*6.5, (hz, len(solved), Path(log.name).read_text()[-2000:])
            spin(0.4)
            count = len(solved); spin(0.4)
            assert len(solved) == count, '不得持续重发旧目标'
            print(f'PASS: IK {hz} Hz, {count} outputs / 6s input; stops after input gap', flush=True)
            stop(process); process = None
            spin(0.3)
        print('PASS: startup gripper / clamping / gravity / rejected targets / stale feedback / joint-only mode', flush=True)
    finally:
        if process: stop(process)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
