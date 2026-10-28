# Copyright (c) 2026
# Licensed under the Apache License, Version 2.0
"""启动 lite_move_controller（运控层）。

⚠️ 前置条件（缺一不可，缺了的表现都在注释里写了）：

  1. **lite_bringup 必须先起来**，而且是 MIT 组 + 授权本节点为数据源：
         ros2 launch lite_bringup lite_bringup.launch.py \\
             mode:=arms controller:=modes control_mode:=mit \\
             source_names:=lite_move_controller
     · 漏 controller:=modes → 加载的是 CSP 控制器，本层的 MIT 五字段无人消费
     · 漏 source_names      → 桥丢弃本层**所有**命令（日志："丢弃未知数据源"）

  2. `canonical_joints` 必须与 bringup 的 `arm_joints`（= 驱动实际关节集合）
     **逐字一致且顺序一致**；`joints_left/right` 永远只是**7 个臂关节**
     （IK 链只有 7 个 revolute，夹爪不在链里）。
     · `joint_mode:=arms`（14）：canonical = [L7, R7]
     · `joint_mode:=arms_grippers`（16）：canonical = [L7, left_gripper, R7,
       right_gripper]（顺序 = lite_bringup 的 `_DAMPING_BY_MODE`）
     ⚠️ 旧版这里有个 bug：`joints_left/right` 在 16 关节模式下被塞成 8 个
        （含夹爪）⇒ 节点拿 8 个关节去建 7 自由度的链 ⇒ 建链失败 ⇒ arms=0。
        现在 `joints_*` 固定 7 个，夹爪单独走 `gripper_joints`。

  3. 逆解模型与参数由 `lite_ik_node` 提供。
     本 launch **不启动** move_group（本层不需要规划场景），只读它的配置。

用法：
    # 默认：常规模式
    ros2 launch lite_move_controller move_controller.launch.py

    # 16 关节 + 夹爪阻抗夹持（夹住一支笔；gripper_position=0 = 夹到底）
    ros2 launch lite_move_controller move_controller.launch.py \\
        joint_mode:=arms_grippers gripper_position:=0.0 gripper_kp:=20 gripper_kd:=3

    # 导纳 / 阻抗（**启动期固定**，要换就重启）
    ros2 launch lite_move_controller move_controller.launch.py mode:=admittance
    ros2 launch lite_move_controller move_controller.launch.py mode:=impedance

    # 使用另一份 lite_dyid 动力学参数
    ros2 launch lite_move_controller move_controller.launch.py \\
        params_json:=$HOME/.ros/lite_dyid/runs/run_XXXX/params.json

启动后（**不会自动接管**，auto_enable 默认 false）：
    ros2 topic echo /lite_move_controller/status --once
    ros2 service call /lite_move_controller/enable std_srvs/srv/Trigger "{}"
    # 然后发目标：
    ros2 topic pub /lite_move_controller/task_target move_msgs/msg/MoveTargetArray ...
    # 回退：
    ros2 service call /lite_move_controller/disable std_srvs/srv/Trigger "{}"
"""

import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

#: 与 lite_bringup 的 `mode:=arms` 一致的 14 关节顺序（**唯一事实来源是 bringup**）
_JOINTS_LEFT = ['left_shoulder_pitch', 'left_shoulder_roll', 'left_shoulder_yaw',
                'left_elbow_pitch', 'left_wrist_yaw', 'left_wrist_roll',
                'left_wrist_pitch']
_JOINTS_RIGHT = ['right_shoulder_pitch', 'right_shoulder_roll', 'right_shoulder_yaw',
                 'right_elbow_pitch', 'right_wrist_yaw', 'right_wrist_roll',
                 'right_wrist_pitch']


def _canonical(mode: str):
    """按 mode 生成 canonical_joints（必须与 lite_bringup 的裁剪规则一致）。"""
    left = list(_JOINTS_LEFT)
    right = list(_JOINTS_RIGHT)
    if mode == 'arms_grippers':
        # 夹爪夹在各自臂**中间**（与 launch 的 _DAMPING_BY_MODE 顺序一致）
        left = left + ['left_gripper']
        right = right + ['right_gripper']
    return left + right


def generate_launch_description():
    # 允许用**外部 YAML** 覆盖包内默认（实验/整定用）：它排在默认文件之后、
    # 注入字典之前，所以 "默认文件 < 外部文件 < 逐项注入" 的优先级成立。
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lite_move_controller'), 'config',
             'lite_move_controller.yaml']),
        description='参数 YAML（默认包内 config/lite_move_controller.yaml）。')
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='regular',
        description="控制律：regular / admittance / impedance（**启动期固定**）。")
    joint_mode_arg = DeclareLaunchArgument(
        'joint_mode', default_value='arms',
        description="关节集合，必须与 lite_bringup 的 mode:= 一致：arms（14）/ arms_grippers（16）。")
    source_arg = DeclareLaunchArgument(
        'source_name', default_value='lite_move_controller',
        description='数据源名（必须已写进 bringup 的 source_names）。')
    params_json_arg = DeclareLaunchArgument(
        'params_json', default_value='',
        description='lite_dyid 的 params.json（空 = 不发动力学前馈）。')
    urdf_arg = DeclareLaunchArgument(
        'urdf_path', default_value='',
        description='URDF 路径（空 = 用 /robot_description）。')
    bounded_arg = DeclareLaunchArgument('bounded_stream_only', default_value='false',
        description='Use atomic fenced ingress; disables legacy arm/gripper ingress and gripper_node.')
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable', default_value='false',
        description='是否起节点后自动 enable（真机建议 false）。')
    rate_arg = DeclareLaunchArgument(
        'control_rate_hz', default_value='100.0',
        description='控制周期（Hz）。')
    enable_ik_node_arg = DeclareLaunchArgument(
        'enable_ik_node', default_value='true',
        description='是否启动独立的微分逆解节点 lite_ik_node（默认 true：使用 DLS 微分逆解 + 零空间避限位）。')
    # ---- 夹爪（prismatic 阻抗夹持）------------------------------------------
    # ⚠️ 只有 joint_mode:=arms_grippers 才会真正生效（否则 canonical 里没有
    #    夹爪关节，节点会整段跳过并打一行 INFO）。
    gripper_position_arg = DeclareLaunchArgument(
        'gripper_position', default_value='0.0',
        description='已弃用的兼容参数；启动保持实测，运动请发 /gripper/command。')
    gripper_kp_arg = DeclareLaunchArgument(
        'gripper_kp', default_value='20.0',
        description='夹爪刚度 [N/m]（prismatic！）。')
    gripper_kd_arg = DeclareLaunchArgument(
        'gripper_kd', default_value='3.0',
        description='夹爪阻尼 [N·s/m]（prismatic！）。')
    gripper_torque_limit_arg = DeclareLaunchArgument(
        'gripper_torque_limit', default_value='0.25',
        description='夹爪力矩上限（N·m，默认 0.25 N·m）。')
    gripper_open_limit_arg = DeclareLaunchArgument(
        'gripper_open_limit', default_value='0.047',
        description='夹爪全开对应的物理行程上限（默认 0.047 m；若传 47 则适配 47 标度）。')

    def _build(context):
        def s(name):
            return LaunchConfiguration(name).perform(context).strip()

        def b(name):
            return s(name).lower() in ('1', 'true', 'yes', 'on')

        jm = s('joint_mode')
        canonical = _canonical(jm)
        # ⚠️ `joints_left/right` **永远**是 7 个臂关节（IK 链只有 7 个 revolute）。
        #    旧版在 arms_grippers 下把它们塞成 8 个（含夹爪）⇒ 节点拿 8 个关节
        #    去建 7 自由度的链 ⇒ 跳过该臂 ⇒ arms=0（真机就是这么失败的）。
        left = list(_JOINTS_LEFT)
        right = list(_JOINTS_RIGHT)

        # ⚠️ 这里**自己写 YAML**（而不是把 dict 交给 launch_ros）：
        #    launch_ros 会把 Python list 归一化成 tuple 并 dump 成
        #    `!!python/tuple`，rclcpp 读不了该键（实测：canonical_joints
        #    "No parameter value set"，节点起不来）；而且它用 `/**` 通配键，
        #    在本机节点上也不生效。自己写可以同时钉死**节点名**与**list 语义**。
        params = {
            'canonical_joints': canonical,
            'joints_left': left,
            'joints_right': right,
            'gripper_position': float(s('gripper_position')),
            'gripper_kp': float(s('gripper_kp')),
            'gripper_kd': float(s('gripper_kd')),
            'gripper_torque_limit': float(s('gripper_torque_limit')),
            'source_name': s('source_name'),
            'control_mode': s('mode'),
            'control_rate_hz': float(s('control_rate_hz')),
            'urdf_path': s('urdf_path'),
            'auto_enable': b('auto_enable'),
            'bounded_stream_only': b('bounded_stream_only'),
            'use_external_ik': b('enable_ik_node'),
        }
        if jm == 'arms_grippers':
            params['gripper_joints'] = ['left_gripper', 'right_gripper']
        # ⚠️ `params_json` **只在显式给出时**才注入：空串会覆盖 YAML/节点里的
        #    默认辨识路径（默认开启动力学前馈），把前馈静默关掉。
        if s('params_json'):
            params['params_json'] = s('params_json')

        with tempfile.NamedTemporaryFile(
                mode='w', prefix='lmc_params_', suffix='.yaml',
                delete=False) as fh:
            yaml.safe_dump({'lite_move_controller': {'ros__parameters': params}},
                           fh, allow_unicode=True, sort_keys=False)
            overrides_path = fh.name

        node = Node(
            package='lite_move_controller',
            executable='move_controller_node',
            name='lite_move_controller',
            output='screen',
            parameters=[s('params_file'), overrides_path],
        )

        nodes = [node]
        if jm == 'arms_grippers' and not b('bounded_stream_only'):
            # 仅在 mode:=arms_grippers 时拉起增量夹爪控制节点
            nodes.append(Node(
                package='lite_move_controller',
                executable='gripper_node',
                name='gripper_node',
                output='screen',
                parameters=[{
                    'gripper_joints': ['left_gripper', 'right_gripper'],
                    'lower_limits': [0.0, 0.0],
                    'upper_limits': [float(s('gripper_open_limit')), float(s('gripper_open_limit'))],
                    'default_stiffness': float(s('gripper_kp')),
                    'default_damping': float(s('gripper_kd')),
                    'publish_rate_hz': 50.0,
                    'command_topic': '/gripper/command',
                    'opening_topic': '/gripper/opening',
                    'joint_states_topic': '/joint_states',
                    'output_topic': '/lite_move_controller/gripper_command',
                }],
            ))
        if b('enable_ik_node'):
            ik_share = FindPackageShare('lite_ik_node').perform(context)
            ik_params = {
                'input_target_topic': '/lite_move_controller/task_target',
                'output_joint_topic': '/lite_move_controller/joint_target',
                'joint_states_topic': '/joint_states',
                'joints_left': left,
                'joints_right': right,
                'max_joint_jump_rad': 0.35,
            }
            if s('urdf_path'):
                ik_params['urdf_path'] = s('urdf_path')
            else:
                ik_params['urdf_path'] = ik_share + '/config/model/lite_000_asm.urdf'
            ik_params['srdf_path'] = ik_share + '/config/model/lite_000_asm.srdf'

            ik_node = Node(
                package='lite_ik_node',
                executable='lite_ik_node',
                name='lite_ik_node',
                output='screen',
                parameters=[ik_params],
            )
            nodes.append(ik_node)

        return nodes

    from launch.actions import OpaqueFunction
    return LaunchDescription([
        mode_arg, joint_mode_arg, source_arg, params_json_arg, urdf_arg,
        auto_enable_arg, bounded_arg, rate_arg, enable_ik_node_arg, params_file_arg,
        gripper_position_arg, gripper_kp_arg, gripper_kd_arg, gripper_torque_limit_arg,
        gripper_open_limit_arg,
        OpaqueFunction(function=_build),
    ])
