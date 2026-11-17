# Lite_move_controller

Lite 双臂运动控制包。它接收连续末端位姿或关节目标，生成平滑的全关节 MIT
命令，并通过 `lite_bridge` 下发。仓库同时包含 `move_msgs`，作为控制器与上游
数据源之间的消息和服务契约。

## 软件分层

```text
上游目标
  │  move_msgs/MoveTargetArray
  ▼
Lite_IK_node
  │  连续关节目标
  ▼
Lite_move_controller
  │  滤波、插值、控制律、动力学前馈、限位
  ▼
lite_bridge → lite_controller → lite_driver

lite_dyid ── params.json ──▶ Lite_move_controller
```

`Lite_IK_node` 提供 KDL/MoveIt 插件运动学库和独立 IK 节点。控制器链接它导出的
运动学库，并在默认启动文件中拉起独立 IK 节点。`lite_dyid` 负责生成机器人自身
的动力学参数；控制器只读取其 `params.json` 产物。

`move_msgs` 保持独立 ROS 包名，因为 IK 节点与控制器都要使用同一套接口。它与
`lite_move_controller` 一起发布在本仓库中，从而保持接口和实现同步版本化。

## 控制原理

输入位姿先进行四元数符号对齐，再转为旋转向量并经过二阶 Butterworth 低通。
IK 节点根据当前关节状态做阻尼最小二乘微分逆解，并利用零空间梯度把冗余关节
拉向限位中部；必要时调用全局 IK 后端兜底。控制器收到关节目标后用三次 Hermite
段连接相邻解，以固定控制频率发布完整关节帧。

控制器支持以下启动期模式：

- `gravity_compensation`：进入内部重力补偿运行状态，拒绝运动和夹爪目标；手臂按实测位置计算 `g(q)`，Kp=0、Kd=1，不加摩擦或惯性项。
- `regular`：关节位置、速度与动力学前馈。
- `admittance`：由估计外力生成位姿偏移，再沿普通跟踪链路下发。
- `impedance`：由末端位姿误差经 `Jᵀ(Kx + Dẋ)` 生成附加关节力矩。

所有模式在发布前统一经过位置、速度、力矩和增益限制。动力学前馈使用 Pinocchio
读取 `lite_dyid` 的质量、质心、惯量与摩擦参数。

## 工作空间布局

推荐将三个仓库并列放在工作空间 `src` 下：

```text
<workspace>/src/
├── Lite_move_controller/   # 本仓库
│   ├── lite_move_controller/
│   └── move_msgs/
├── Lite_IK_node/           # lite_ik_node 包
└── lite_dyid/
```

控制器还需要现有底层包：`lite_bridge`、`lite_controller`、`lite_driver`、
`lite_description`、`robstride_msgs` 和 `lite_bringup`。

## 构建

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to lite_move_controller
source install/setup.bash
```

依赖方向为：

```text
move_msgs → lite_ik_node → lite_move_controller
lite_dyid ──生成参数文件──▶ lite_move_controller
```

本链路不依赖 `lite_moveit` 配置包。IK 使用的 URDF、SRDF 和
`kinematics.yaml` 随 `lite_ik_node` 一起安装。

## 启动规范

运行运控层前，**必须先启动底层驱动与桥层**，并且必须通过 `source_names:=lite_move_controller`
将运控层加入桥层的数据源授权白名单（否则桥会丢弃运控层的全部命令）。

### 1. 启动底层 Bringup（终端 1）

```bash
# 推荐：16 轴模式（双臂 14 轴 + 双夹爪 2 轴）
ros2 launch lite_bringup lite_bringup.launch.py \
    mode:=arms_grippers controller:=modes control_mode:=mit \
    source_names:=lite_move_controller

# 纯双臂 14 轴模式（实机未装夹爪时使用）：
ros2 launch lite_bringup lite_bringup.launch.py \
    mode:=arms controller:=modes control_mode:=mit \
    source_names:=lite_move_controller
```

### 2. 启动运控层 Move Controller（终端 2）

```bash
# 16 轴模式（默认自动拉起独立夹爪增量控制节点 gripper_node）：
ros2 launch lite_move_controller move_controller.launch.py \
    mode:=regular joint_mode:=arms_grippers

# 纯双臂 14 轴模式（不启动 gripper_node，旧逻辑零改变）：
ros2 launch lite_move_controller move_controller.launch.py \
    mode:=regular joint_mode:=arms
```

其他可用控制律模式（启动期参数固定）：
- `mode:=admittance`：导纳控制
- `mode:=impedance`：末端阻抗控制

节点完成授权和模式回读后进入 STANDBY：底层保持 MIT TRACKING，以 100 Hz
保持当前实测位置并更新重力前馈，不接受运动目标。enable 只开放目标输入；
disable 清除目标、重新锚定实测并回待命，保留 TRACKING 与授权。故障、失鲜或
被抢占仍停止下发，允许底层安全降级。

实测臂关节超出模型限位（容许 0.01 rad 的端点误差）会进入 FAULT，启动时不
锁源、不切 TRACKING；运行时立即停发。不能通过裁剪越界实测来建立待命目标，
否则会在接管时产生位置跳变。需核对机器人姿态与本机标定后再恢复。确认状态后启用：

```bash
ros2 topic echo /lite_move_controller/status --once
ros2 service call /lite_move_controller/enable std_srvs/srv/Trigger "{}"
```

### 3. 夹爪独立控制接口（仅在 mode:=arms_grippers 下可用）

由 `gripper_node` 向上暴露 0~1 归一化输入（0.0 代表完全闭合，1.0 代表完全张开）：

- **逐爪控制**：发布至 `/gripper/command`（`sensor_msgs/msg/JointState`）
  ```bash
  ros2 topic pub /gripper/command sensor_msgs/msg/JointState \
    "{name: ['left_gripper', 'right_gripper'], position: [1.0, 0.5]}" --once
  ```
- **双爪全局控制**：发布至 `/gripper/opening`（`std_msgs/msg/Float64`）
  ```bash
  ros2 topic pub /gripper/opening std_msgs/msg/Float64 "{data: 1.0}" --once   # 全开
  ros2 topic pub /gripper/opening std_msgs/msg/Float64 "{data: 0.5}" --once   # 半开
  ros2 topic pub /gripper/opening std_msgs/msg/Float64 "{data: 0.0}" --once   # 全闭
  ```

### 4. 外部接口示范：左圆右方

先启动 bringup 和 move_controller（见文末完整命令），再运行安装后的示例：

```bash
ros2 run lite_move_controller run_circle_square.py --duration 200 --speed 0.01
```

示例调用公开 `ik_query` 预查起点和轨迹采样点，发送关节目标预摆位，实测接近
起点后发送左圆、右圆角方形的末端位姿。插值和动力学由控制器负责，示例不加载
内部模型。默认在 X-Z 平面绘制；`--plane horizontal` 改为 X-Y 平面。
夹爪默认每 10 秒依次发送归一化 1、0.5、0；`--no-gripper` 关闭该示范。

## 动力学辨识参数

仓库内置当前 Lite 构型的已验证参数：

```text
lite_move_controller/config/dynamics/identified_params.json
```

默认配置直接使用这份文件，安装后由包路径自动定位，无需本机绝对路径。

如果机械臂构型发生变化，例如更换连杆、负载、电机、减速器或关节零位，应重新
执行 `lite_dyid`。辨识完成后，把新结果覆盖到指定位置：

```bash
cp ~/.ros/lite_dyid/runs/run_<时间戳>/params.json \
  <workspace>/src/Lite_move_controller/lite_move_controller/config/dynamics/identified_params.json

cd <workspace>
colcon build --packages-select lite_move_controller
```

也可以在启动时临时指定另一份结果：

```bash
ros2 launch lite_move_controller move_controller.launch.py \
  params_json:=/absolute/path/to/params.json
```

参数文件中的关节顺序必须与当前 `canonical_joints` 一致。建议把一次有效辨识的
`params.json` 与对应构型提交在同一个控制器版本中，保证示范可复现。

## 主要接口

| 方向 | 接口 | 类型 |
|---|---|---|
| 输入 | `/lite_move_controller/task_target` | `move_msgs/msg/MoveTargetArray` |
| IK 输出/控制器输入 | `/lite_move_controller/joint_target` | `move_msgs/msg/MoveTargetArray` |
| 状态输入 | `/joint_states` | `sensor_msgs/msg/JointState` |
| 桥命令输出 | `/lite_bridge/motion_command` | `robstride_msgs/msg/MotionCommand` |
| 状态输出 | `/lite_move_controller/status` | `move_msgs/msg/MoveStatus` |
| 服务 | `/lite_move_controller/enable` | `std_srvs/srv/Trigger` |
| 服务 | `/lite_move_controller/disable` | `std_srvs/srv/Trigger` |
| 服务 | `/lite_move_controller/get_status` | `move_msgs/srv/GetMoveStatus` |
| 服务 | `/lite_move_controller/ik_query` | `move_msgs/srv/QueryIk` |

主要配置位于 `config/lite_move_controller.yaml`。启动文件会根据 `joint_mode`
注入 14 或 16 关节的 canonical 顺序，并将控制器与 IK 节点接到同一组话题。

## 验证

```bash
./build/lite_move_controller/test_core
./build/lite_move_controller/test_filter_shape
./build/lite_move_controller/test_gripper
./build/lite_move_controller/test_dynamics
```

## 注意事项

- `joint_mode`、桥的 canonical 关节集合和启动参数必须一致。
- 更换机器人构型后必须同步更新 `Lite_IK_node` 的模型资产并重新执行 `lite_dyid`。
- 启动测试前先确认桥当前数据源和控制模式；测试结束可调用 `disable` 退出接管。


## 2026-11 接口示例与运行约定

运行态默认下发 100 Hz，DWELL 仍为 20 Hz。独立 IK 默认按 25 Hz 消费每臂最新位姿，
`ik_rate_hz:=...` 可在启动时调整；关节目标直接透传。每份位姿只求解一次，排队和求解
耗时均计入目标有效期，不用重复发布旧解掩盖断流。

`enable_ik_node:=false` 后不执行任何逆解，也不回退内部 IK；`task_target` 和
`joint_target` 仍接受 `TARGET_JOINTS`，位姿目标被拒绝，`ik_query` 返回关闭原因。

纯重力补偿通过现有启动入口选择：

```bash
ros2 launch lite_move_controller move_controller.launch.py mode:=gravity_compensation joint_mode:=arms_grippers
ros2 service call /lite_move_controller/enable std_srvs/srv/Trigger '{}'
```

该模式复用原有锁源、接管、disable、反馈失鲜、抢占和故障处理，无运动目标时持续
补偿，不进入 DWELL。模型参数加载失败会拒绝启动。内部状态为 GRAVITY_COMPENSATION；
为保持消息契约，对外仍为 `state=5 / state_name=RUNNING`，通过已有的
`control_mode=gravity_compensation` 和 `state_detail` 区分。夹爪保持接管时实测开度，
不参与手臂重力计算。激活斜坡期间补偿和阻尼渐入，稳定后手臂 Kd=1，仍受安全限幅。

默认动力学数据随包保存在 `config/dynamics/identified_params.json`，安装时一起发布。
它与 `lite_dyid` 的 `run_20261019_234605/params.json` 一致，生成时间为
2026-10-20T08:54:39+00:00，SHA-256：
`150849c04691b8734d98792d37201244920abda926d8f466d6955d161fe495f6`。
默认 launch 解析为安装包内的绝对路径，不依赖当前目录或辨识工作目录；
显式 `params_json:=...` 仍可覆盖。`get_status` 的 verbose 回读报告实际加载路径。

夹爪只增加目标位置裁剪：归一化入口裁剪到 [0,1]，内部物理开度也限制到 URDF 范围；
配置不能扩大该范围。驱动继续将物理开度映射到标定的闭合/张开端点。
不增加边界减力、接触检测或新的夹持策略。未收到实测/指令的夹爪不发布初始化零目标。
`gripper_torque_limit` 目前仅约束前馈字段，不代表完整 MIT 力矩上限；底层既有保护保持原样。

左圆右方示例可从安装包直接运行；操作者应先启动驱动及本节点并确保处于 STANDBY：

```bash
ros2 launch lite_move_controller move_controller.launch.py mode:=impedance joint_mode:=arms_grippers
ros2 run lite_move_controller run_circle_square.py --duration 200 --no-gripper
```

示例只使用公开的 `ik_query`、`get_status`、`enable`、`disable`、`task_target`、
`status`，以及包内夹爪节点的公开归一化入口 `/gripper/command`。它不访问底层话题/服务、不加载 URDF、
不启动或终止其它节点。先预查轨迹采样点，再发送关节预摆位，实测接近后开始末端
轨迹；状态异常立即停止目标并请求 disable。预查仅验证采样点可达性，不是碰撞规划。

`--stats-file` 保留原末端轨迹 CSV 列名和顺序，记录目标坐标；公开接口未提供的
`plan`/`act` 坐标留空，不再依赖内部模型重算或虚构末端误差。关节反馈仍可从
`status` 获取。旧 `--urdf` 参数保留兼容，但不读取文件。运行结束和 Ctrl+C 都请求 disable。
新版电机层的 MIT HOLD 是故障退回用的纯阻尼，不等于锁住当前位置；健康
STANDBY/disable 不再经过 HOLD。预摆位使用五次时间曲线，默认至少 30 s、
最大关节速度 0.10 rad/s、加速度 0.15 rad/s²；图形默认速度 0.01 m/s。

真机启动必须显式使用本机标定文件，不得用更新远端仓库附带的标定覆盖本机数据。
同一套驱动、运控、示例需使用相同独立 ROS_DOMAIN_ID；本机测试设置
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST。运控发现重复命令/状态/机器人描述
发布者会进入 FAULT，避免同名 ROS 节点串入其它机器的命令。

14/16 轴需显式配对：底层 `mode:=arms/arms_grippers` 对应运控
`joint_mode:=arms/arms_grippers`。本次不修改底层默认值或模式切换机制。
