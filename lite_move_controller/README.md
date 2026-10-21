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

控制器支持三种启动期模式：

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

> [!CAUTION]
> **重要环境要求：必须使用系统 Python 环境编译，严禁在 Anaconda / Conda 环境下执行编译！**
> 
> 如果终端处于 Conda 虚拟环境（如终端前缀包含 `(base)`），请在编译前务必执行：
> ```bash
> conda deactivate
> ```
> **原因**：Anaconda 自带了更高版本的 `libyaml-cpp.so.0.9`，而 Ubuntu 系统及 ROS 2 使用的是 `libyaml-cpp.so.0.8`。如果在 Conda 激活状态下编译，CMake 会优先链接 Anaconda 的 `yaml-cpp 0.9`，导致运行时出现致命动态库错误：
> `dlopen error: libyaml-cpp.so.0.9: cannot open shared object file: No such file or directory`，使硬件驱动与控制器插件（`liblite_driver.so`、`liblite_controller.so`）全部无法被 `controller_manager` 加载。

```bash
cd <workspace>
conda deactivate  # 务必退出 conda 环境
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

## 启动

### 方式一：一揽子脚本自动拉起（推荐）

只要底层硬件与驱动桥层（`lite_bringup`）已经启动并处于 MIT 模式，直接运行一揽子测试脚本即可自动拉起运控、激活使能并执行轨迹：

```bash
# 步骤 1：启动底层 bringup（若未启动）
ros2 launch lite_bringup lite_bringup.launch.py \
  mode:=arms controller:=modes control_mode:=mit \
  source_names:=lite_move_controller

# 步骤 2：一揽子运行左圆右方测试（自动完成节点拉起、使能、轨迹执行与退出回收）
python3 src/Lite_move_controller/lite_move_controller/scripts/run_circle_square.py
```

**一揽子脚本能力**：
1. **自动清理与拉起**：自动清理系统内残留的 `move_controller_node` / `lite_ik_node` 孤儿进程，并在后台拉起 `move_controller.launch.py`（启动日志实时重定向到 `tmp/move_controller_launch.log`）。
2. **状态感知与自动使能**：等待节点完成 URDF/Pinocchio 解析进入 `STANDBY` 状态后，自动调用 `/lite_move_controller/enable` 使其进入 `TRACKING` 模式（若遇到 `FAULT` 自动尝试 `clear_fault`）。
3. **轨迹执行与报表导出**：自动完成预摆位与末端形状跟踪，结束后打印全链路误差统计并保存至 `tmp/circle_square_tracking_stats.csv`。
4. **安全收尾**：正常结束或 Ctrl+C 中断时，自动调用 `/lite_move_controller/disable` 释放控制权，并优雅清理后台拉起的 launch 进程树。

常用参数：
- `--mode {regular,admittance,impedance}`：指定控制律（默认 `regular`）
- `--joint-mode {arms,arms_grippers}`：指定关节模式（默认 `arms`）
- `--speed 0.05`：指定末端线速度（m/s）
- `--duration 200`：测试时长（秒）
- `--no-launch`：不自动拉起后台 launch（适用于已在独立终端手动启动了 `move_controller.launch.py` 的场景）

### 方式二：手动分步启动

若需在独立终端观察控制器和 IK 节点的全部实时日志，可分步启动：

```bash
# 1. 启动运控层与 IK 节点
ros2 launch lite_move_controller move_controller.launch.py \
  mode:=regular \
  joint_mode:=arms

# 2. 节点默认处于 STANDBY 待命状态，确认后手动激活：
ros2 topic echo /lite_move_controller/status --once
ros2 service call /lite_move_controller/enable std_srvs/srv/Trigger "{}"

# 3. 运行测试脚本（指定 --no-launch 复用已启动的节点）：
python3 src/Lite_move_controller/lite_move_controller/scripts/run_circle_square.py --no-launch

# 4. 测试结束或需要释放控制权时：
ros2 service call /lite_move_controller/disable std_srvs/srv/Trigger "{}"
```

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
