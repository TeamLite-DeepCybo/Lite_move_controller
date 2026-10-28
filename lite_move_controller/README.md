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

节点默认处于 STANDBY 待命状态。确认状态后启用：

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

### 4. 一键示范：左圆右方 + 夹爪三态循环

启动带夹爪的 bringup 后，在终端中直接运行一键脚本：

```bash
# 标准启动：16 轴阻抗控制 + 动力学前馈 + 夹爪 10s 三态循环
bash src/lite_move_controller/scripts/run_circle_square.sh

# 环境变量微调示例：
DURATION=60 SPEED=0.04 bash src/lite_move_controller/scripts/run_circle_square.sh   # 运行 60s，线速度 0.04 m/s
PLANE=horizontal bash src/lite_move_controller/scripts/run_circle_square.sh         # 在 X-Y 水平桌面绘制
ENABLE_GRIPPER=0 bash src/lite_move_controller/scripts/run_circle_square.sh         # 禁用夹爪开闭示范（仅纯臂绘制）
```

---

## 左圆右方案例原理与运行指南

「左圆右方」是运控层的综合验收示范：左臂在三维空间绘制严格水平向前的圆形轨迹，右臂绘制圆角正方形轨迹，同时夹爪进行三态开闭动作。该案例全面检验了**运动学微分逆解、笛卡尔阻抗控制、Pinocchio 动力学前馈补偿、四道闸安全限幅、夹爪解耦控制与三级误差解耦分析**。

### 1. 案例执行流程与命令

整个测试流程分为启动准备、预摆位、位姿轨迹跟踪三个阶段：

```mermaid
flowchart LR
    A["底层 Bringup<br/>(16 轴 MIT 模式)"] --> B["运控层启动<br/>(move_controller + ik + gripper)"]
    B --> C["阶段 1: 预摆位 (8s)<br/>关节空间平滑规划至起点"]
    C --> D["阶段 2: 轨迹跟踪 (200s)<br/>左圆右方位姿模式 + 夹爪三态循环"]
    D --> E["数据结算与导出<br/>三级误差解耦报告 & CSV 保存"]
```

#### 完整操作步骤

**终端 1：底层 Bringup（带夹爪 16 轴模式）**
```bash
ros2 launch lite_bringup lite_bringup.launch.py \
    mode:=arms_grippers controller:=modes control_mode:=mit \
    source_names:=lite_move_controller
```

**终端 2：运行一键测试脚本**
```bash
bash src/lite_move_controller/scripts/run_circle_square.sh
```

如需直接调用底层 Python 驱动脚本进行高度定制化测试：
```bash
python3 src/lite_move_controller/scripts/run_circle_square.py \
    --prepose-sec 8.0 \
    --duration 200.0 \
    --speed 0.05 \
    --circle-d 0.12 \
    --square-side 0.08 \
    --corner-r 0.015 \
    --dist-sym 0.20 \
    --plane vertical \
    --enable-gripper \
    --stats-file tmp/circle_square_tracking_stats.csv
```

### 2. 核心数学与控制原理

#### ① 阶段 1：关节空间平滑预摆位（Prepose）
- **起点解算**：利用本地 URDF 与 Pinocchio 对两臂目标起点（左圆右方起始位姿，姿态严格水平向前 $q=[0.7071, 0, 0, 0.7071]$）求解带限位约束的逆运动学（$q_{pre}$）。
- **三次 Hermite 关节规划**：从当前实际构型 $q_{meas}$ 到预摆位目标 $q_{pre}$ 进行多项式平滑插值，速度与加速度连续，防止在从任意收纳姿态切入形状时产生笛卡尔空间冲击或跨奇异点跳变。

#### ② 阶段 2：任务空间连续轨迹生成
- **左臂圆形（Circle）**：
  $$p_L(t) = c_L + \begin{bmatrix} r \cos(\omega t) \\ 0 \\ r \sin(\omega t) \end{bmatrix}, \quad \omega = \frac{v}{r}, \quad r = 0.06\text{ m}$$
- **右臂圆角正方形（Filleted Square）**：
  采用边长 $a = 0.08\text{ m}$、圆角半径 $r_c = 0.015\text{ m}$ 的连续曲率轨迹。通过分段直线与圆弧过渡，消除直角转弯处的速度阶跃与加速度奇点。
- **平滑过渡（Blend）**：在预摆位终点与形状起点之间加入 2.0s 的多项式混合过渡（Blend），确保切入几何轨迹时零冲击。

#### ③ 任务空间阻抗控制与刚度动态映射
- 控制器接收笛卡尔位姿目标，微分逆解节点 `lite_ik_node` 计算目标关节角；
- 运控层实时计算解析雅可比矩阵 $J(q)$，根据任务空间配置的虚拟刚度 $K_x = [350, 350, 350, 4, 4, 4]$ 与阻尼 $D_x$，通过能量等价原则将其投影为关节空间对角刚度与阻尼：
  $$K_{p, j_k} = \sum_{i=1}^6 J_{ik}^2 K_{x, i}, \quad K_{d, j_k} = \sum_{i=1}^6 J_{ik}^2 D_{x, i}$$
- 这一映射消除了笛卡尔外环弹簧与 MIT 关节内环弹簧的“双弹簧并联冲突”，实现动态随动柔顺。

#### ④ 动力学前馈补偿（Pinocchio 与 dyid）
- 14 个手臂关节全部注入非线性重力与库仑/粘性摩擦力矩前馈：
  $$\tau_{ff}(q, \dot{q}) = g(q) + \tau_{friction}(\dot{q})$$
- 动力学模型使用 `lite_dyid` 离线辨识的高精度刚体惯量与摩擦参数（`identified_params.json`），使关节大部分静态重力由前馈力矩 $\tau_{ff}$ 承担，MIT 刚度环仅负责克服外力扰动与微小跟踪误差。

#### ⑤ 夹爪解耦控制原理
- **独立控制通道**：夹爪作为 16 轴模式下的增量控制节点，独立订阅 `/gripper/opening`（0.0 全闭 $\sim$ 1.0 全开），并按 URDF 限位 $[0.0, 0.047]\text{ m}$ 进行线性折算。
- **动力学解耦**：夹爪电机力矩前馈恒为 0（$\tau_{ff, gripper} = 0.0\text{ N}\cdot\text{m}$），力矩上限由安全闸严格限制为 0.25 N·m，绝不干扰机械臂 14 轴的动力学补偿。
- **三态循环示范**：在 200s 的画圆过程中，夹爪每 10 秒在全开 (1.0)、半开 (0.5)、全闭 (0.0) 之间平滑循环，验证手臂画圆与末端抓取的协同解耦能力。

#### ⑥ 三级轨迹跟踪误差全链路解耦分析
脚本全程以 100 Hz 采集三级轨迹并保存至 CSV 文件，并在测试结束时输出解耦评估报告：
- **$p_d(t)$（期望轨迹）**：几何生成的理论目标点；
- **$p_{plan}(t)$（规划轨迹）**：由运控层下发命令 `/lite_bridge/motion_command` 经正运动学计算出的末端位置；
- **$p_{act}(t)$（实测轨迹）**：由电机编码器回读 `/joint_states` 经正运动学计算出的末端物理位置。

数学空间满足严格的向量三角形解耦关系：
$$e_{total} = p_{act} - p_d = \underbrace{(p_{plan} - p_d)}_{e_{plan}\text{ (规划滤波滞后)}} + \underbrace{(p_{act} - p_{plan})}_{e_{ctrl}\text{ (运控执行误差)}}$$
- **$e_{plan}$**：反映目标滤波器的低通截止延迟、IK 插值收敛速度与可行性；
- **$e_{ctrl}$**：反映阻抗刚度/阻尼响应特性、重力与摩擦补偿残差以及驱动伺服带宽。
- 该分析使现场调试人员能清晰定位误差来源于“规划滤波迟滞”还是“力控跟踪残差”。

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
