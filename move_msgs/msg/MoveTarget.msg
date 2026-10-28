# MoveTarget —— 运控层（lite_move_controller）的**流式**终端目标
#
# ===========================================================================
# 设计意图：上游只描述"终端想要什么"，不关心应用什么控制律
# ===========================================================================
#
# 上游数据源（遥操 / 视觉伺服 / 在线规划 / 测试数据源）以 10~200 Hz **流式**
# 连续下发本消息；运控层负责：
#   · 位姿滤波 → 逆解（IK）→ 三次密插 → 关节目标
#   · 叠加动力学前馈（g / 摩擦 / 惯量）与所选控制律
#   · 组装成 robstride_msgs/MotionCommand（MIT 五字段）下发给 lite_bridge
#
# ⚠️ 本消息**不携带控制律选择**：常规 / 导纳 / 阻抗在 lite_move_controller 里
#    是**启动期参数**（`mode:=regular|admittance|impedance`），运行期固定。
#    理由：三套控制律的 Kp/Kd/τ_ff 语义不同，在线切换会出现"力的阶跃"，
#    而且不利于各自独立整定/回归。要换律就重启节点（秒级，不动机器人）。
#
# ⚠️ 本包（move_msgs）与 robstride_msgs **刻意解耦**：
#    · 本消息描述"运控层与上游之间的契约"，与电机协议无关；
#    · 运控层**向下**仍然严格遵循 robstride_msgs（MotionCommand / JointCommand），
#      一层对一层负责，不要把两层契约混在一个包里。

std_msgs/Header header

# 目标类型：位姿（默认）或**关节位置**。
#
# ⚠️ 关节模式是为了"预摆位/回位"这类不需要 IK 的动作：从当前关节角平滑
#    走到给定关节角，再切回位姿模式继续跟随。两种模式在**同一个 TRACKING**
#    内切换，不经过 HOLD（HOLD 是零力拖动，会漂移）。
uint8 TARGET_POSE = 0
uint8 TARGET_JOINTS = 1
uint8 target_type

# 目标机械臂："left" / "right"（必须与运控层的 arm 配置一致）。
# 未知值 → 整帧拒绝（不猜）。
string arm

# 位姿所在坐标系。⚠️ 必须是运控层**认识且已配置**的坐标系
# （默认 "world_root"）；不认识就整帧拒绝。
string frame_id

# --- 关节模式（target_type == TARGET_JOINTS）-------------------------------
# 臂关节的期望位置（rad），长度必须 = 该臂关节数（7），顺序 = joints_left/right。
# 空 = 用位姿模式。非空时忽略 position/orientation（IK 不参与）。
float64[] joint_positions

# --- 期望终端位姿 -----------------------------------------------------------
#
# `position` 是位置（m）；`orientation` 是四元数（x,y,z,w）。
#
# ⚠️ 为什么用四元数而不是欧拉角：
#   欧拉角有万向锁、且插值不唯一；运控层收到后会做**符号对齐**（q·q_prev ≥ 0）
#   再转成旋转向量做滤波，因此上游不必自己保证连续性 —— 但**必须**是单位四元数
#   （‖q‖ 偏离 1 超过 1e-3 → 整帧拒绝，而不是静默归一化）。
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

# --- 可选的每帧控制参数覆盖 ------------------------------------------------
#
# `params_valid = false`（默认）⇒ 全部使用运控层的启动期配置。
# `params_valid = true` ⇒ 本帧覆盖下列参数；**未覆盖的项仍用启动期配置**。
#
# ⚠️ 为什么允许覆盖：同一个模式在接触/自由空间需要不同的柔度，
#    让上游能"边走边调"比重启节点现实。但覆盖值仍会被运控层的硬上限夹住
#    （`limit_guard`），上游无法把刚度/力矩推到危险值。
bool params_valid

# 任务空间刚度（N/m, N·m/rad），6 维：x,y,z + 绕 x,y,z。
# 空 = 用启动期配置。
float64[] stiffness_cart

# 任务空间阻尼（N·s/m, N·m·s/rad），6 维。空 = 用配置。
float64[] damping_cart

# 导纳参数：[M(6), Ka(6)]，空 = 用配置（仅 admittance 模式有意义）。
# Ka 越小越"跟手"（K_a = 0 ⇒ 自由拖动）；K_a 大 ⇒ 近似刚体跟随。
float64[] admittance

# 前馈外力（N, N·m），6 维。⚠️ 目前**必须为空或全 0**：
#   本机没有力传感器（见设计文档），运控层的外力来自
#   `τ_ext = τ_meas − τ_model` 的估计。本字段是给将来装 F/T 传感器预留的接口，
#   现在填了会被忽略并打一条限频告警（不静默生效）。
float64[] wrench_feedforward

# --- 夹爪 -------------------------------------------------------------------
#
# DEPRECATED: retained for wire compatibility; ignored in joint AND pose paths.
# Use /gripper/command (JointState, normalized 0..1) for independent grippers.
# This scalar cannot distinguish omitted from explicit zero. Do not use it as
# a command; explicit zero on the NEW named topic remains a valid close target.
float64 gripper

# 本帧有效期（毫秒）。0 = 用运控层的默认值。
# ⚠️ 运控层的自检超时（t_target_timeout_ms）会与它取较小者：
#    某个上游崩溃时，宁可早退到"保持"也不要晚。
float64 timeout_ms

# 序号，便于丢帧检测与日志对齐（上游单调递增即可；不校验连续性）。
uint64 sequence
