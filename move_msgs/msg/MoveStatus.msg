# MoveStatus —— 运控层向上反馈的**唯一**全景状态
#
# ===========================================================================
# 上游需要知道的四件事（缺一不可）
# ===========================================================================
#
# ① **我的命令被接受了吗**（状态机档位 + 是否被桥授权）
# ② **IK 解出来了吗、有没有饱和**（`ik_status` / `saturated_joints`）
# ③ **任务目标打满了吗**（`task_residual` / `ee_residual`）—— 打不满时上游可以
#    用 `MoveTarget.position += ee_residual` 迭代逼近，而不必自己猜
# ④ **我在补偿什么、补得对不对**（`tau_model` vs `tau_measured`）
#
# ⚠️ 状态机档位与"是否被授权"是**两个不同的轴**（设计文档 §2.4）：
#    本层 `RUNNING` + 桥 `active_source` 不是自己 = 被抢占，
#    这时命令会被桥丢弃，上游必须看得见，否则会以为"命令生效了"。
#
# ⚠️ 反馈频率（默认 20 Hz）**低于**下发频率（100~200 Hz）：
#    状态里有 14×5 个数组，按下发频率发会让 DDS 过载，而人/上游看不了那么快。

std_msgs/Header header

# --- ① 状态机 ---------------------------------------------------------------
#
# 0=IDLE 未初始化，1=LOADING 载入模型/IK/参数，2=STANDBY 就绪但未接管,
# 3=ARMING 正在锁源，4=SWITCHING 正在请求桥切模式，5=RUNNING 正常下发，
# 6=SUSPENDED 被别的源抢占（**已停发**），7=DWELL 目标超时→低频保持,
# 8=FAULT 安全违例（已停发），9=ERROR 启动期不可恢复错误
uint8 STATE_IDLE = 0
uint8 STATE_LOADING = 1
uint8 STATE_STANDBY = 2
uint8 STATE_ARMING = 3
uint8 STATE_SWITCHING = 4
uint8 STATE_RUNNING = 5
uint8 STATE_SUSPENDED = 6
uint8 STATE_DWELL = 7
uint8 STATE_FAULT = 8
uint8 STATE_ERROR = 9
uint8 state
string state_name

# 为什么停在当前状态 / 为什么拒绝迁移（人类可读，空 = 正常运行）。
# ⚠️ 这个字段是刻意设计的：桥的 get_status 早已证明"说不出原因"最难排查。
string state_detail

# --- 本层当前配置（启动期固定）----------------------------------------------
string control_mode            # regular / admittance / impedance
string dynamics_model          # static_friction_only / static_friction_inertia

# 是否已具备"能算 M(q)"的惯性参数。false ⇒ 导纳的惯性整形/阻抗的 D 推导
# **自动退化为名义质量**（并在日志里明写），不会静默假装有惯性。
bool dynamics_has_inertia

# --- ② 与桥的关系 -----------------------------------------------------------
bool bridge_authorized         # 桥回读：现在是不是听我的
string bridge_active_source    # 桥回读：实际持有控制权的源
string bridge_arbitration_state
uint8 bridge_mode              # 桥回读的模式号（2 = MIT TRACKING 才对）
string bridge_mode_name

# --- ③ 逐臂 -----------------------------------------------------------------
string[] arms

# 逆解状态（**三个正交轴**，设计文档 §4）：
#   ik_status:  0=OK 精确可达，1=RESIDUAL 有解但任务打不满，
#               2=LIMIT_CLAMPED 解被限位裁剪，3=FAILED 没有可用新解
uint8[] ik_status
string[] ik_status_name
# 位置残差（m）与姿态残差（rad）：target − 实际可达点。**永远有限**
float64[] ik_residual_pos
float64[] ik_residual_rot
# 被限位夹住的关节名（空 = 没有被夹）
string[] ik_saturated_joints
# 本帧实际动作：0=SEND_SOLVED 1=SEND_RESIDUAL 2=HOLD_LAST
uint8[] ik_action
uint8 IK_ACTION_SEND_SOLVED = 0
uint8 IK_ACTION_SEND_RESIDUAL = 1
uint8 IK_ACTION_HOLD_LAST = 2

# 导纳内部状态：顺从偏移（m, rad），6 维/臂
float64[] admittance_offset
# 外力估计（N, N·m），6 维/臂。⚠️ 由 τ_ext 反推，**不是**真力测量
float64[] estimated_wrench

# --- ④ 下发内容与实测对照 ---------------------------------------------------
# ⚠️ 顺序与运控层的 canonical_joints 一致（= 桥的 canonical_joints）
string[] joint_names

# 期望位置 / 实测位置（rad）
float64[] q_desired
float64[] q_measured

# 模型前馈力矩（N·m）与实测力矩（N·m）
float64[] tau_model
float64[] tau_measured

# 电机侧刚度 / 阻尼（逐关节）
float64[] stiffness
float64[] damping

# 激活斜坡进度 0~1（0 = 首帧：Kp=Kd=0 且位置 = 实测 ⇒ 力矩恒为 0）
float64 ramp_alpha

# --- 诊断计数（单调递增，用于发现"偶发但持续"的问题）------------------------
uint64 frames_published        # 成功下发的帧数
uint64 frames_rejected         # 因自身校验失败而未下发的帧数
uint64 target_timeouts         # 目标超时次数（→ DWELL）
uint64 limit_clamped           # 被 limit_guard 夹住的累计次数（非 0 要查）
uint64 filter_resets           # 因时间戳跳变/四元数翻转等触发的滤波复位次数
