# EeResidual —— 终端残差（target − 实际可达点）
#
# ===========================================================================
# 单独一条消息的理由：**迭代逼近**是最常用的上游动作
# ===========================================================================
#
# IK 打不满目标时（超出工作空间 / 姿态不可达 / 撞限位），运控层会把目标
# **投影到最近可达点**并继续下发（不停止、不跳变）。上游若想真到达那个点，
# 最自然的做法是迭代：
#
#     x_new = x_old + residual        # 逐次把残差加回目标
#
# 所以本消息给出的是"可加量"：`position_residual = target − reachable`。
# 上游可以直接 `MoveTarget.position += position_residual` 再来一帧。
#
# ⚠️ 与 MoveStatus 里的 `ik_residual_*` 是**同一份数据** —— 这里单独发是因为
#    迭代修正只需要这两项，不值得让上游去解析整个 14×5 的状态数组。
#    ⚠️ 但它**不替代** MoveStatus：饱和关节名、状态机档位、力矩对照仍要看状态。
std_msgs/Header header

# 哪条臂（与 MoveTarget.arm 一致）
string arm

# true = 本帧 IK **精确可达**（残差在阈值内），此时下面两项恒为 0。
# ⚠️ 上游应当**先看这个标志**，不要用"残差接近 0"去判断 —— 阈值是运控层的配置。
bool exact

# 位置残差（m，在 `frame_id` 下）
geometry_msgs/Vector3 position_residual

# 姿态残差（rad，轴角表示的旋转向量）
geometry_msgs/Vector3 orientation_residual

# 残差范数（便于上游直接设阈值）
float64 position_residual_norm
float64 orientation_residual_norm
