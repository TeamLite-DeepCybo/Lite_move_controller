# InteractionEvent —— 交互事件（接触 / 脱离 / 外力越界）
#
# ===========================================================================
# ⚠️⚠️ 本机**没有力传感器**，所以"真外力"字段现在恒为 0（占位）
# ===========================================================================
#
# 事件的判据来自**力矩残差**：
#
#     τ_ext = τ_measured − τ_model        （τ_model = g + C·q̇ + 摩擦）
#     F_ext = J(q)^{+T} · τ_ext           （只保留"能产生末端运动"的分量）
#
# 这个估计有两个已知局限（写在这里以免被误用）：
#   ① **只能观测能产生末端运动的力矩分量** —— 沿"手臂轴线"的纯压缩力、
#      以及纯零空间内的力，在 J 的零空间里，**看不见**；
#   ② τ_measured 含减速箱摩擦与温漂，所以 F_ext 只适合做**阈值事件检测**
#      （进入接触 / 脱离），**不适合**当力测量值用于力控。
#
# 将来装了 F/T 传感器：把真实 wrench 填进 `wrench` 并把 `wrench_is_estimated`
# 置 false，控制律结构不变（设计时已按这个接口预留）。
std_msgs/Header header

string arm

# 0=ENTER_CONTACT 进入接触，1=LEAVE_CONTACT 脱离接触，
# 2=WrenchExceeded 外力估计超阈，3=ADMITTANCE_SATURATED 顺从偏移到限幅
uint8 EVENT_ENTER_CONTACT = 0
uint8 EVENT_LEAVE_CONTACT = 1
uint8 EVENT_WRENCH_EXCEEDED = 2
uint8 EVENT_ADMITTANCE_SATURATED = 3
uint8 event
string event_name

# 触发本次事件的估计外力（N, N·m，6 维：力 + 力矩）
float64[] wrench

# ⚠️ true = 上面是**由力矩残差反推的估计值**（当前唯一可能）；false = 真测量。
bool wrench_is_estimated

# 触发阈值（与运控层配置一致，便于上游判断"离阈值还有多远"）
float64 threshold
# 超过阈值的持续时长（秒）—— 用于区分"瞬时尖峰"与"真的接触"
float64 duration_sec
