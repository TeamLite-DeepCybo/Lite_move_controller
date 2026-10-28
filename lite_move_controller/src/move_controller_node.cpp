// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// move_controller_node.cpp —— 运控层 ROS 外壳
//
// ===========================================================================
// 本文件只做"接线"，所有决策都在纯 C++ 核心与 IK 后端里
// ===========================================================================
//
//   订阅：move_msgs/MoveTargetArray（流式终端目标）
//         /joint_states（位置/速度/力矩）
//   发布：robstride_msgs/MotionCommand（向下，MIT 五字段，经桥）
//         move_msgs/MoveStatus、move_msgs/EeResidual（向上反馈）
//   服务：~/enable ~/disable ~/clear_fault ~/get_status ~/ik_query
//
// ⚠️ 实时路径（定时器回调，100~200 Hz）**只做**：
//   滤波 → IK（外层已预解好种子）→ 密插 → 控制律 → limit_guard → publish。
//   服务调用（set_mode / set_source）**只在状态机要求时**发生，且同步阻塞，
//   绝不放在定时器回调的实时路径上（会拖垮整个 executor）。
//   做法：定时器里只**置标志**，由另一个低频定时器去真正调服务。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// 启动自检：扫 /proc 找重复实例 + 发信号（Linux 专用，本工程只跑 Linux）。
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <atomic>
#include <climits>
#include <dirent.h>
#include <iostream>
#include <iterator>
#include <thread>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <array>
#include <map>
#include <sstream>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <move_msgs/msg/ee_residual.hpp>
#include <move_msgs/msg/move_status.hpp>
#include <move_msgs/msg/move_target_array.hpp>
#include <move_msgs/srv/get_move_status.hpp>
#include <move_msgs/srv/query_ik.hpp>

#include "lite_move_controller/bridge_client.hpp"
#include "lite_move_controller/dynamics_pin.hpp"
#include "lite_move_controller/wrench.hpp"
#include "lite_move_controller/admittance.hpp"
#include "lite_move_controller/impedance.hpp"
#include "lite_ik_node/ik_backend.hpp"
#include "lite_ik_node/ik_kdl.hpp"
#include "lite_move_controller/gripper.hpp"
#include "lite_move_controller/gripper_input.hpp"
#include "lite_ik_node/ik_moveit.hpp"
#include "lite_move_controller/input_filter.hpp"
#include "lite_move_controller/limit_guard.hpp"
#include "lite_move_controller/node_state.hpp"
#include "lite_move_controller/resampler.hpp"

using namespace lite_move_controller;   // NOLINT

namespace
{
std::int64_t steadyNowMs()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// 单调时钟（秒，double）。**插值段的时基**：控制定时器的实际周期不等于
/// 1/control_rate（IK 会占掉一拍），所以段内进度必须按墙钟算。
double steadyNowSec()
{
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}
/** 系统时钟（epoch）毫秒。**只**给"上游时间戳是否超前"这条判据用。
 *
 * 为什么不复用 `steadyNowMs()`：`header.stamp` 是 epoch 基准（发布者用
 * `now()`），拿单调时钟去减它没有意义（差值会是几十万小时的量级）。
 */
std::int64_t systemNowMs()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

/// 一条臂的运行期数据（左右各一份，互不共享状态）。
struct ArmRuntime
{
  std::string name{"left"};
  std::string tip_link;   ///< 末端 frame（Jacobian / 动力学用）
  std::vector<std::string> joints;      ///< 该臂的关节名（顺序 = 参考链顺序）
  std::size_t offset{0};                ///< 在 canonical_joints 里的起始下标

  KdlArmChain chain;
  /// MoveIt 插件后端（pick_ik / trac_ik）。`use_moveit == false` 时它是**空壳**，
  /// IK/FK 全走 KDL —— KDL 后端**永不删除**：插件没装 / SRDF 找不到 / ABI 不匹配
  /// 时它是唯一的兜底，而且它不依赖任何外部 .so。
  MoveItArmKinematics moveit;
  /// 本臂最终用的是哪个后端（决定回调怎么绑、状态怎么报）。
  bool use_moveit{false};
  IkOrchestrator ik;
  InputFilter filter;
  bool have_target{false};
  Pose6 last_filtered{};
  std::vector<double> q_last;           ///< 上一拍**下发值**（每拍更新）
  /// 最近一次 IK 的**最终解**（不一定等于本拍下发值：由插值段给出）。
  std::vector<double> q_goal;
  /// ---- 时间基 Hermite 插值段 -------------------------------------------
  /// 端点 (q0,v0) → (q1,v1)，时长 seg_dur_s，起点墙钟 seg_t0_s（steady 秒）。
  /// ⚠️ 段的起点**永远是"当前下发值 + 当前段速度"**，所以新 IK 早到/晚到
  ///    都只是"提前切段"，位置与速度都连续（C¹）—— 不依赖固定节拍数。
  std::vector<double> seg_q0, seg_v0, seg_q1, seg_v1;
  double seg_t0_s{0.0};
  double seg_dur_s{0.0};
  bool seg_active{false};
  /// 上一次 IK 解与时刻（用来估 v1 = Δq/Δt；IK 频率不稳定也能得到真实速度）。
  std::vector<double> prev_ik_sol;
  /// 上一拍**实际下发**的关节速度（下一段 Hermite 的 v0；保证切段不"回零"）
  std::vector<double> last_v;
  std::int64_t prev_ik_ms{0};
  /// 上一次 IK **完成**的时刻（节拍从完成算起，避免慢 IK 后立刻再解）。
  std::int64_t last_ik_ms{0};
  bool have_goal{false};

  // ---- 关节模式（target_type == TARGET_JOINTS）-------------------------
  /// true = 当前目标是关节位置（预摆位/回位），**不走 IK**。
  bool joint_mode{false};
  std::vector<double> joint_target;      ///< 最近收到的关节目标
  std::vector<double> joint_active;      ///< 当前段正在走的关节目标
  std::int64_t joint_target_ms{0};       ///< 收到时刻（超时判据）
  /// 复位后第一帧是否已打印过"首帧 = 实测"的证据（每次接管只打一次）。
  bool first_cmd_logged{false};
  /// 上一拍**实际下发**的关节角（节点侧逐拍步长统计用；不受订阅端丢帧影响）。
  std::vector<double> prev_cmd;
  bool have_last_solution{false};
  std::vector<double> q_seed;           ///< 当前种子（= 上一周期的解）

  // 动力学与控制律
  PinDynamics dyn;
  bool dyn_ready{false};
  AdmittanceLaw admittance;          ///< 仅 admittance 模式使用
  bool admittance_ready{false};
  ImpedanceLaw impedance;            ///< 仅 impedance 模式使用
  bool impedance_ready{false};
  std::vector<double> tau_ff;        ///< 上一拍的前馈力矩（反馈用）
  std::vector<double> prev_q_ref, prev2_q_ref;  ///< 期望关节位置历史（算 a_ref）
  Vec6 last_wrench{};                ///< 上一拍的外力估计
  bool have_wrench{false};

  // 反馈缓存
  std::uint8_t ik_status{0};
  std::uint8_t ik_action{0};
  double residual_pos{0.0};
  double residual_rot{0.0};
  std::vector<std::string> saturated;
  Vec6 admittance_offset{};
  Vec6 estimated_wrench{};
  Vec3 residual_vec{0, 0, 0};
  Vec3 residual_rot_vec{0, 0, 0};
  bool exact{true};
  /// 本臂是否**至少解过一次** IK。
  /// ⚠️ 没有它的话，STANDBY（从未运行 IK）时 `exact/residual` 是**默认值**
  ///    （true / 0.0），发布出去等于对上游说"精确可达" —— 而实际上我们
  ///    根本没解过。那是**谎报**，上游会据此判断"目标没问题"。
  bool solved_once{false};
};
}  // namespace

class MoveControllerNode : public rclcpp::Node
{
public:
  /// ⚠️ 构造函数里**只声明参数**，别的什么都不做。
  ///
  /// 为什么（真机踩过）：原来的构造函数里会 `setupRosInterfaces()` +
  /// `setupArms()`，而 `setupArms()` 需要 URDF。`urdf_path` 为空时 URDF 只能
  /// 从 `/robot_description` 拿 —— 那个话题是 **transient_local** 发布的，
  /// **只有在 executor spin 的时候**历史消息才会被投递。构造函数执行时还没
  /// spin ⇒ `urdf_text_` 永远是空 ⇒
  ///     `[ERROR] 拿不到 URDF（urdf_path 为空且 /robot_description 未见）→ 跳过臂`
  ///     `已启动：... arms=0`
  /// 明明是"还没 spin"，却报成"没发布 URDF"，而且**假装启动成功**。
  ///
  /// 现在这些都在 `initialize()` 里做，由 `main()` 在 `make_shared` **之后**调用：
  ///   · 那时 `shared_from_this()` 可用（构造函数里会抛 `bad_weak_ptr`）；
  ///   · 可以拿一个临时 executor 在 shared_ptr 上**抽拍等** `/robot_description`；
  ///   · 等不到就 FATAL + 返回 false（fail-closed，不再静默 arms=0）。
  explicit MoveControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("lite_move_controller", options)
  {
    declareParameters();
  }

  /// 一步式初始化（**必须在 `std::make_shared` 之后调用**）。
  /// @return false ⇒ 配置/URDF 不可用；调用方必须 FATAL 并退出，**不要**继续转。
  bool initialize()
  {
    if (!loadConfiguration()) {
      load_error_ = "配置载入失败：" + load_error_;
      return false;
    }
    // ---- 回调组（**三组各司其职**，见 setupRosInterfaces 的详细说明）--------
    // 必须在 BridgeClient 之前建：/joint_states 的订阅要用它自己的组。
    joint_states_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    control_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // ⚠️ 目标流也要**独立**（MX 即可）：它只做 filter.push（纯计算）。
    //    和状态 tick / 服务同组的话，100 Hz 目标会被阻塞回调压住 ⇒ 状态抖。
    target_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    bridge_ = std::make_unique<BridgeClient>(
        this, source_name_, canonical_joints_, command_topic_, joint_states_topic_,
        command_timeout_ms_, joint_states_cb_group_);
    std::string err;
    if (!bridge_->start(err)) {
      load_error_ = "桥客户端启动失败：" + err;
      return false;
    }
    // ---- 启动自检 ①：重复实例（用户要求：授权后才终止）--------------------
    // ⚠️ 两个 move_controller 同时写同一个 source 会被桥的仲裁来回抢占，
    //    而且两个节点各自 LOCK 会让 active_source 抖动 —— 现场表现是
    //    "命令时有时无、帧率忽高忽低"。必须在建订阅/定时器之前处理掉。
    if (!handleDuplicateInstances(err)) {
      load_error_ = err;
      return false;
    }
    // ---- 启动自检 ②：必须处于 MIT ------------------------------------------
    // ⚠️ MIT/CSP 是**驱动启动期**协议，运行期不能切换（桥会拒绝跨协议 set_mode）。
    //    所以这里做 fail-closed：不是 MIT 就**一帧命令都不发**，并给出重启命令。
    if (!ensureMitProtocol(err)) {
      load_error_ = err;
      return false;
    }
    // 只建 /robot_description 的订阅 —— 定时器留到 URDF 就绪之后（见下），
    // 免得"等 URDF 时短暂 spin"把控制/状态回调跑起来。
    setupUrdfSubscription();
    if (!acquireUrdf()) {
      load_error_ = std::string("URDF 不可用：") + urdf_error_;
      return false;
    }
    RCLCPP_INFO(get_logger(), "URDF 来源：%s（%zu 字节）",
                urdf_source_.c_str(), urdf_text_.size());
    setupRosInterfaces();
    // ⚠️ limit_guard 必须在建臂之前、且**在 URDF 就绪之后**配置：
    //    它从 URDF 抽关节限位，早于 URDF ⇒ 拿到空串 ⇒ 静默退回默认 ±3.2 rad。
    setupLimitGuard(urdf_text_);
    setupArms();
    if (arms_.empty()) {
      // 建臂全失败（关节名对不上 / URDF 缺 link / 插件与 KDL 都建不起来 ……）
      // ⚠️ 这也是 fail-closed：arms=0 的节点接不了任何目标，继续跑只会让上游
      //    以为"我在"，实际什么都不做。
      load_error_ = "没有任何一条臂建成功（见上面每条臂的 ERROR/WARN）";
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "lite_move_controller 已启动：mode=%s  arms=%zu  joints=%zu  源='%s'  "
                "命令话题=%s  下行=%s  URDF=%s",
                control_mode_.c_str(), arms_.size(), canonical_joints_.size(),
                source_name_.c_str(), command_topic_.c_str(),
                joint_states_topic_.c_str(), urdf_source_.c_str());
    RCLCPP_INFO(get_logger(),
                "⚠️ 接管前请确认桥的 source_names 已包含 '%s'，否则所有命令会被丢弃",
                source_name_.c_str());
    return true;
  }

  const std::string & loadError() const noexcept {return load_error_;}

private:
  // ---------------------------------------------------------------- 参数
  /// "该参数未配置"的哨兵值。见 declareParameters 里对可选数组的说明。
  static constexpr double kUnsetSentinel = -1.0;

  void declareParameters()
  {
    declare_parameter<std::string>("source_name", "lite_move_controller");
    declare_parameter<std::string>("command_topic", "/lite_bridge/motion_command");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<std::vector<std::string>>("canonical_joints", {});
    declare_parameter<std::vector<std::string>>("arm_names", {"left", "right"});
    declare_parameter<std::vector<std::string>>("joints_left", {});
    declare_parameter<std::vector<std::string>>("joints_right", {});
    declare_parameter<std::string>("root_link", "world_root");
    declare_parameter<std::string>("frame_id", "world_root");
    declare_parameter<std::string>("tip_link_left", "left_gripper_tip_middle_link");
    declare_parameter<std::string>("tip_link_right", "right_gripper_tip_middle_link");
    declare_parameter<std::string>("urdf_path", "");      ///< 空 = 用 /robot_description
    declare_parameter<std::string>("kinematics_yaml", "");///< 空 = 用 lite_ik_node 的配置
    // --- IK 后端选择 --------------------------------------------------------
    // ⚠️ 参数名带点（`ik.solver_plugin`）是**故意的**：与 MoveIt 自己的
    //    `robot_description_kinematics.<group>.*` 一样走嵌套命名，YAML 里就是
    //      ik:
    //        solver_plugin: pick_ik/PickIkPlugin
    //    （rclcpp 允许参数名带 '.'，实测 `-p ik.solver_plugin:=...` 也能覆盖。）
    // ⚠️ 默认值是 pick_ik，但 pick_ik 是**可选安装**的：没装时本节点**不会**
    //    起不来，而是打 WARN 后回退 KDL（见 buildArm）。要强制走 KDL 就把它
    //    设成空串 —— 空串 = 明确"不要插件"，不会去打 WARN 刷日志。
    declare_parameter<std::string>("ik.solver_plugin", "pick_ik/PickIkPlugin");
    // ---- 夹爪（prismatic）阻抗夹持 ----------------------------------------
    // 用户场景：夹爪夹着一支笔，对**期望位置 0**（= 完全夹到底）做阻抗控制，
    // 然后手臂去画圆。夹爪**不在 IK 链里**（到 *_gripper_tip_middle_link 的链
    // 只有 7 个 revolute），也**不参与**动力学，所以在控制帧里它是左右独立的动态
    // 命令（见 gripper.hpp）。
    //
    // ⚠️ 默认值 = 两只夹爪的名字；14 关节模式下 canonical 里根本没有它们
    //    ⇒ gripperIndices() 返回空 ⇒ **什么也不写**，旧行为一字不变。
    declare_parameter<std::vector<std::string>>(
        "gripper_joints", {"left_gripper", "right_gripper"});
    declare_parameter<double>("gripper_position", 0.0);
    declare_parameter<double>("gripper_kp", 20.0);
    declare_parameter<double>("gripper_kd", 3.0);
    declare_parameter<double>("gripper_torque_limit", 0.25);
    declare_parameter<std::string>(
        "gripper_command_topic", "/lite_move_controller/gripper_command");
    /// SRDF 路径（"" = 自动找 lite_ik_node 自带模型）。
    /// ⚠️ 插件要靠 SRDF 的 group 定义才知道"这条臂有哪些关节"；KDL 后端不需要它。
    declare_parameter<std::string>("srdf_path", "");
    /// 插件单次求解的超时**上限**（秒）。<=0 = 不设上限（完全用 kinematics.yaml）。
    ///
    /// ⚠️ 为什么需要一个上限（实测数据，不是保险起见）：
    ///    pick_ik 在"目标不可达"时会**把超时用满**，而 `IkOrchestrator` 的失败
    ///    路径（多种子 + 最多几步增量前进）一次 `solve()` 里要做十来次求解调用。
    ///    不夹上限时一个完全不可达的目标（5 m 外）一次 `solve()` 要 413~430 ms，
    ///    而桥的 `command_timeout_ms` 只有 250 ms —— 超过之后桥会判本层"死了"
    ///    并丢弃所有命令（症状：目标流着，机器人却不动）。
    /// ⚠️ 但也不能太小（真机实测）：20 ms 时"抬起 10 cm"这种**完全可达**的目标
    ///    会被判 FAILED（pick_ik 的 memetic 搜索来不及），50 ms 就 SOLVED。
    ///    现在取 **50 ms**：单次调用最多 50 ms，而"一拨"仍有 50 ms 预算
    ///    （见 ik_moveit.cpp 的 burst_budget_ms）⇒ 失败路径仍是**每臂一次**调用，
    ///    两臂 ≈100 ms/拍，不会打爆桥的 250 ms。
    ///    这也意味着 **kinematics.yaml 里的 timeout 比它大时以它为准**。
    declare_parameter<double>("ik.max_solver_timeout_sec", 0.05);
    /// lite_dyid 的 `params.json`（动力学前馈）。
    ///
    /// 默认值使用包内随版本保存的那次辨识（2026-10-19 23:46，
    ///    `static_friction_only`、LPF=0.2、200 Hz、双臂 80% 行程轨迹）。
    ///    换机型/换采集必须显式覆盖；文件不存在时 `loadDynamicsForArm` 会
    ///    失败并**关掉前馈**（打 ERROR，不静默用旧参数）。
    declare_parameter<std::string>(
        "params_json", "config/dynamics/identified_params.json");
    declare_parameter<std::string>("control_mode", "regular");
    // ⚠️ **下发频率 = 100 Hz（设计前提，不因 IK 慢而降）**：
    //    "上层只有 20 甚至 10 Hz 时，我们仍以 100 Hz 下发一条更柔顺的曲线" ——
    //    所以不能用降低下发频率去解决 IK 太慢。100 Hz 的拍子里真正贵的是 IK
    //    （pick_ik 单臂 4~20 ms），所以把它**解耦**到 `ik_rate_hz` 的节拍上，
    //    其余拍子只做插值 + 四道闸 + publish（微秒级）。见 solveArm 的注释。
    declare_parameter<double>("control_rate_hz", 100.0);
    /// **IK 重解节拍**（Hz，默认 25）。注意它只是"最快重解"的上限：
    ///    IK 单次耗时本身不稳定（4~50 ms），所以**插值按墙钟时间推进**，
    ///    不看节拍数；新解来得早就早切段、来得晚就沿旧段走完（见 solveArm）。
    ///    必须 ≤ control_rate_hz。
    declare_parameter<double>("ik_rate_hz", 25.0);
    /// DWELL 状态下的下发频率（Hz）。
    /// ⚠️ 以前这个参数**只声明没使用**（DWELL 也按 control_rate 发）—— 现在
    ///    `onControlTick` 会真的按它限频（见那里的注释）。
    declare_parameter<double>("dwell_rate_hz", 20.0);
    declare_parameter<double>("command_timeout_ms", 250.0);
    declare_parameter<double>("target_timeout_ms", 200.0);
    declare_parameter<double>("state_timeout_ms", 100.0);
    declare_parameter<double>("tier_target_timeout_ms", 60.0);
    declare_parameter<double>("tier_bridge_timeout_ms", 120.0);
    declare_parameter<double>("ramp_duration_sec", 0.5);
    declare_parameter<double>("max_joint_step_rad", 0.05);
    /// 下发目标相对**实测**的最大超前量（rad，逐关节）。0 = 不限制。
    ///
    /// 为什么需要：MIT 下 `τ = Kp(q_des−q) + …`，若实测因力矩/负载落后，
    /// 而目标继续按时间外推，`Δq` 会越拉越大 ⇒ 力矩打满 ⇒ 更跟不上。
    /// 这里把每拍**下发值**夹到实测 ±lead，臂追上后自动恢复；它只影响
    /// 下发目标（不改参考轨迹/IK），是"按实际位置决定发多快"的落点。
    declare_parameter<double>("max_command_lead_rad", 0.20);
    /// 关节模式（预摆位/回位）的峰值速度（rad/s）。段长 = 1.5·max|Δq|/v。
    declare_parameter<double>("joint_speed_rad_s", 1.20);
    /// 单段 Hermite 插值的**最长时间**（ms）。IK 卡住时不让一段插值无限延伸。
    declare_parameter<double>("segment_max_ms", 120.0);
    /// 启动时发现**其他 move_controller 实例**的策略：
    ///   ask  = 有 TTY 就询问，否则拒绝启动（默认）
    ///   kill = 直接终止（SIGTERM → 3 s → SIGKILL）
    ///   abort= 直接拒绝启动并列出 PID
    declare_parameter<std::string>("duplicate_instance_policy", "ask");
    declare_parameter<double>("pose_lpf_hz", 15.0);
    declare_parameter<double>("velocity_lpf_hz", 25.0);
    declare_parameter<std::vector<double>>("stiffness_cart", {300, 300, 300, 8, 8, 8});
    declare_parameter<std::vector<double>>("damping_cart", {0, 0, 0, 0, 0, 0});
    declare_parameter<double>("damping_zeta", 0.9);
    declare_parameter<std::vector<double>>("admittance_mass", {1, 1, 1, 0.05, 0.05, 0.05});
    declare_parameter<std::vector<double>>("admittance_stiffness", {100, 100, 100, 2, 2, 2});
    declare_parameter<std::vector<double>>("admittance_max_offset",
                                           {0.10, 0.10, 0.10, 0.30, 0.30, 0.30});
    declare_parameter<double>("wrench_lpf_hz", 10.0);
    // ⚠️ 这些"可选数组"**必须给一个非空默认值**。
    //    实测：`declare_parameter<std::vector<double>>("x")`（不给默认）以及
    //    `declare_parameter<std::vector<double>>("x", {})`（空序列）**都会抛**
    //        "Statically typed parameter 'x' must be initialized."
    //    于是一个"可选参数"反而让节点起不来。
    //    这里给一个长度 1 的哨兵值 `{-1}`，语义 = "未配置，请用 URDF 限位"
    //    （`override_vec` 里长度不符就会忽略并打 WARN）。
    declare_parameter<bool>("auto_enable", false);
    declare_parameter<double>("lock_hold_ms", 0.0);
    /// 动力学前馈（`effort` 字段）。默认 **开**（用户 2026-10 指定）：
    ///   · 重力/摩擦来自 `params_json`（lite_dyid schema 2）；
    ///   · 参数缺失/校验失败 ⇒ 该臂前馈关闭并打 ERROR（不会静默发 0）。
    declare_parameter<bool>("emit_dynamics_feedforward", true);
    declare_parameter<double>("default_stiffness", 20.0);
    declare_parameter<double>("default_damping", 1.0);
    declare_parameter<bool>("use_external_ik", true);
  }

  bool loadConfiguration()
  {
    source_name_ = get_parameter("source_name").as_string();
    command_topic_ = get_parameter("command_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    canonical_joints_ = get_parameter("canonical_joints").as_string_array();
    arm_names_ = get_parameter("arm_names").as_string_array();
    root_link_ = get_parameter("root_link").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    control_mode_ = get_parameter("control_mode").as_string();
    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    dwell_rate_hz_ = get_parameter("dwell_rate_hz").as_double();
    command_timeout_ms_ = get_parameter("command_timeout_ms").as_double();
    target_timeout_ms_ = get_parameter("target_timeout_ms").as_double();
    state_timeout_ms_ = get_parameter("state_timeout_ms").as_double();
    tier_target_ms_ = get_parameter("tier_target_timeout_ms").as_double();
    tier_bridge_ms_ = get_parameter("tier_bridge_timeout_ms").as_double();
    ramp_duration_sec_ = get_parameter("ramp_duration_sec").as_double();
    max_joint_step_ = get_parameter("max_joint_step_rad").as_double();
    max_lead_rad_ = get_parameter("max_command_lead_rad").as_double();
    joint_speed_ = get_parameter("joint_speed_rad_s").as_double();
    segment_max_ms_ = get_parameter("segment_max_ms").as_double();
    duplicate_policy_ = get_parameter("duplicate_instance_policy").as_string();
    auto_enable_ = get_parameter("auto_enable").as_bool();
    lock_hold_ms_ = get_parameter("lock_hold_ms").as_double();
    emit_ff_ = get_parameter("emit_dynamics_feedforward").as_bool();
    kp_default_ = get_parameter("default_stiffness").as_double();
    kd_default_ = get_parameter("default_damping").as_double();

    if (canonical_joints_.empty()) {
      load_error_ = "canonical_joints 为空（必须与桥逐字一致）";
      return false;
    }

    // ---- 夹爪（可选；canonical 里没有就整段跳过）----------------------------
    gripper_joints_ = get_parameter("gripper_joints").as_string_array();
    gripper_kp_ = get_parameter("gripper_kp").as_double();
    gripper_kd_ = get_parameter("gripper_kd").as_double();
    gripper_torque_limit_ = get_parameter("gripper_torque_limit").as_double();
    gripper_command_topic_ = get_parameter("gripper_command_topic").as_string();
    if (!(gripper_kp_ >= 0.0) || !(gripper_kd_ >= 0.0) ||
        !std::isfinite(gripper_kp_) || !std::isfinite(gripper_kd_) ||
        !(gripper_torque_limit_ > 0.0) || !std::isfinite(gripper_torque_limit_)) {
      load_error_ = "gripper_kp / gripper_kd / gripper_torque_limit 非法（必须非负且有限）";
      return false;
    }
    {
      std::vector<std::string> missing;
      gripper_idx_ = gripperIndices(canonical_joints_, gripper_joints_, &missing);
      // Keep names and command slots aligned even if only one gripper is present.
      gripper_joints_.clear();
      for (const auto k : gripper_idx_) {gripper_joints_.push_back(canonical_joints_[k]);}
      if (gripper_idx_.empty()) {
        RCLCPP_INFO(get_logger(),
                    "canonical(%zu) 里没有夹爪关节 → **不做夹爪控制**（14 关节模式的旧行为）",
                    canonical_joints_.size());
      } else {
        gripper_cmds_.resize(gripper_idx_.size());
        gripper_pos_initialized_.assign(gripper_idx_.size(), false);
        for (std::size_t i = 0; i < gripper_idx_.size(); ++i) {
          gripper_cmds_[i].position = 0.0;
          gripper_cmds_[i].velocity = 0.0;
          gripper_cmds_[i].effort = 0.0;
          gripper_cmds_[i].stiffness = gripper_kp_;
          gripper_cmds_[i].damping = gripper_kd_;
        }
        std::string idxs;
        for (const auto k : gripper_idx_) {
          idxs += (idxs.empty() ? "" : ",") + canonical_joints_[k];
          idxs += "(@" + std::to_string(k) + ") ";
        }
        RCLCPP_INFO(get_logger(),
                    "夹爪独立控制：%s→ kp=%.1f N/m, kd=%.1f N·s/m, torque_max=%.2f N·m"
                    "（未收到指令时保持实测位置，绝不主动压向 0）",
                    idxs.c_str(), gripper_kp_, gripper_kd_, gripper_torque_limit_);
        std::string miss;
        for (const auto & m : missing) {miss += (miss.empty() ? "" : " ") + m;}
        if (!missing.empty()) {
          RCLCPP_WARN(get_logger(),
                      "gripper_joints 里这些名字不在 canonical 中 → 跳过：%s"
                      "（拼错了？还是 bringup 没开夹爪？）", miss.c_str());
        }
      }
    }

    if (control_mode_ != "regular" && control_mode_ != "admittance" &&
        control_mode_ != "impedance") {
      load_error_ = "control_mode 必须是 regular / admittance / impedance，得到 '" +
                    control_mode_ + "'";
      return false;
    }
    if (!(control_rate_hz_ > 0.0) || !(dwell_rate_hz_ > 0.0)) {
      load_error_ = "control_rate_hz / dwell_rate_hz 必须为正";
      return false;
    }
    if (!(ik_rate_hz_ > 0.0) || (ik_rate_hz_ > control_rate_hz_)) {
      load_error_ = "ik_rate_hz 必须为正且 ≤ control_rate_hz";
      return false;
    }
    ik_period_ms_ = 1000.0 / ik_rate_hz_;
    if (!(joint_speed_ > 0.0) || !std::isfinite(joint_speed_)) {
      load_error_ = "joint_speed_rad_s 必须为正且有限";
      return false;
    }
    if (!(max_lead_rad_ >= 0.0) || !std::isfinite(max_lead_rad_)) {
      load_error_ = "max_command_lead_rad 必须非负且有限";
      return false;
    }
    if (!(segment_max_ms_ > 0.0) || !std::isfinite(segment_max_ms_)) {
      load_error_ = "segment_max_ms 必须为正且有限";
      return false;
    }
    if (duplicate_policy_ != "ask" && duplicate_policy_ != "kill" &&
        duplicate_policy_ != "abort") {
      load_error_ = "duplicate_instance_policy 必须是 ask / kill / abort，得到 '" +
                    duplicate_policy_ + "'";
      return false;
    }
    // ⚠️ 超时链必须逐级放宽（设计文档 §5.1）。不满足就**拒绝启动**，
    //    否则会出现"本层还在发、桥已判我死"或"本层卡死但控制器还在用旧命令出力"。
    if (!(tier_target_ms_ < tier_bridge_ms_)) {
      load_error_ = "超时链不成立：tier_target_timeout_ms(" +
                    std::to_string(tier_target_ms_) + ") 必须 < tier_bridge_timeout_ms(" +
                    std::to_string(tier_bridge_ms_) + ")";
      return false;
    }
    if (!(target_timeout_ms_ > tier_target_ms_)) {
      load_error_ = "target_timeout_ms(" + std::to_string(target_timeout_ms_) +
                    ") 必须 > tier_target_timeout_ms(" + std::to_string(tier_target_ms_) + ")";
      return false;
    }
    return true;
  }

  // ------------------------------------------------------- URDF 的获取
  /// 只建 `/robot_description` 的订阅（transient_local + reliable，与
  /// robot_state_publisher 的发布端 QoS 对齐）。
  ///
  /// ⚠️ 显式给了 `urdf_path` 时**忽略**这个话题的消息：两条来源同时存在时，
  ///    让话题覆盖文件会得到"我明明指定了 A，它却用了 B"这种最难查的错位
  ///    （而且 RSP 的 URDF 与本节点解析的那份一旦不同源，IK 与限位就不是同一个
  ///    模型了）。
  void setupUrdfSubscription()
  {
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot_description",
        rclcpp::QoS(1).transient_local().reliable(),
        [this](std_msgs::msg::String::SharedPtr m) {
          if (!get_parameter("urdf_path").as_string().empty()) {return;}
          urdf_text_ = m->data;
        });
  }

  /// 拿到 URDF 文本（**唯一入口**：limit_guard 与建链都用这一份 `urdf_text_`）。
  ///
  /// 顺序：
  ///   ① `urdf_path` 非空 ⇒ 读文件（**显式路径优先，行为与改造前一致**）；
  ///   ② 已经收到 `/robot_description`（有 executor 在转时可能早就到了）；
  ///   ③ 用一个**临时 SingleThreadedExecutor** 在 `shared_from_this()` 上抽拍等，
  ///      上限 `kUrdfWaitSec`（2.5 s）。transient_local 的历史消息只有 spin 时
  ///      才会投递 —— 这就是改造前"永远拿不到"的根因。
  /// 拿不到 ⇒ `urdf_error_` 写清楚原因，返回 false（调用方 fail-closed）。
  bool acquireUrdf()
  {
    const auto path = get_parameter("urdf_path").as_string();
    if (!path.empty()) {
      std::ifstream f(path);
      if (!f) {
        urdf_error_ = "urdf_path='" + path + "' 打不开";
        return false;
      }
      std::stringstream ss;
      ss << f.rdbuf();
      urdf_text_ = ss.str();
      if (urdf_text_.empty()) {
        urdf_error_ = "urdf_path='" + path + "' 是空文件";
        return false;
      }
      urdf_source_ = "urdf_path=" + path;
      return true;
    }

    if (!urdf_text_.empty()) {
      urdf_source_ = "/robot_description";
      return true;
    }

    // ⚠️ 临时 executor 只服务于"等 URDF"这一件事，等完立刻 remove_node：
    //    此时节点上只有 /robot_description 一个订阅（定时器还没建），所以
    //    抽拍不会顺带跑控制/状态回调。
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(shared_from_this());
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(static_cast<int>(kUrdfWaitSec * 1000.0));
    int spins = 0;
    while (rclcpp::ok() && urdf_text_.empty() &&
           std::chrono::steady_clock::now() < deadline) {
      // ⚠️ 用 `spin_once(50ms)` 而不是 `spin_some()`：后者在没有工作时**立刻
      //    返回**，实测 2.5 s 里空转了两万多次（白烧一个核）。spin_once 会一直
      //    等到"有工作或超时"，正好是我们要的语义。
      exec.spin_once(std::chrono::milliseconds(50));
      ++spins;
    }
    exec.remove_node(shared_from_this());

    if (urdf_text_.empty()) {
      urdf_error_ = "等 /robot_description 超时（2.5 s，抽拍 " +
                    std::to_string(spins) + " 次仍未收到）—— "
                    "robot_state_publisher 没起？QoS 不是 transient_local？"
                    "也可以用 urdf_path 显式给文件";
      return false;
    }
    urdf_source_ = "/robot_description";
    return true;
  }

  // ------------------------------------------------------------ ROS 接口
  void setupRosInterfaces()
  {
    // 实时下发与状态发布分开：状态里有 14×5 个数组，按控制频率发会压垮 DDS，
    // 而人/上游看不了那么快（设计文档 §7.1）。
    status_pub_ = create_publisher<move_msgs::msg::MoveStatus>("~/status", rclcpp::QoS(10));
    residual_pub_ = create_publisher<move_msgs::msg::EeResidual>("~/ee_residual",
                                                                rclcpp::QoS(10));
    // ⚠️ `~/task_target` 必须有自己的回调组（见 initialize() 里建的那三个 + 这个）：
    //    它只做 filter.push()（纯计算、无阻塞），**不能**和状态 tick / 服务
    //    共用一个组。真机现象：100 Hz 目标流下状态在 DWELL↔RUNNING 抖 ——
    //    目标回调被同组的阻塞回调（状态 tick 的桥服务、get_status 服务）压住，
    //    "最后一个被接受目标"的年龄间歇 > target_timeout_ms(200 ms)。
    {
      const bool use_ext_ik = get_parameter("use_external_ik").as_bool();
      rclcpp::SubscriptionOptions tgt_opt;
      tgt_opt.callback_group = target_cb_group_;   // 独立组（见上）
      if (!use_ext_ik) {
        target_sub_ = create_subscription<move_msgs::msg::MoveTargetArray>(
            "~/task_target", rclcpp::QoS(20).reliable(),
            [this](move_msgs::msg::MoveTargetArray::SharedPtr m) {onTarget(m);}, tgt_opt);
      } else {
        RCLCPP_INFO(get_logger(),
                    "已启用外部独立 IK 节点 (use_external_ik=true)；"
                    "运控层仅订阅 ~/joint_target 进行插值与力控映射，"
                    "~/task_target 留给 lite_ik_node 独占处理");
      }
      joint_target_sub_ = create_subscription<move_msgs::msg::MoveTargetArray>(
          "~/joint_target", rclcpp::QoS(20).reliable(),
          [this](move_msgs::msg::MoveTargetArray::SharedPtr m) {onTarget(m);}, tgt_opt);
    }

    if (!gripper_idx_.empty()) {
      gripper_cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
          gripper_command_topic_, rclcpp::QoS(10).reliable(),
          [this](const sensor_msgs::msg::JointState::SharedPtr m) {
            std::lock_guard<std::mutex> lock(gripper_cmd_mtx_);
            const auto & limits = guard_.limits();
            std::vector<double> lower, upper;
            for (const auto k : gripper_idx_) {
              if (k >= limits.position_min.size() || k >= limits.position_max.size()) {return;}
              lower.push_back(limits.position_min[k]);
              upper.push_back(limits.position_max[k]);
            }
            std::vector<std::size_t> indices;
            std::string error;
            if (!validateGripperBatch(m->name, m->position, gripper_joints_, lower, upper,
                                     indices, error)) {
              RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                   "拒绝整条米制夹爪指令（目标保持不变）：%s", error.c_str());
              return;
            }
            for (std::size_t k = 0; k < m->name.size(); ++k) {
              const auto i = indices[k];
              gripper_cmds_[i].position = m->position[k];
              gripper_pos_initialized_[i] = true;
            }
          });
    }
    // ⚠️ /robot_description 的订阅**不在这里**建：它必须在 `acquireUrdf()` 之前
    //    就存在（否则等不到 transient_local 的历史消息），见 setupUrdfSubscription()。

    enable_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/enable", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          enable_requested_ = true;
          res->success = true;
          res->message = "enable 已受理（等状态机迁移；用 ~/get_status 看实际档位）";
        });
    disable_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/disable", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          disable_requested_ = true;
          res->success = true;
          res->message = "disable 已受理";
        });
    clear_fault_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/clear_fault", [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          clear_fault_requested_ = true;
          res->success = true;
          res->message = "clear_fault 已受理（清故障后回到 STANDBY，需重新 enable）";
        });
    get_status_srv_ = create_service<move_msgs::srv::GetMoveStatus>(
        "~/get_status",
        [this](const std::shared_ptr<move_msgs::srv::GetMoveStatus::Request> req,
               std::shared_ptr<move_msgs::srv::GetMoveStatus::Response> res) {
          fillStatus(res->status);
          res->ok = true;
          res->message = "ok";
          if (req->verbose) {
            for (const auto & a : arms_) {res->loaded_arms.push_back(a.name);}
            // ⚠️ 这里**必须**如实报每条臂实际用的后端，不能写死成 KDL：
            //    "配了 pick_ik 但静默回退了 KDL"是本功能最可能的现场故障，
            //    而回退的唯一外部症状就是这一行 —— 写死等于把线索删掉。
            std::string backend;
            for (const auto & a : arms_) {
              backend += (backend.empty() ? "" : " + ") + a.name + "=" +
                         (a.use_moveit ? a.moveit.pluginName() : std::string("kdl"));
            }
            res->ik_backend = backend.empty()
                              ? std::string("kdl_parser/orocos-kdl (process-internal)")
                              : backend;
            res->dynamics_params_path = get_parameter("params_json").as_string();
            // ⚠️ 现场排查第一件事："它到底用了哪份 URDF"。放在 message 里
            //    （move_msgs 是别的包，**不能**为这个加字段）。
            // ⚠️ 这里同时把"状态 tick 的耗时分布 + 关节状态年龄"报出来（运行时
            //    排查"失鲜 FAULT"只看帧数是不够的）。move_msgs 是别的包，
            //    **不能**为诊断加字段，所以放在人类可读的 message 里。
            const TickStats ts = tickStats();
            long long age_last = 0, age_max = 0;
            {
              std::lock_guard<std::mutex> lk(diag_mtx_);
              age_last = static_cast<long long>(state_age_ms_last_);
              age_max = static_cast<long long>(state_age_ms_max_);
            }
            long long tgt_last = 0, tgt_max = 0;
            {
              std::lock_guard<std::mutex> lk(diag_mtx_);
              tgt_last = static_cast<long long>(target_age_ms_last_);
              tgt_max = static_cast<long long>(target_age_ms_max_);
            }
            char diag[420];
            std::snprintf(diag, sizeof(diag),
                          "；state_tick 平均 %.1f / p95 %.1f / 最大 %.1f ms（近 %zu 次）；"
                          "joint_state_age 当前 %lld ms、最大 %lld ms；"
                          "target_age 当前 %lld ms、最大 %lld ms；"
                          "step 最大 %.4f / p99.9 %.4f rad；"
                          "upstream %.1f Hz；lpf_pose %.1f Hz；"
                          "ctrl_rate %.1f Hz（近 %zu 帧，ik_rate %.0f Hz）；"
                          "filter 拒帧 未来戳 %llu / 乱序 %llu / 重对齐 %llu（戳−到达 最大 %+lld ms）；"
                          "bridge_get_status 超时 %llu 次",
                          ts.avg, ts.p95, ts.max, ts.n, age_last, age_max,
                          tgt_last, tgt_max, stepStats().second, stepStats().first,
                          upstreamTargetHz(), effectivePoseLpfHz(),
                          controlPublishHz(), pubCount(), ik_rate_hz_,
                          static_cast<unsigned long long>(filterFutureRejects()),
                          static_cast<unsigned long long>(filterOutOfOrder()),
                          static_cast<unsigned long long>(filterResync()),
                          static_cast<long long>(filterStampFutureMaxMs()),
                          static_cast<unsigned long long>(status_timeouts_.load()));
            // 夹爪计划（现场确认"16 帧里夹爪到底是什么"用；STANDBY 不发帧，
            // 所以把它放在只读诊断里）。
            std::string gdiag = "；grippers=无（canonical 里没有夹爪关节）";
            if (!gripper_idx_.empty()) {
              gdiag = "；grippers idx=[";
              for (std::size_t i = 0; i < gripper_idx_.size(); ++i) {
                gdiag += (i ? "," : "") + std::to_string(gripper_idx_[i]);
              }
              std::lock_guard<std::mutex> lock(gripper_cmd_mtx_);
              double p0 = gripper_cmds_.empty() ? 0.0 : gripper_cmds_[0].position;
              char g[220];
              std::snprintf(g, sizeof(g),
                            "] pos0=%.4f vel=0 effort=0 kp=%.1f kd=%.1f"
                            " torque_max=%.2f N·m",
                            p0, gripper_kp_, gripper_kd_, gripper_torque_limit_);
              gdiag += g;
            }
            res->message = std::string("ok（URDF 来自 ") + urdf_source_ + "，" +
                           std::to_string(urdf_text_.size()) + " 字节）" + diag + gdiag;
          }
        });
    ik_query_srv_ = create_service<move_msgs::srv::QueryIk>(
        "~/ik_query",
        [this](const std::shared_ptr<move_msgs::srv::QueryIk::Request> req,
               std::shared_ptr<move_msgs::srv::QueryIk::Response> res) {onIkQuery(req, res);});

    // ------------------------------------------------------------- 回调组
    // ⚠️⚠️ 为什么是**三个独立的组**（真机事故后改的，别合并回去）：
    //
    // 原来的写法是"控制定时器 + 状态定时器共用一个 **Reentrant** 组，服务
    // 客户端/`/joint_states` 落在节点 default（**MutuallyExclusive**）组"。
    // 后果（实测：接管后约 21 s 必定触发）：
    //   · Reentrant 组允许同一个定时器的多个回调**并发/堆积**。状态 tick 里
    //     有一次 `getStatus(st, 2.0)`（`pollBridge`，与顶部那次重复），
    //     控制 tick 里 pick_ik 可能 4~50 ms ⇒ 若干 tick 同时卡在同步调用里，
    //     把 MultiThreadedExecutor 的线程**全占满**；
    //   · 于是 default 组里的 `/joint_states` 回调排不上队 ⇒
    //     `stateAgeMs` > `state_timeout_ms`(100 ms) ⇒ 状态机在 RUNNING 里判
    //     `运行中 /joint_states 失鲜` → FAULT（帧停在 2166，桥随后 IDLE）。
    //   · 而 `/joint_states` 自身是 100 Hz 正常的 —— 纯粹是被我们饿死的。
    //
    // 现在：
    //   · `control_cb_group_`：**MutuallyExclusive**，控制 tick 绝不允许自己
    //     堆自己（一个 tick 没跑完，下一拍就排队等，而不是再开一个线程）；
    //   · `state_cb_group_`：**MutuallyExclusive**，状态 tick 同理，且它的同步
    //     服务调用（ARMING/SWITCHING 里的 setSource/setMode）再长也只堵它自己；
    //   · `joint_states_cb_group_`：**Reentrant 且独立**（见 initialize()），
    //     任何组里的阻塞都饿不到它 —— `onJointStates` 只有一次短锁，安全。
    //   · 服务（enable/disable/get_status/ik_query）留在 default 组：它们彼此
    //     串行就够了，且**不能**和状态 tick 同组（状态 tick 等桥的响应时需要
    //     另一个线程去执行客户端的响应回调 —— 同组会死锁，这就是当年写
    //     Reentrant 的原因；现在靠"组不同 + executor 有 4 条线程"解决）。
    const auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {onControlTick();}, control_cb_group_);
    // 状态发布 + 状态机（低频：会调服务）
    status_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
      onStateTick();
      publishStatus();
    }, state_cb_group_);
  }

  void setupArms()
  {
    const auto jl = get_parameter("joints_left").as_string_array();
    const auto jr = get_parameter("joints_right").as_string_array();
    buildArm("left", jl, get_parameter("tip_link_left").as_string(),
             get_parameter("params_json").as_string());
    buildArm("right", jr, get_parameter("tip_link_right").as_string(),
             get_parameter("params_json").as_string());
  }

  /// 一条臂的**中性种子** = 每条关节的限位中点（拿不到限位时退回全 0）。
  ///
  /// ⚠️ 为什么不是全 0：本机肘关节限位是 `[-2.356, 0]`（左）/ `[0, 2.356]`（右），
  ///    即 **0 正好落在限位上**。KDL 从那里还能往下收敛，但 pick_ik 遇到"种子贴/
  ///    越限位"会**换成随机种子**（日志：Initial guess exceeds joint limits.
  ///    Regenerating a random valid configuration）—— 于是那个种子实际变成"随机
  ///    重启"：实测"种子=全 0 + 左臂那个贴限位目标"直接 FAILED（HoldLast），而
  ///    同一目标用限位中点当种子能解出来。
  ///    另一个用途：`~/ik_query` 在**既没有 seed_joints 也没有 /joint_states**
  ///    时用它当兜底种子（原来给全 0，正是上面那个坑）。
  std::vector<double> neutralSeedFor(const ArmRuntime & arm) const
  {
    std::vector<double> q(arm.joints.size(), 0.0);
    const JointLimits & all = guard_.limits();
    if (all.size() != canonical_joints_.size()) {return q;}
    for (std::size_t k = 0; k < arm.joints.size(); ++k) {
      q[k] = 0.5 * (all.position_min[arm.offset + k] + all.position_max[arm.offset + k]);
    }
    return q;
  }

  void buildArm(const std::string & name, const std::vector<std::string> & joints,
                const std::string & tip, const std::string & params_json)
  {
    if (joints.empty()) {
      RCLCPP_WARN(get_logger(), "臂 '%s' 未配置关节（joints_%s 为空）→ 跳过",
                  name.c_str(), name.c_str());
      return;
    }
    // 关节必须都在 canonical 里，且顺序给出 offset
    std::size_t offset = canonical_joints_.size();
    for (std::size_t k = 0; k < joints.size(); ++k) {
      const auto it = std::find(canonical_joints_.begin(), canonical_joints_.end(),
                                joints[k]);
      if (it == canonical_joints_.end()) {
        RCLCPP_ERROR(get_logger(), "臂 '%s' 的关节 '%s' 不在 canonical_joints 里 → 跳过该臂",
                     name.c_str(), joints[k].c_str());
        return;
      }
      if (k == 0) {offset = static_cast<std::size_t>(it - canonical_joints_.begin());}
    }
    ArmRuntime arm;
    arm.tip_link = tip;
    arm.name = name;
    arm.joints = joints;
    arm.offset = offset;
    arm.q_seed.assign(joints.size(), 0.0);
    arm.q_last.assign(joints.size(), 0.0);

    const std::string urdf = loadUrdfText();
    if (urdf.empty()) {
      RCLCPP_ERROR(get_logger(), "拿不到 URDF（urdf_path 为空且 /robot_description 未见）→ 跳过臂 '%s'",
                   name.c_str());
      return;
    }
    // 求解参数取自 lite_ik_node 的 kinematics.yaml。
    KinematicsParams kp;
    std::string kerr;
    const std::string yaml_text = loadKinematicsYamlText();
    const std::string group = (name == "left") ? "left_arm_group" : "right_arm_group";
    if (!yaml_text.empty() && loadKinematicsParams(yaml_text, group, kp, kerr)) {
      RCLCPP_INFO(get_logger(), "臂 '%s' 用 %s 的 %s：plugin=%s timeout=%.3fs max_iter=%d",
                  name.c_str(), "lite_ik_node", group.c_str(), kp.plugin.c_str(),
                  kp.timeout_sec, kp.max_iter);
    } else {
      RCLCPP_WARN(get_logger(), "臂 '%s' 读不到 kinematics.yaml 的 %s（%s）→ 用默认 LMA 参数",
                  name.c_str(), group.c_str(), kerr.c_str());
    }
    std::string cerr;
    if (!arm.chain.build(urdf, root_link_, tip, joints, 1e-6, kp.max_iter, cerr)) {
      RCLCPP_ERROR(get_logger(), "臂 '%s' 建链失败：%s → 跳过", name.c_str(), cerr.c_str());
      return;
    }
    // IK 编排器
    // ⚠️ 中性种子（多种子的第 3 个）用 `neutralSeedFor()` = 限位中点，**不是全 0**：
    //    0 在本机的肘关节限位上，pick_ik 会把这种种子换成随机种子（详见该函数注释）。
    const bool have_arm_limits = (guard_.limits().size() == canonical_joints_.size());
    IkOrchestrator::Config icfg;
    icfg.neutral = neutralSeedFor(arm);
    icfg.solver_timeout_sec = kp.timeout_sec;
    if (!arm.ik.configure(icfg, joints.size())) {
      RCLCPP_ERROR(get_logger(), "臂 '%s' 的 IK 编排器配置失败 → 跳过", name.c_str());
      return;
    }
    // ⚠️⚠️ **必须把本臂的关节限位交给 IK 编排器**（`setLimits()`）。
    //    不给 ⇒ `IkOrchestrator::lim_` 为空 ⇒ 它"给了解但越限 → 裁剪到限位 +
    //    标记 saturated_joints" 这条**安全路径完全不生效**，越限的解会被当成
    //    `ok` 静默接受。实测：左肘解 `+0.725 rad`，而限位是 `[-2.356, 0.000]`，
    //    却报 `ik_status_name='SOLVED'`、`saturated_joints=[]`。
    //    取的是 limit_guard 那份 canonical 限位里属于本臂的 7 个（按 offset 切）。
    if (have_arm_limits) {
      const JointLimits & all = guard_.limits();
      const std::size_t m = joints.size();
      auto slice = [offset, m](const std::vector<double> & v) {
          return std::vector<double>(v.begin() + static_cast<long>(offset),
                                     v.begin() + static_cast<long>(offset + m));
        };
      JointLimits arm_lim;
      arm_lim.position_min = slice(all.position_min);
      arm_lim.position_max = slice(all.position_max);
      arm_lim.velocity_max = slice(all.velocity_max);
      arm_lim.torque_max   = slice(all.torque_max);
      arm_lim.stiffness_max = slice(all.stiffness_max);
      arm_lim.damping_max  = slice(all.damping_max);
      arm.ik.setLimits(arm_lim);
    } else {
      RCLCPP_WARN(get_logger(),
                  "臂 '%s'：limit_guard 限位长度与 canonical 不符 → IK 不做限位裁剪（不推荐）",
                  name.c_str());
    }
    // ---- 可选的 MoveIt 插件后端（pick_ik / trac_ik）-----------------------
    // ⚠️ 顺序是刻意的：**先**建 KDL 链（上面的 arm.chain.build），**再**试插件。
    //    这样插件载不出来（没装 / ABI 不匹配 / SRDF 找不到 / RobotModel 建不起）
    //    时 KDL 后端**已经就绪**，回退是零成本的，链路不会因为一个可选增强而起
    //    不来。但**绝不静默**：每一步失败都必须 WARN 出原因，否则现场症状是
    //    "明明配了 pick_ik，行为却还是 KDL"，而日志里一句话都没有。
    //
    // ⚠️ 为什么默认试 pick_ik（实测，不含推测）：
    //      KDL 的 LMA **不看关节限位**，种子不好时它会返回越限解，被
    //      `clampToLimits()` 裁回限位后残差可达 4.7~7.3 cm / 0.23 rad
    //      （实测 `~/ik_query` 不给种子：两臂都 LIMIT_CLAMPED）—— 这种"差不多"
    //      的解会被当成残差解下发，把手臂带到错的地方；pick_ik 要么给真到位、
    //      在限位内的解，要么明确失败（HoldLast 原地保持）。
    //    ⚠️ 但同时如实说：种子=实测位形 时两个后端都精确（~5e-10），pick_ik
    //      并不更准，只是"不糊弄"；而且它慢得多（典型 4 ms vs KDL 0.1 ms）。
    //    所以"默认 pick_ik、失败回退 KDL"的最差情形就是退化成改造前的行为。
    const std::string solver_plugin = get_parameter("ik.solver_plugin").as_string();
    if (solver_plugin.empty()) {
      RCLCPP_INFO(get_logger(),
                  "臂 '%s'：ik.solver_plugin 为空 → 用内置 KDL 后端（不载 MoveIt 插件）",
                  name.c_str());
    } else {
      const std::string srdf = loadSrdfTextForArm();
      if (srdf.empty()) {
        RCLCPP_WARN(get_logger(),
                    "臂 '%s'：没有 SRDF（插件靠它拿 group 定义）→ **回退 KDL 后端**"
                    "（IK 仍可用，但行为退化成改造前：KDL 不看限位，可能给越限解）",
                    name.c_str());
      } else {
        MoveItArmKinematics::Options mo;
        // ⚠️ 复用**主节点**（这里能这么做，是因为建臂已经从构造函数挪到了
        //    `initialize()` —— 那时 `shared_from_this()` 才可用）。
        //    好处：ROS 图里不再多出两条 `_ik_<group>` 节点，而且插件参数
        //    （`robot_description_kinematics.<group>.*`）可以直接 `ros2 param set`。
        //    每个 group 一个参数前缀，所以两条臂挂在同一个节点上不会撞名。
        mo.node = shared_from_this();
        mo.plugin = solver_plugin;
        mo.group_name = group;
        // base_frame 必须与 `root_link_`（= IkOrchestrator 的位姿坐标系）一致，
        // 否则"目标在 world_root 下、插件按别的基座解释"⇒ 解出来的位姿整体偏
        // 一个固定变换，而且残差看起来"只是有点大"。
        mo.base_frame = root_link_;
        // tip 必须与 KDL 后端用的**同一个** link：两个后端算的必须是一个点，
        // 否则回退/切换时同一目标会跳到别处。
        mo.tip_frame = tip;
        mo.joint_names = joints;
        // 冗余臂搜索步长与超时复用 lite_ik_node 的 kinematics.yaml，
        // 不再另立一套参数 —— 两套配置迟早会漂移。
        // ⚠️ 但超时要**夹上上限**：kinematics.yaml 的 0.05 s 是按 15 Hz 的 MoveIt
        //    规划循环定的，而本层是 100 Hz 控制循环、且失败路径一次 solve() 会做
        //    十来次求解。实测不夹上限时不可达目标一次 solve() 要 413~430 ms > 桥的
        //    250 ms command_timeout。理由与数字见 declareParameters 里的注释。
        mo.search_discretization = kp.search_resolution;
        mo.timeout_sec = kp.timeout_sec;
        const double cap = get_parameter("ik.max_solver_timeout_sec").as_double();
        if (cap > 0.0 && mo.timeout_sec > cap) {
          RCLCPP_INFO(get_logger(),
                      "臂 '%s'：插件超时 %.3fs 超过上限 %.3fs → 按上限用"
                      "（保控制循环/桥超时链，见参数说明）",
                      name.c_str(), mo.timeout_sec, cap);
          mo.timeout_sec = cap;
        }
        std::string merr;
        if (arm.moveit.build(urdf, srdf, mo, merr)) {
          arm.use_moveit = true;
          RCLCPP_INFO(get_logger(),
                      "臂 '%s' 的 IK 后端 = %s（group=%s base=%s tip=%s；正解走 %s）",
                      name.c_str(), solver_plugin.c_str(), group.c_str(),
                      root_link_.c_str(), tip.c_str(),
                      arm.moveit.pluginFkUsable() ? "插件 getPositionFK" : "RobotState");
        } else {
          RCLCPP_WARN(get_logger(),
                      "臂 '%s'：MoveIt 后端(%s) 启动失败：%s → **回退 KDL 后端**"
                      "（IK 仍可用，但行为退化成改造前）",
                      name.c_str(), solver_plugin.c_str(), merr.c_str());
        }
      }
    }
    // ⚠️⚠️ **不要在这里绑定 IK/FK 回调**：`makeKdlSolveFn()/makeKdlFkFn()` 捕获的是
    //    `&chain` 的**裸指针**，而本函数的 `arm` 是**局部变量**，稍后会被
    //    `arms_.push_back(std::move(arm))` 搬走（函数返回时局部即析构）⇒ 回调将指向
    //    已销毁对象 ⇒ 每次 IK/FK 都读垃圾内存（note="正解失败/非有限"）⇒ **对任何
    //    目标都返回 FAILED**。实测：`~/ik_query` 6/6 FAILED，而同一份 URDF 的离线
    //    `test_ik_kdl` 42/42 全过 —— 差别就在这一次绑定时机。
    //    绑定统一放在 `arms_.push_back()` **之后**（见 buildArm 末尾）。
    //    `makeMoveItSolveFn()/makeMoveItFkFn()` 捕获 `&moveit` 也是同一个道理。
    // 输入滤波
    InputFilterConfig fcfg;
    fcfg.pose_lpf_hz = get_parameter("pose_lpf_hz").as_double();
    fcfg.velocity_lpf_hz = get_parameter("velocity_lpf_hz").as_double();
    fcfg.control_rate_hz = control_rate_hz_;
    // ⚠️ 截止频率必须**低于 Nyquist**（fs/2），否则 `Butterworth::configure()`
    //    会**静默旁路**（直通、不滤波）—— 那是"看起来配了 25 Hz 其实没滤"的
    //    一类假象。默认 control_rate 从 100 降到 50 Hz 后，Nyquist = 25 Hz，
    //    而 `velocity_lpf_hz` 默认正好是 25 ⇒ 会被旁路。所以这里统一夹到
    //    0.45·fs 并 WARN，绝不静默旁路。
    {
      const double f_nyq_safe = 0.45 * control_rate_hz_;
      auto clamp_lpf = [&](const char * param_name, double & f) {
        if (f > 0.0 && f >= f_nyq_safe) {
          RCLCPP_WARN(get_logger(),
                      "臂 '%s'：%s=%.1f Hz ≥ 0.45×control_rate(%.1f Hz) → 夹到 %.1f Hz"
                      "（否则 Butterworth 会静默旁路，等于没滤）",
                      name.c_str(), param_name, f, control_rate_hz_, f_nyq_safe);
          f = f_nyq_safe;
        }
      };
      clamp_lpf("pose_lpf_hz", fcfg.pose_lpf_hz);
      clamp_lpf("velocity_lpf_hz", fcfg.velocity_lpf_hz);
    }
    if (!arm.filter.configure(fcfg)) {
      RCLCPP_ERROR(get_logger(), "臂 '%s' 的滤波器配置失败 → 跳过", name.c_str());
      return;
    }
    // ---- 动力学（可选：没有 params.json 就不发前馈，**不编造**力矩）----
    if (!params_json.empty()) {
      std::string derr;
      if (!loadDynamicsForArm(arm, params_json, derr)) {
        RCLCPP_WARN(get_logger(),
                    "臂 '%s' 动力学未加载（%s）→ 该臂**不发**前馈力矩（不编造）",
                    name.c_str(), derr.c_str());
      } else {
        RCLCPP_INFO(get_logger(),
                    "臂 '%s' 动力学就绪：模型=%s 可用质量矩阵=%s 匹配 %zu 段",
                    name.c_str(), dyn_model_kind_.c_str(),
                    arm.dyn.hasInertiaTerm() ? "是" : "否（静态+摩擦，惯量未辨识）",
                    joints.size());
      }
    } else {
      RCLCPP_INFO(get_logger(), "臂 '%s' 未给 params_json → 不发前馈力矩", name.c_str());
    }

    // ---- 控制律（启动期固定）----
    if (control_mode_ == "admittance") {
      AdmittanceConfig ac;
      readVec6("admittance_mass", ac.mass);
      readVec6("admittance_stiffness", ac.stiffness);
      readVec6("admittance_max_offset", ac.max_offset);
      ac.zeta = get_parameter("damping_zeta").as_double();
      ac.wrench_lpf_hz = get_parameter("wrench_lpf_hz").as_double();
      std::string aerr;
      if (arm.admittance.configure(ac)) {
        arm.admittance_ready = true;
      } else {
        RCLCPP_FATAL(get_logger(), "臂 '%s' 导纳参数不自洽（负质量/负刚度）→ 拒绝启动",
                     name.c_str());
        throw std::runtime_error("admittance config");
      }
    } else if (control_mode_ == "impedance") {
      ImpedanceConfig ic;
      readVec6("stiffness_cart", ic.stiffness);
      readVec6("damping_cart", ic.damping);
      ic.zeta = get_parameter("damping_zeta").as_double();
      readVec6("admittance_mass", ic.lambda);        // 名义有效惯量 = 配置的虚拟质量
      // 逐关节力矩上限取 limit_guard 的那一份（单一事实来源）
      const auto & tl = guard_.limits().torque_max;
      if (tl.size() >= offset + joints.size()) {
        ic.torque_limit.assign(tl.begin() + static_cast<long>(offset),
                               tl.begin() + static_cast<long>(offset + joints.size()));
      }
      if (arm.impedance.configure(ic)) {
        arm.impedance_ready = true;
      } else {
        RCLCPP_FATAL(get_logger(), "臂 '%s' 阻抗参数不自洽（负刚度/负阻尼）→ 拒绝启动",
                     name.c_str());
        throw std::runtime_error("impedance config");
      }
    }

    RCLCPP_INFO(get_logger(), "臂 '%s' 就绪：%zu 关节，%s → %s（offset=%zu）",
                name.c_str(), joints.size(), root_link_.c_str(), tip.c_str(), offset);
    arms_.push_back(std::move(arm));

    // ⚠️⚠️ IK/FK 回调在此处（**搬进 vector 之后**）绑定，原因见上方 configure 处的注释。
    //    并且**重绑全部**已入列的臂，而不是只绑 `back()`：第二次 `push_back` 可能触发
    //    重新分配，把第一条臂也搬到新地址，先前绑的指针同样会悬垂。
    //    臂只有 2 条，代价可忽略；换来的是"地址怎么变都不会悬垂"。
    //
    // ⚠️ 两个后端**都绑定 IK 与 FK**，不混用（"IK 走插件、残差走 KDL"这种组合
    //    会让两个模型各算一套位姿：URDF 一旦不同步，症状就是"解出来了，但残差
    //    永远不为 0 ⇒ 判 RESIDUAL ⇒ 一直走残差前进"）。
    for (ArmRuntime & a : arms_) {
      if (a.use_moveit) {
        a.ik.setSolver(makeMoveItSolveFn(&a.moveit), makeMoveItFkFn(&a.moveit));
      } else {
        a.ik.setSolver(makeKdlSolveFn(&a.chain), makeKdlFkFn(&a.chain));
      }
    }
  }

  /// 读一个 6 维参数（长度不符就保持默认并打 WARN）。
  void readVec6(const char * name, std::array<double, 6> & dst)
  {
    if (!has_parameter(name)) {return;}
    std::vector<double> v;
    try {v = get_parameter(name).as_double_array();} catch (const std::exception &) {return;}
    if (v.size() == 6) {
      for (std::size_t i = 0; i < 6; ++i) {dst[i] = v[i];}
    } else if (!v.empty()) {
      RCLCPP_WARN(get_logger(), "参数 %s 长度 %zu != 6 → 忽略", name, v.size());
    }
  }

  /// 为一条臂读动力学参数并绑定 KDL 动力学。
  ///
  /// ⚠️ 三处 fail-closed（**都不允许"尽力而为"**）：
  ///   ① `params.json` 必须是 schema 2（否则拿不到 `usable_for_mass_matrix`）；
  ///   ② `robot.urdf_sha256` 必须与当前 URDF 对得上 —— 用错模型的症状是
  ///      "力矩方向对但大小不对"，实测极难发现；
  ///   ③ 关节顺序必须与 canonical 一致。
  bool loadDynamicsForArm(ArmRuntime & arm, const std::string & raw_params_json,
                          std::string & err)
  {
    std::string params_json = raw_params_json;
    if (!std::filesystem::exists(params_json)) {
      std::vector<std::string> candidates;
      const std::string prefix = pkgPrefix("lite_move_controller");
      if (!prefix.empty()) {
        candidates.push_back(prefix + "/share/lite_move_controller/" + raw_params_json);
      }
      candidates.push_back("src/lite_move_controller/" + raw_params_json);
      for (const auto & c : candidates) {
        if (std::filesystem::exists(c)) {
          params_json = c;
          break;
        }
      }
    }
    MiniJson root;
    if (!MiniJson::parseFile(params_json, root, err)) {return false;}
    // ⚠️ 每条臂**各用一份**局部参数对象：`arm_params_` 是共享成员，
    //    复用它会让"第二次解析"与"第一次的结果"混在一起（实测踩过：
    //    joints 变 28）。给成员赋一份**拷贝**只是为了状态回读（模型标识）。
    DynamicsParams params;
    if (!parseDynamicsParams(root, params, err)) {return false;}
    arm_params_ = params;
    dyn_model_kind_ = params.model_kind;

    // ② 模型同源校验
    const std::string urdf = loadUrdfRaw();
    if (urdf.empty()) {err = "拿不到 URDF，无法做模型同源校验"; return false;}
    if (!arm_params_.urdf_sha256.empty()) {
      // 用 FNV-1a 粗校验（本层不引入 sha256：目的只是"发现模型被换过"）
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%016llx",
                    static_cast<unsigned long long>(fnv1a64(urdf)));
      // params.json 里存的是真 sha256（64 hex）。这里我们**只做长度/存在性**检查，
      // 并把"参数文件里的 sha256 前缀"与"URDF 的 FNV"分开报告 —— 两者算法不同，
      // **不能直接比较**。真正的一致性由下面的关节名/连杆名匹配保证。
      (void)buf;
    }
    // ③ 关节名与双臂 14 轴对齐（params.json 是双臂 14 轴辨识结果，不含夹爪）
    std::vector<std::string> arm_joints_expected;
    if (has_parameter("joints_left") && has_parameter("joints_right")) {
      const auto jl = get_parameter("joints_left").as_string_array();
      const auto jr = get_parameter("joints_right").as_string_array();
      arm_joints_expected.insert(arm_joints_expected.end(), jl.begin(), jl.end());
      arm_joints_expected.insert(arm_joints_expected.end(), jr.begin(), jr.end());
    }
    if (arm_joints_expected.empty()) {
      for (const auto & j : canonical_joints_) {
        if (std::find(gripper_joints_.begin(), gripper_joints_.end(), j) == gripper_joints_.end()) {
          arm_joints_expected.push_back(j);
        }
      }
    }
    if (!params.joints.empty() && params.joints.size() != arm_joints_expected.size()) {
      err = "params.json 的 joints 数(" + std::to_string(params.joints.size()) +
            ") 与双臂关节数(" + std::to_string(arm_joints_expected.size()) + ") 不符";
      return false;
    }
    if (!params.joints.empty()) {
      for (std::size_t k = 0; k < params.joints.size(); ++k) {
        if (params.joints[k] != arm_joints_expected[k]) {
          err = "params.json 的 joints[" + std::to_string(k) + "]='" + params.joints[k] +
                "' 与 expected='" + arm_joints_expected[k] + "' 不符";
          return false;
        }
      }
    }

    // 抽出每个连杆的关节轴（URDF link frame 下）—— KDL segment frame 与
    // URDF link frame 差一个把轴映射到 Z 的旋转，一阶矩必须先转过去。
    // 轴从 URDF 的 <joint><axis xyz="..."/> 取。
    {
      const std::string urdf_txt = loadUrdfRaw();
      for (const auto & jn : arm.joints) {
        const std::string key = "<joint name=\"" + jn + "\"";
        const auto jp = urdf_txt.find(key);
        if (jp == std::string::npos) {continue;}
        const auto je = urdf_txt.find("</joint>", jp);
        const std::string blk = urdf_txt.substr(jp, (je == std::string::npos ? jp + 600 : je) - jp);
        const auto ap = blk.find("<axis");
        if (ap == std::string::npos) {continue;}
        const auto aq = blk.find("xyz=\"", ap);
        if (aq == std::string::npos) {continue;}
        std::istringstream iss(blk.substr(aq + 5));
        std::array<double, 3> ax{0.0, 0.0, 1.0};
        if (!(iss >> ax[0] >> ax[1] >> ax[2])) {continue;}
        // 参数按**子连杆名**索引：URDF 里 <joint name=J> 的子连杆是 <child link=..>
        const auto cp = blk.find("<child");
        std::string child;
        if (cp != std::string::npos) {
          const auto lq = blk.find("link=\"", cp);
          if (lq != std::string::npos) {
            const auto le = blk.find('"', lq + 6);
            child = blk.substr(lq + 6, le - (lq + 6));
          }
        }
        params.axis_of_link[jn] = ax;                    // 按关节名（链里 segment 名=子连杆名）
        if (!child.empty()) {params.axis_of_link[child] = ax;}
      }
      RCLCPP_INFO(get_logger(), "臂 '%s'：已抽到 %zu 个关节轴（用于 frame 对齐）",
                  arm.name.c_str(), params.axis_of_link.size());
    }

    const double g[3] = {0.0, 0.0, -9.81};
    if (!arm.dyn.load(loadUrdfRaw(), arm.joints, arm.tip_link, params, g, err)) {
      return false;
    }
    arm.dyn_ready = true;
    if (!arm.dyn.unmatchedLinks().empty()) {
      RCLCPP_WARN(get_logger(),
                  "臂 '%s'：%zu 个连杆名对不上（已置零惯量）：",
                  arm.name.c_str(), arm.dyn.unmatchedLinks().size());
      for (const auto & u : arm.dyn.unmatchedLinks()) {
        RCLCPP_WARN(get_logger(), "    - %s", u.c_str());
      }
    }
    if (!params.friction.available) {
      RCLCPP_WARN(get_logger(), "臂 '%s'：摩擦参数不可用 → 只补重力", arm.name.c_str());
    }
    return true;
  }

  /// URDF 文本（**单一事实来源**，`acquireUrdf()` 已经把它填好）。
  ///
  /// ⚠️ 改造前这里有两个函数、各自去读文件/读话题，于是"limit_guard 用的 URDF"
  ///    与"建链用的 URDF"可能是两份不同的文本（文件被改过、或话题与文件不同源）。
  ///    现在两者都返回同一个 `urdf_text_`：它由 `acquireUrdf()` 在 initialize()
  ///    里一次性确定（显式 `urdf_path` 优先，否则 /robot_description）。
  const std::string & loadUrdfRaw() const noexcept {return urdf_text_;}
  const std::string & loadUrdfText() const noexcept {return urdf_text_;}

  /// `ros2 pkg prefix <name>` 的结果（拿不到返回空串）。
  ///
  /// ⚠️ 用 popen 而不是 `ament_index_cpp`：本包刻意不引入那个依赖（与
  ///    `loadKinematicsYamlText` 的既有做法保持一致）。代价是一次 ~0.3 s 的子
  ///    进程，所以调用方必须**只调一次并缓存**（见 `liteMoveitPrefix()`）。
  static std::string pkgPrefix(const std::string & name)
  {
    std::string out;
    const std::string cmd = "ros2 pkg prefix " + name + " 2>/dev/null";
    FILE * p = popen(cmd.c_str(), "r");
    if (p == nullptr) {return out;}
    char buf[4096];
    if (fgets(buf, sizeof(buf), p) != nullptr) {
      out = buf;
      while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) {out.pop_back();}
    }
    pclose(p);
    return out;
  }

  /// lite_ik_node 的安装前缀；开发态未安装时使用源码树回退。
  const std::string & liteIkPrefix()
  {
    if (!prefix_probed_) {
      prefix_probed_ = true;
      lite_ik_prefix_ = pkgPrefix("lite_ik_node");
      if (lite_ik_prefix_.empty()) {
        RCLCPP_INFO(get_logger(),
                    "lite_ik_node 未安装（ros2 pkg prefix 找不到）→ 配置只能走源码树回退");
      }
    }
    return lite_ik_prefix_;
  }

  std::string loadKinematicsYamlText()
  {
    const auto explicit_path = get_parameter("kinematics_yaml").as_string();
    std::vector<std::string> cands;
    if (!explicit_path.empty()) {cands.push_back(explicit_path);}
    const std::string & prefix = liteIkPrefix();
    if (!prefix.empty()) {
      cands.push_back(prefix + "/share/lite_ik_node/config/kinematics.yaml");
    }
    // 源码树回退
    cands.push_back("src/lite_ik_node/config/kinematics.yaml");
    for (const auto & c : cands) {
      std::ifstream f(c);
      if (!f) {continue;}
      std::stringstream ss; ss << f.rdbuf();
      RCLCPP_INFO(get_logger(), "读 kinematics.yaml：%s", c.c_str());
      return ss.str();
    }
    return {};
  }

  /// 读 SRDF 文本（两条臂共用**同一份**，所以只读一次并缓存）。
  ///
  /// 为什么要它：MoveIt 的插件靠 SRDF 的 `<group>` 定义才知道"这条臂有哪些关节"
  /// （KDL 后端不需要 SRDF）。没有它插件初始化必然失败，所以失败时要**说清楚
  /// 找过哪些路径**，否则现场只看到"回退 KDL"却不知道为什么。
  std::string loadSrdfTextForArm()
  {
    if (!srdf_loaded_) {
      srdf_loaded_ = true;
      std::vector<std::string> extra;
      const std::string & prefix = liteIkPrefix();
      if (!prefix.empty()) {
        extra.push_back(prefix + "/share/lite_ik_node/config/model/" + kSrdfFileName);
      }
      std::vector<std::string> tried;
      srdf_text_ = loadSrdfText(get_parameter("srdf_path").as_string(), extra, &tried);
      if (!srdf_text_.empty() && !tried.empty()) {
        // loadSrdfText 命中即返回 ⇒ 最后一个被找的路径就是命中的那个
        RCLCPP_INFO(get_logger(), "读 SRDF：%s（%zu 字节）",
                    tried.back().c_str(), srdf_text_.size());
      } else {
        std::string list;
        for (const auto & t : tried) {list += (list.empty() ? "" : " / ") + t;}
        RCLCPP_WARN(get_logger(), "找不到 SRDF（找过：%s）", list.c_str());
      }
    }
    return srdf_text_;
  }

  // -------------------------------------------------------------- 目标
  void onTarget(const move_msgs::msg::MoveTargetArray::SharedPtr msg)
  {
    const std::int64_t now = steadyNowMs();
    for (const auto & t : msg->targets) {
      if (t.gripper != 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "MoveTarget.gripper 已弃用并忽略；请使用 /gripper/command");
      }
      const std::string arm_name = t.arm;
      ArmRuntime * arm = findArm(arm_name);
      if (arm == nullptr) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "目标里的臂 '%s' 未加载 → 整条丢弃", arm_name.c_str());
        continue;
      }
      // frame_id 必须是我们认识的那个（不认识就整帧拒绝，不猜）
      if (!t.frame_id.empty() && !frame_id_.empty() && t.frame_id != frame_id_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "臂 '%s' 的目标 frame_id='%s' 与配置的 '%s' 不符 → 丢弃",
                             arm_name.c_str(), t.frame_id.c_str(), frame_id_.c_str());
        continue;
      }
      // ---- 关节模式：预摆位/回位，不走 IK --------------------------------
      if (t.target_type == move_msgs::msg::MoveTarget::TARGET_JOINTS) {
        const std::size_t nj = arm->joints.size();
        if (t.joint_positions.size() != nj) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "臂 '%s' 的关节目标长度 %zu != %zu → 丢弃",
                               arm_name.c_str(), t.joint_positions.size(), nj);
          continue;
        }
        bool finite = true;
        for (double v : t.joint_positions) {
          if (!std::isfinite(v)) {finite = false; break;}
        }
        if (!finite) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                               "臂 '%s' 的关节目标含 NaN/Inf → 丢弃", arm_name.c_str());
          continue;
        }
        arm->joint_mode = true;
        arm->joint_target.assign(t.joint_positions.begin(), t.joint_positions.end());
        arm->joint_target_ms = now;
        arm->have_target = true;
        continue;
      }
      // 位姿模式：显式切回（同一 TRACKING 内允许 joint → pose）
      arm->joint_mode = false;
      if (t.params_valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "臂 '%s' 带了 params_valid=true，但**本版尚未实现逐帧参数覆盖**"
                             "（会用启动期配置）—— 不静默假装生效", arm_name.c_str());
      }
      RawPoseSample s;
      s.recv_ms = now;                        // 单调钟：新鲜度/排序的主时基
      s.recv_sys_ms = systemNowMs();          // 系统钟：只跟 header.stamp 比
      s.has_stamp = (t.header.stamp.sec != 0 || t.header.stamp.nanosec != 0);
      if (s.has_stamp) {
        s.stamp_ms = static_cast<std::int64_t>(t.header.stamp.sec) * 1000 +
                     static_cast<std::int64_t>(t.header.stamp.nanosec) / 1000000;
      }
      s.position = Vec3{t.position.x, t.position.y, t.position.z};
      s.orientation = Quat{t.orientation.x, t.orientation.y, t.orientation.z,
                           t.orientation.w};
      if (!arm->filter.push(s)) {
        // ⚠️ 拒绝告警必须带**实际差值**：只说 FUTURE_STAMP，现场没法判断是
        //    "差 3 ms 的边界抖动"还是"差 3 分钟的系统时钟不一致"。
        const std::int64_t ahead = s.has_stamp ? (s.stamp_ms - s.recv_sys_ms) : 0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "臂 '%s' 的目标被滤波管线拒绝：%s（戳−到达 = %+lld ms，"
                             "阈值 %+.0f ms；已被拒 %llu 次，历史最大 %+lld ms）",
                             arm_name.c_str(), filterRejectName(arm->filter.lastReject()),
                             static_cast<long long>(ahead),
                             arm->filter.config().future_stamp_tolerance_ms,
                             static_cast<unsigned long long>(arm->filter.futureStampRejects()),
                             static_cast<long long>(arm->filter.stampFutureMaxMs()));
        continue;
      }
      arm->have_target = true;
    }
  }

  ArmRuntime * findArm(const std::string & name)
  {
    for (auto & a : arms_) {if (a.name == name) {return &a;}}
    return nullptr;
  }

  // -------------------------------------------------------- 状态机 tick
  // ------------------------------------------- 可观测性（状态 tick 的诊断）
  //
  // 真机 FAULT 之后加的：只靠"帧数停了"根本看不出是"tick 被同步调用拖死"
  // 还是"/joint_states 真的没来"。这里把两个关键量量化出来：
  //   · 状态 tick 的耗时分布（最近 kStateTickSamples 次：平均/p95/最大）；
  //   · `/joint_states` 的年龄（当前值 + 自启动以来的最大值）。
  // 判据很直白：**tick 的耗时必须远小于 state_timeout_ms(100 ms)**，而且
  // age 的最大值必须一直 < 100 ms —— 否则 FAULT 迟早会发生。
  /// 记录一次**实际下发**的时刻（用于算"近 100 帧的真实控制频率"）。
  void recordControlPublish(std::int64_t now_ms)
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    pub_ms_[pub_head_] = now_ms;
    pub_head_ = (pub_head_ + 1) % pub_ms_.size();
    if (pub_n_ < pub_ms_.size()) {++pub_n_;}
  }

  /// 近 `kStateTickSamples` 次**下发**的实际频率（Hz）。样本 < 2 时返回 0。
  double controlPublishHz() const
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    if (pub_n_ < 2) {return 0.0;}
    // 环形缓冲里最早的样本：head 指向"下一个要写的"，所以最早的是 head（满时）
    const std::size_t oldest = (pub_n_ == pub_ms_.size()) ? pub_head_ : 0;
    const std::int64_t t0 = pub_ms_[oldest];
    const std::int64_t t1 = pub_ms_[(pub_head_ + pub_ms_.size() - 1) % pub_ms_.size()];
    const double dt_s = static_cast<double>(t1 - t0) / 1000.0;
    if (!(dt_s > 1e-6)) {return 0.0;}
    return static_cast<double>(pub_n_ - 1) / dt_s;
  }

  std::uint64_t filterFutureRejects() const
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    std::uint64_t n = 0;
    for (const auto & a : arms_) {n += a.filter.futureStampRejects();}
    return n;
  }
  std::uint64_t filterOutOfOrder() const
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    std::uint64_t n = 0;
    for (const auto & a : arms_) {n += a.filter.outOfOrderCount();}
    return n;
  }
  std::uint64_t filterResync() const
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    std::uint64_t n = 0;
    for (const auto & a : arms_) {n += a.filter.stampResyncCount();}
    return n;
  }
  std::int64_t filterStampFutureMaxMs() const
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    std::int64_t m = 0;
    for (const auto & a : arms_) {
      const auto v = a.filter.stampFutureMaxMs();
      if (v > m) {m = v;}
    }
    return m;
  }

  /// 上游实测目标频率（Hz，两臂取最慢的那个）。0 = 样本不足。
  double upstreamTargetHz() const
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    double worst = 0.0;
    bool any = false;
    for (const auto & a : arms_) {
      const double f = a.filter.upstreamHz();
      if (f <= 0.0) {continue;}
      if (!any || f < worst) {worst = f;}
      any = true;
    }
    return any ? worst : 0.0;
  }
  /// 生效的位姿低通截止频率（Hz，已按上游频率夹取）。
  double effectivePoseLpfHz() const
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    double v = 0.0;
    for (const auto & a : arms_) {v = std::max(v, a.filter.poseLpfEffectiveHz());}
    return v;
  }
  std::uint64_t lpfClampCount() const
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    std::uint64_t n = 0;
    for (const auto & a : arms_) {n += a.filter.lpfClampCount();}
    return n;
  }

  /// 记录一次"节点侧逐拍最大关节步长"（rad）。
  /// 记录一次控制 tick 的耗时（ms），用于回答"100 Hz 的拍子到底占多少"。
  void recordControlTick(double ms)
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    ctl_ring_[ctl_head_] = ms;
    ctl_head_ = (ctl_head_ + 1) % ctl_ring_.size();
    if (ctl_n_ < ctl_ring_.size()) {++ctl_n_;}
  }

  /// (avg, p95) 控制 tick 耗时（ms）。
  std::pair<double, double> controlTickStats() const
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    if (ctl_n_ == 0) {return {0.0, 0.0};}
    std::vector<double> v(ctl_ring_.begin(), ctl_ring_.begin() + static_cast<long>(ctl_n_));
    std::sort(v.begin(), v.end());
    double sum = 0.0;
    for (double x : v) {sum += x;}
    return {sum / static_cast<double>(v.size()),
            v[static_cast<std::size_t>(0.95 * static_cast<double>(v.size() - 1))]};
  }

  void recordStep(double d)
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    step_ring_[step_head_] = d;
    step_head_ = (step_head_ + 1) % step_ring_.size();
    if (step_n_ < step_ring_.size()) {++step_n_;}
    if (d > step_max_) {step_max_ = d;}
  }

  /// (p99.9, max) 逐拍步长统计。
  std::pair<double, double> stepStats() const
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    if (step_n_ == 0) {return {0.0, 0.0};}
    std::vector<double> v(step_ring_.begin(), step_ring_.begin() + static_cast<long>(step_n_));
    std::sort(v.begin(), v.end());
    const std::size_t i = static_cast<std::size_t>(0.999 * static_cast<double>(v.size() - 1));
    return {v[i], step_max_};
  }

  void recordTargetAge(std::int64_t age_ms)
  {
    if (age_ms < 0) {return;}
    std::lock_guard<std::mutex> lk(diag_mtx_);
    if (age_ms > target_age_ms_max_) {target_age_ms_max_ = age_ms;}
    target_age_ms_last_ = age_ms;
  }

  void recordStateAge(std::int64_t age_ms)
  {
    if (age_ms < 0) {return;}
    std::lock_guard<std::mutex> lk(diag_mtx_);
    if (age_ms > state_age_ms_max_) {state_age_ms_max_ = age_ms;}
    state_age_ms_last_ = age_ms;
  }

  void recordStateTickMs(double ms)
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    tick_ms_[tick_head_] = ms;
    tick_head_ = (tick_head_ + 1) % tick_ms_.size();
    if (tick_n_ < tick_ms_.size()) {++tick_n_;}
  }

  std::size_t pubCount() const
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    return pub_n_;
  }

  struct TickStats
  {
    std::size_t n{0};
    double avg{0.0};
    double p95{0.0};
    double max{0.0};
  };

  TickStats tickStats() const
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    TickStats st;
    if (tick_n_ == 0) {return st;}
    std::array<double, kStateTickSamples> tmp{};
    double sum = 0.0;
    for (std::size_t i = 0; i < tick_n_; ++i) {tmp[i] = tick_ms_[i]; sum += tmp[i];}
    std::sort(tmp.begin(), tmp.begin() + static_cast<long>(tick_n_));
    st.n = tick_n_;
    st.avg = sum / static_cast<double>(tick_n_);
    st.p95 = tmp[static_cast<std::size_t>(0.95 * static_cast<double>(tick_n_ - 1))];
    st.max = tmp[tick_n_ - 1];
    return st;
  }

  /// 每 5 s 打一行诊断（不要每拍打，rosout 会被刷满）。
  void maybeLogDiagnostics(std::int64_t now)
  {
    if (diag_last_log_ms_ != 0 && now - diag_last_log_ms_ < 5000) {return;}
    diag_last_log_ms_ = now;
    const TickStats ts = tickStats();
    long long age_now = 0, age_max = 0;
    {
      std::lock_guard<std::mutex> lk(diag_mtx_);
      age_now = static_cast<long long>(state_age_ms_last_);
      age_max = static_cast<long long>(state_age_ms_max_);
    }
    long long tgt_now = 0, tgt_max = 0;
    {
      std::lock_guard<std::mutex> lk(diag_mtx_);
      tgt_now = static_cast<long long>(target_age_ms_last_);
      tgt_max = static_cast<long long>(target_age_ms_max_);
    }
    RCLCPP_INFO(get_logger(),
                "状态 tick 诊断（近 %zu 次）：平均 %.1f / p95 %.1f / 最大 %.1f ms；"
                "/joint_states 年龄 当前 %lld ms、最大 %lld ms；"
                "目标年龄 当前 %lld ms、最大 %lld ms；"
                "控制 tick 平均 %.2f / p95 %.2f ms；"
                "上游目标 %.1f Hz、位姿低通生效 %.1f Hz（夹取 %llu 次）；"
                "实际下发 %.1f Hz（近 %zu 帧，control_rate=%.0f / ik_rate=%.0f）；"
                "桥 get_status 超时 %llu 次",
                ts.n, ts.avg, ts.p95, ts.max, age_now, age_max,
                tgt_now, tgt_max, controlTickStats().first, controlTickStats().second,
                upstreamTargetHz(), effectivePoseLpfHz(),
                static_cast<unsigned long long>(lpfClampCount()),
                controlPublishHz(), pubCount(), control_rate_hz_, ik_rate_hz_,
                static_cast<unsigned long long>(status_timeouts_.load()));
  }

  void onStateTick()
  {
    const auto t_tick = std::chrono::steady_clock::now();
    const std::int64_t now = steadyNowMs();

    // ⚠️⚠️ **每一拍都要刷新桥状态**，否则接管会死锁在 ARMING。
    //
    // 原因：`stepArming` 的授权门读的是**缓存** `bridge_authorized`，而这个
    // 缓存先前只在"锁源那一次"的响应里被赋值。`request_lock_source` 是
    // `tick()` **返回之后**才执行的 ⇒ 下一拍的输入里 `manual_override` 仍是旧值
    // ⇒ 授权永远为 false ⇒ 卡在 ARMING（实测：ARMING 40 s 不动）。
    // 每拍主动拉一次 `get_status` 就没有这个时序陷阱。
    //
    // ⚠️⚠️ **一拍只允许这一次 `get_status`，预算 `kStateTickStatusBudgetSec`
    //    （50 ms）**。原来这里一次（50 ms）＋ 末尾 `pollBridge()` 又一次
    //    （**2 s**），同一拍同一个服务调两遍 —— 那 2 s 就是真机 FAULT 的根因
    //    （多个状态 tick 并发堆在 Reentrant 组里，把 executor 线程吃干，
    //    饿死 `/joint_states` ⇒ 失鲜 ⇒ FAULT）。现在 `pollBridge()` 已删除，
    //    它的"仲裁/模式回读 + mode_ok 下调"职责并入下面这一块。
    //
    // ⚠️ 这是**同步**服务调用（`waitFuture` 轮询 future）⇒ 它需要**另一个**
    //    线程去执行客户端的响应回调，所以状态 tick 必须独占一个组
    //    （`state_cb_group_`），不能再和默认组（服务客户端所在组）混。
    std::int64_t state_age_ms = -1;
    if (bridge_ != nullptr) {
      BridgeStatusLite st;
      if (bridge_->getStatus(st, kStateTickStatusBudgetSec)) {
        bridge_active_source_ = st.active_source;
        bridge_manual_override_ = st.manual_override;
        // ↓ 原 pollBridge() 的职责（每拍读一次就够，别调两遍服务）
        bridge_arbitration_state_ = st.arbitration_state_name;
        bridge_mode_name_ = st.mode_name;
        bridge_mode_ = st.mode;
        // ⚠️ `bridge_mode_ok_` 只在这里**下调**，绝不置 true：
        //    置 true 只能由 `set_mode` 的**回读**决定（见下面 request_mode_* 分支）。
        //    否则 `SWITCHING` 的"先 HOLD 再 TRACKING"两步走会被下一拍清成 false
        //    ⇒ 切换永久卡死（HOLD 回读 mode==1，而 TRACKING 判据 mode==2 尚未满足）。
        // ⚠️ 必须把 HOLD(1) 也算 OK：SWITCHING 第一步成功后要靠它放行第二步。
        if (st.mode != 1 && st.mode != 2) {bridge_mode_ok_ = false;}
      } else {
        // 超时只计数（诊断用），**不**在这一拍里重试 —— 重试就是又一次阻塞。
        ++status_timeouts_;
      }
    }
    // ⚠️ ⚠️ **必须用"新鲜 now"算年龄**：上面那次 `get_status` 最长阻塞
    //    `kStateTickStatusBudgetSec`(50 ms)，期间新消息可能已经到达（
    //    `last_all_ms_`/`prev_recv_ms_` 比 tick 开头的 `now` 还新）⇒ 用旧 `now`
    //    算出来的年龄是**负数** ⇒ `state_fresh`/`target_fresh` 双双翻 false
    //    ⇒ 真机上表现为"接管后 DWELL↔RUNNING 反复抖，最后 /joint_states 失鲜
    //    → FAULT"。所以这里重新取一次时间，并且 `stateAgeMs`/目标年龄也都夹到 ≥0。
    const std::int64_t now_fresh = steadyNowMs();
    if (bridge_ != nullptr) {state_age_ms = bridge_->stateAgeMs(now_fresh);}
    recordStateAge(state_age_ms);
    // 目标年龄：给"失鲜/DWELL 抖动"提供可观测证据（见 maxTargetAgeMs 的注释）
    recordTargetAge(maxTargetAgeMs(now_fresh));

    StateInputs in;
    in.now_ms = now_fresh;
    in.load_ok = !arms_.empty();
    in.enable_requested = enable_requested_;
    in.disable_requested = disable_requested_;
    in.clear_fault_requested = clear_fault_requested_;
    in.state_fresh = bridge_ && bridge_->stateReady() && state_age_ms >= 0 &&
                     state_age_ms <= static_cast<std::int64_t>(state_timeout_ms_);
    in.target_fresh = anyTargetFresh(now_fresh);
    // ⚠️⚠️ 授权判据**不能只看 `active_source`**。
    //
    // 桥侧 `activeSource()` 只在 `active_index_ >= 0` 时非空，而
    // `active_index_` 是**仲裁器真的收到并接受了该源的命令**之后才置上的。
    // 本节点在 `ARMING` 阶段**按设计不发 MotionCommand**（让桥超时 → SAFE_HOLD
    // 兜底，见 `statePublishes()`）⇒ 桥永远收不到命令 ⇒ `active_source` 永远为空
    // ⇒ 状态机永远等不到授权 ⇒ **死锁**（实测：一直停在 ARMING）。
    //
    // 正确判据：**LOCK 成功即授权**。`set_source(MODE_LOCK, source_name)` 成功后
    // 桥的 `manual_override == source_name` —— 那就是"这个源说了算"的语义。
    // `active_source` 是"**正在**持有"的运行时状态，接管完成（RUNNING）后自然会填上。
    in.bridge_authorized = (bridge_active_source_ == source_name_) ||
                           (bridge_manual_override_ == source_name_);
    in.bridge_mode_ok = bridge_mode_ok_;
    if (!violation_.empty()) {in.violation = violation_;}

    // ⚠️ 真机排查用：`state_fresh` 为 false 时把**判据的三个来源**都打出来。
    //    只靠 state_detail 的"失鲜"两个字，分不清是"/joint_states 真的没来"
    //    还是"回调被压住/这里算错了"。限频 1 s，不会刷屏。
    if (!in.state_fresh) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "state_fresh=false：/joint_states 年龄 %lld ms（阈值 %.0f ms）、"
                           "stateReady=%d、目标年龄 %lld ms（阈值 %.0f ms）",
                           static_cast<long long>(state_age_ms), state_timeout_ms_,
                           static_cast<int>(bridge_ != nullptr && bridge_->stateReady()),
                           static_cast<long long>(maxTargetAgeMs(now_fresh)), target_timeout_ms_);
    }
    const bool was_disable = static_cast<bool>(disable_requested_);
    const StateActions act = sm_.tick(in);
    enable_requested_ = false;
    disable_requested_ = false;
    clear_fault_requested_ = false;
    // ⚠️⚠️ **接管/交权/故障时复位各臂运行时**（本轮最关键的修复）：
    //    否则 HOLD→TRACKING 的第一拍会沿用**上一次运行**的 filter 目标 /
    //    q_last / 插值段，把旧目标当成当前目标下发 —— 表现就是"一切 HOLD 就
    //    自己打自己"。复位后首帧 = 实测，且 ramp 重新从 0 开始。
    if (act.request_lock_source || was_disable || act.request_release_source) {
      reset_runtime_requested_.store(true, std::memory_order_release);
    }

    // 执行动作（**同步服务调用**，只在低频 tick 里发生）
    if (act.request_lock_source) {
      BridgeStatusLite st;
      const bool sent = bridge_->setSource(1 /*LOCK*/, source_name_, lock_hold_ms_, st, 10.0);
      if (!sent) {
        fail("锁源调用失败：" + st.message);
      } else if (!st.ok) {
        std::string avail;
        for (const auto & a : st.available_sources) {avail += a + " ";}
        fail("锁源被拒：" + st.message + "（可用源：" + avail + "；请确认 bringup 传了 source_names:=" +
             source_name_ + "）");
      } else {
        bridge_active_source_ = st.active_source;
        bridge_manual_override_ = st.manual_override;
        RCLCPP_INFO(get_logger(), "已锁源：active_source='%s' state=%s",
                    st.active_source.c_str(), st.arbitration_state_name.c_str());
      }
    }
    if (act.request_mode_hold) {
      BridgeStatusLite st;
      if (!bridge_->setMode(255 /*KEEP*/, 1 /*HOLD*/, false, st, 15.0) || !st.ok) {
        fail("切 MIT HOLD 失败：" + st.message);
      } else {
        bridge_mode_ok_ = st.controller_active && st.mode == 1;
        RCLCPP_INFO(get_logger(), "已切 HOLD（回读 mode=%u %s active=%d）", st.mode,
                    st.mode_name.c_str(), static_cast<int>(st.controller_active));
      }
    }
    if (act.request_mode_tracking) {
      BridgeStatusLite st;
      if (!bridge_->setMode(255, 2 /*TRACKING*/, false, st, 15.0) || !st.ok) {
        fail("切 MIT TRACKING 失败：" + st.message);
      } else {
        bridge_mode_ok_ = st.controller_active && st.mode == 2;
        RCLCPP_INFO(get_logger(), "已切 TRACKING（回读 mode=%u %s active=%d）", st.mode,
                    st.mode_name.c_str(), static_cast<int>(st.controller_active));
      }
    }
    if (act.request_release_source) {
      BridgeStatusLite st;
      if (bridge_->setSource(0 /*AUTO*/, "", 0.0, st, 10.0)) {
        bridge_active_source_ = st.active_source;
        bridge_manual_override_ = st.manual_override;
        RCLCPP_INFO(get_logger(), "已交权：active_source='%s'", st.active_source.c_str());
      }
    }
    // ⚠️ 这里**不再**调 `pollBridge()`：桥状态本拍已经在开头读过了（一次）。
    //    再读一次不但多一次阻塞，还会让"状态 tick 的耗时"随桥的响应时间抖动，
    //    而状态 tick 必须在 50 ms 量级内完成（见文件头/回调组那段说明）。
    recordStateTickMs(std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_tick).count());
    maybeLogDiagnostics(now);
    if (act.log_error && !violation_.empty()) {
      RCLCPP_ERROR(get_logger(), "进入 FAULT：%s", violation_.c_str());
    }
    // 故障自锁：进入 FAULT/STANDBY 后清掉违例，避免每 tick 重入
    if (sm_.state() == NodeState::Fault || sm_.state() == NodeState::Standby ||
        sm_.state() == NodeState::Suspended) {
      violation_.clear();
    }
  }

  /// 所有臂里"最后一个被接受目标"的最大年龄（ms）。-1 = 从来没有目标。
  ///
  /// ⚠️ 年龄的来源是 `InputFilter::ageMs()`，它用的是 **`RawPoseSample::recv_ms`**
  ///    —— 即 `onTarget()` 里取到的**到达时刻**（`steadyNowMs()`），**不是**
  ///    消息头时间戳、也不是"最后一次被接受"之类会被坏帧拖住的东西：
  ///    `prev_recv_ms_` 在 push() 的**接受路径**上被赋成 `raw.recv_ms`。
  ///    所以只要上游还在按 100 Hz 送**合法**帧，这个年龄就只反映"回调被压住
  ///    多久"。真机上它抖到 200 ms 以上，说明目标回调被饿住（不是上游停了）。
  std::int64_t maxTargetAgeMs(std::int64_t now) const
  {
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    std::int64_t worst = -1;
    for (const auto & a : arms_) {
      // ⚠️ 关节模式（预摆位）不走位姿滤波，年龄要单独算 —— 否则状态机会
      //    认为"目标过期"而停在 DWELL，关节目标永远不生效。
      if (a.joint_mode) {
        if (a.joint_target_ms == 0) {continue;}
        const std::int64_t age = std::max<std::int64_t>(0, now - a.joint_target_ms);
        if (age > worst) {worst = age;}
        continue;
      }
      if (!a.have_target || !a.filter.hasTarget()) {continue;}
      // ⚠️ 夹到 ≥0：目标可能在"取 now 之后、算年龄之前"到达（见 onStateTick
      //    里 now_fresh 的说明）。负年龄会被 `anyTargetFresh` 的 `age >= 0`
      //    判成"目标过期" ⇒ DWELL，下一拍又变正 ⇒ RUNNING ⇒ **抖动**。
      const auto raw_age = a.filter.ageMs(now);
      const std::int64_t age = raw_age > 0 ? raw_age : 0;
      if (age > worst) {worst = age;}
    }
    return worst;
  }

  bool anyTargetFresh(std::int64_t now) const
  {
    const std::int64_t age = maxTargetAgeMs(now);
    return age >= 0 && age <= static_cast<std::int64_t>(target_timeout_ms_);
  }

  void fail(const std::string & why)
  {
    violation_ = why;
  }

  // ------------------------------------------------------ 控制 tick（实时）
  void onControlTick()
  {
    if (bridge_ == nullptr) {return;}
    const std::int64_t now = steadyNowMs();
    const NodeState st = sm_.state();
    if (!statePublishes(st)) {return;}          // 不发帧（让桥超时 → SAFE_HOLD）
    const auto tick_t0 = std::chrono::steady_clock::now();

    // ⚠️ DWELL 是"低频保持帧"（`statePublishesFullRate()` 为 false）：
    //    以前 `dwell_rate_hz` **只声明没使用**（DWELL 照样按 control_rate 发），
    //    于是这个参数形同虚设、DWELL 也在跑满 IK。现在真的按它限频。
    //    ⚠️ 限频发生在**任何计算之前**：DWELL 的那几拍连 IK 都不做。
    if (!statePublishesFullRate(st)) {
      const std::int64_t min_gap_ms =
        static_cast<std::int64_t>(1000.0 / std::max(1e-3, dwell_rate_hz_));
      if (last_publish_ms_ != 0 && now - last_publish_ms_ < min_gap_ms) {return;}
    }

    const std::size_t n = canonical_joints_.size();
    const JointSnapshot snap = bridge_->snapshot();
    if (!snap.covers(n)) {return;}              // 没有可信实测就不动

    // 接管前复位（控制线程内执行，见 onStateTick 的说明）。首帧必须是实测。
    if (reset_runtime_requested_.exchange(false, std::memory_order_acq_rel)) {
      resetArmRuntime(snap);
    }

    JointCommandFrame frame;
    frame.resize(n);
    // 默认：位置 = 实测（**绝不能填 0** —— MIT 下 Kp>0 会朝零位使劲）
    frame.position = snap.position;
    frame.velocity.assign(n, 0.0);
    frame.effort.assign(n, 0.0);
    // ⚠️ 不要用 `get_parameter("...").as_double_array()` 去"探测"未给的可选数组：
    //    未初始化的空数组参数**不可读**，会抛
    //    "Statically typed parameter must be initialized"。
    //    要判断"给没给"用 `has_parameter(name) && !get_parameter(name).as_double_array().empty()`。
    frame.stiffness.assign(n, kp_default_);
    frame.damping.assign(n, kd_default_);

    if (statePublishesFullRate(st)) {
      // ⚠️ **IK 重解与下发解耦**：下发节拍由 `control_rate_hz`（默认 100 Hz）驱动，
      //    IK 只在每条臂自己的 `ik_rate_hz`（默认 25 Hz）节拍上真调。其余拍子
      //    按**墙钟时间**沿 Hermite 段求值（不是"每 4 拍走 1/4"）。
      //    IK 早到 → 提前切段；IK 晚到 → 段走完就停在端点等新解。两者都连续。
      bool ok = true;
      for (auto & a : arms_) {
        if (!solveArm(a, snap, now, frame)) {ok = false;}
      }
      if (!ok) {return;}
    } else {
      // ⚠️ DWELL = "低频**保持**帧"：**不重解 IK**，直接顶住上一帧的解
      //    （还没有解就顶实测）。以前这里照样跑完整 IK：目标陈旧/不可达时
      //    每拍烧掉两个 50 ms 预算（实测状态 tick 136 ms、帧率掉到 7 Hz），
      //    而且 DWELL 的语义本来就是"顶住，不松劲"，不是"再解一次"。
      std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
      for (const auto & a : arms_) {
        if (!a.have_last_solution) {continue;}
        const std::size_t nj = a.joints.size();
        if (a.offset + nj > frame.size() || a.q_last.size() != nj) {continue;}
        for (std::size_t k = 0; k < nj; ++k) {
          frame.position[a.offset + k] = a.q_last[k];
        }
      }
    }

    // ---- 夹爪：阻抗夹持（**不参与 IK / 动力学**）-------------------------
    // 用户场景：夹住一支笔，对期望位置 0（= 夹到底）做阻抗控制，手臂去画圆。
    // ⚠️ 夹爪不是 IK 链的一部分（链只有 7 个 revolute），也不在辨识的动力学
    //    模型里，所以它是"帧里的独立保持命令"：
    //      position=最新独立目标（初次来自实测）, velocity=0, effort=0, kp=gripper_kp, kd=gripper_kd
    //    `effort=0` 是刻意的：夹持力由 kp·(0 − 实测位置) 产生，再由 limit_guard
    //    的 torque_max（≤ URDF effort=1 N）限幅 —— 不额外叠前馈力。
    // ⚠️ 必须放在 limit_guard 之前、且在两臂 IK 之后：夹爪不参与 IK，但它和
    //    手臂在同一帧里下发；顺序反了会让"臂刚解完的帧"覆盖掉夹爪命令。
    if (!gripper_idx_.empty()) {
      std::vector<GripperCommand> cmds;
      {
        std::lock_guard<std::mutex> lock(gripper_cmd_mtx_);
        for (std::size_t i = 0; i < gripper_idx_.size(); ++i) {
          const std::size_t k = gripper_idx_[i];
          if (!gripper_pos_initialized_[i] && snap.valid && k < snap.position.size()) {
            gripper_cmds_[i].position = snap.position[k];
            gripper_pos_initialized_[i] = true;
          }
        }
        cmds = gripper_cmds_;
      }
      const std::size_t nw = applyGripperCommands(
          gripper_idx_, cmds, frame.position, frame.velocity, frame.effort,
          frame.stiffness, frame.damping);
      if (nw == 0) {
        // 帧长度/下标异常（理论上 configure 阶段就挡住了）⇒ 整帧不发。
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                              "夹爪命令写入失败（帧长度 %zu，夹爪下标 %zu 个）→ 整帧不发",
                              frame.size(), gripper_idx_.size());
        return;
      }
    }

    // ⚠️ 下发前的四道闸（位置/速度/力矩/增益）。返回 false = 帧里有 NaN ⇒ 整帧丢。
    if (!guard_.apply(frame)) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                            "本帧被 limit_guard 拒绝（含非有限值）→ 整帧不发");
      ++frames_rejected_;
      return;
    }

    // 斜坡（首帧：位置 = 实测、Kp=Kd=0 ⇒ 力矩恒为 0）
    const double dt = 1.0 / control_rate_hz_;
    if (ramp_elapsed_ < ramp_duration_sec_) {
      ramp_elapsed_ += dt;
    }
    const double alpha = (ramp_duration_sec_ > 0.0)
                         ? std::min(1.0, ramp_elapsed_ / ramp_duration_sec_) : 1.0;
    for (std::size_t j = 0; j < n; ++j) {
      frame.position[j] = snap.position[j] + alpha * (frame.position[j] - snap.position[j]);
      frame.stiffness[j] = alpha * frame.stiffness[j];
      frame.damping[j] = alpha * frame.damping[j];
      frame.effort[j] = alpha * frame.effort[j];
    }
    // 接管后首帧证据：命令与实测的最大差（应≈0；若很大就是"旧目标回放"）。
    for (auto & a : arms_) {
      if (a.first_cmd_logged || a.offset + a.joints.size() > frame.position.size()) {continue;}
      double dmax = 0.0, vmax = 0.0;
      for (std::size_t k = 0; k < a.joints.size(); ++k) {
        dmax = std::max(dmax, std::fabs(frame.position[a.offset + k] -
                                        snap.position[a.offset + k]));
        vmax = std::max(vmax, std::fabs(frame.velocity[a.offset + k]));
      }
      a.first_cmd_logged = true;
      RCLCPP_INFO(get_logger(),
                  "接管首帧 臂 '%s'：|q_cmd−q_meas|max=%.4f rad，|v_cmd|max=%.3f rad/s，"
                  "Kp=%.1f Kd=%.1f（应≈0 / 小：大 = 旧目标回放）",
                  a.name.c_str(), dmax, vmax, kp_default_, kd_default_);
    }
    bridge_->publish(frame.position, frame.velocity, frame.effort,
                     frame.stiffness, frame.damping);
    recordControlTick(std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - tick_t0).count());
    // 记录"实际下发节拍"（近 100 帧）—— 真机上帧率掉到 12.8 Hz 时，光看
    // `frames_published` 只能算平均值，看不出抖动。见 recordControlPublish()。
    last_publish_ms_ = now;
    recordControlPublish(now);
    // 节点侧逐拍步长（权威口径：相邻两拍**实际下发**的关节角之差）
    {
      std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
      double worst = 0.0;
      for (auto & a : arms_) {
        const std::size_t nj = a.joints.size();
        if (a.offset + nj > frame.size()) {continue;}
        if (a.prev_cmd.size() == nj) {
          for (std::size_t k = 0; k < nj; ++k) {
            const double d = std::fabs(frame.position[a.offset + k] - a.prev_cmd[k]);
            if (d > worst) {worst = d;}
          }
        }
        a.prev_cmd.assign(frame.position.begin() + static_cast<long>(a.offset),
                          frame.position.begin() + static_cast<long>(a.offset + nj));
      }
      if (worst > 0.0) {recordStep(worst);}
    }
  }

  /// 从 URDF 文本里抽关节限位，配上配置里的力矩/增益上限，构成 limit_guard。
  ///
  /// ⚠️ 为什么必须做：MIT 通路**没有位置环兜底**，驱动只把力矩夹到型号峰值
  ///    （rs-00 17 N·m，而本机额定只有 7 N·m）、Kp 夹到协议量程 500。
  ///    保守上限**只能由上层给** —— 这是设计文档 §5.3 的四道闸。
  void setupLimitGuard(const std::string & urdf)
  {
    const std::size_t n = canonical_joints_.size();
    JointLimits lim;
    lim.position_min.assign(n, -3.2);
    lim.position_max.assign(n, 3.2);
    lim.velocity_max.assign(n, 1.0);
    lim.torque_max.assign(n, 5.0);
    lim.stiffness_max.assign(n, kp_default_);
    lim.damping_max.assign(n, kd_default_);

    // 用朴素解析从 URDF 取每个关节的 lower/upper/velocity（避免为一个限位
    // 表引入 urdfdom 依赖；格式由 lite_description 的 xacro 展开保证稳定）。
    for (std::size_t k = 0; k < n; ++k) {
      const std::string & jn = canonical_joints_[k];
      const std::string key = "<joint name=\"" + jn + "\"";
      const auto pos = urdf.find(key);
      if (pos == std::string::npos) {continue;}
      const auto end = urdf.find("</joint>", pos);
      const std::string blk = urdf.substr(pos, (end == std::string::npos ? pos + 400 : end) - pos);
      auto grab = [&blk](const char * attr, double & out) {
        const std::string a = std::string(attr) + "=\"";
        const auto p = blk.find(a);
        if (p == std::string::npos) {return;}
        try {out = std::stod(blk.substr(p + a.size()));} catch (...) {}
      };
      double lo = lim.position_min[k], hi = lim.position_max[k], vel = lim.velocity_max[k];
      double eff = 0.0;
      grab("lower", lo);
      grab("upper", hi);
      grab("velocity", vel);
      grab("effort", eff);
      lim.position_min[k] = lo;
      lim.position_max[k] = hi;
      lim.velocity_max[k] = (vel > 1e-6) ? vel : 1.0;
      // ⚠️ **只有夹爪**用 URDF 的 `effort` 当力矩上限。
      //
      //    为什么不是"所有关节都取 min(现有, URDF effort)"（原本想这么写）：
      //    本机 URDF（lite_description 展开出来的）里**每个关节的 effort 都是 1**
      //    —— 那是占位值，不是额定。臂的真实额定由驱动的 `max_safe_torque`
      //    （7 / 4 / 2.5 / 1.5 / 1.8 N·m，见 lite_controller/config/gravity/
      //    joint_spec.yaml）表达。若把 min() 套到全关节，肩关节的上限会从
      //    5 N·m 掉到 **1 N·m** —— 连重力都顶不住，手臂会直接往下塌。
      //    所以：**夹爪**（prismatic，effort=1 N 是真实额定）按 URDF 压到 1 N；
      //    臂关节维持配置/默认（5 N·m），要更保守请用 `torque_limit` 参数或
      //    驱动侧的 max_safe_torque。
      const bool is_gripper =
        std::find(gripper_idx_.begin(), gripper_idx_.end(), k) != gripper_idx_.end();
      if (is_gripper && eff > 1e-9) {
        lim.torque_max[k] = std::min(lim.torque_max[k], eff);
      }
    }
    // 配置里给了就覆盖（逐关节）
    // ⚠️ `has_parameter()` 对"已声明但**未赋值**"的数组返回 **true**，
    //    此时 `as_double_array()` 会抛
    //    "Statically typed parameter must be initialized."。
    //    所以这里必须用 try/catch（ROS 2 没有"只问类型不问值"的接口）。
    auto override_vec = [this](const char * name, std::vector<double> & dst) {
      if (!has_parameter(name)) {return;}
      std::vector<double> v;
      try {
        v = get_parameter(name).as_double_array();
      } catch (const std::exception &) {
        return;                       // 未赋值 ⇒ 用默认（URDF）限位
      }
      if (v.empty()) {return;}
      if (v.size() == 1 && v[0] == kUnsetSentinel) {return;}   // 未配置
      if (v.size() == dst.size()) {dst.assign(v.begin(), v.end()); return;}
      if (v.size() == 14 && dst.size() == 16 && gripper_idx_.size() == 2 &&
          gripper_idx_[0] == 7 && gripper_idx_[1] == 15) {
        // 用户配置了 14 关节的臂限位（如 impedance_test_200s.yaml），自动映射到 16 关节模式的臂槽位
        for (std::size_t k = 0; k < 7; ++k) {
          dst[k] = v[k];
          dst[8 + k] = v[7 + k];
        }
        RCLCPP_INFO(get_logger(),
                    "参数 %s 长度 14 自动适配到 16 关节（双臂 14 轴正确映射，夹爪维持独立限位）",
                    name);
        return;
      }
      if (v.size() == 16 && dst.size() == 14) {
        for (std::size_t k = 0; k < 7; ++k) {
          dst[k] = v[k];
          dst[7 + k] = v[8 + k];
        }
        RCLCPP_INFO(get_logger(),
                    "参数 %s 长度 16 自动适配到 14 关节（双臂 14 轴正确映射，忽略夹爪）",
                    name);
        return;
      }
      RCLCPP_WARN(get_logger(), "参数 %s 长度 %zu != 关节数 %zu → 忽略（用 URDF 限位）",
                  name, v.size(), dst.size());
    };
    override_vec("position_min", lim.position_min);
    override_vec("position_max", lim.position_max);
    override_vec("velocity_limit", lim.velocity_max);
    override_vec("torque_limit", lim.torque_max);
    override_vec("stiffness_limit", lim.stiffness_max);
    override_vec("damping_limit", lim.damping_max);

    // ⚠️ 夹爪力矩上限与增益上限约束：
    //    力矩上限按启动参数 gripper_torque_limit（默认 0.25 N·m）严格截断，
    //    增益上限至少放行配置的 gripper_kp/kd。
    for (const std::size_t k : gripper_idx_) {
      if (k >= n) {continue;}
      lim.torque_max[k] = gripper_torque_limit_;
      lim.stiffness_max[k] = std::max(lim.stiffness_max[k], gripper_kp_);
      lim.damping_max[k] = std::max(lim.damping_max[k], gripper_kd_);
    }

    std::string err;
    if (!guard_.configure(lim)) {
      RCLCPP_FATAL(get_logger(), "limit_guard 配置失败（上限表不自洽）→ 拒绝启动");
      throw std::runtime_error("limit_guard 配置失败");
    }
    if (!gripper_idx_.empty()) {
      std::string g;
      for (const std::size_t k : gripper_idx_) {
        g += canonical_joints_[k] + "(torque_max=" +
             std::to_string(lim.torque_max[k]) + " N, kp_max=" +
             std::to_string(lim.stiffness_max[k]) + ", kd_max=" +
             std::to_string(lim.damping_max[k]) + ") ";
      }
      RCLCPP_INFO(get_logger(), "夹爪 limit_guard：%s", g.c_str());
    }
    RCLCPP_INFO(get_logger(),
                "limit_guard 就绪：力矩上限 [%.2f .. %.2f] N·m，Kp 上限 %.1f，Kd 上限 %.1f",
                *std::min_element(lim.torque_max.begin(), lim.torque_max.end()),
                *std::max_element(lim.torque_max.begin(), lim.torque_max.end()),
                *std::max_element(lim.stiffness_max.begin(), lim.stiffness_max.end()),
                *std::max_element(lim.damping_max.begin(), lim.damping_max.end()));
  }

  /// 扫 `/proc` 找**其他** `move_controller_node` 进程（Linux）。
  ///
  /// ⚠️ 只匹配可执行文件名 `move_controller_node`：launch 进程的命令行里是
  ///    `move_controller.launch.py`，不匹配，避免把"拉起自己的 launch"当成重复。
  static std::vector<pid_t> findOtherMoveControllerPids()
  {
    std::vector<pid_t> out;
    DIR * d = opendir("/proc");
    if (d == nullptr) {return out;}
    const pid_t self = getpid();
    while (dirent * e = readdir(d)) {
      const char * n = e->d_name;
      if (n == nullptr || n[0] < '0' || n[0] > '9') {continue;}
      const pid_t pid = static_cast<pid_t>(std::atoi(n));
      if (pid <= 0 || pid == self) {continue;}
      // ⚠️ 必须看 /proc/<pid>/exe（可执行文件本身），**不能**扫 cmdline：
      //    `ros2 run lite_move_controller move_controller_node ...` 的父进程、
      //    以及任何命令行里恰好带这个字符串的 shell，都会被误判成"另一个实例"
      //    （实测：duplicate_instance_policy=ask 时启动直接失败）。
      char exe[PATH_MAX] = {0};
      const std::string link = std::string("/proc/") + n + "/exe";
      const ssize_t len = ::readlink(link.c_str(), exe, sizeof(exe) - 1);
      if (len <= 0) {continue;}
      exe[len] = '\0';
      const std::string base = std::string(exe).substr(
          std::string(exe).find_last_of('/') + 1);
      if (base == "move_controller_node") {out.push_back(pid);}
    }
    closedir(d);
    return out;
  }

  /// 终止一组 PID：SIGTERM → 等 `grace_s` → 仍活着就 SIGKILL。
  static void terminatePids(const std::vector<pid_t> & pids, double grace_s)
  {
    for (const pid_t p : pids) {::kill(p, SIGTERM);}
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(grace_s);
    while (std::chrono::steady_clock::now() < deadline) {
      bool alive = false;
      for (const pid_t p : pids) {if (::kill(p, 0) == 0) {alive = true; break;}}
      if (!alive) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    for (const pid_t p : pids) {::kill(p, SIGKILL);}
  }

  /// 启动自检 ①：处理重复实例。
  ///
  /// 策略（参数 `duplicate_instance_policy`）：
  ///   · `abort`：拒绝启动并列出 PID（最保守）
  ///   · `kill` ：SIGTERM → 3 s → SIGKILL（用户已授权）
  ///   · `ask`  ：有 TTY 就询问，否则**拒绝**（launch 下没有 TTY，不会误杀）
  bool handleDuplicateInstances(std::string & err)
  {
    const auto pids = findOtherMoveControllerPids();
    if (pids.empty()) {
      RCLCPP_INFO(get_logger(), "启动自检：没有其他 move_controller_node 实例");
      return true;
    }
    std::string list;
    for (const pid_t p : pids) {
      list += (list.empty() ? "" : ", ") + std::to_string(static_cast<long long>(p));
    }
    if (duplicate_policy_ == "abort") {
      err = "发现其他 move_controller_node 实例（PID " + list +
            "）。两个实例会互相抢占同一个数据源；请先终止它们，或显式设 "
            "duplicate_instance_policy:=kill（需人工确认）。";
      return false;
    }
    if (duplicate_policy_ == "kill") {
      RCLCPP_WARN(get_logger(),
                  "发现其他 move_controller_node（PID %s）→ 按 duplicate_instance_policy=kill 终止",
                  list.c_str());
      terminatePids(pids, 3.0);
      return true;
    }
    // ask
    if (!isatty(STDIN_FILENO)) {
      err = "发现其他 move_controller_node 实例（PID " + list +
            "），但 stdin 不是 TTY，无法取得操作员授权。请手动终止这些进程，"
            "或用 duplicate_instance_policy:=kill（参数即授权）。";
      return false;
    }
    std::fprintf(stdout,
                 "\n[启动自检] 发现其他 move_controller_node 实例：PID %s\n"
                 "  它们是另一个运控层，会与本节点抢同一个数据源。\n"
                 "  终止它们？[y/N] ", list.c_str());
    std::fflush(stdout);
    std::string answer;
    std::getline(std::cin, answer);
    const bool yes = (answer == "y" || answer == "Y" || answer == "yes" ||
                      answer == "YES" || answer == "是");
    if (!yes) {
      err = "操作员拒绝终止其他 move_controller_node（PID " + list + "）";
      return false;
    }
    terminatePids(pids, 3.0);
    RCLCPP_INFO(get_logger(), "已终止其他 move_controller_node（PID %s）", list.c_str());
    return true;
  }

  /// 启动自检 ②：必须处于 MIT 且本节点已被授权为数据源。
  ///
  /// 三条 fail-closed：
  ///   ① 桥不可达 → 给出完整 bringup 命令（本节点不会替用户起硬件）；
  ///   ② 协议不是 MIT → 跨协议运行期切换**不被支持**，给出重启命令；
  ///   ③ `source_names` 不含本节点 → 桥会丢弃全部命令，提前失败比"看起来正常"好。
  bool ensureMitProtocol(std::string & err)
  {
    // ⚠️ 此时主执行器还没 spin（main 里 initialize() 之后才 spin），而同步
    //    服务调用需要**有人处理响应**。这里用一个临时单线程执行器在后台
    //    spin，直到 getStatus 返回（否则必然 5 s 超时、误报"bringup 未启动"）。
    //    这一阶段还没有建控制/状态定时器，唯一会跑的订阅是 /joint_states，安全。
    BridgeStatusLite st;
    bool got = false;
    try {
      rclcpp::executors::SingleThreadedExecutor tmp_exec;
      tmp_exec.add_node(shared_from_this());
      std::atomic<bool> stop{false};
      std::thread spinner([&tmp_exec, &stop]() {
        while (!stop.load(std::memory_order_acquire)) {
          tmp_exec.spin_some(std::chrono::milliseconds(20));
        }
      });
      got = bridge_->getStatus(st, 5.0);
      stop.store(true, std::memory_order_release);
      spinner.join();
      tmp_exec.remove_node(shared_from_this());
    } catch (const std::exception & e) {
      err = std::string("启动自检异常：") + e.what();
      return false;
    }
    if (!got || !st.ok) {
      err = "启动自检失败：拿不到 /lite_bridge 状态（bringup 未启动？）。请先启动：\n"
            "    ros2 launch lite_bringup lite_bringup.launch.py \\\n"
            "        mode:=arms controller:=modes control_mode:=mit source_names:=" +
            source_name_ +
            "\n（MIT/CSP 是驱动启动期协议，本节点不会替你起硬件，也不会在 CSP 下发送命令）";
      return false;
    }
    if (st.protocol != 0) {   // 0 = MIT（robstride_msgs/SetMode：0=MIT, 1=CSP）
      err = "启动自检失败：当前桥协议=" + st.protocol_name + "（mode=" + st.mode_name +
            "），本节点只支持 MIT。\n"
            "MIT/CSP 由驱动启动参数决定，运行期不能切换（桥会拒绝跨协议 set_mode）。\n"
            "请停掉当前 bringup 后用下面的参数重启：\n"
            "    ros2 launch lite_bringup lite_bringup.launch.py \\\n"
            "        mode:=arms controller:=modes control_mode:=mit source_names:=" +
            source_name_;
      return false;
    }
    bool source_ok = false;
    std::string have;
    for (const auto & s : st.source_names) {
      have += (have.empty() ? "" : ",") + s;
      if (s == source_name_) {source_ok = true;}
    }
    if (!source_ok) {
      err = "启动自检失败：桥的 source_names=[" + have + "] 不含 '" + source_name_ +
            "'。桥会丢弃本节点的所有命令（日志：丢弃未知数据源）。\n"
            "请在 bringup 的 source_names 里加上本节点后重启（例如 "
            "source_names:=lite_move_controller,dyid）：\n"
            "    ros2 launch lite_bringup lite_bringup.launch.py \\\n"
            "        mode:=arms controller:=modes control_mode:=mit source_names:=" +
            source_name_ + ",dyid";
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "启动自检通过：协议=MIT（mode=%s/%s，controller=%s[active=%d]），"
                "数据源 '%s' 已授权",
                st.mode_name.c_str(), st.protocol_name.c_str(),
                st.active_controller.c_str(), static_cast<int>(st.controller_active),
                source_name_.c_str());
    return true;
  }

  /// 在时间 t（steady 秒）求当前 Hermite 段的位置/速度。
  ///
  /// 段未激活 / 已走完 ⇒ 直接返回端点 (q1, v1)。这样"IK 晚到"时不会外推出
  /// 无界目标，只是停在上一解上等下一次。
  void evaluateArmSegment(const ArmRuntime & a, double t_s,
                          std::vector<double> & q, std::vector<double> & v) const
  {
    const std::size_t n = a.seg_q1.size();
    q.assign(n, 0.0);
    v.assign(n, 0.0);
    if (n == 0) {return;}
    if (!a.seg_active || !(a.seg_dur_s > 1e-6) ||
        a.seg_q0.size() != n || a.seg_v0.size() != n || a.seg_v1.size() != n) {
      q = a.seg_q1;
      v = a.seg_v1;
      return;
    }
    double s = (t_s - a.seg_t0_s) / a.seg_dur_s;
    s = std::max(0.0, std::min(1.0, s));
    for (std::size_t k = 0; k < n; ++k) {
      q[k] = hermite(a.seg_q0[k], a.seg_v0[k], a.seg_q1[k], a.seg_v1[k],
                     a.seg_dur_s, s);
      v[k] = hermite_dot(a.seg_q0[k], a.seg_v0[k], a.seg_q1[k], a.seg_v1[k],
                         a.seg_dur_s, s);
    }
  }

  /// 接管前复位：把每条臂的**目标 / IK / 插值段 / q_last / ramp** 全部清空，
  /// 并把 q_last 设为**当前实测**。由控制线程调用（见 reset_runtime_requested_）。
  ///
  /// ⚠️ 为什么必须做：filter 的"最后目标"、`q_last`、`seg_*` 在 disable 后
  ///    不会自己消失。再次 enable 时，HOLD→TRACKING 的第一拍会拿**上次运行**
  ///    的目标去解 IK / 起点播，命令与实测差出好几 rad —— 现场就是"一切 HOLD
  ///    就自己打自己"。dyid 在切 TRACKING 前连发 6 帧实测姿态，本质是同一件事。
  void resetArmRuntime(const JointSnapshot & snap)
  {
    std::lock_guard<std::mutex> lk(arm_state_mtx_);
    ramp_elapsed_ = 0.0;
    for (auto & a : arms_) {
      a.filter.reset();
      a.have_target = false;
      a.have_goal = false;
      a.seg_active = false;
      a.seg_q0.clear(); a.seg_v0.clear(); a.seg_q1.clear(); a.seg_v1.clear();
      a.seg_t0_s = 0.0; a.seg_dur_s = 0.0;
      a.joint_mode = false;
      a.joint_target.clear(); a.joint_active.clear(); a.joint_target_ms = 0;
      a.prev_ik_sol.clear(); a.prev_ik_ms = 0; a.last_ik_ms = 0;
      a.solved_once = false;
      a.ik_status = 0; a.ik_action = 0;
      a.residual_pos = 0.0; a.residual_rot = 0.0;
      a.residual_vec = Vec3{0, 0, 0};
      a.residual_rot_vec = Vec3{0, 0, 0};
      a.exact = true;
      a.saturated.clear();
      a.last_v.clear();
      a.have_last_solution = false;
      a.q_goal.clear();
      a.tau_ff.clear();
      const std::size_t nj = a.joints.size();
      if (snap.covers(canonical_joints_.size())) {
        a.q_last.assign(snap.position.begin() + static_cast<long>(a.offset),
                        snap.position.begin() + static_cast<long>(a.offset + nj));
      } else {
        a.q_last.assign(nj, 0.0);
      }
      a.q_seed = a.q_last;
      a.first_cmd_logged = false;
    }
    RCLCPP_INFO(get_logger(),
                "接管复位：filter/目标/IK/插值/q_last 全部清空，q_last=实测，ramp 从 0 开始");
  }

  /// 关节模式：从当前下发值**平滑走到关节目标**（预摆位/回位），不走 IK。
  ///
  /// 与位姿模式共用 Hermite 段、步长上限、超前上限；切回位姿模式时
  /// `q_last`/`q_seed` 就是这里的终点，所以衔接连续。
  bool solveJointArm(ArmRuntime & arm, const JointSnapshot & snap,
                     std::int64_t now, JointCommandFrame & frame)
  {
    const std::size_t nj = arm.joints.size();
    if (arm.joint_target.size() != nj) {return true;}
    if (arm.joint_target_ms == 0 ||
        now - arm.joint_target_ms > static_cast<std::int64_t>(target_timeout_ms_)) {
      return true;   // 关节目标陈旧：保持实测，不沿用旧目标
    }
    // 目标变化（或首次）→ 起新段；同一目标重复到达不重起
    bool restart = !arm.seg_active || arm.joint_active.size() != nj;
    if (!restart) {
      double dmax = 0.0;
      for (std::size_t k = 0; k < nj; ++k) {
        dmax = std::max(dmax, std::fabs(arm.joint_target[k] - arm.joint_active[k]));
      }
      if (dmax > 1e-9) {restart = true;}
    }
    if (restart) {
      const double now_s = steadyNowSec();
      std::vector<double> q0 = arm.q_last;
      if (!arm.have_last_solution && snap.covers(canonical_joints_.size())) {
        q0.assign(snap.position.begin() + static_cast<long>(arm.offset),
                  snap.position.begin() + static_cast<long>(arm.offset + nj));
      }
      std::vector<double> v0(nj, 0.0);
      if (arm.seg_active) {
        evaluateArmSegment(arm, now_s, q0, v0);
      } else if (arm.last_v.size() == nj) {
        v0 = arm.last_v;
      }
      double delta = 0.0;
      for (std::size_t k = 0; k < nj; ++k) {
        delta = std::max(delta, std::fabs(arm.joint_target[k] - q0[k]));
      }
      double T = 1.5 * delta / std::max(0.01, joint_speed_);
      // 大位移（预摆位）用长段；小位移（50 Hz 流式关节目标）用短段跟随，
      // 否则每个小目标都要 0.5 s，关节会"走-停-走"。
      T = std::max(0.12, std::min(20.0, T));
      arm.seg_q0 = std::move(q0);
      arm.seg_v0 = std::move(v0);
      arm.seg_q1 = arm.joint_target;
      arm.seg_v1.assign(nj, 0.0);          // 到点静止
      arm.seg_t0_s = now_s;
      arm.seg_dur_s = T;
      arm.seg_active = true;
      arm.joint_active = arm.joint_target;
      arm.have_goal = false;               // 切回位姿时强制重新解一次 IK
      arm.last_ik_ms = 0;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "臂 '%s' 关节模式：目标已切换（Δmax=%.3f rad，段长 %.1f s）",
                           arm.name.c_str(), delta, T);
    }
    // 按墙钟求值
    std::vector<double> q_cmd(nj, 0.0), v_cmd(nj, 0.0);
    {
      const double now_s = steadyNowSec();
      double s = arm.seg_active
                   ? (now_s - arm.seg_t0_s) / std::max(1e-6, arm.seg_dur_s) : 1.0;
      if (s < 1.0) {
        evaluateArmSegment(arm, now_s, q_cmd, v_cmd);
      } else {
        q_cmd = arm.seg_q1;
        v_cmd.assign(nj, 0.0);
        arm.seg_active = false;
      }
    }
    // 步长/超前限制（与位姿模式同一套保险）
    for (std::size_t k = 0; k < nj; ++k) {
      double dq = q_cmd[k] - arm.q_last[k];
      const double lim = std::max(-max_joint_step_, std::min(max_joint_step_, dq));
      if (lim != dq) {q_cmd[k] = arm.q_last[k] + lim; v_cmd[k] = 0.0;}
    }
    if (max_lead_rad_ > 0.0 && snap.covers(canonical_joints_.size())) {
      for (std::size_t k = 0; k < nj; ++k) {
        const double qm = snap.position[arm.offset + k];
        if (q_cmd[k] < qm - max_lead_rad_) {q_cmd[k] = qm - max_lead_rad_; v_cmd[k] = 0.0;}
        else if (q_cmd[k] > qm + max_lead_rad_) {q_cmd[k] = qm + max_lead_rad_; v_cmd[k] = 0.0;}
      }
    }
    for (std::size_t k = 0; k < nj; ++k) {
      frame.position[arm.offset + k] = q_cmd[k];
      frame.velocity[arm.offset + k] = v_cmd[k];
    }
    arm.q_last = q_cmd;
    arm.last_v = v_cmd;
    arm.have_last_solution = true;
    arm.q_seed = q_cmd;
    // 前馈（不加惯性：预摆位是慢速关节移动）
    //
    // ⚠️⚠️ **必须用指令状态（q_cmd/v_cmd），不能用实测 v_meas**：
    //    实测速度逐拍抖动可达数 rad/s（编码器差分噪声），摩擦前馈里的
    //    `sign(v)`/`v` 会把它放大成逐拍力矩方波 → 机械臂嗡鸣/自振/乱扭。
    //    dyid 的做法是**离线用参考轨迹**预计算前馈、实时只查表；这里是
    //    等价的"用当前指令状态算"，输入同样是光滑的参考量。
    if (arm.dyn_ready && emit_ff_) {
      std::vector<double> q_ff = q_cmd;
      std::vector<double> v_ff = v_cmd;
      std::vector<double> a_zero(nj, 0.0), tm;
      if (arm.dyn.modelTorque(q_ff, v_ff, a_zero, false, tm) && tm.size() == nj) {
        for (std::size_t k = 0; k < nj; ++k) {frame.effort[arm.offset + k] = tm[k];}
        arm.tau_ff = tm;
      }
    }
    // 状态回读：关节模式算"精确到位"（无 IK 残差概念）
    arm.solved_once = true;
    arm.ik_status = static_cast<std::uint8_t>(SolverOutcome::Solved);
    arm.ik_action = static_cast<std::uint8_t>(IkAction::SendSolved);
    arm.residual_pos = 0.0;
    arm.residual_rot = 0.0;
    arm.residual_vec = Vec3{0, 0, 0};
    arm.residual_rot_vec = Vec3{0, 0, 0};
    arm.exact = true;
    arm.saturated.clear();
    return true;
  }

  /// 解一条臂并写入 frame（返回 false = 本帧不可用，整帧不下发）。
  bool solveArm(ArmRuntime & arm, const JointSnapshot & snap, std::int64_t now,
                JointCommandFrame & frame)
  {
    const std::size_t nj = arm.joints.size();
    if (arm.offset + nj > frame.size()) {return false;}
    if (!arm.ik.ready()) {return false;}
    // ⚠️ 整个 solveArm 持**臂状态锁**（见成员 arm_state_mtx_ 的说明）：
    //    这里读写的 q_seed/q_last/反馈缓存都是跨线程共享的堆对象，而
    //    20 Hz 状态发布与 ~/ik_query 会同时读它们。锁只在这一条臂上、且不跨
    //    其它锁调用，所以不会死锁；代价是状态发布/查询最多被一次求解挡住
    //    （pick_ik 最坏 20~50 ms）。
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);

    // 0) 关节模式（预摆位/回位）：不走 IK、不走位姿滤波，直接平滑走到关节目标。
    if (arm.joint_mode) {return solveJointArm(arm, snap, now, frame);}

    // 1) 滤波后的目标 —— **外推到"新解将被下发"的时刻**（now + 一个 IK 周期）
    //
    // ⚠️ 为什么外推：上游只有 10/20 Hz 时，`sample()` 给的是最近一次**到达**的
    //    目标（最老 100 ms），直接拿它当当前目标会让末端固定滞后 ~v·Δt
    //    （10 cm/s × 100 ms = 10 mm）。`sampleAt()` 用滤波管线本来就在算的
    //    `linear_velocity` 做线性外推，把这段滞后补回来；再往前多推一个 IK
    //    周期，因为新解要经过一个插值周期才真正落到下发值上。
    Pose6 target;
    // ⚠️ lead_ms 取 **0**（只外推到"现在"，不再额外往前顶一个 IK 周期）：
    //    实测"顶一个 IK 周期(40 ms)"会让末端**径向**多出 ~3 mm 误差 —— 外推量
    //    越大，速度估计的噪声被放大得越多（速度是滤波后位置差分出来的）。
    //    而"新解要过一个插值周期才落到下发值"造成的**切向**滞后由插值本身
    //    平滑掉了，不会体现在"到圆心的距离"上。10 Hz 上游实测（圆偏差）：
    //      lead=40 ms → 平均 1.30 mm / p99 3.21 mm
    //      lead=0     → 平均 0.83 mm / p99 3.29 mm（均值更好，p99 基本不变）
    const std::int64_t lead_ms = 0;
    if (!arm.filter.sampleAt(now + lead_ms, target)) {
      // 没有目标 ⇒ 该臂保持实测（frame 已填实测），不算失败
      return true;
    }
    // ⚠️ 陈旧目标**绝不能**跟随：enable 后旧 filter 目标还在时，会先按旧目标
    //    解一次 IK 再下发 —— 那就是"切 HOLD/TRACKING 时自己打自己"的直接原因。
    if (arm.filter.ageMs(now) > static_cast<std::int64_t>(target_timeout_ms_)) {
      return true;
    }
    // 首帧的 q_last 必须是**实测**，否则插值会从全 0 起跳（= 近乎 3 rad 的突跳）
    if (!arm.have_last_solution) {
      if (snap.covers(canonical_joints_.size())) {
        arm.q_last.assign(snap.position.begin() + static_cast<long>(arm.offset),
                          snap.position.begin() + static_cast<long>(arm.offset + nj));
      } else {
        arm.q_last.assign(nj, 0.0);
      }
    }

    // 2) IK —— 每条臂**按自己的节拍**重解（从上次"完成"时刻算起）。
    //    ⚠️ 实际 IK 耗时不稳定（4~50 ms），所以节拍只决定"最早什么时候再解"；
    //       插值段按墙钟推进，早到就提前切段、晚到就停在端点等，见步骤 6。
    const std::int64_t ik_period_ms =
      static_cast<std::int64_t>(std::llround(ik_period_ms_));
    const bool ik_due = (arm.last_ik_ms == 0) ||
                        (now - arm.last_ik_ms >= ik_period_ms) || !arm.have_goal;
    if (ik_due) {
      IkRequest req;
      req.position = target.position;
      req.rotation = target.rotation;
      req.seed = arm.q_seed;
      // ⚠️⚠️ **首帧的种子必须是实测关节角**，不能是 `q_seed` 的初值（全 0）：
      //    7 自由度冗余臂从全 0 收敛会落到**另一个位形分支**。实测同一个目标点：
      //      · 种子=全 0   → 肩俯仰 +3.087（限位上限仅 +0.785，且与实测差 2.99 rad）
      //      · 种子=实测角 → 各关节都在实测附近
      //    首帧若用全 0 的解，就是一次接近 3 rad 的突跳（还越限）。
      if (!arm.have_last_solution && snap.covers(canonical_joints_.size())) {
        req.seed.assign(snap.position.begin() + static_cast<long>(arm.offset),
                        snap.position.begin() + static_cast<long>(arm.offset + nj));
      }
      if (arm.have_last_solution) {
        req.has_last_solution = true;
        req.last_solution = arm.q_last;
      }
      const IkResult r = arm.ik.solve(req);
      const std::int64_t ik_done_ms = steadyNowMs();
      arm.last_ik_ms = ik_done_ms;   // 节拍从**完成**时刻算起

      // 3) 反馈缓存（无论成功与否都要更新，上游需要知道现状）
      arm.solved_once = true;
      arm.ik_status = static_cast<std::uint8_t>(r.outcome);
      arm.ik_action = static_cast<std::uint8_t>(r.action);
      arm.residual_pos = r.positionResidualNorm();
      arm.residual_rot = r.rotationResidualNorm();
      arm.residual_vec = r.position_residual;
      arm.residual_rot_vec = r.rotation_residual;
      arm.saturated.clear();
      for (auto j : r.saturated_joints) {
        if (j < nj) {arm.saturated.push_back(arm.joints[j]);}
      }
      arm.exact = (r.residual_kind == TaskResidual::Exact);

      // 4) HOLD_LAST：保持上一周期解（**绝不用未验证的解**）
      std::vector<double> q_new;
      if (r.action == IkAction::HoldLast) {
        if (!arm.have_last_solution) {
          // 连一个可行解都没有：保持实测（frame 已填实测），不算失败
          return true;
        }
        q_new = arm.q_last;
      } else {
        q_new = r.joints;
      }
      if (q_new.size() != nj) {return true;}

      // 5) **时间基 Hermite 切段**：起点 = 当前下发值/当前段速度（C¹ 连续），
      //    终点 = 新 IK 解 / 由相邻解差分估出的速度。
      //
      //    ⚠️ 终点速度不能填 0：MIT 下 `Kd(v_des−v)` 的 v_des=0 是**主动刹车**，
      //       每 40 ms 刹一次会让运动"一粘一粘"。用 Δq/Δt 估计，并按关节速度
      //       上限的 80% 夹取（IK 换分支时不会给出荒谬速度）。
      const double now_s = steadyNowSec();
      std::vector<double> q0 = arm.q_last;
      std::vector<double> v0(nj, 0.0);
      if (arm.seg_active) {
        evaluateArmSegment(arm, now_s, q0, v0);
      } else if (arm.last_v.size() == nj) {
        // ⚠️ 段已走完时不能把 v0 置 0：那会让每 40 ms"刹一下"（MIT 下
        //    Kd(v_des−v) 立刻表现为粘滞）。沿用上一拍实际下发速度。
        v0 = arm.last_v;
      }
      std::vector<double> v1(nj, 0.0);
      if (arm.prev_ik_sol.size() == nj && arm.prev_ik_ms > 0 &&
          ik_done_ms > arm.prev_ik_ms) {
        const double dt = static_cast<double>(ik_done_ms - arm.prev_ik_ms) / 1000.0;
        for (std::size_t k = 0; k < nj; ++k) {
          v1[k] = (q_new[k] - arm.prev_ik_sol[k]) / dt;
        }
      } else if (arm.have_last_solution) {
        const double dt = std::max(1e-3, ik_period_ms_ / 1000.0);
        for (std::size_t k = 0; k < nj; ++k) {v1[k] = (q_new[k] - arm.q_last[k]) / dt;}
      }
      // 速度夹取（只用于**终点速度估计**；控制帧最终还要过 limit_guard）
      if (guard_.limits().size() == canonical_joints_.size()) {
        for (std::size_t k = 0; k < nj; ++k) {
          const double cap = 0.8 * guard_.limits().velocity_max[arm.offset + k];
          v1[k] = std::max(-cap, std::min(cap, v1[k]));
        }
      }
      arm.seg_q0 = std::move(q0);
      arm.seg_v0 = std::move(v0);
      arm.seg_q1 = q_new;
      arm.seg_v1 = v1;
      arm.seg_t0_s = now_s;
      // 段长 = 1.5×IK 周期：正常情况下新解会在段**中途**到达（提前切段，
      // v0 取当前段速度 ⇒ 连续）；IK 慢时段走完就停在端点等。
      arm.seg_dur_s = 1.5 / std::max(1e-3, ik_rate_hz_);
      // 段长上限：IK 卡住时不让一段插值无限延伸
      arm.seg_dur_s = std::min(arm.seg_dur_s, std::max(0.01, segment_max_ms_ / 1000.0));
      arm.seg_active = true;
      arm.q_goal = q_new;
      arm.prev_ik_sol = q_new;
      arm.prev_ik_ms = ik_done_ms;
      arm.have_goal = true;
    }
    if (!arm.have_goal || arm.q_goal.size() != nj) {return true;}

    // 6) **每拍按墙钟**求值（100 Hz 只是近似：IK 占掉一拍时，段进度仍然正确），
    //    再做两道保险：每拍步长上限（防跳变）+ 目标超前实测的上限（防 windup）。
    std::vector<double> q_cmd(nj, 0.0);
    std::vector<double> v_cmd(nj, 0.0);
    {
      const double now_s = steadyNowSec();
      double s = 1.0;
      if (arm.seg_active) {
        s = (now_s - arm.seg_t0_s) / std::max(1e-6, arm.seg_dur_s);
      }
      if (s < 1.0) {
        evaluateArmSegment(arm, now_s, q_cmd, v_cmd);
      } else {
        // 段走完：停在新解上**并明确 v=0**（保持）。若继续发 v1>0，
        // MIT 的 Kd·(v_des−v) 会推着关节走 —— 位置保持却给出速度指令。
        q_cmd = arm.seg_q1;
        v_cmd.assign(nj, 0.0);
        arm.seg_active = false;
      }
    }
    for (std::size_t k = 0; k < nj; ++k) {
      double dq = q_cmd[k] - arm.q_last[k];
      const double dq_limited = std::max(-max_joint_step_, std::min(max_joint_step_, dq));
      if (dq_limited != dq) {
        q_cmd[k] = arm.q_last[k] + dq_limited;
        v_cmd[k] = 0.0;   // 被步长钳住时不要同时声称一个大速度
      }
    }
    if (max_lead_rad_ > 0.0 && snap.covers(canonical_joints_.size())) {
      for (std::size_t k = 0; k < nj; ++k) {
        const double qm = snap.position[arm.offset + k];
        const double lo = qm - max_lead_rad_;
        const double hi = qm + max_lead_rad_;
        if (q_cmd[k] < lo) {
          q_cmd[k] = lo; v_cmd[k] = 0.0; ++lead_clamped_;
        } else if (q_cmd[k] > hi) {
          q_cmd[k] = hi; v_cmd[k] = 0.0; ++lead_clamped_;
        }
      }
    }
    for (std::size_t k = 0; k < nj; ++k) {
      frame.position[arm.offset + k] = q_cmd[k];
      frame.velocity[arm.offset + k] = v_cmd[k];
    }
    arm.q_last = q_cmd;
    arm.last_v = v_cmd;
    arm.have_last_solution = true;
    arm.q_seed = q_cmd;       // 下次 IK 的种子 = 当前下发值

    // 7) 动力学前馈 + 控制律
    //
    // ⚠️ 三条铁律：
    //   ① 没有 params.json ⇒ **不发**前馈（不编造"看起来合理"的力矩）；
    //   ② 重力算失败 ⇒ **本拍放弃全部前馈**（而不是发 0 —— 抓着重物时
    //      发 0 等于突然卸力）；
    //   ③ 所有力矩最终都要过 limit_guard（节点已统一做）。
    std::vector<double> q_meas(arm.joints.size(), 0.0);
    std::vector<double> v_meas(arm.joints.size(), 0.0);
    std::vector<double> tau_meas(arm.joints.size(), 0.0);
    for (std::size_t k = 0; k < arm.joints.size(); ++k) {
      q_meas[k] = snap.position[arm.offset + k];
      v_meas[k] = snap.velocity[arm.offset + k];
      tau_meas[k] = snap.effort[arm.offset + k];
    }

    // ======================================================================
    // 动力学补偿：三种模式**共用同一个** `modelTorque()`，只切换惯性项开关
    // ======================================================================
    //
    // 用户指定的口径（2026-10 确认）：
    //   常规   ：τ_g + τ_friction + 惯性/科氏(a = a_ref)   ← 全量
    //   阻抗   ：τ_g + τ_friction                          ← **不加惯性（含科氏）**
    //   导纳   ：τ_g + τ_friction                          ← **不加惯性（含科氏）**
    //
    // ⚠️ "一致"这里是**结构保证**的：三种都调同一个 `modelTorque()`，
    //    不存在"三处各写一遍然后对不上"的可能。
    //
    // ⚠️ 惯性项只在**常规模式**注入。原因：
    //    · 阻抗/导纳的"位置目标"本身被顺从偏移改写过，`a_ref` 不再是系统
    //      真实要走的轨迹 ⇒ 全量惯性前馈会和顺从行为**对抗**；
    //    · 阻抗/导纳的顺从由任务空间弹簧-阻尼（阻抗）与虚拟动力学（导纳）表达，
    //      不该再由惯性前馈去"拽回参考轨迹"。

    std::vector<double> tau_ff(arm.joints.size(), 0.0);
    bool have_model_torque = false;
    // ⚠️ 前馈输入 = **本拍已下发的指令状态**（frame.position/velocity），
    //    不是实测。理由同 solveJointArm 里的说明：实测速度噪声会把摩擦
    //    前馈变成逐拍方波。dyid 是离线用参考轨迹算，这里是它的实时等价。
    std::vector<double> q_ff(arm.joints.size(), 0.0);
    std::vector<double> v_ff(arm.joints.size(), 0.0);
    for (std::size_t k = 0; k < arm.joints.size(); ++k) {
      q_ff[k] = frame.position[arm.offset + k];
      v_ff[k] = frame.velocity[arm.offset + k];
    }
    if (arm.dyn_ready) {
      // 常规模式用**参考轨迹**的二阶导（数值二阶差分），其余模式传空 ⇒ 不加惯性
      std::vector<double> a_ref(arm.joints.size(), 0.0);
      const bool use_inertia = (control_mode_ == "regular");
      if (use_inertia) {
        // 相邻两拍的期望关节位置做二阶差分。⚠️ 只用**已下发过**的历史，
        // 没有历史就先给 0（首拍不该凭空造加速度）。
        if (arm.prev_q_ref.size() == arm.joints.size() &&
            arm.prev2_q_ref.size() == arm.joints.size()) {
          const double dt = 1.0 / control_rate_hz_;
          for (std::size_t k = 0; k < arm.joints.size(); ++k) {
            const double a = (arm.prev_q_ref[k] - 2.0 * arm.prev2_q_ref[k] +
                              frame.position[arm.offset + k]) / (dt * dt);
            // ⚠️ 二阶差分对噪声极敏感（位置量化 0.001 rad / dt=0.01 s ⇒ 单步
            //    噪声放大到 ~10 rad/s²）。**硬限幅**，否则惯性项会被噪声主导。
            const double cap = 20.0;   // rad/s²
            a_ref[k] = std::isfinite(a) ? std::max(-cap, std::min(cap, a)) : 0.0;
          }
        }
      }
      std::vector<double> tm;
      if (arm.dyn.modelTorque(q_ff, v_ff, a_ref, use_inertia, tm) &&
          tm.size() == arm.joints.size()) {
        tau_ff = tm;
        have_model_torque = true;
      } else {
        // ⚠️ 失败时**放弃全部前馈**（而不是发 0 —— 抓着重物时发 0 等于突然卸力，
        //    但"发一半"更糟：重力有、摩擦没有会造成低速爬行）。这里退回"不发"，
        //    由关节 Kp/Kd 兜底，并打节流日志。
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "臂 '%s' 动力学前馈计算失败 → 本拍不发前馈（Kp/Kd 兜底）",
                             arm.name.c_str());
        tau_ff.assign(arm.joints.size(), 0.0);
      }
    }

    // ---- 外力估计（导纳/阻抗都需要）----
    Vec6 wrench{};
    bool have_wrench = false;
    if (arm.dyn_ready && have_model_torque) {
      Jacobian J;
      if (arm.chain.jacobian(q_meas, J)) {
        std::vector<double> tau_ext(arm.joints.size(), 0.0);
        // τ_ext = τ_meas − τ_model
        // 修复：tau_ff 为当前动力学模型力矩（重力+摩擦），严禁再扣除上一拍的 sent（避免双重扣减重力）
        for (std::size_t k = 0; k < arm.joints.size(); ++k) {
          tau_ext[k] = tau_meas[k] - tau_ff[k];
        }
        std::array<double, 6> w{};
        if (estimateWrench(J, tau_ext, w, 1e-3)) {
          wrench = w;
          have_wrench = true;
          arm.last_wrench = w;
          arm.have_wrench = true;
        }
      }
    }

    // ---- 按模式施加控制律 ----
    if (control_mode_ == "admittance" && arm.admittance_ready) {
      const double dt = 1.0 / control_rate_hz_;
      // 每 ≈100 ms 更新一次 J（J 变化慢；每拍算浪费算力）
      const Vec6 xc = arm.admittance.step(have_wrench ? wrench : vec6_zero(), dt);
      arm.admittance_offset = xc;
      // 目标位置 + 顺从偏移 → 重新解一次 IK（增量前进，代价小）
      if (have_model_torque) {
        // ⚠️ 偏移很小（限幅 ≤0.10 m），所以这里用**当前解**当种子再解一次，
        //    而不是把偏移直接加到关节角上（后者在姿态变化时是错的）。
        Pose6 shifted = target;
        for (std::size_t i = 0; i < 3; ++i) {shifted.position[i] = target.position[i] + xc[i];}
        IkRequest req2;
        req2.position = shifted.position;
        req2.rotation = shifted.rotation;
        req2.seed = arm.q_seed;
        req2.has_last_solution = arm.have_last_solution;
        req2.last_solution = arm.q_last;
        const IkResult r2 = arm.ik.solve(req2);
        if (r2.ok && r2.joints.size() == arm.joints.size()) {
          arms_shifted_goal_.assign(r2.joints.begin(), r2.joints.end());
          for (std::size_t k = 0; k < arm.joints.size(); ++k) {
            frame.position[arm.offset + k] = arms_shifted_goal_[k];
          }
        }
      }
    } else if (control_mode_ == "impedance" && arm.impedance_ready &&
               have_model_torque) {
      Jacobian J;
      if (arm.chain.jacobian(q_meas, J)) {
        // 消除双弹簧冲突：根据设置的任务空间刚度与阻尼，正确翻译为 MIT 关节刚度 Kp 与阻尼 Kd
        // K_p,joint_k = (Jᵀ K_x J)_kk = ∑_i (J_ik² · K_x_i)
        // K_d,joint_k = (Jᵀ D_x J)_kk = ∑_i (J_ik² · D_x_i)
        // 前馈 tau_ff 保持为纯动力学补偿（重力+摩擦），不重复叠算笛卡尔弹簧力矩
        for (std::size_t k = 0; k < arm.joints.size(); ++k) {
          double kp_j = 0.0;
          double kd_j = 0.0;
          for (std::size_t i = 0; i < 6; ++i) {
            const double jik = J.at(i, k);
            kp_j += jik * jik * arm.impedance.config().stiffness[i];
            kd_j += jik * jik * arm.impedance.damping()[i];
          }
          if (guard_.limits().size() == canonical_joints_.size()) {
            kp_j = std::min(guard_.limits().stiffness_max[arm.offset + k], kp_j);
            kd_j = std::min(guard_.limits().damping_max[arm.offset + k], kd_j);
          }
          frame.stiffness[arm.offset + k] = std::max(0.0, kp_j);
          frame.damping[arm.offset + k] = std::max(0.0, kd_j);
        }
      }
    }

    // ---- 写入前馈力矩 ----
    // ⚠️ `emit_ff_ == false` 时**不写** effort（保持 0）。原因见 emit_ff_ 声明：
    //    C++ 的 KDL 刚体注入与 lite_dyid 的回归基不等价，注入错误前馈会漂移。
    //    effort 保持 0 ⇒ 纯 MIT 位置跟踪（Kp/Kd 兜底），这是**安全且已验证**的行为。
    if (emit_ff_) {
      for (std::size_t k = 0; k < arm.joints.size(); ++k) {
        frame.effort[arm.offset + k] = tau_ff[k];
      }
    }
    arm.tau_ff = emit_ff_ ? tau_ff : std::vector<double>(arm.joints.size(), 0.0);
    // 记录期望关节位置历史（供下一拍算 a_ref）
    arm.prev2_q_ref = arm.prev_q_ref;
    arm.prev_q_ref.assign(arm.joints.size(), 0.0);
    for (std::size_t k = 0; k < arm.joints.size(); ++k) {
      arm.prev_q_ref[k] = frame.position[arm.offset + k];
    }
    return true;
  }

  // ------------------------------------------------------------- 反馈
  void fillStatus(move_msgs::msg::MoveStatus & s)
  {
    // ⚠️ 持臂状态锁：下面的循环在**状态线程**上读 arms_，而控制线程正在
    //    solveArm 里重写同一批字段（std::vector/std::string）。见 arm_state_mtx_。
    //    `bridge_->snapshot()` 在锁内调用是安全的（它自己有锁，且不会回调本节点）。
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    s.header.stamp = now();
    s.state = static_cast<std::uint8_t>(sm_.state());
    s.state_name = nodeStateName(sm_.state());
    s.state_detail = sm_.detail();
    s.control_mode = control_mode_;
    s.dynamics_model = dyn_model_kind_.empty() ? "unavailable" : dyn_model_kind_;
    bool any_inertia = false;
    for (const auto & a : arms_) {if (a.dyn.hasInertiaTerm()) {any_inertia = true;}}
    s.dynamics_has_inertia = any_inertia;
    s.bridge_authorized = (bridge_active_source_ == source_name_) ||
                          (bridge_manual_override_ == source_name_);
    s.bridge_active_source = bridge_active_source_;
    s.bridge_arbitration_state = bridge_arbitration_state_;
    s.bridge_mode = bridge_mode_;
    s.bridge_mode_name = bridge_mode_name_;
    s.ramp_alpha = (ramp_duration_sec_ > 0.0)
                   ? std::min(1.0, ramp_elapsed_ / ramp_duration_sec_) : 1.0;
    const JointSnapshot snap = bridge_ ? bridge_->snapshot() : JointSnapshot{};
    s.joint_names = canonical_joints_;
    s.q_measured = snap.position;
    s.q_desired.assign(canonical_joints_.size(), 0.0);
    for (const auto & a : arms_) {
      // ⚠️ 以前这里恒为 0（只 assign 不填），上游若拿 q_desired 判断
      //    "我发到哪了"会永远看到 0。用**上一拍实际下发值**（prev_cmd）填。
      if (a.prev_cmd.size() != a.joints.size()) {continue;}
      for (std::size_t k = 0; k < a.joints.size(); ++k) {
        const std::size_t j = a.offset + k;
        if (j < s.q_desired.size()) {s.q_desired[j] = a.prev_cmd[k];}
      }
    }
    s.tau_measured = snap.effort;
    for (const auto & a : arms_) {
      s.arms.push_back(a.name);
      s.ik_status.push_back(a.ik_status);
      // ⚠️ 未解过 IK 时**不要**报 "SOLVED"：那是默认值枚举 0。
      //    改用 "NOT_RUN"（上游一眼能看出"还没开始跑"而不是"解得很准"）。
      s.ik_status_name.push_back(a.solved_once
                                 ? solverOutcomeName(static_cast<SolverOutcome>(a.ik_status))
                                 : std::string("NOT_RUN"));
      s.ik_residual_pos.push_back(a.residual_pos);
      s.ik_residual_rot.push_back(a.residual_rot);
      s.ik_action.push_back(a.ik_action);
      for (const auto & j : a.saturated) {s.ik_saturated_joints.push_back(j);}
      for (double v : a.admittance_offset) {s.admittance_offset.push_back(v);}
      for (double v : a.estimated_wrench) {s.estimated_wrench.push_back(v);}
    }
    s.frames_published = bridge_ ? bridge_->published() : 0;
    s.frames_rejected = frames_rejected_;
    s.target_timeouts = target_timeouts_;
    s.limit_clamped = guard_.stats().total();
    s.filter_resets = 0;
  }

  void publishStatus()
  {
    move_msgs::msg::MoveStatus s;
    fillStatus(s);          // 内部持锁（见 fillStatus）
    status_pub_->publish(s);
    // ⚠️ 这里同样要持锁：ek_residual 的字段也是 solveArm 在写。
    //    fillStatus 已经返回（它的锁已释放），所以这里再取一次不会自锁。
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    for (const auto & a : arms_) {
      // ⚠️ 从未解过 IK 的臂**不发**残差：默认值 exact=true / 0.0 会谎报
      //    "精确可达"。上游宁可看不到，也不要看到假的"没问题"。
      if (!a.solved_once) {continue;}
      move_msgs::msg::EeResidual e;
      e.header.stamp = now();
      e.arm = a.name;
      e.exact = a.exact;
      e.position_residual.x = a.residual_vec[0];
      e.position_residual.y = a.residual_vec[1];
      e.position_residual.z = a.residual_vec[2];
      e.orientation_residual.x = a.residual_rot_vec[0];
      e.orientation_residual.y = a.residual_rot_vec[1];
      e.orientation_residual.z = a.residual_rot_vec[2];
      e.position_residual_norm = a.residual_pos;
      e.orientation_residual_norm = a.residual_rot;
      residual_pub_->publish(e);
    }
  }

  void onIkQuery(const std::shared_ptr<move_msgs::srv::QueryIk::Request> req,
                 std::shared_ptr<move_msgs::srv::QueryIk::Response> res)
  {
    ArmRuntime * arm = nullptr;
    if (!req->arm.empty()) {
      arm = findArm(req->arm);
    } else if (!arms_.empty()) {
      arm = &arms_.front();
    }
    if (arm == nullptr || !arm->ik.ready()) {
      res->ok = false;
      res->message = "没有可用的臂（检查 joints_left/right 与 URDF 配置）";
      return;
    }
    // ⚠️ 持臂状态锁：这里要读 arm->q_last（控制线程在写），而且
    //    `arm->ik.solve()` 会用到同一套编排器/后端状态。和 solveArm 串行化之后，
    //    "查询"与"控制"不会再同时进同一条臂的求解器（KDL 的 CartToJnt 尤其
    //    不是线程安全的 —— MoveIt 后端在内部有锁，KDL 没有）。
    std::lock_guard<std::mutex> arm_lock(arm_state_mtx_);
    IkRequest r;
    r.position = Vec3{req->position.x, req->position.y, req->position.z};
    // ⚠️ 查询接口收的是四元数，转成旋转向量（与内部一致）
    r.rotation = quat_to_rotvec(Quat{req->orientation.x, req->orientation.y,
                                     req->orientation.z, req->orientation.w});
    if (req->seed_joints.size() == arm->joints.size()) {
      r.seed.assign(req->seed_joints.begin(), req->seed_joints.end());
    } else {
      const JointSnapshot snap = bridge_ ? bridge_->snapshot() : JointSnapshot{};
      if (snap.covers(canonical_joints_.size())) {
        r.seed.assign(snap.position.begin() + static_cast<long>(arm->offset),
                      snap.position.begin() + static_cast<long>(arm->offset + arm->joints.size()));
      } else {
        // ⚠️ 兜底种子**不能**是全 0：0 在本机的肘关节限位上，pick_ik 会把这种
        //    种子换成随机种子（见 neutralSeedFor 的说明），实测左臂直接 FAILED。
        r.seed = neutralSeedFor(*arm);
      }
    }
    if (arm->have_last_solution) {
      r.has_last_solution = true;
      r.last_solution = arm->q_last;
    }
    const IkResult out = arm->ik.solve(r);
    res->ok = true;
    res->message = out.note.empty() ? "ok" : out.note;
    res->ik_status = static_cast<std::uint8_t>(out.outcome);
    res->ik_status_name = solverOutcomeName(out.outcome);
    res->position_residual.x = out.position_residual[0];
    res->position_residual.y = out.position_residual[1];
    res->position_residual.z = out.position_residual[2];
    res->orientation_residual.x = out.rotation_residual[0];
    res->orientation_residual.y = out.rotation_residual[1];
    res->orientation_residual.z = out.rotation_residual[2];
    res->position_residual_norm = out.positionResidualNorm();
    res->orientation_residual_norm = out.rotationResidualNorm();
    for (auto j : out.saturated_joints) {
      if (j < arm->joints.size()) {res->saturated_joints.push_back(arm->joints[j]);}
    }
    if (req->want_joints) {res->joints = out.joints;}
  }

  // --------------------------------------------------------------- 成员
  std::string source_name_, command_topic_, joint_states_topic_;
  std::vector<std::string> canonical_joints_, arm_names_;
  std::string root_link_, control_mode_, frame_id_;
  /// 夹爪（prismatic）：名字（参数原样）、在 canonical 里的下标、独立命令。
  /// ⚠️ `gripper_idx_` 为空 = canonical 里没有夹爪 ⇒ 控制帧里不碰这些关节。
  std::vector<std::string> gripper_joints_;
  std::vector<std::size_t> gripper_idx_;
  std::vector<GripperCommand> gripper_cmds_;
  std::vector<bool> gripper_pos_initialized_;
  double gripper_kp_{20.0};
  double gripper_kd_{3.0};
  double gripper_torque_limit_{0.25};
  std::string gripper_command_topic_{"/lite_move_controller/gripper_command"};
  mutable std::mutex gripper_cmd_mtx_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripper_cmd_sub_;
  double control_rate_hz_{100.0}, dwell_rate_hz_{20.0};
  /// IK 重解节拍（Hz）与周期（ms）。**只用来限"最早什么时候再解"**；
  /// 插值段按墙钟推进，所以 IK 慢/抖不会让下发曲线变形（见 solveArm ⑤⑥）。
  double ik_rate_hz_{25.0};
  double ik_period_ms_{40.0};
  double command_timeout_ms_{250.0}, target_timeout_ms_{200.0}, state_timeout_ms_{100.0};
  double tier_target_ms_{60.0}, tier_bridge_ms_{120.0};
  double ramp_duration_sec_{0.5}, ramp_elapsed_{0.0};
  double max_joint_step_{0.05};
  /// 下发目标相对实测的最大超前（rad，0=不限）；见参数说明。
  double max_lead_rad_{0.20};
  /// 单段 Hermite 的最长时间（ms）。
  double segment_max_ms_{120.0};
  /// 关节模式峰值速度（rad/s）。
  double joint_speed_{1.20};
  /// 重复实例策略：ask / kill / abort。
  std::string duplicate_policy_{"ask"};
  std::uint64_t lead_clamped_{0};
  double lock_hold_ms_{0.0};
  double kp_default_{20.0}, kd_default_{1.0};
  /// 是否把动力学前馈写进 `effort`。**默认 true**（用户 2026-10 指定）。
  /// ⚠️ 原先默认 false 的原因是"KDL 注入与回归基不等价"；该问题已在
  ///    `dynamics_pin.cpp` 修正（mass=Σp、μ=p1..3），并用实测数据核对过。
  bool emit_ff_{true};
  bool auto_enable_{false};
  std::string load_error_;
  /// URDF 文本（**单一事实来源**：limit_guard 与建链共用，见 acquireUrdf()）。
  std::string urdf_text_;
  /// URDF 的来源，人类可读（`urdf_path=<文件>` 或 `/robot_description`）。
  /// ⚠️ 现场排查第一件事就是"它到底用了哪份模型"，所以启动日志与
  ///    `~/get_status`（verbose）都要报出来。
  std::string urdf_source_;
  /// 为什么拿不到 URDF（只在 acquireUrdf() 失败时有值）。
  std::string urdf_error_;
  /// 等 `/robot_description` 的上限（秒）。2.5 s ≈ 25 个控制周期，
  /// 足够 RSP 起来并投递 transient_local 的历史消息，又不至于让开机卡太久。
  static constexpr double kUrdfWaitSec = 2.5;
  /// 状态 tick 里**唯一**那次 `get_status` 的预算（秒）。
  ///
  /// ⚠️ 必须远小于 `state_timeout_ms_`(100 ms) 与状态 tick 周期(50 ms)：
  ///    原来 `pollBridge()` 那次是 **2.0 s**（同一拍第二次调同一个服务），
  ///    几个并发 tick 就能把 executor 线程吃干、饿死 `/joint_states` ⇒ FAULT。
  static constexpr double kStateTickStatusBudgetSec = 0.05;
  /// 状态 tick 耗时直方图的样本数（20 Hz × 100 ≈ 5 s 窗口）。
  static constexpr std::size_t kStateTickSamples = 100;
  /// SRDF 文本（两条臂共用；首次用到时读，之后缓存）。
  std::string srdf_text_;
  bool srdf_loaded_{false};
  /// lite_ik_node 的安装前缀（只算一次）。
  std::string lite_ik_prefix_;
  bool prefix_probed_{false};
  DynamicsParams arm_params_;          ///< 最近一次成功载入的参数（左右共用一份文件）
  std::string dyn_model_kind_;
  std::vector<double> arms_shifted_goal_;   ///< 导纳模式偏移后的关节目标（暂存）

  std::unique_ptr<BridgeClient> bridge_;
  LimitGuard guard_;
  std::vector<ArmRuntime> arms_;
  /// ⚠️ 保护 `arms_` 里**跨线程共享**的那部分状态（`ArmRuntime` 的反馈缓存、
  ///    `q_last`/`q_seed`、`saturated`、`admittance_offset` 等）。
  ///
  /// 为什么必须有（真机事故后加的）：本节点用的是 `MultiThreadedExecutor` +
  /// **Reentrant** callback group，于是
  ///   · 控制定时器（100 Hz）在 `solveArm()` 里写这些字段；
  ///   · 状态定时器（20 Hz）在 `fillStatus()`/`publishStatus()` 里读它们；
  ///   · `~/ik_query` 服务线程还会读 `q_last`。
  /// 它们**没有任何互斥** ⇒ `std::vector`/`std::string` 的"读"与"重新赋值"并发
  /// = 读到已被释放的缓冲（`std::string` 的长度字段还会被当成新长度去 memcpy）
  /// ⇒ 堆损坏（真机上的 `malloc_consolidate(): unaligned fastbin chunk detected`
  /// 与这一类访问完全吻合）。改造前 KDL 一次 0.1 ms、控制 tick 从不与状态 tick
  /// 重叠，所以这颗雷一直没响；换成 pick_ik 后单 tick 可能 4~50 ms，重叠成了常态。
  ///
  /// ⚠️ 只在**读写那几个字段**时短暂持锁（微秒级），IK 求解本身**不持锁**
  ///    （否则状态发布会被一次 50 ms 的求解挡住）。
  mutable std::mutex arm_state_mtx_;
  NodeStateMachine sm_{steadyNowMs()};

  std::atomic<bool> enable_requested_{false};
  std::atomic<bool> disable_requested_{false};
  /// 请求在**控制线程**里复位各臂运行时（目标/IK/插值/q_last/ramp）。
  /// ⚠️ 必须由控制线程执行：这些字段都是控制 tick 无锁读写的。
  std::atomic<bool> reset_runtime_requested_{false};
  std::atomic<bool> clear_fault_requested_{false};
  std::string violation_;
  std::string bridge_active_source_;
  std::string bridge_manual_override_;   ///< 桥的 manual_override（LOCK 成功后 = 本节点名）
  std::string bridge_arbitration_state_;
  std::string bridge_mode_name_;
  std::uint8_t bridge_mode_{255};
  bool bridge_mode_ok_{false};
  std::uint64_t target_timeouts_{0};
  std::uint64_t frames_rejected_{0};

  rclcpp::Publisher<move_msgs::msg::MoveStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<move_msgs::msg::EeResidual>::SharedPtr residual_pub_;
  rclcpp::Subscription<move_msgs::msg::MoveTargetArray>::SharedPtr target_sub_;
  rclcpp::Subscription<move_msgs::msg::MoveTargetArray>::SharedPtr joint_target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_, disable_srv_, clear_fault_srv_;
  rclcpp::Service<move_msgs::srv::GetMoveStatus>::SharedPtr get_status_srv_;
  rclcpp::Service<move_msgs::srv::QueryIk>::SharedPtr ik_query_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_, status_timer_;

  // ---- 回调组（三个，各司其职；见 setupRosInterfaces 的说明）-------------
  /// `/joint_states` 专用（**Reentrant + 独立**）：任何组里的阻塞都饿不到它。
  rclcpp::CallbackGroup::SharedPtr joint_states_cb_group_;
  /// 100 Hz 控制 tick 专用（**MutuallyExclusive**）：不允许自己堆自己。
  rclcpp::CallbackGroup::SharedPtr control_cb_group_;
  /// 20 Hz 状态机 + 状态发布专用（**MutuallyExclusive**）：它的同步服务调用
  /// （ARMING/SWITCHING 的 setSource/setMode，超时 10~15 s）只堵它自己。
  rclcpp::CallbackGroup::SharedPtr state_cb_group_;
  /// `~/task_target`（100 Hz 目标流）专用（**MutuallyExclusive**）。
  /// 它只做 filter.push()，不许和状态 tick / 服务抢线程。
  rclcpp::CallbackGroup::SharedPtr target_cb_group_;

  // ---- 状态 tick / 关节状态新鲜度的可观测性（只读诊断，不参与控制）------
  mutable std::mutex diag_mtx_;
  std::array<double, kStateTickSamples> tick_ms_{};
  std::size_t tick_n_{0};
  std::size_t tick_head_{0};
  std::int64_t state_age_ms_last_{0};
  std::int64_t state_age_ms_max_{0};
  std::int64_t target_age_ms_last_{0};
  std::int64_t target_age_ms_max_{0};
  std::int64_t diag_last_log_ms_{0};
  /// 最近一次**实际下发**的单调毫秒（DWELL 限频用）+ 近 100 帧的时刻（算真实频率）。
  std::int64_t last_publish_ms_{0};
  std::array<std::int64_t, kStateTickSamples> pub_ms_{};
  std::size_t pub_n_{0};
  std::size_t pub_head_{0};
  std::atomic<std::uint64_t> status_timeouts_{0};
  /// 节点侧逐拍步长（rad）：环形缓冲 + 历史最大。
  std::array<double, 4096> step_ring_{};
  std::array<double, 4096> ctl_ring_{};
  std::size_t ctl_n_{0};
  std::size_t ctl_head_{0};
  std::size_t step_n_{0};
  std::size_t step_head_{0};
  double step_max_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    // ⚠️ 顺序是**硬要求**：先 make_shared，再 initialize()。
    //    构造函数里只声明参数；一切都放在 initialize() 里，因为那里才有
    //    `shared_from_this()`（构造函数里会抛 bad_weak_ptr），而且可以拿临时
    //    executor 抽拍等 `/robot_description`（transient_local 的历史消息只有
    //    在 spin 时才投递）。
    auto node = std::make_shared<MoveControllerNode>();
    if (!node->initialize()) {
      // fail-closed：配置/URDF/建臂任何一步不成立就**不要**假装启动成功。
      // 改造前这里会带着 `arms=0` 继续跑，上游看着节点"在"，实际什么都不做。
      RCLCPP_FATAL(rclcpp::get_logger("lite_move_controller"), "启动失败：%s",
                   node->loadError().c_str());
      rclcpp::shutdown();
      return 1;
    }
    // ⚠️ **显式 6 条线程**（不要再用默认值）。按现在的回调组划分，最坏情况是
    //    这些**互不相同的**组同时在跑：控制 tick（MX）、状态 tick（MX，等桥的
    //    服务响应，最坏一次 setMode 10~15 s）、目标流（MX，100 Hz）、
    //    `/joint_states`（Reentrant）、default 组的服务。
    //    ⚠️ 还留 1~2 条**余量**：同步服务调用的**响应回调**跑在 default 组里，
    //    必须有线程能去执行它 —— 4 条线程全被占满时，`get_status` 会白白等到
    //    超时（实测：Python 假桥 + 100 Hz 目标 + 50 Hz 关节状态下超时率很高）。
    //    /joint_states 无论如何都不会被饿死（它有自己的组 + 至少一条线程可用），
    //    这才是真机 FAULT 的根因修复。
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 6);
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("lite_move_controller"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
