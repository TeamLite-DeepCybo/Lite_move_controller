// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// node_state.hpp —— 上层编排状态机（**纯逻辑**，可穷举单测）
//
// ===========================================================================
// 为什么状态机必须是纯逻辑（而不是写在 ROS 节点里）
// ===========================================================================
//
// 本项目已有两条同类理由（`lite_bridge/source_arbiter.hpp`、
// `lite_controller/activation_ramp.hpp`）：**守卫是安全关键逻辑，必须能穷举测试**。
// 若它内部调用 `rclcpp::Clock::now()` 或直接发服务，测试就得靠 sleep 与真桥 ——
// 那会变成 flaky 的时序测试，而不是"守卫不满足就一定不迁移"的证明。
//
// 所以这里所有时间都是**调用方传入的毫秒**，所有"动作"都表达为**标志**
// （`Actions`），由 ROS 外壳去执行。
//
// ===========================================================================
// 状态与迁移（设计文档 §7.2）
// ===========================================================================
//
//     IDLE ──load_ok──▶ STANDBY ──enable──▶ ARMING ──armed──▶ SWITCHING
//       │                  ▲                   │                 │
//       │                  │                   │fail             │mode_ok
////      │               disable                └──────▶ STANDBY   ▼
//       │                  │                                  RUNNING ⇄ SUSPENDED
//       │                  │                                     │  ▲
//       │                  │                            target_timeout│  │target_ok
//       │                  │                                     ▼  │
//       │                  │                                   DWELL
//       │                  │                                     │
//       │                  └──────────────disable────────────────┘
//       │
//       └─load_fail──▶ ERROR          （任意状态 ──violation──▶ FAULT ──clear──▶ STANDBY）
//
// **迁移守卫**（不满足就**停在原状态并给出原因**，绝不"猜"）：
//
//   STANDBY → ARMING    ：必须已加载（模型/IK/限位）、状态新鲜、无未清故障
//   ARMING  → SWITCHING ：桥已确认授权（`set_source{LOCK}` 回读）
//   SWITCHING → RUNNING ：桥回读 mode==2(MIT TRACKING) 且 controller_active
//   RUNNING → SUSPENDED ：桥回读 active_source ≠ 自己（被抢占）
//   RUNNING → DWELL     ：目标超时
//   FAULT   → STANDBY   ：**只能**经 clear_fault，且必须重新 enable
//                         （不允许从 FAULT 直接回 RUNNING —— 那等于"自动恢复出力"）
//
// ⚠️ **一次只发一个模式请求**：`SWITCHING` 里失败**不重试**（重试会撞上桥的
//    迁移守则并可能留下半切状态）。失败 → 退回 STANDBY 并给出原因。
#pragma once

#include <cstdint>
#include <string>

namespace lite_move_controller
{

/// 状态机的档位。
///
/// ⚠️ 数值必须与 `move_msgs/msg/MoveStatus.msg` 的 `STATE_*` 常量**逐一相同**
///    （对外契约）。`include/lite_move_controller/node_state_ros.hpp` 里有一条
///    `static_assert` 把这条约束变成**编译期**错误 —— 否则它会在运行期变成一个
///    "状态显示错了"的难查问题。
enum class NodeState : std::uint8_t
{
  Idle = 0,
  Loading = 1,
  Standby = 2,
  Arming = 3,
  Switching = 4,
  Running = 5,
  Suspended = 6,
  Dwell = 7,
  Fault = 8,
  Error = 9,
};

inline const char * nodeStateName(NodeState s) noexcept
{
  switch (s) {
    case NodeState::Idle:      return "IDLE";
    case NodeState::Loading:   return "LOADING";
    case NodeState::Standby:   return "STANDBY";
    case NodeState::Arming:    return "ARMING";
    case NodeState::Switching: return "SWITCHING";
    case NodeState::Running:   return "RUNNING";
    case NodeState::Suspended: return "SUSPENDED";
    case NodeState::Dwell:     return "DWELL";
    case NodeState::Fault:     return "FAULT";
    case NodeState::Error:     return "ERROR";
  }
  return "UNKNOWN";
}

/// 该状态下**是否允许下发** MotionCommand。
///
/// ⚠️ 这条是本文件最重要的一条规则：
///   · `STANDBY` / `ARMING` / `SWITCHING` / `SUSPENDED` / `FAULT` / `ERROR`
///     **一律不发帧**。理由：让桥的 `source_timeouts_ms` 超时 → `SAFE_HOLD`
///     兜底。**发"全 0 软帧"是错的** —— MIT 全 0 = Kp=Kd=0 = 松劲 = 手臂塌下来。
///   · `DWELL` 发**低频保持帧**（位置 = 最后目标），语义与桥的
///     `timeout_action: hold` 一致（顶住，不松劲）。
///   · `RUNNING` 全速下发。
inline bool statePublishes(NodeState s) noexcept
{
  return s == NodeState::Running || s == NodeState::Dwell;
}

/// 全速下发（而非 DWELL 的低频保持）。
inline bool statePublishesFullRate(NodeState s) noexcept
{
  return s == NodeState::Running;
}

/// 一次 tick 的输入。所有时间都是**调用方给的单调毫秒**。
struct StateInputs
{
  std::int64_t now_ms{0};

  bool load_ok{false};           ///< 模型/IK/参数/限位全部就绪
  std::string load_error;        ///< load_ok=false 时的原因

  bool enable_requested{false};  ///< 收到 ~/enable
  bool disable_requested{false}; ///< 收到 ~/disable
  bool clear_fault_requested{false};

  bool state_fresh{false};       ///< /joint_states 在超时内（否则不能激活）

  bool bridge_authorized{false}; ///< 桥回读：现在听我的
  bool bridge_mode_ok{false};    ///< 桥回读 mode==MIT TRACKING 且 controller active
  bool bridge_source_changed{false}; ///< 桥回读 active_source 变了（抢占/释放）

  bool target_fresh{false};      ///< 目标在超时内
  std::string violation;         ///< 非空 = 安全违例（NaN / 位置超差 / 求解失败连击…）
};

/// 状态机要求 ROS 外壳去做的动作。
///
/// ⚠️ 用**标志**而不是直接调服务：这样"是否请求了切模式"可以被单测断言。
struct StateActions
{
  bool request_lock_source{false};   ///< set_source{MODE_LOCK, 自己}
  bool request_mode_hold{false};     ///< set_mode{→1}（MIT HOLD）
  bool request_mode_tracking{false}; ///< set_mode{→2}（MIT TRACKING）
  bool request_release_source{false};///< set_source{AUTO}
  bool request_standby{false};       ///< 桥的 ~/standby（安全档）
  bool log_error{false};
};

/// 状态机本体。
class NodeStateMachine
{
public:
  /// 构造即进入 `Idle`。
  explicit NodeStateMachine(std::int64_t now_ms = 0) noexcept {last_change_ms_ = now_ms;}

  NodeState state() const noexcept {return state_;}
  const std::string & detail() const noexcept {return detail_;}
  std::int64_t last_change_ms() const noexcept {return last_change_ms_;}
  std::uint64_t transitions() const noexcept {return transitions_;}

  /// 推进一拍。返回本拍要求执行的动作。
  StateActions tick(const StateInputs & in) noexcept
  {
    StateActions act;
    // ⚠️ 桥回读**只有一个来源**：入参。早先同时保留 note*() 写成员 +
    //    读入参，两条路会分叉 —— 单测里就表现为"设了授权却还在 ARMING"。
    //    这里在入口统一刷一次成员，后面的 step* 只读成员，杜绝分叉。
    bridge_authorized_ = in.bridge_authorized;
    bridge_mode_ok_ = in.bridge_mode_ok;
    // ---- 全局优先级 1：安全违例（最高，任何状态都可能被打到 FAULT）----
    if (!in.violation.empty() &&
        state_ != NodeState::Fault && state_ != NodeState::Error) {
      transitionTo(NodeState::Fault, in.now_ms, in.violation);
      // 进 FAULT 时**要求交权**（我们不再可信，别占着源）
      act.request_release_source = true;
      act.log_error = true;
      return act;
    }
    // ---- 全局优先级 2：load 失败 → ERROR（不可恢复，需重启）----
    if (state_ == NodeState::Idle && in.load_ok == false && !in.load_error.empty()) {
      transitionTo(NodeState::Error, in.now_ms, in.load_error);
      act.log_error = true;
      return act;
    }

    switch (state_) {
      case NodeState::Idle:      act = stepIdle(in); break;
      case NodeState::Loading:   act = stepLoading(in); break;
      case NodeState::Standby:   act = stepStandby(in); break;
      case NodeState::Arming:    act = stepArming(in); break;
      case NodeState::Switching: act = stepSwitching(in); break;
      case NodeState::Running:   act = stepRunning(in); break;
      case NodeState::Suspended: act = stepSuspended(in); break;
      case NodeState::Dwell:     act = stepDwell(in); break;
      case NodeState::Fault:     act = stepFault(in); break;
      case NodeState::Error:     /* 需重启，什么都不做 */ break;
    }
    return act;
  }

private:
  StateActions stepIdle(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.load_ok) {transitionTo(NodeState::Loading, in.now_ms, "载入中…");}
    return act;
  }

  StateActions stepLoading(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.load_ok) {
      transitionTo(NodeState::Standby, in.now_ms, "已就绪（未接管）");
    }
    return act;
  }

  StateActions stepStandby(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.disable_requested) {return act;}          // 已在 STANDBY，幂等
    if (!in.enable_requested) {return act;}
    // 守卫：状态必须新鲜 —— 拿不到实测关节角就不能接管
    //（否则可能把"实测 0"当成真值，正是 hold_controller 的 C2 那类错误）
    if (!in.state_fresh) {
      detail_ = "enable 被拒：/joint_states 不新鲜或未覆盖全部关节";
      return act;
    }
    transitionTo(NodeState::Arming, in.now_ms, "请求锁源…");
    act.request_lock_source = true;
    return act;
  }

  StateActions stepArming(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.disable_requested) {
      transitionTo(NodeState::Standby, in.now_ms, "用户取消（ARMING 阶段）");
      act.request_release_source = true;
      return act;
    }
    if (!in.state_fresh) {
      transitionTo(NodeState::Standby, in.now_ms, "锁源期间状态失鲜，放弃接管");
      act.request_release_source = true;
      return act;
    }
    if (!in.bridge_authorized) {return act;}          // 等桥回读
    transitionTo(NodeState::Switching, in.now_ms, "请求桥切模式…");
    // 迁移守则：→2 只能从 1 进。所以先请求 HOLD，由 stepSwitching 再请求 TRACKING。
    act.request_mode_hold = true;
    return act;
  }

  StateActions stepSwitching(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.disable_requested) {
      transitionTo(NodeState::Standby, in.now_ms, "用户取消（SWITCHING 阶段）");
      act.request_mode_hold = true;      // 回到 HOLD 更安全
      act.request_release_source = true;
      return act;
    }
    if (!in.bridge_authorized) {
      // 授权丢了（被抢占）⇒ 放弃切换，退回 STANDBY
      transitionTo(NodeState::Standby, in.now_ms, "切换中失去授权，放弃");
      act.request_release_source = true;
      return act;
    }
    if (!in.bridge_mode_ok) {
      // 第一次进来：HOLD 已请求，等 mode_ok；若 HOLD 都没成功，超时由外壳处理
      if (!hold_requested_) {
        hold_requested_ = true;
        act.request_mode_hold = true;
      }
      return act;
    }
    // mode_ok 且我们请求的是 TRACKING
    if (!tracking_requested_) {
      tracking_requested_ = true;
      act.request_mode_tracking = true;
      detail_ = "已请求 MIT TRACKING，等回读…";
      return act;
    }
    // HOLD 与 TRACKING 都已请求且回读 mode_ok ⇒ 可以进入 RUNNING
    hold_requested_ = false;
    tracking_requested_ = false;
    transitionTo(NodeState::Running, in.now_ms, "已接管（MIT TRACKING）");
    return act;
  }

  StateActions stepRunning(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.disable_requested) {
      transitionTo(NodeState::Standby, in.now_ms, "用户 disable");
      act.request_mode_hold = true;      // 停发前先回 HOLD，避免松劲
      act.request_release_source = true;
      return act;
    }
    if (!in.state_fresh) {
      transitionTo(NodeState::Fault, in.now_ms, "运行中 /joint_states 失鲜");
      act.request_release_source = true;
      act.log_error = true;
      return act;
    }
    if (!in.bridge_authorized) {
      transitionTo(NodeState::Suspended, in.now_ms, "被其它数据源抢占（已停发）");
      return act;
    }
    if (!in.target_fresh) {
      transitionTo(NodeState::Dwell, in.now_ms, "目标超时 → 低频保持");
      return act;
    }
    return act;
  }

  StateActions stepSuspended(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.disable_requested) {
      transitionTo(NodeState::Standby, in.now_ms, "用户 disable（被抢占中）");
      act.request_release_source = true;
      return act;
    }
    if (in.bridge_authorized) {
      // 重新拿回授权：**必须重新走一遍模式确认**，不能直接回 RUNNING
      //（期间桥可能被切到别的模式）
      transitionTo(NodeState::Arming, in.now_ms, "重新获得授权，重新确认模式");
      act.request_lock_source = true;
      return act;
    }
    return act;
  }

  StateActions stepDwell(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.disable_requested) {
      transitionTo(NodeState::Standby, in.now_ms, "用户 disable（DWELL）");
      act.request_mode_hold = true;
      act.request_release_source = true;
      return act;
    }
    if (!in.bridge_authorized) {
      transitionTo(NodeState::Suspended, in.now_ms, "DWELL 中被抢占");
      return act;
    }
    if (in.target_fresh) {
      transitionTo(NodeState::Running, in.now_ms, "目标恢复 → 全速下发");
    }
    return act;
  }

  StateActions stepFault(const StateInputs & in) noexcept
  {
    StateActions act;
    // ⚠️ 只能经 clear_fault 离开 FAULT，且**回到 STANDBY**（不是 RUNNING）：
    //    从 FAULT 直接回 RUNNING 等于"自动恢复出力"，而违例的原因还没被查。
    if (in.clear_fault_requested) {
      if (!in.state_fresh) {
        detail_ = "clear_fault 被拒：/joint_states 仍不新鲜";
        return act;
      }
      transitionTo(NodeState::Standby, in.now_ms, "已清除故障（需重新 enable）");
    }
    return act;
  }

  void transitionTo(NodeState s, std::int64_t now_ms, const std::string & why) noexcept
  {
    state_ = s;
    detail_ = why;
    last_change_ms_ = now_ms;
    ++transitions_;
    if (s == NodeState::Running) {hold_requested_ = false; tracking_requested_ = false;}
  }

  NodeState state_{NodeState::Idle};
  std::string detail_{"未初始化"};
  std::int64_t last_change_ms_{0};
  std::uint64_t transitions_{0};

  // 由外壳回填的桥回读
  bool bridge_authorized_{false};
  bool bridge_mode_ok_{false};

  // SWITCHING 的两步去重（避免每拍重复请求同一个模式）
  bool hold_requested_{false};
  bool tracking_requested_{false};
};

}  // namespace lite_move_controller
