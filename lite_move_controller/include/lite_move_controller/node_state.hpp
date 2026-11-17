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
// 健康待命也维持 MIT TRACKING：启动先锁源、直接切 TRACKING，再进入 STANDBY。
// enable 只开放目标处理；disable 清除目标并保持当前实测位置，不交权、不切 HOLD。
// FAULT / SUSPENDED 停发，底层安全降级仍由桥决定，绝不覆盖故障保护。
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
  GravityCompensation = 10,  ///< 内部运行状态；对外映射到既有 RUNNING。
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
    case NodeState::GravityCompensation: return "GRAVITY_COMPENSATION";
  }
  return "UNKNOWN";
}

/// 允许输出的状态还必须由 ROS 外壳检查授权、MIT TRACKING 和反馈新鲜度。
inline NodeState publicNodeState(NodeState s) noexcept
{
  return s == NodeState::GravityCompensation ? NodeState::Running : s;
}

inline bool statePublishes(NodeState s) noexcept
{
  return s == NodeState::Standby || s == NodeState::Switching ||
         s == NodeState::Running || s == NodeState::Dwell ||
         s == NodeState::GravityCompensation;
}

/// 全速下发（而非 DWELL 的低频保持）。
inline bool statePublishesFullRate(NodeState s) noexcept
{
  return s == NodeState::Standby || s == NodeState::Switching ||
         s == NodeState::Running || s == NodeState::GravityCompensation;
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
  bool gravity_compensation{false}; ///< 启动期选择，不依赖运动目标。
  std::string violation;         ///< 非空 = 安全违例（NaN / 位置超差 / 求解失败连击…）
};

/// 状态机要求 ROS 外壳去做的动作。
///
/// ⚠️ 用**标志**而不是直接调服务：这样"是否请求了切模式"可以被单测断言。
struct StateActions
{
  bool request_lock_source{false};   ///< set_source{MODE_LOCK, 自己}
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
    if (in.disable_requested) {run_requested_ = false;}
    else if (in.enable_requested && state_ != NodeState::Fault) {run_requested_ = true;}
    // ⚠️ 桥回读**只有一个来源**：入参。早先同时保留 note*() 写成员 +
    //    读入参，两条路会分叉 —— 单测里就表现为"设了授权却还在 ARMING"。
    //    这里在入口统一刷一次成员，后面的 step* 只读成员，杜绝分叉。
    bridge_authorized_ = in.bridge_authorized;
    bridge_mode_ok_ = in.bridge_mode_ok;
    // ---- 全局优先级 1：安全违例（最高，任何状态都可能被打到 FAULT）----
    if (!in.violation.empty() &&
        state_ != NodeState::Fault && state_ != NodeState::Error) {
      run_requested_ = false;
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
      case NodeState::GravityCompensation: act = stepRunning(in); break;
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
    if (in.load_ok && in.state_fresh) {
      transitionTo(NodeState::Arming, in.now_ms, "初始化 MIT 待命保持，等待锁源");
      act.request_lock_source = true;
    }
    return act;
  }

  StateActions stepStandby(const StateInputs & in) noexcept
  {
    StateActions act;
    if (!in.state_fresh) {
      transitionTo(NodeState::Fault, in.now_ms, "待命中 /joint_states 不新鲜");
      act.request_release_source = true;
    } else if (!in.bridge_authorized || !in.bridge_mode_ok) {
      transitionTo(NodeState::Suspended, in.now_ms, "待命保持失去授权或 MIT TRACKING");
    } else if (run_requested_) {
      transitionTo(in.gravity_compensation ? NodeState::GravityCompensation : NodeState::Running,
                   in.now_ms, in.gravity_compensation ? "纯重力补偿（Kp=0，Kd=1；拒绝运动目标）" : "已开放运动目标");
    }
    return act;
  }

  StateActions stepArming(const StateInputs & in) noexcept
  {
    StateActions act;
    if (!in.state_fresh) {
      transitionTo(NodeState::Fault, in.now_ms, "锁源期间反馈失鲜");
      act.request_release_source = true;
      return act;
    }
    if (!in.bridge_authorized) {return act;}
    transitionTo(NodeState::Switching, in.now_ms, "请求 MIT TRACKING，等待实际回读");
    act.request_mode_tracking = true;
    return act;
  }

  StateActions stepSwitching(const StateInputs & in) noexcept
  {
    StateActions act;
    if (!in.state_fresh) {
      transitionTo(NodeState::Fault, in.now_ms, "切换期间反馈失鲜");
      act.request_release_source = true;
    } else if (!in.bridge_authorized) {
      transitionTo(NodeState::Suspended, in.now_ms, "切换中失去授权");
    } else if (in.bridge_mode_ok) {
      transitionTo(NodeState::Standby, in.now_ms, "MIT TRACKING 待命保持（运动输入关闭）");
    }
    return act;
  }

  StateActions stepRunning(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.disable_requested) {
      transitionTo(NodeState::Standby, in.now_ms, "用户 disable");
      // 健康 disable 仅关闭目标输入，继续 MIT 保持。
      return act;
    }
    if (!in.state_fresh) {
      transitionTo(NodeState::Fault, in.now_ms, "运行中 /joint_states 失鲜");
      act.request_release_source = true;
      act.log_error = true;
      return act;
    }
    if (!in.bridge_authorized || !in.bridge_mode_ok) {
      transitionTo(NodeState::Suspended, in.now_ms, "失去授权或 MIT TRACKING（已停发）");
      return act;
    }
    if (state_ != NodeState::GravityCompensation && !in.target_fresh) {
      transitionTo(NodeState::Dwell, in.now_ms, "目标超时 → 低频保持");
      return act;
    }
    return act;
  }

  StateActions stepSuspended(const StateInputs & in) noexcept
  {
    StateActions act;
    // 不自动抢回控制权；显式 enable 才重新锁源。
    if (in.enable_requested && in.state_fresh) {
      transitionTo(NodeState::Arming, in.now_ms, "显式恢复，重新锁源");
      act.request_lock_source = true;
    }
    return act;
  }

  StateActions stepDwell(const StateInputs & in) noexcept
  {
    StateActions act;
    if (in.disable_requested) {
      transitionTo(NodeState::Standby, in.now_ms, "用户 disable（DWELL）");
      // DWELL → STANDBY 保留 MIT TRACKING 与授权。
      return act;
    }
    if (!in.state_fresh) {
      transitionTo(NodeState::Fault, in.now_ms, "DWELL 中 /joint_states 失鲜");
      act.request_release_source = true;
      act.log_error = true;
      return act;
    }
    if (!in.bridge_authorized || !in.bridge_mode_ok) {
      transitionTo(NodeState::Suspended, in.now_ms, "DWELL 失去授权或 MIT TRACKING");
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
      run_requested_ = false;
      transitionTo(NodeState::Arming, in.now_ms, "已清故障，重建待命保持；运动需重新 enable");
      act.request_lock_source = true;
    }
    return act;
  }

  void transitionTo(NodeState s, std::int64_t now_ms, const std::string & why) noexcept
  {
    state_ = s;
    detail_ = why;
    last_change_ms_ = now_ms;
    ++transitions_;
    if (s == NodeState::Fault || s == NodeState::Suspended) {run_requested_ = false;}
  }

  NodeState state_{NodeState::Idle};
  std::string detail_{"未初始化"};
  std::int64_t last_change_ms_{0};
  std::uint64_t transitions_{0};

  // 由外壳回填的桥回读
  bool bridge_authorized_{false};
  bool bridge_mode_ok_{false};

  bool run_requested_{false};
};

}  // namespace lite_move_controller
