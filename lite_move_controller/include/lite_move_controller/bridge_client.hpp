// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// bridge_client.hpp —— 运控层与 lite_bridge 之间的**唯一**接口
//
// ===========================================================================
// 职责边界（用户明确要求：只与桥通讯，不碰下层模式）
// ===========================================================================
//
// 本文件是运控层**唯一**会碰下层的地方，而且只做三件事：
//   ① 把 `MotionCommand` 发到 `/lite_bridge/motion_command`
//   ② 调桥已经提供的服务：`set_mode` / `set_source` / `get_status` / `standby`
//   ③ 读 `/joint_states`（位置/速度/**力矩**）
//
// **不做**：编 CAN 帧、加载/卸载控制器、直接调 `controller_manager`、
// 发明新模式、碰驱动。
//
// ===========================================================================
// 两条轴（与 bridge/docs/桥层设计.md 一致，别混淆）
// ===========================================================================
//
//   轴1 协议 protocol（mit=0 / csp=1）—— 启动期固定，运行期不可切
//   轴2 模式 mode —— MIT 组 0 ZERO_TORQUE / 1 HOLD / 2 TRACKING
//
// ⚠️ **迁移守则**：`→2 TRACKING` **只能从 1 HOLD 进**。所以接管必须走
//    `set_mode(→1)` → 校验 → `set_mode(→2)` → 校验 两步，不能一步到位。
//
// ⚠️ 本节点是桥的**一个数据源**：`MotionCommand.source` 必须与桥的
//    `source_names` 白名单一致，否则**所有**命令被丢弃并打
//    "丢弃未知数据源"。这一条是实测踩过的（见 dyid 的 RUNBOOK）。
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/header.hpp>

#include <robstride_msgs/msg/motion_command.hpp>
#include <robstride_msgs/srv/get_bridge_status.hpp>
#include <robstride_msgs/srv/set_mode.hpp>
#include <robstride_msgs/srv/set_source.hpp>

namespace lite_move_controller
{

/// 关节状态快照（按 canonical 顺序）。
struct JointSnapshot
{
  bool valid{false};
  std::int64_t stamp_ms{0};
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;

  bool covers(std::size_t n) const noexcept
  {
    return valid && position.size() == n && velocity.size() == n &&
           effort.size() == n;
  }
};

/// 桥的回读结果（**绝不乐观上报**：全部取自实际状态）。
struct BridgeStatusLite
{
  bool ok{false};
  std::string message;
  std::uint8_t protocol{255};
  std::string protocol_name;
  std::uint8_t mode{255};
  std::string mode_name;
  bool controller_active{false};
  std::string active_controller;
  std::string active_source;
  std::uint8_t arbitration_state{255};
  std::string arbitration_state_name;

  /// 当前受**人工干预**（LOCK / HANDOVER）的源名；空 = 无干预。
  ///
  /// ⚠️ 这是"接管授权"的**可靠判据**，比 `active_source` 可靠：
  ///    `active_source` 只有仲裁器真的收到并接受了该源的命令后才非空，而
  ///    本节点在 ARMING 阶段按设计**不发命令** ⇒ 只看它必然死锁（实测）。
  ///    `set_source(MODE_LOCK, name)` 成功后 `manual_override == name`。
  std::string manual_override;
  /// ⚠️ 两个 srv 的"源列表"字段名**不同**，别混用：
  ///   · `SetSource`      → `available_sources`（供"源名拼错时自查"）
  ///   · `GetBridgeStatus`→ `source_names`（已授权的源）
  std::vector<std::string> available_sources;   ///< SetSource 响应
  std::vector<std::string> source_names;        ///< GetBridgeStatus 响应
};

/// 桥客户端：发命令 + 调服务 + 收关节状态。
///
/// ⚠️ 服务调用是**同步阻塞**的（用 future + 超时），且只在 `SWITCHING`/`ARMING`
///    状态下发生（低频、非实时路径）。**实时下发路径（100 Hz）只有 publish()**。
class BridgeClient
{
public:
  /// @param js_cb_group `/joint_states` 订阅用的回调组。
  ///
  /// ⚠️ **必须给它一个独立于服务调用/定时器的回调组**（真机踩过）：
  ///    不给 ⇒ 落在节点的 **default MutuallyExclusive** 组里；而状态/控制定时器
  ///    一旦在别的组里长时间阻塞（同步服务调用最坏 2 s），MultiThreadedExecutor
  ///    的线程会被占满，`/joint_states`（100 Hz）的回调就**排不上队** ⇒
  ///    `stateAgeMs` 超过 `state_timeout_ms` ⇒ 状态机在 RUNNING 里判
  ///    "运行中 /joint_states 失鲜" → FAULT。实测接管后约 21 s 触发。
  ///    nullptr = 保持旧行为（用节点默认组），只有离线小程序会这么用。
  BridgeClient(rclcpp::Node * node, std::string source_name,
               std::vector<std::string> canonical_joints,
               std::string command_topic, std::string joint_states_topic,
               double command_timeout_ms,
               rclcpp::CallbackGroup::SharedPtr js_cb_group = nullptr);

  /// 建立订阅与 publisher。返回 false ⇒ 配置不可用。
  bool start(std::string & error_out);

  /// 是否收到过满足"覆盖全部 canonical 关节"的 `/joint_states`。
  bool stateReady() const noexcept {return have_all_;}

  /// 取最新快照（拷贝，短锁）。
  JointSnapshot snapshot() const;

  /// 距最近一次收到**完整**关节状态过去了多少毫秒（调用方判新鲜度）。
  std::int64_t stateAgeMs(std::int64_t now_ms) const noexcept;

  /// 下发一帧 MIT 命令（**实时路径**：只做一次 publish，不加锁做重活）。
  ///
  /// ⚠️ 五个数组**必须全部填满**且长度 = 关节数：接收方 `tracking_controller`
  ///    对 `JointCommand` 做**整帧**校验，任一数组长度不符就整帧丢弃。
  void publish(const std::vector<double> & position,
               const std::vector<double> & velocity,
               const std::vector<double> & effort,
               const std::vector<double> & stiffness,
               const std::vector<double> & damping);

  /// `~/set_mode`（同步等待，带超时）。
  bool setMode(std::uint8_t protocol, std::uint8_t mode, bool force,
               BridgeStatusLite & out, double timeout_sec);

  /// `~/set_source`。
  /// @param mode 0=AUTO 1=LOCK 2=HANDOVER 3=PREEMPT
  bool setSource(std::uint8_t mode, const std::string & source, double hold_ms,
                 BridgeStatusLite & out, double timeout_sec);

  /// `~/get_status`。
  bool getStatus(BridgeStatusLite & out, double timeout_sec);

  const std::string & sourceName() const noexcept {return source_name_;}
  const std::vector<std::string> & canonicalJoints() const noexcept
  {
    return canonical_joints_;
  }
  std::uint64_t published() const noexcept {return published_.load();}
  std::uint64_t rejectedUnknownSource() const noexcept
  {
    return rejected_unknown_.load();
  }

private:
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg);

  /// 等一个服务 future（任意响应类型）。
  /// ⚠️ 早先写成 `std::shared_future<void>&` —— 而 `async_send_request().future`
  ///    是 `shared_future<SharedResponse>`，类型不匹配会编译失败。用模板才对。
  template<typename Fut>
  bool waitFuture(Fut & fut, double timeout_sec) const
  {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline) {
      if (fut.wait_for(std::chrono::milliseconds(5)) == std::future_status::ready) {
        return true;
      }
    }
    return false;
  }

  rclcpp::Node * node_{nullptr};
  /// `/joint_states` 的独立回调组（见构造函数参数的说明）；nullptr = 节点默认组。
  rclcpp::CallbackGroup::SharedPtr js_cb_group_;
  std::string source_name_;
  std::vector<std::string> canonical_joints_;
  std::string command_topic_;
  std::string joint_states_topic_;
  double command_timeout_ms_{300.0};

  rclcpp::Publisher<robstride_msgs::msg::MotionCommand>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Client<robstride_msgs::srv::SetMode>::SharedPtr set_mode_cli_;
  rclcpp::Client<robstride_msgs::srv::SetSource>::SharedPtr set_source_cli_;
  rclcpp::Client<robstride_msgs::srv::GetBridgeStatus>::SharedPtr get_status_cli_;

  mutable std::mutex mu_;
  JointSnapshot snap_{};
  std::vector<int> index_of_canonical_;   ///< canonical 关节在消息里的下标；-1 = 缺
  bool have_all_{false};
  std::int64_t last_all_ms_{0};

  std::atomic<std::uint64_t> published_{0};
  std::atomic<std::uint64_t> rejected_unknown_{0};
  std::atomic<std::uint64_t> incomplete_frames_{0};
};

}  // namespace lite_move_controller
