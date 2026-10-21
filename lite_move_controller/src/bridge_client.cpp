// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// bridge_client.cpp —— 见头文件的分层说明
#include "lite_move_controller/bridge_client.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <chrono>
#include <future>
#include <thread>

namespace lite_move_controller
{

namespace
{
std::int64_t steadyNowMs()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

BridgeClient::BridgeClient(rclcpp::Node * node, std::string source_name,
                           std::vector<std::string> canonical_joints,
                           std::string command_topic, std::string joint_states_topic,
                           double command_timeout_ms,
                           rclcpp::CallbackGroup::SharedPtr js_cb_group)
: node_(node),
  js_cb_group_(std::move(js_cb_group)),
  source_name_(std::move(source_name)),
  canonical_joints_(std::move(canonical_joints)),
  command_topic_(std::move(command_topic)),
  joint_states_topic_(std::move(joint_states_topic)),
  command_timeout_ms_(command_timeout_ms)
{
  index_of_canonical_.assign(canonical_joints_.size(), -1);
  snap_.position.assign(canonical_joints_.size(), 0.0);
  snap_.velocity.assign(canonical_joints_.size(), 0.0);
  snap_.effort.assign(canonical_joints_.size(), 0.0);
}

bool BridgeClient::start(std::string & error_out)
{
  if (node_ == nullptr) {error_out = "node 为空"; return false;}
  if (source_name_.empty()) {error_out = "source_name 为空"; return false;}
  if (canonical_joints_.empty()) {error_out = "canonical_joints 为空"; return false;}
  if (command_topic_.empty()) {error_out = "command_topic 为空"; return false;}

  // 与桥一致：reliable(10)
  pub_ = node_->create_publisher<robstride_msgs::msg::MotionCommand>(
      command_topic_, rclcpp::QoS(10).reliable());
  // ⚠️ 订阅放进**独立的回调组**（见头文件构造参数的说明）：
  //    否则它落在节点 default（MutuallyExclusive）组里，会被同组里的其它回调
  //    （以及被占满的 executor 线程）饿住，表现为"关节状态失鲜 → FAULT"。
  {
    rclcpp::SubscriptionOptions js_opt;
    js_opt.callback_group = js_cb_group_;      // nullptr = 旧行为（节点默认组）
    js_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, rclcpp::QoS(20).reliable(),
        [this](sensor_msgs::msg::JointState::SharedPtr m) {onJointStates(m);}, js_opt);
  }

  set_mode_cli_ = node_->create_client<robstride_msgs::srv::SetMode>("/lite_bridge/set_mode");
  set_source_cli_ = node_->create_client<robstride_msgs::srv::SetSource>("/lite_bridge/set_source");
  get_status_cli_ = node_->create_client<robstride_msgs::srv::GetBridgeStatus>(
      "/lite_bridge/get_status");
  return true;
}

void BridgeClient::onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // ⚠️ 回调里只做"索引 + 拷贝"，不做任何可能阻塞的事。
  //    `/joint_states` 是 100 Hz，这里慢了会拖垮整个 executor。
  const std::size_t n = canonical_joints_.size();
  std::vector<double> p(n, 0.0), v(n, 0.0), e(n, 0.0);
  std::vector<char> seen(n, 0);

  // 建一次 name → 消息下标 的映射（每帧重建，因为消息里的顺序不保证稳定；
  // 关节数只有 14，代价可忽略，换来"顺序无关"的健壮性）
  for (std::size_t i = 0; i < msg->name.size(); ++i) {
    for (std::size_t k = 0; k < n; ++k) {
      if (msg->name[i] != canonical_joints_[k]) {continue;}
      if (i < msg->position.size()) {p[k] = msg->position[i];}
      if (i < msg->velocity.size()) {v[k] = msg->velocity[i];}
      if (i < msg->effort.size()) {e[k] = msg->effort[i];}
      seen[k] = 1;
      break;
    }
  }
  std::size_t covered = 0;
  for (char c : seen) {if (c) {++covered;}}

  std::lock_guard<std::mutex> lk(mu_);
  if (covered == n) {
    snap_.valid = true;
    snap_.position = std::move(p);
    snap_.velocity = std::move(v);
    snap_.effort = std::move(e);
    snap_.stamp_ms = steadyNowMs();
    last_all_ms_ = snap_.stamp_ms;
    have_all_ = true;
  }
  // 覆盖不全时**不更新**快照 —— 宁可让调用方看到"陈旧"，
  // 也不要把"缺失关节填 0"当成实测值（填 0 = 朝零位使劲，本项目踩过）。
}

JointSnapshot BridgeClient::snapshot() const
{
  std::lock_guard<std::mutex> lk(mu_);
  return snap_;
}

std::int64_t BridgeClient::stateAgeMs(std::int64_t now_ms) const noexcept
{
  std::lock_guard<std::mutex> lk(mu_);
  if (!have_all_) {return -1;}
  // ⚠️ 年龄**不允许为负**：调用方（状态 tick）会先取一个 `now` 再去做一次可能
  //    阻塞 50 ms 的 `get_status`，期间新消息可能到 —— 于是 `last_all_ms_ > now`
  //    ⇒ `now - last_all_ms_` 是**负数**。而"负年龄"没有任何物理意义，
  //    上游判据（`age >= 0 && age <= timeout`）却会把它当成"失鲜" ⇒ FAULT，
  //    或者把 `target_fresh` 打成 false ⇒ DWELL↔RUNNING 抖动（真机实测）。
  //    这里夹到 0（= 刚刚到达），调用方那边也会用**新鲜一点的 now** 重算。
  const std::int64_t age = now_ms - last_all_ms_;
  return age > 0 ? age : 0;
}

void BridgeClient::publish(const std::vector<double> & position,
                           const std::vector<double> & velocity,
                           const std::vector<double> & effort,
                           const std::vector<double> & stiffness,
                           const std::vector<double> & damping)
{
  const std::size_t n = canonical_joints_.size();
  // 长度必须全部对齐：接收方(`tracking_controller`)对 JointCommand 做**整帧**校验，
  // 任一数组长度不符 → 整帧丢弃（不是部分生效）。
  if (position.size() != n || velocity.size() != n || effort.size() != n ||
      stiffness.size() != n || damping.size() != n) {
    return;                       // 调用方应已校验；这里静默丢弃并由计数反映
  }
  robstride_msgs::msg::MotionCommand m;
  m.header.stamp = node_->now();
  m.source = source_name_;
  m.mode = robstride_msgs::msg::MotionCommand::MODE_MIT;
  m.timeout_ms = command_timeout_ms_;
  m.joint_names = canonical_joints_;
  m.position = position;
  m.velocity = velocity;
  m.effort = effort;
  m.kp = stiffness;
  m.kd = damping;
  pub_->publish(m);
  published_.fetch_add(1);
}

bool BridgeClient::setMode(std::uint8_t protocol, std::uint8_t mode, bool force,
                           BridgeStatusLite & out, double timeout_sec)
{
  out = BridgeStatusLite{};
  if (!set_mode_cli_->wait_for_service(std::chrono::seconds(3))) {
    out.message = "服务不可用：/lite_bridge/set_mode（lite_bringup 起了吗？）";
    return false;
  }
  auto req = std::make_shared<robstride_msgs::srv::SetMode::Request>();
  req->protocol = protocol;
  req->mode = mode;
  req->force = force;
  auto fut = set_mode_cli_->async_send_request(req).future.share();
  if (!waitFuture(fut, timeout_sec)) {
    out.message = "set_mode 超时（桥内部最长阻塞约 15 s）";
    return false;
  }
  const auto res = fut.get();
  out.ok = res->ok;
  out.message = res->message;
  out.protocol = res->actual_protocol;
  out.protocol_name = res->actual_protocol_name;
  out.mode = res->actual_mode;
  out.mode_name = res->actual_mode_name;
  out.controller_active = res->controller_active;
  out.active_controller = res->active_controller;
  return true;
}

bool BridgeClient::setSource(std::uint8_t mode, const std::string & source,
                             double hold_ms, BridgeStatusLite & out, double timeout_sec)
{
  out = BridgeStatusLite{};
  if (!set_source_cli_->wait_for_service(std::chrono::seconds(3))) {
    out.message = "服务不可用：/lite_bridge/set_source";
    return false;
  }
  auto req = std::make_shared<robstride_msgs::srv::SetSource::Request>();
  req->mode = mode;
  req->source = source;
  req->hold_ms = hold_ms;
  auto fut = set_source_cli_->async_send_request(req).future.share();
  if (!waitFuture(fut, timeout_sec)) {
    out.message = "set_source 超时";
    return false;
  }
  const auto res = fut.get();
  out.ok = res->ok;
  out.message = res->message;
  out.active_source = res->active_source;
  // ⚠️ 必须回填 `manual_override`：它是 `ARMING` 的**授权判据**
  //    （见 move_controller_node.cpp 的 `in.bridge_authorized`）。
  //    漏了它 ⇒ 本函数的调用方拿到的 `manual_override` 是空串
  //    ⇒ 锁源"成功"但状态机判为未授权 ⇒ **永远卡在 ARMING**（实测复现）。
  out.manual_override = res->manual_override;
  out.arbitration_state = res->arbitration_state;
  out.arbitration_state_name = res->arbitration_state_name;
  out.available_sources = res->available_sources;
  return true;
}

bool BridgeClient::getStatus(BridgeStatusLite & out, double timeout_sec)
{
  out = BridgeStatusLite{};
  // ⚠️ `wait_for_service` 也必须吃**同一个**预算（原来写死 3 s）：
  //    状态 tick 每拍都会调本函数，若桥的服务暂时没被发现，写死的 3 s 会让
  //    tick 直接卡死 3 s（比 waitFuture 那 2 s 还糟）。这里取 min(预算, 3 s)：
  //    状态 tick 传 0.05 ⇒ 最多 50 ms；ARMING/SWITCHING 的 setSource/setMode
  //    仍各自保留 3 s 的服务发现预算（它们不是热路径）。
  const double svc_wait = std::min(timeout_sec, 3.0);
  if (!get_status_cli_->wait_for_service(std::chrono::duration<double>(svc_wait))) {
    out.message = "服务不可用：/lite_bridge/get_status";
    return false;
  }
  auto req = std::make_shared<robstride_msgs::srv::GetBridgeStatus::Request>();
  auto fut = get_status_cli_->async_send_request(req).future.share();
  if (!waitFuture(fut, timeout_sec)) {
    out.message = "get_status 超时";
    return false;
  }
  const auto res = fut.get();
  out.ok = true;
  out.protocol = res->protocol;
  out.protocol_name = res->protocol_name;
  out.mode = res->mode;
  out.mode_name = res->mode_name;
  out.controller_active = res->controller_active;
  out.active_controller = res->active_controller;
  out.active_source = res->active_source;
  // ⚠️⚠️ 这一行是**时序关键**，不能省：
  //    `onStateTick()` 顶部**每拍**都用本函数的回读刷新缓存
  //    （`bridge_manual_override_ = st.manual_override`）。
  //    若这里不回填 ⇒ 每拍都把 LOCK 刚设上的 `manual_override` **清成空串**
  //    ⇒ `bridge_authorized` 恒为 false ⇒ 卡在 ARMING 并无限重发锁源请求。
  //    两个 srv（SetSource / GetBridgeStatus）都有该字段，必须**都**回填。
  out.manual_override = res->manual_override;
  out.arbitration_state = res->arbitration_state;
  out.arbitration_state_name = res->arbitration_state_name;
  out.source_names = res->source_names;
  return true;
}

}  // namespace lite_move_controller
