// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// gripper_node.cpp —— 夹爪增量控制节点
//
// ===========================================================================
// 职责与设计
// ===========================================================================
//
// 1. 独立解耦：
//    仅在 mode:=arms_grippers 模式下被 launch 启动；在 mode:=arms 纯臂模式下
//    不启动，完全不干扰双臂原有控制流程。
//
// 2. 向上接口（0~1 归一化输入）：
//    · 话题 /gripper/command (sensor_msgs/JointState): 逐夹爪归一化开度 [0.0~1.0]
//    · 话题 /gripper/opening (std_msgs/Float64): 全局归一化开度 [0.0~1.0] 快捷接口
//    0.0 = 完全闭合，1.0 = 完全张开。
//
// 3. 物理映射与保持现状：
//    读取 URDF/配置限位（默认 [0.0, 0.047] m），将 0~1 线性换算为真实物理开度（米）。
//    启动时从 /joint_states 采样当前真实开度作为初始目标，未收到用户指令前
//    绝不主动强行压向 0，保持现状。
//
// 4. 向下对齐：
//    以稳定节拍发布物理米制 JointState 至内部话题 /lite_move_controller/gripper_command，
//    由运控层主节点直接汇聚进 16 轴全量帧。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <stdexcept>

#include "lite_move_controller/gripper_input.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

namespace lite_move_controller
{

class GripperNode : public rclcpp::Node
{
public:
  explicit GripperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("gripper_node", options)
  {
    declare_parameter<std::vector<std::string>>(
      "gripper_joints", {"left_gripper", "right_gripper"});
    declare_parameter<std::vector<double>>("lower_limits", {0.0, 0.0});
    declare_parameter<std::vector<double>>("upper_limits", {0.047, 0.047});
    declare_parameter<double>("default_stiffness", 20.0);
    declare_parameter<double>("default_damping", 3.0);
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<std::string>("command_topic", "/gripper/command");
    declare_parameter<std::string>("opening_topic", "/gripper/opening");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<std::string>(
      "output_topic", "/lite_move_controller/gripper_command");

    gripper_joints_ = get_parameter("gripper_joints").as_string_array();
    lower_limits_ = get_parameter("lower_limits").as_double_array();
    upper_limits_ = get_parameter("upper_limits").as_double_array();
    const double rate_hz = get_parameter("publish_rate_hz").as_double();
    const std::string cmd_topic = get_parameter("command_topic").as_string();
    const std::string opn_topic = get_parameter("opening_topic").as_string();
    const std::string js_topic = get_parameter("joint_states_topic").as_string();
    const std::string out_topic = get_parameter("output_topic").as_string();

    const std::size_t num_joints = gripper_joints_.size();
    std::string error;
    if (!validateGripperConfiguration(gripper_joints_, lower_limits_, upper_limits_, error)) {
      throw std::invalid_argument("invalid gripper configuration: " + error);
    }
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be finite and positive");
    }

    target_positions_.assign(num_joints, 0.0);
    target_initialized_.assign(num_joints, false);

    for (std::size_t i = 0; i < num_joints; ++i) {
      RCLCPP_INFO(get_logger(),
                  "夹爪 [%zu] '%s'：限位 [%.4f .. %.4f] m",
                  i, gripper_joints_[i].c_str(), lower_limits_[i], upper_limits_[i]);
    }

    pub_gripper_cmd_ = create_publisher<sensor_msgs::msg::JointState>(
      out_topic, rclcpp::QoS(10).reliable());

    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      js_topic, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        onJointStates(msg);
      });

    sub_gripper_cmd_ = create_subscription<sensor_msgs::msg::JointState>(
      cmd_topic, rclcpp::QoS(10).reliable(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        onGripperCommand(msg);
      });

    sub_gripper_opening_ = create_subscription<std_msgs::msg::Float64>(
      opn_topic, rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        onGripperOpening(msg);
      });

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { publishLoop(); });

    RCLCPP_INFO(get_logger(),
                "gripper_node 已就绪：订阅 '%s'（0~1归一化JointState）、'%s'（0~1单浮点），"
                "汇聚发布到 '%s'（%.1f Hz）",
                cmd_topic.c_str(), opn_topic.c_str(), out_topic.c_str(), rate_hz);
  }

private:
  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (std::size_t i = 0; i < gripper_joints_.size(); ++i) {
      if (target_initialized_[i]) {
        continue;
      }
      for (std::size_t k = 0; k < msg->name.size(); ++k) {
        if (msg->name[k] == gripper_joints_[i] && k < msg->position.size()) {
          const double pos_meas = msg->position[k];
          if (std::isfinite(pos_meas)) {
            target_positions_[i] = std::clamp(pos_meas, lower_limits_[i], upper_limits_[i]);
            target_initialized_[i] = true;
            RCLCPP_INFO(get_logger(),
                        "夹爪 '%s' 初始目标已锚定至实测位置：%.4f m（保持现状，不主动夹紧）",
                        gripper_joints_[i].c_str(), target_positions_[i]);
          }
          break;
        }
      }
    }
  }

  void onGripperCommand(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::size_t> indices;
    std::string error;
    const std::vector<double> lower(gripper_joints_.size(), 0.0);
    const std::vector<double> upper(gripper_joints_.size(), 1.0);
    if (!validateGripperBatch(msg->name, msg->position, gripper_joints_, lower, upper,
                             indices, error)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "拒绝整条夹爪指令（目标保持不变）：%s", error.c_str());
      return;
    }
    for (std::size_t k = 0; k < msg->name.size(); ++k) {
      const auto i = indices[k];
      const double u = msg->position[k];
      target_positions_[i] = lower_limits_[i] + u * (upper_limits_[i] - lower_limits_[i]);
      target_initialized_[i] = true;
    }
  }

  void onGripperOpening(const std_msgs::msg::Float64::SharedPtr msg)
  {
    if (!std::isfinite(msg->data) || msg->data < 0.0 || msg->data > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "拒绝双爪开度：必须为 0..1 有限值（目标保持不变）");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    const double u_clamped = msg->data;
    for (std::size_t i = 0; i < gripper_joints_.size(); ++i) {
      const double pos_m =
        lower_limits_[i] + u_clamped * (upper_limits_[i] - lower_limits_[i]);
      target_positions_[i] = pos_m;
      target_initialized_[i] = true;
    }
    RCLCPP_DEBUG(get_logger(),
                 "广播所有夹爪目标：归一化 %.2f", u_clamped);
  }

  void publishLoop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor_msgs::msg::JointState out;
    out.header.stamp = now();
    for (std::size_t i = 0; i < gripper_joints_.size(); ++i) {
      if (!target_initialized_[i]) {continue;}
      out.name.push_back(gripper_joints_[i]);
      out.position.push_back(target_positions_[i]);
    }
    if (out.name.empty()) {return;}
    out.velocity.assign(out.name.size(), 0.0);
    out.effort.assign(out.name.size(), 0.0);
    pub_gripper_cmd_->publish(out);
  }

  std::vector<std::string> gripper_joints_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> target_positions_;
  std::vector<bool> target_initialized_;
  std::mutex mutex_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_gripper_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_gripper_cmd_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_gripper_opening_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace lite_move_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lite_move_controller::GripperNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

