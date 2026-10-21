// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// test_state_contract.cpp —— 把"内部枚举 == 对外 msg 常量"变成**编译期**约束
//
// ⚠️ 为什么值得单独一个文件：`move_msgs/msg/MoveStatus.msg` 的 STATE_* 常量是
//    **对外契约**（上游按它解释状态），而内部用的是 `NodeState` 枚举。
//    两者一旦漂移，症状是"状态显示错了"——值班时最难查的一类问题，
//    而且不会有任何报错。static_assert 把它变成编译失败。
//
// 本文件只在能拿到 move_msgs 生成头时参与编译（CMake 里条件加入）。
#include "lite_move_controller/node_state.hpp"

#include <move_msgs/msg/move_status.hpp>

using move_msgs::msg::MoveStatus;

static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Idle) ==
              MoveStatus::STATE_IDLE, "NodeState::Idle 必须 == STATE_IDLE");
static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Loading) ==
              MoveStatus::STATE_LOADING, "NodeState::Loading 必须 == STATE_LOADING");
static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Standby) ==
              MoveStatus::STATE_STANDBY, "NodeState::Standby 必须 == STATE_STANDBY");
static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Arming) ==
              MoveStatus::STATE_ARMING, "NodeState::Arming 必须 == STATE_ARMING");
static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Switching) ==
              MoveStatus::STATE_SWITCHING, "NodeState::Switching 必须 == STATE_SWITCHING");
static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Running) ==
              MoveStatus::STATE_RUNNING, "NodeState::Running 必须 == STATE_RUNNING");
static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Suspended) ==
              MoveStatus::STATE_SUSPENDED, "NodeState::Suspended 必须 == STATE_SUSPENDED");
static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Dwell) ==
              MoveStatus::STATE_DWELL, "NodeState::Dwell 必须 == STATE_DWELL");
static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Fault) ==
              MoveStatus::STATE_FAULT, "NodeState::Fault 必须 == STATE_FAULT");
static_assert(static_cast<std::uint8_t>(lite_move_controller::NodeState::Error) ==
              MoveStatus::STATE_ERROR, "NodeState::Error 必须 == STATE_ERROR");

int main() {return 0;}
