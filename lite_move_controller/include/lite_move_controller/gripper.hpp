// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// gripper.hpp —— 夹爪（prismatic）阻抗夹持的**纯逻辑**（零 ROS、可裸测）
//
// ===========================================================================
// 为什么单独抽出来
// ===========================================================================
//
// 用户场景：夹爪夹着一支笔，"对期望位置 0 做阻抗控制"（0 = 完全夹到底），
// 然后手臂去画圆。夹爪**不是** IK 链的一部分（到 `*_gripper_tip_middle_link`
// 的链只有 7 个 revolute），也**不参与**动力学（辨识参数里没有它），所以它
// 在控制帧里是一段左右独立、持续保持的命令：
//
//     position = 最新独立目标（未指定时保持；初次来自实测）
//     velocity = 0, effort = 0
//     kp = gripper_kp（N/m）, kd = gripper_kd（N·s/m）
//
// 这段逻辑一旦出错，症状是"手臂在动、夹爪松了/夹爆了"，而日志里什么都看不
// 出来 —— 所以把"下标映射 + 填充"做成**纯函数**，用裸 g++ 单测钉住：
//   · canonical 里没有夹爪（14 关节模式）⇒ 什么也不写，旧行为一字不变；
//   · 只有一只夹爪在 canonical 里（现场配错）⇒ 只写那一只，不越界；
//   · 帧长度不够 ⇒ 不写、返回 0（宁可不动，也不越界）。
//
// ⚠️ 单位（prismatic，与 revolute 完全不同）：
//   position [m], velocity [m/s], effort [N], kp [N/m], kd [N·s/m]。
//   驱动侧 MIT 通路对 effort 还有**型号上限**（本机夹爪 URDF `effort=1`），
//   所以 `gripper_kp=200 N/m` 的实际含义是"很快压到 1 N 就顶住"——
//   是**轻夹**，不是"200 N 的夹持力"。夹持力上限由 limit_guard 的
//   `torque_max`（取 min(配置, URDF effort)）决定，不是 kp。
#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace lite_move_controller
{

/// 夹爪命令（一帧内固定不变）。
struct GripperCommand
{
  double position{0.0};    ///< [m] 期望位置；0 = 完全夹到底
  double velocity{0.0};    ///< [m/s]
  double effort{0.0};      ///< [N] 前馈力（这里恒 0，夹持力靠 kp·Δx 产生）
  double stiffness{0.0};   ///< [N/m]
  double damping{0.0};     ///< [N·s/m]
};

/// 在 canonical 里找出夹爪关节的下标（按 `gripper_joints` 的顺序）。
///
/// @param canonical      节点的 canonical 关节（14 或 16）
/// @param gripper_joints 参数 `gripper_joints`（默认 left/right_gripper）
/// @param missing_out    可选：不在 canonical 里的名字（现场排查"我明明配了，怎么没夹"）
/// @return 下标列表；**空 = 不做夹爪控制**（14 关节模式就是这条路径）
///
/// ⚠️ **顺序**按 `gripper_joints` 给定，而不是 canonical 顺序：这样日志/诊断
///    里的"第 0 个夹爪"永远是 left_gripper，不会因为 canonical 怎么排而变。
inline std::vector<std::size_t> gripperIndices(
  const std::vector<std::string> & canonical,
  const std::vector<std::string> & gripper_joints,
  std::vector<std::string> * missing_out = nullptr)
{
  std::vector<std::size_t> idx;
  idx.reserve(gripper_joints.size());
  for (const auto & name : gripper_joints) {
    if (name.empty()) {continue;}
    bool found = false;
    for (std::size_t k = 0; k < canonical.size(); ++k) {
      if (canonical[k] == name) {
        idx.push_back(k);
        found = true;
        break;
      }
    }
    if (!found && missing_out != nullptr) {missing_out->push_back(name);}
  }
  return idx;
}

/// 把多个夹爪的独立命令写进一帧（纯函数）。
///
/// @param idx   `gripperIndices()` 的结果
/// @param cmds  夹爪命令列表，长度必须精确等于 idx.size()
/// @param position/velocity/effort/stiffness/damping  帧的五个数组（长度 = canonical 数）
/// @return 实际写入了几个关节（0 = 没写：idx 为空、cmds 长度不匹配或任一数组长度不够）
inline std::size_t applyGripperCommands(
  const std::vector<std::size_t> & idx,
  const std::vector<GripperCommand> & cmds,
  std::vector<double> & position,
  std::vector<double> & velocity,
  std::vector<double> & effort,
  std::vector<double> & stiffness,
  std::vector<double> & damping)
{
  if (idx.empty() || cmds.size() != idx.size()) {return 0;}
  const std::size_t n = position.size();
  if (velocity.size() != n || effort.size() != n ||
      stiffness.size() != n || damping.size() != n) {
    return 0;
  }
  for (const std::size_t k : idx) {
    if (k >= n) {return 0;}          // 下标越界 ⇒ 整体放弃（不写半帧）
  }
  for (std::size_t i = 0; i < idx.size(); ++i) {
    const std::size_t k = idx[i];
    position[k] = cmds[i].position;
    velocity[k] = cmds[i].velocity;
    effort[k] = cmds[i].effort;
    stiffness[k] = cmds[i].stiffness;
    damping[k] = cmds[i].damping;
  }
  return idx.size();
}

/// 把统一夹爪命令写进一帧（纯函数，向下兼容）。
///
/// @param idx   `gripperIndices()` 的结果
/// @param cmd   统一夹爪命令
/// @param position/velocity/effort/stiffness/damping  帧的五个数组（长度 = canonical 数）
/// @return 实际写入了几个关节（0 = 没写：idx 为空或任一数组长度不够）
///
/// ⚠️ 长度不够时**整体放弃**而不是"能写几个写几个"：半帧夹爪命令比不写更危险
///    （一只夹爪有刚度、另一只没有，手臂画圆时受力不对称）。
inline std::size_t applyGripperCommands(
  const std::vector<std::size_t> & idx,
  const GripperCommand & cmd,
  std::vector<double> & position,
  std::vector<double> & velocity,
  std::vector<double> & effort,
  std::vector<double> & stiffness,
  std::vector<double> & damping)
{
  std::vector<GripperCommand> cmds(idx.size(), cmd);
  return applyGripperCommands(idx, cmds, position, velocity, effort, stiffness, damping);
}

}  // namespace lite_move_controller
