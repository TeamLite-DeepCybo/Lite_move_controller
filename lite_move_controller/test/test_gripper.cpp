// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// test_gripper.cpp —— 夹爪填充逻辑的**裸单测**（零 ROS，可 g++ 直接跑）
//
// 为什么单独测：夹爪命令写错不会报错 —— 手臂照常画圆，只是笔掉了或者夹爆了，
// 日志里什么都看不出来。所以把"下标映射 + 写帧"抽成纯函数（gripper.hpp），
// 在这里把 14/16 关节两种模式、缺关节、帧长度不符、下标越界全部钉住。
//
// 运行： ./build/lite_move_controller/test_gripper
#include "lite_move_controller/gripper.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace lite_move_controller;

static int g_n = 0, g_fail = 0;
#define CHECK(c, ...) do { ++g_n; if (!(c)) { ++g_fail; \
  std::printf("  FAIL %d  ", __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)

// 与 lite_bringup 的 `_DAMPING_BY_MODE` 一致的 16 关节顺序
static const std::vector<std::string> kLeft = {
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow_pitch", "left_wrist_yaw", "left_wrist_roll", "left_wrist_pitch"};
static const std::vector<std::string> kRight = {
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow_pitch", "right_wrist_yaw", "right_wrist_roll", "right_wrist_pitch"};

static std::vector<std::string> canonical14()
{
  std::vector<std::string> c = kLeft;
  c.insert(c.end(), kRight.begin(), kRight.end());
  return c;
}

static std::vector<std::string> canonical16()
{
  std::vector<std::string> c = kLeft;
  c.push_back("left_gripper");
  c.insert(c.end(), kRight.begin(), kRight.end());
  c.push_back("right_gripper");
  return c;
}

int main()
{
  const std::vector<std::string> kGrippers = {"left_gripper", "right_gripper"};

  // ---- ① 16 关节：夹爪下标必须是 7 与 15（bringup 把夹爪夹在臂中间）----
  {
    const auto c = canonical16();
    CHECK(c.size() == 16, "16 关节 canonical 长度应为 16，得到 %zu", c.size());
    CHECK(c[7] == "left_gripper", "canonical[7] 应是 left_gripper，得到 %s", c[7].c_str());
    CHECK(c[15] == "right_gripper", "canonical[15] 应是 right_gripper，得到 %s",
          c[15].c_str());
    std::vector<std::string> missing;
    const auto idx = gripperIndices(c, kGrippers, &missing);
    CHECK(idx.size() == 2, "应找到 2 个夹爪，得到 %zu", idx.size());
    CHECK(idx.size() == 2 && idx[0] == 7, "left_gripper 下标应为 7");
    CHECK(idx.size() == 2 && idx[1] == 15, "right_gripper 下标应为 15");
    CHECK(missing.empty(), "不应有缺失，得到 %zu", missing.size());
    std::printf("[16 关节] 夹爪下标 = ");
    for (auto k : idx) {std::printf("%zu ", k);}
    std::printf("\n");
  }

  // ---- ② 14 关节：canonical 里没有夹爪 ⇒ 一个下标都不返回（旧行为）----
  {
    std::vector<std::string> missing;
    const auto idx = gripperIndices(canonical14(), kGrippers, &missing);
    CHECK(idx.empty(), "14 关节模式不应有夹爪下标，得到 %zu", idx.size());
    CHECK(missing.size() == 2, "缺失列表应有两个名字，得到 %zu", missing.size());
  }

  // ---- ③ 顺序按 gripper_joints 给定（不是 canonical 顺序）----
  {
    const auto c = canonical16();
    const auto idx = gripperIndices(c, {"right_gripper", "left_gripper"});
    CHECK(idx.size() == 2, "应找到 2 个，得到 %zu", idx.size());
    CHECK(idx.size() == 2 && idx[0] == 15 && idx[1] == 7,
          "顺序应按参数给：得到 %zu,%zu", idx.size() == 2 ? idx[0] : 0,
          idx.size() == 2 ? idx[1] : 0);
  }

  // ---- ④ 空参数 / 空名字 ----
  {
    CHECK(gripperIndices(canonical16(), {}).empty(), "空 gripper_joints 应无下标");
    CHECK(gripperIndices(canonical16(), {""}).empty(), "空串名字应被跳过");
    CHECK(gripperIndices({}, kGrippers).empty(), "空 canonical 应无下标");
  }

  // ---- ⑤ 只配一只夹爪（现场配错时不能把另一只算进来）----
  {
    const auto idx = gripperIndices(canonical16(), {"left_gripper"});
    CHECK(idx.size() == 1 && idx[0] == 7, "只应有 left_gripper(7)");
  }

  // ---- ⑥ 写帧：夹爪被写成 0/0/0/200/5，臂关节**一位都不动** ----
  {
    const auto c = canonical16();
    const auto idx = gripperIndices(c, kGrippers);
    GripperCommand cmd;
    cmd.position = 0.0;
    cmd.velocity = 0.0;
    cmd.effort = 0.0;
    cmd.stiffness = 200.0;
    cmd.damping = 5.0;

    std::vector<double> pos(16, 0.123), vel(16, 0.456), eff(16, 0.789),
                        kp(16, 11.0), kd(16, 22.0);
    // 臂关节的"哨兵值"：写完之后必须原样
    const std::vector<std::size_t> arm_idx = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14};
    const auto n = applyGripperCommands(idx, cmd, pos, vel, eff, kp, kd);
    CHECK(n == 2, "应写入 2 个夹爪，得到 %zu", n);
    for (const auto k : {std::size_t{7}, std::size_t{15}}) {
      CHECK(pos[k] == 0.0, "夹爪[%zu] position 应为 0，得到 %f", k, pos[k]);
      CHECK(vel[k] == 0.0, "夹爪[%zu] velocity 应为 0，得到 %f", k, vel[k]);
      CHECK(eff[k] == 0.0, "夹爪[%zu] effort 应为 0，得到 %f", k, eff[k]);
      CHECK(kp[k] == 200.0, "夹爪[%zu] kp 应为 200，得到 %f", k, kp[k]);
      CHECK(kd[k] == 5.0, "夹爪[%zu] kd 应为 5，得到 %f", k, kd[k]);
    }
    for (const auto k : arm_idx) {
      CHECK(pos[k] == 0.123 && vel[k] == 0.456 && eff[k] == 0.789 &&
            kp[k] == 11.0 && kd[k] == 22.0,
            "臂关节[%zu] 不应被夹爪逻辑改动", k);
    }
  }

  // ---- ⑦ 14 关节模式下 idx 为空 ⇒ 写帧是**无操作**（不碰任何值）----
  {
    GripperCommand cmd;
    cmd.stiffness = 200.0;
    cmd.damping = 5.0;
    std::vector<double> pos(14, 1.0), vel(14, 2.0), eff(14, 3.0), kp(14, 4.0), kd(14, 5.0);
    const auto n = applyGripperCommands({}, cmd, pos, vel, eff, kp, kd);
    CHECK(n == 0, "空下标应写入 0 个，得到 %zu", n);
    for (std::size_t k = 0; k < 14; ++k) {
      CHECK(pos[k] == 1.0 && vel[k] == 2.0 && eff[k] == 3.0 && kp[k] == 4.0 && kd[k] == 5.0,
            "14 关节模式不应改动任何值（关节 %zu）", k);
    }
  }

  // ---- ⑧ 帧长度不一致 / 下标越界 ⇒ 整体放弃（绝不写半帧）----
  {
    GripperCommand cmd;
    cmd.stiffness = 200.0;
    std::vector<double> pos(16, 0.0), vel(16, 0.0), eff(16, 0.0);
    std::vector<double> kp(15, 9.0), kd(16, 9.0);       // kp 少一格
    CHECK(applyGripperCommands({7, 15}, cmd, pos, vel, eff, kp, kd) == 0,
          "长度不一致时应写入 0 个");
    CHECK(kp[0] == 9.0 && pos[7] == 0.0, "长度不一致时不应写任何字段");

    std::vector<double> kp2(16, 9.0), kd2(16, 9.0);
    CHECK(applyGripperCommands({7, 16}, cmd, pos, vel, eff, kp2, kd2) == 0,
          "下标越界(16)时应写入 0 个");
    CHECK(pos[7] == 0.0 && kp2[7] == 9.0, "下标越界时不应写任何字段");
  }

  // ---- ⑨ 非零期望位置也要如实写入（用户可能想夹在半开）----
  {
    const auto idx = gripperIndices(canonical16(), kGrippers);
    GripperCommand cmd;
    cmd.position = 0.023;      // 半开
    cmd.stiffness = 150.0;
    cmd.damping = 3.0;
    std::vector<double> pos(16, 1.0), vel(16, 1.0), eff(16, 1.0), kp(16, 1.0), kd(16, 1.0);
    CHECK(applyGripperCommands(idx, cmd, pos, vel, eff, kp, kd) == 2, "应写入 2 个");
    CHECK(std::fabs(pos[7] - 0.023) < 1e-12, "position 应如实写入 0.023，得到 %f", pos[7]);
    CHECK(std::fabs(pos[15] - 0.023) < 1e-12, "position 应如实写入 0.023，得到 %f", pos[15]);
  }

  // ---- ⑩ 左右夹爪独立开度写入（向量重载）----
  {
    const auto idx = gripperIndices(canonical16(), kGrippers);
    std::vector<GripperCommand> cmds(2);
    cmds[0].position = 0.047; // 左爪全开
    cmds[0].stiffness = 20.0;
    cmds[0].damping = 3.0;
    cmds[1].position = 0.0;   // 右爪全闭
    cmds[1].stiffness = 30.0;
    cmds[1].damping = 4.0;
    std::vector<double> pos(16, 0.0), vel(16, 0.0), eff(16, 0.0), kp(16, 0.0), kd(16, 0.0);
    CHECK(applyGripperCommands(idx, cmds, pos, vel, eff, kp, kd) == 2, "应写入 2 个");
    CHECK(std::fabs(pos[7] - 0.047) < 1e-12, "左爪 position 应为 0.047");
    CHECK(std::fabs(kp[7] - 20.0) < 1e-12, "左爪 kp 应为 20.0");
    CHECK(std::fabs(pos[15] - 0.0) < 1e-12, "右爪 position 应为 0.0");
    CHECK(std::fabs(kp[15] - 30.0) < 1e-12, "右爪 kp 应为 30.0");

    // 长度不匹配测试
    cmds.pop_back();
    CHECK(applyGripperCommands(idx, cmds, pos, vel, eff, kp, kd) == 0, "cmds 长度不匹配时应返回 0");
  }

  std::printf("\n%d checks, %d failures\n", g_n, g_fail);
  return g_fail == 0 ? 0 : 1;
}
