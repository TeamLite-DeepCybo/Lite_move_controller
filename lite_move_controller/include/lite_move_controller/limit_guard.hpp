// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// limit_guard.hpp —— 下发前的**三道闸**（位置 / 速度 / 力矩 + 增益）
//
// ===========================================================================
// 为什么必须由本层做（而不是靠驱动）
// ===========================================================================
//
// MIT 通路**没有位置环兜底**：力矩是直通量 `τ = Kp·Δp + Kd·Δv + τ_ff`。
// 驱动侧只把力矩夹到**型号峰值**（rs-02 14 N·m / rs-00 17 N·m）、
// Kp 夹到**协议量程 500** —— 而本机的额定力矩只有 2.5~7 N·m。
// 文档里提过的 `mit_kp_max` **从未实现**（见 lite_bridge/docs/桥层设计.md §7.1）。
//
// ⇒ 保守的逐关节上限**只能由上层给**。本文件就是那一层的单一事实来源。
//
// ===========================================================================
// 三道闸与它们的取值来源
// ===========================================================================
//
// | 闸 | 上限来源 | 越界后果 |
// |---|---|---|
// | 位置 | URDF 关节限位（`robot_description`）| 撞机械限位 → 堵转过载 |
// | 速度 | URDF `velocity` 与驱动 `limit_spd` 的**较小者** | 电机追不上，位置环饱和 |
// | 力矩 | `joint_spec.yaml` 的 `max_safe_torque`（肩 7.0 / 肘 4.0 / 腕 1.5~1.8）| 满力矩顶死 |
// | Kp/Kd | 配置的逐关节上限（**必须给**，不能退回协议量程 500）| `Kp=500` + 大偏差 = 立刻满力矩 |
//
// ⚠️ 所有闸都是**逐关节**的，不是标量：本机肩 7.0 N·m、腕 1.5 N·m，差 4.7 倍，
//    用一个标量必然要么夹不住肩、要么把腕夹废。
//
// ===========================================================================
// 计数与"不静默"
// ===========================================================================
//
// 每次夹取都**计数**（`clamped_*`）。非零不是"正常现象"，而是信号：
//   · `clamped_torque` 持续增长 ⇒ 模型/增益不匹配，或有持续外力；
//   · `clamped_position` 非零 ⇒ 轨迹/IK 超出限位，**设计有问题**；
//   · `clamped_gain` 非零 ⇒ 上游给的刚度超过本层允许值。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "lite_ik_node/kinematics_types.hpp"

namespace lite_move_controller
{
/// 一帧的下发内容（本层内部表示，最终转成 robstride_msgs/MotionCommand）。
struct JointCommandFrame
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> effort;
  std::vector<double> stiffness;
  std::vector<double> damping;

  void resize(std::size_t n)
  {
    position.assign(n, 0.0);
    velocity.assign(n, 0.0);
    effort.assign(n, 0.0);
    stiffness.assign(n, 0.0);
    damping.assign(n, 0.0);
  }
  std::size_t size() const noexcept {return position.size();}
};

/// 夹取统计。
struct ClampStats
{
  std::uint64_t clamped_torque{0};
  std::uint64_t clamped_velocity{0};
  std::uint64_t clamped_position{0};
  std::uint64_t clamped_gain{0};
  std::uint64_t nonfinite_rejected{0};
  /// 软限位渐弱生效的（关节×帧）次数。**不计入 `total()`**（它不是"夹取"，
  /// 而是"在带内主动减弱"，语义上与四道闸的越界夹取不同）。
  std::uint64_t soft_tapered{0};

  std::uint64_t total() const noexcept
  {
    return clamped_torque + clamped_velocity + clamped_position +
           clamped_gain + nonfinite_rejected;
  }
};

/// 三道闸。纯逻辑、无 ROS。
class LimitGuard
{
public:
  /// 配置。返回 false ⇒ 上限集合不自洽，**必须拒绝启动**（fail-closed）。
  bool configure(const JointLimits & limits) noexcept
  {
    if (!limits.valid()) {return false;}
    lim_ = limits;
    stats_ = ClampStats{};
    return true;
  }

  const JointLimits & limits() const noexcept {return lim_;}
  const ClampStats & stats() const noexcept {return stats_;}

  /// 夹取一帧（原地修改）。
  ///
  /// ⚠️ 返回 false 表示**帧里有非有限值**，调用方必须**整帧丢弃**
  ///    （不能"修好再发" —— 非有限值说明上游/IK 出了问题，猜测它的意图更危险）。
  bool apply(JointCommandFrame & frame) noexcept
  {
    const std::size_t n = lim_.size();
    if (frame.size() != n) {++stats_.nonfinite_rejected; return false;}

    for (std::size_t j = 0; j < n; ++j) {
      // ---- 数值合法性（第一优先级：NaN/Inf 一律整帧拒绝）----
      if (!std::isfinite(frame.position[j]) || !std::isfinite(frame.velocity[j]) ||
          !std::isfinite(frame.effort[j]) || !std::isfinite(frame.stiffness[j]) ||
          !std::isfinite(frame.damping[j])) {
        ++stats_.nonfinite_rejected;
        return false;
      }
      // ---- ① 位置（含**软限位**）----
      //
      // band == 0（未配置）⇒ 与旧版逐字等价：硬夹到 [lo, hi]。
      // band  > 0        ⇒ 只在越出 [lo-band, hi+band] 时硬夹，并在带内
      //                     算一个渐弱系数 s ∈ [0,1] 去压 Kp/Kd/τ_ff。
      const double lo = lim_.position_min[j];
      const double hi = lim_.position_max[j];
      const double band = lim_.softBand(j);
      double soft = 1.0;
      if (band > 0.0) {
        const double d = std::min(frame.position[j] - lo, hi - frame.position[j]);
        // smoothstep：d<=0（已到/越限）→ 0；d>=band（带外）→ 1
        const double x = std::min(1.0, std::max(0.0, d / band));
        soft = x * x * (3.0 - 2.0 * x);
        if (frame.position[j] < lo - band) {
          frame.position[j] = lo - band; ++stats_.clamped_position;
        } else if (frame.position[j] > hi + band) {
          frame.position[j] = hi + band; ++stats_.clamped_position;
        }
      } else if (frame.position[j] < lo) {
        frame.position[j] = lo; ++stats_.clamped_position;
      } else if (frame.position[j] > hi) {
        frame.position[j] = hi; ++stats_.clamped_position;
      }
      // ---- ② 速度 ----
      const double vmax = lim_.velocity_max[j];
      if (frame.velocity[j] > vmax) {frame.velocity[j] = vmax; ++stats_.clamped_velocity;}
      else if (frame.velocity[j] < -vmax) {frame.velocity[j] = -vmax; ++stats_.clamped_velocity;}
      // ---- ③ 力矩 ----
      const double tmax = lim_.torque_max[j];
      if (frame.effort[j] > tmax) {frame.effort[j] = tmax; ++stats_.clamped_torque;}
      else if (frame.effort[j] < -tmax) {frame.effort[j] = -tmax; ++stats_.clamped_torque;}
      // ---- ④ 增益（Kp/Kd 必须**非负**；负增益 = 正反馈 = 发散）----
      if (frame.stiffness[j] < 0.0) {frame.stiffness[j] = 0.0; ++stats_.clamped_gain;}
      if (frame.damping[j] < 0.0) {frame.damping[j] = 0.0; ++stats_.clamped_gain;}
      const double kmax = lim_.stiffness_max[j];
      if (frame.stiffness[j] > kmax) {frame.stiffness[j] = kmax; ++stats_.clamped_gain;}
      const double dmax = lim_.damping_max[j];
      if (frame.damping[j] > dmax) {frame.damping[j] = dmax; ++stats_.clamped_gain;}

      // ---- ⑤ 软限位渐弱（band>0 时才生效）----
      //  在限位带内把 Kp / Kd / τ_ff **一起**压小：位置环的"顶死"力矩与
      //  前馈力矩同时消失，关节只会软软地靠住机械限位。
      //  ⚠️ 阻尼也一起压：只压 Kp 会让关节在带内变成**欠阻尼**，撞限位时更容易弹。
      if (band > 0.0 && soft < 1.0) {
        frame.stiffness[j] *= soft;
        frame.damping[j] *= soft;
        frame.effort[j] *= soft;
        ++stats_.soft_tapered;
      }
    }
    return true;
  }

  /// 位置是否在限位内（不修改任何东西）。供 IK 后端/自检用。
  bool position_in_bounds(const std::vector<double> & q, double margin = 0.0) const noexcept
  {
    if (q.size() != lim_.size()) {return false;}
    for (std::size_t j = 0; j < q.size(); ++j) {
      if (q[j] < lim_.position_min[j] + margin) {return false;}
      if (q[j] > lim_.position_max[j] - margin) {return false;}
    }
    return true;
  }

  /// 哪些关节贴着限位（余量 < `margin`）。供 `ik_saturated_joints` 反馈用。
  std::vector<std::size_t> saturated_joints(const std::vector<double> & q,
                                            double margin) const noexcept
  {
    std::vector<std::size_t> out;
    if (q.size() != lim_.size()) {return out;}
    for (std::size_t j = 0; j < q.size(); ++j) {
      if (q[j] - lim_.position_min[j] < margin ||
          lim_.position_max[j] - q[j] < margin) {
        out.push_back(j);
      }
    }
    return out;
  }

  void reset_stats() noexcept {stats_ = ClampStats{};}

private:
  JointLimits lim_{};
  ClampStats stats_{};
};

}  // namespace lite_move_controller
