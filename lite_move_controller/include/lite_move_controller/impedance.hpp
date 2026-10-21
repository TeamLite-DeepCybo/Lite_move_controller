// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// impedance.hpp —— 任务空间阻抗控制律
//
// ===========================================================================
// 与导纳的**本质区别**（一句话，设计文档 §5.5）
// ===========================================================================
//
//   阻抗：检测到偏差 → **输出力矩**（`τ = Jᵀ·K_x·Δx`），位置目标不动。
//   导纳：检测到外力 → **移动位置目标**（`x_d + x_c`），再让位置环去追。
//
// 所以在 MIT 帧里两者落点不同：
//   阻抗 → 主要落在 `effort` 字段（并保留较低的 Kp/Kd）；
//   导纳 → 主要落在 `position` 字段（并保留较低的 Kp/Kd）。
//
// ===========================================================================
// 控制律
// ===========================================================================
//
//     F_task = K_x · Δx + D_x · Δẋ                       (6 维笛卡尔)
//     τ_imp  = J(q)ᵀ · F_task                            (7 维关节力矩)
//
// 其中：
//   Δx   = "期望位姿 ⊖ 实测位姿"的 6 维误差（位置 + **旋转向量**姿态误差）
//   Δẋ   = 期望速度 − 实测速度（6 维，任务空间）
//   K_x  = 任务空间刚度（N/m, N·m/rad），由上层给
//   D_x  = 任务空间阻尼，未显式给时按 `2ζ√(K_x·Λ)` 推
//   J    = 6×7 工具帧几何雅可比（由 IK 后端提供）
//
// ===========================================================================
// 三条必须处理的细节
// ===========================================================================
//
// ① **姿态误差必须用旋转向量**，不能用欧拉角差：欧拉角有万向锁、且
//    "角差"在多圈情况下有 ±2π 跳变，会让阻抗力突然反向。
//
// ② **零空间不受影响**：`Jᵀ·F_task` 只产生"能改变末端"的力矩分量，
//    这是 Jacobian-transpose 方法的天然性质（不需要显式投影）。
//
// ③ **限幅**：`τ_imp` 必须在最后逐关节夹到 `max_safe_torque`
//    —— MIT 通路驱动只按型号量程夹（峰值可达 14~17 N·m），没有软件夹层。
//    这一层由 `limit_guard` 统一做，本文件只给出未夹的原始值。
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "lite_ik_node/kinematics_types.hpp"
#include "lite_move_controller/admittance.hpp"   // Vec6 / Mat6 工具

namespace lite_move_controller
{

/// 阻抗控制的配置。
struct ImpedanceConfig
{
  /// 任务空间刚度（N/m, N·m/rad）。**允许任务空间出现 0（该轴纯柔顺）**，
  /// 但不允许负值（= 正反馈，会把机械臂推出去）。
  Vec6 stiffness{300.0, 300.0, 300.0, 8.0, 8.0, 8.0};
  /// 任务空间阻尼。全 0 时按 `2ζ√(K·Λ)` 推。
  Vec6 damping{0, 0, 0, 0, 0, 0};
  /// 推阻尼用的阻尼比。
  double zeta{0.9};
  /// 有效惯量 Λ（用于推阻尼）。缺省用 `nominal_lambda`。
  Vec6 lambda{1.0, 1.0, 1.0, 0.05, 0.05, 0.05};
  /// 逐关节力矩上限（N·m）。空 = 不夹（**不推荐**，真机上应当总是给）。
  JointVec torque_limit;
};

/// 阻抗律的输出。
struct ImpedanceResult
{
  Vec6 wrench{};
  JointVec tau;
  bool ok{false};
  bool clamped_by_local_limit{false};
  std::size_t clamped_joints{0};
};

/// 任务空间阻抗控制律（纯逻辑）。
class ImpedanceLaw
{
public:
  bool configure(const ImpedanceConfig & cfg) noexcept
  {
    cfg_ = cfg;
    for (double k : cfg_.stiffness) {
      if (!std::isfinite(k) || k < 0.0) {return false;}
    }
    for (double d : cfg_.damping) {
      if (!std::isfinite(d) || d < 0.0) {return false;}
    }
    if (!std::isfinite(cfg_.zeta) || cfg_.zeta <= 0.0) {return false;}
    for (double v : cfg_.torque_limit) {
      if (!std::isfinite(v) || v <= 0.0) {return false;}
    }
    d_ = cfg_.damping;
    bool given = false;
    for (double v : d_) {if (v > 0.0) {given = true; break;}}
    if (!given) {d_ = damping_from_stiffness(cfg_.stiffness, cfg_.lambda, cfg_.zeta);}
    configured_ = true;
    return true;
  }

  /// 计算任务空间力矩并映射到关节。
  ///
  /// @param J     几何雅可比（6×n）。`!valid()` 或与 tau.size() 不符 → 返回 ok=false。
  /// @param dx    任务空间位姿误差（位置 m + **旋转向量** rad）。
  /// @param dxdot 任务空间速度误差（m/s, rad/s）。可全 0。
  /// @param tau   输出（长度 = n）。
  ImpedanceResult compute(const Jacobian & J, const Vec6 & dx, const Vec6 & dxdot,
                          std::size_t n_joints) const noexcept
  {
    ImpedanceResult out;
    if (!configured_ || !J.valid() || J.cols != n_joints || n_joints == 0) {return out;}
    if (!vec6_finite(dx) || !vec6_finite(dxdot)) {return out;}

    // ① 任务空间弹簧-阻尼力
    for (std::size_t i = 0; i < 6; ++i) {
      out.wrench[i] = cfg_.stiffness[i] * dx[i] + d_[i] * dxdot[i];
    }
    if (!vec6_finite(out.wrench)) {return out;}

    // ② τ = Jᵀ · F
    out.tau.assign(n_joints, 0.0);
    for (std::size_t j = 0; j < n_joints; ++j) {
      double s = 0.0;
      for (std::size_t i = 0; i < 6; ++i) {s += J.at(i, j) * out.wrench[i];}
      out.tau[j] = s;
    }
    // ③ 逐关节夹到 max_safe_torque（本地保险；limit_guard 还会再夹一次前馈）
    if (cfg_.torque_limit.size() == n_joints) {
      for (std::size_t j = 0; j < n_joints; ++j) {
        if (!std::isfinite(out.tau[j])) {return out;}
        const double lim = cfg_.torque_limit[j];
        if (out.tau[j] > lim) {out.tau[j] = lim; ++out.clamped_joints;
          out.clamped_by_local_limit = true;}
        else if (out.tau[j] < -lim) {out.tau[j] = -lim; ++out.clamped_joints;
          out.clamped_by_local_limit = true;}
      }
    }
    out.ok = true;
    return out;
  }

  const Vec6 & damping() const noexcept {return d_;}
  const ImpedanceConfig & config() const noexcept {return cfg_;}

private:
  ImpedanceConfig cfg_{};
  Vec6 d_{};
  bool configured_{false};
};

}  // namespace lite_move_controller
