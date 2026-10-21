// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// dynamics_pin.hpp —— 用 **pinocchio** 实现动力学（与 lite_dyid 辨识侧同库同 API）
//
// ===========================================================================
// 为什么改用 pinocchio（而不是手写 KDL 物理）
// ===========================================================================
//
// `lite_dyid` 的辨识是用 pinocchio 做的：
//   · 重力：`pin.computeGeneralizedGravity(gmodel, q)`，`gmodel` 只含 (m, μ)；
//   · 惯性：`pin.computeJointTorqueRegressor(M, D, q, v, a)` 的**6 个惯量列**；
//   · 参数基：标准 10 参数/连杆 `[m, mcx, mcy, mcz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`。
//
// 我先前的实现是手写 KDL 刚体注入，**与辨识侧不等价**（质心跑到 39.8 米外、
// 力矩差到 28 倍），根因是我误把"一阶矩"当"质心"，又自己猜了 KDL 的 frame 约定。
// 结论：**不要重实现物理**。用同一个库、同一句 API，一致性由构造保证。
//
// ===========================================================================
// 单点动力学模型（与 fastsolve.py 的 tgblk / Ablk 逐项对应）
// ===========================================================================
//
//   τ(q,v,a) = τ_g(q)                                  ← gmodel（只含 m, μ）
//            + [ rnea(full, q,v,a) − rnea(full, q,0,0) ]  ← 惯性 + 科氏 + 离心
//            + τ_friction(v)                            ← c1±/c2
//
// 其中 `full` 模型每连杆的 10 参数 = `gravity.params` 的 (m, μ) **拼接**
// `inertia.raw_fit.pi_I` 的 6 个惯量分量。这正是 `params.json` 里那句
// note 的要求："请整体写回模型后用 rnea 使用，不要单独解读 Ixx"。
//
// ===========================================================================
// 三种模式的补偿口径（用户指定，**结构保证一致**）
// ===========================================================================
//
//   常规   ：τ_g + τ_friction + 惯性/科氏(a = a_ref)   ← 全量
//   阻抗   ：τ_g + τ_friction                          ← 不加惯性（含科氏）
//   导纳   ：τ_g + τ_friction                          ← 不加惯性（含科氏）
//
// 三种模式都调用**同一个** `modelTorque()`，只切换 `withInertia` 开关 ——
// 这样"一致"不是靠三处各写一遍对齐，而是由同一个函数保证。
#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "lite_move_controller/dynamics.hpp"
#include "lite_move_controller/impedance.hpp"   // Jacobian
#include "lite_move_controller/input_filter.hpp" // Vec3

namespace lite_move_controller
{

/// 用 pinocchio 计算的一条臂的动力学 + 雅可比。
class PinDynamics
{
public:
  PinDynamics();
  ~PinDynamics();
  PinDynamics(const PinDynamics &) = delete;
  PinDynamics & operator=(const PinDynamics &) = delete;
  PinDynamics(PinDynamics &&) noexcept;
  PinDynamics & operator=(PinDynamics &&) noexcept;

  /// 建模型并注入辨识参数。
  ///
  /// @param urdf_text      URDF 全文（来自 /robot_description 或 urdf_path）
  /// @param joint_names    本臂 7 个关节（**顺序即输出顺序**）
  /// @param tip_frame      末端 frame 名（求 Jacobian 用）
  /// @param params         `lite_dyid` 的 schema 2 参数
  /// @param gravity        重力向量（世界系，通常 {0,0,-9.81}）
  bool load(const std::string & urdf_text,
            const std::vector<std::string> & joint_names,
            const std::string & tip_frame,
            const DynamicsParams & params,
            const double gravity[3],
            std::string & error_out);

  bool ready() const noexcept {return ready_;}
  std::size_t dof() const noexcept {return dof_;}
  const std::vector<std::string> & jointNames() const noexcept {return joints_;}

  /// 是否具备可用的惯量项（`pi_I` 存在且维数匹配）。
  bool hasInertiaTerm() const noexcept {return has_inertia_term_;}

  /// τ_g(q)。重力只依赖 (m, μ)，与惯量张量**无关**。
  bool gravityTorque(const std::vector<double> & q, std::vector<double> & tau_g) const;

  /// **统一**的模型力矩：重力 + 摩擦 [+ 惯性/科氏]。
  ///
  /// @param with_inertia  `false` 时**完全不加**惯性/科氏项（阻抗/导纳用）
  /// @param a            关节加速度（`with_inertia=false` 时忽略）
  bool modelTorque(const std::vector<double> & q,
                   const std::vector<double> & v,
                   const std::vector<double> & a,
                   bool with_inertia,
                   std::vector<double> & tau_out) const;

  /// 6×n 几何雅可比（末端在**世界系**，与已验证的 KDL 约定一致）。
  bool jacobian(const std::vector<double> & q, Jacobian & J) const;

  /// 单次 `modelTorque` 的耗时（µs）—— 用于判断是否需要降频缓存。
  double lastCostUs() const noexcept {return last_cost_us_;}

  /// 未能注入的连杆（名字对不上）——**调用方必须检查**。
  const std::vector<std::string> & unmatchedLinks() const noexcept {return unmatched_;}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<std::string> joints_;
  std::vector<std::string> unmatched_;
  std::size_t dof_{0};
  bool ready_{false};
  bool has_inertia_term_{false};
  mutable double last_cost_us_{0.0};
};

}  // namespace lite_move_controller
