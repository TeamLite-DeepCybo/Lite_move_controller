// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// dynamics_kdl.hpp —— 把 `dynamics.hpp` 的参数接到 KDL 的动力学/雅可比求解器上
//
// ===========================================================================
// 提供四件事（导纳与阻抗都靠它们）
// ===========================================================================
//
//   ① `gravityTorque(q)`     —— τ_g(q)，用于前馈（**与 lite_controller 的
//       GravityCompensator 同一条路线**：把辨识的 mass/com 写回模型再算 rnea）
//   ② `jacobian(q)`          —— 6×n 几何雅可比（末端在 root 下）
//   ③ `estimateWrench(...)`  —— `τ_ext = τ_meas − τ_model` → `F_ext = (Jᵀ)⁺ τ_ext`
//   ④ `tauFromWrench(...)`   —— `τ = Jᵀ F`（阻抗律用）
//
// ===========================================================================
// ⚠️ 惯量张量的使用限制（必须遵守）
// ===========================================================================
//
// `lite_dyid` 在**静态+摩擦**模型下把 `I` 全置 0，并把
// `inertia.usable_for_mass_matrix` 置 false。此时：
//   · 重力前馈**不受影响**（τ_g 只依赖 mass 与 com）；
//   · 但**不能**用它算 M(q)。本文件因此**不提供** `massMatrix()`：见 `ready()`。
//   · 导纳的虚拟质量用**配置的名义值**（`M_d` 是调参量，不是辨识量）。
//
// 装了惯量参数（`--with-inertia`）之后，逐分量仍**非唯一**（数值秩 < 参数个数），
// 所以正确用法是"整体写回模型后用 rnea"，而不是单独解读某个 Ixx。
// 本文件按这个语义用：把 I 写进 KDL segment 的 RigidBodyInertia。
//
// ===========================================================================
// ⚠️ 坐标系（KDL 与 URDF 的差异，踩过才知道）
// ===========================================================================
//
// KDL 的 segment frame 与 URDF 的 link frame **可以有差别**（KDL 会把关节轴
// 对齐到 Z）。而 `params.json` 里的 `com` 是**URDF link frame** 下的。
// 直接把 URDF 的 com 写进 KDL segment 会得到**方向错的力矩**（大小对、方向错，
// 表现为"某个姿态下补偿反向"）。
//
// 因此本文件**不自己拼装**：用 `kdl_parser` 建出来的链（它已经处理了 frame
// 对齐），把 `mass` 与 `com` 按**连杆名**映射到链上的 segment 的 joint 名，
// 再整体替换该段的 `RigidBodyInertia`。⚠️ 名字对不上的连杆会被**跳过并计数**，
// 调用方必须检查 `unmatchedLinks()` —— 静默少补一个连杆 = 那个关节永远顶不住自重。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "lite_move_controller/dynamics.hpp"
#include "lite_ik_node/ik_kdl.hpp"               // KdlArmChain
#include "lite_move_controller/impedance.hpp"    // Jacobian
#include "lite_move_controller/input_filter.hpp" // Vec3

namespace KDL
{
class ChainDynParam;
class ChainJntToJacSolver;
}  // namespace KDL

namespace lite_move_controller
{

/// 一条臂的动力学/雅可比模型。
class KdlDynamics
{
public:
  KdlDynamics();
  ~KdlDynamics();
  KdlDynamics(const KdlDynamics &) = delete;
  KdlDynamics & operator=(const KdlDynamics &) = delete;
  KdlDynamics(KdlDynamics &&) noexcept;
  KdlDynamics & operator=(KdlDynamics &&) noexcept;

  /// 绑定到一条已建好的链，并把 `params` 里的质量/质心写进去。
  ///
  /// @param chain      已经 `build()` 成功的 KDL 链（本类**不持有**它，只读）
  /// @param params     `lite_dyid` 的 schema 2 参数
  /// @param gravity    重力向量（世界系，通常 {0, 0, -9.81}）
  /// @param error_out  失败原因
  ///
  /// ⚠️ 连杆参数的映射用**段名 → params 的连杆名**：KDL 建链时会把关节轴对齐
  ///    到 Z，segment frame 与 URDF link frame **可以不同**；直接按 URDF 的
  ///    com 拼装会得到"方向错的力矩"（大小对、方向错）。所以只做"按名替换
  ///    整段 RigidBodyInertia"，不自己做 frame 变换。
  bool load(const KdlArmChain & chain,
            const DynamicsParams & params,
            const double gravity[3],
            std::string & error_out);

  bool ready() const noexcept {return ready_;}
  std::size_t dof() const noexcept {return dof_;}

  /// 是否具备可用的惯量张量（`params.usable_for_mass_matrix`）。
  /// `false` ⇒ 调用方**不得**用它做惯性整形，应退回名义质量。
  bool hasInertia() const noexcept {return has_inertia_;}

  /// 名字对不上、被跳过的连杆（**必须检查**：静默少补 = 那个关节永远顶不住自重）。
  const std::vector<std::string> & unmatchedLinks() const noexcept {return unmatched_;}

  /// τ_g(q)。失败返回 false（调用方必须**放弃本拍前馈**而不是发 0 ——
  /// 发 0 在抓着重物时等于突然卸力）。
  bool gravityTorque(const std::vector<double> & q, std::vector<double> & tau_g) const;

  /// 6×n 几何雅可比（末端在 root link 下）。
  bool jacobian(const std::vector<double> & q, Jacobian & J) const;

  /// `τ = Jᵀ · F`（阻抗律用）。
  bool tauFromWrench(const Jacobian & J, const std::array<double, 6> & wrench,
                     std::vector<double> & tau) const;

  /// `F = (Jᵀ)⁺ · τ`（外力估计用）。
  ///
  /// 实现：`(Jᵀ)⁺ = J·(J·Jᵀ + λ²I)⁻¹` —— 阻尼最小二乘（DLS）。
  /// ⚠️ 为什么必须阻尼：7 自由度臂在某些位形下 J 的条件数极大，
  ///    无阻尼的伪逆会给出**巨大的**假外力（实测会直接把导纳推飞）。
  ///
  /// @param lambda 阻尼系数（0 < λ，默认 1e-3；越大越保守）
  bool estimateWrench(const Jacobian & J, const std::vector<double> & tau_ext,
                      std::array<double, 6> & wrench_out, double lambda) const;

  /// 采样得到的"该位形下 J 是否病态"（最小奇异值的粗估）。
  /// 用于 `ik_backend`/状态的 `degraded` 判定与日志。
  double conditioningHint(const Jacobian & J) const;

  const std::vector<std::string> & jointNames() const noexcept {return joints_;}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<std::string> joints_;
  std::vector<std::string> unmatched_;
  std::size_t dof_{0};
  bool ready_{false};
  bool has_inertia_{false};
};

}  // namespace lite_move_controller
