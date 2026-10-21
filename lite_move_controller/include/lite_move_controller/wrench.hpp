// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// wrench.hpp —— 关节力矩 ↔ 末端外力 的映射（**与用哪个物理库无关**）
//
// 这两个函数先前挂在 `KdlDynamics` 上，但它们的数学**只依赖 `Jacobian`**：
//
//     τ = Jᵀ · F                    （阻抗：任务空间弹簧-阻尼 → 关节力矩）
//     F = J·(J·Jᵀ + λ²I)⁻¹ · τ      （外力估计：由 τ_ext 反推）
//
// 改成自由函数后，换物理库（KDL → pinocchio）不会牵动它们，也不会
// 出现"因为换了库所以外力估计没了"这种耦合。
//
// ⚠️ 反解必须用**阻尼**最小二乘（DLS）：7 轴臂在某些位形下 `J` 条件数极大，
//    无阻尼伪逆会给出**巨大的**假外力（实测会把导纳直接推飞）。
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "lite_move_controller/admittance.hpp"   // Mat6 / Vec6 / solve6
#include "lite_move_controller/impedance.hpp"    // Jacobian

namespace lite_move_controller
{

/// `τ = Jᵀ · F`。失败（J 不合法 / 非有限）返回 false。
inline bool tauFromWrench(const Jacobian & J, const std::array<double, 6> & wrench,
                          std::vector<double> & tau)
{
  if (!J.valid()) {return false;}
  tau.assign(J.cols, 0.0);
  for (std::size_t j = 0; j < J.cols; ++j) {
    double s = 0.0;
    for (std::size_t r = 0; r < 6; ++r) {s += J.at(r, j) * wrench[r];}
    if (!std::isfinite(s)) {return false;}
    tau[j] = s;
  }
  return true;
}

/// `F = J·(J·Jᵀ + λ²I)⁻¹ · τ`（阻尼最小二乘）。
///
/// @param lambda 阻尼系数（>0；默认由调用方给 1e-3）
inline bool estimateWrench(const Jacobian & J, const std::vector<double> & tau,
                           std::array<double, 6> & wrench_out, double lambda)
{
  if (!J.valid() || tau.size() != J.cols) {return false;}
  // A = J·Jᵀ + λ²I（6×6，对称正定）
  Mat6 A{};
  for (std::size_t r = 0; r < 6; ++r) {
    for (std::size_t c = 0; c < 6; ++c) {
      double s = 0.0;
      for (std::size_t j = 0; j < J.cols; ++j) {s += J.at(r, j) * J.at(c, j);}
      A[r * 6 + c] = s;
    }
  }
  const double lam = (lambda > 0.0) ? lambda : 1e-6;
  for (std::size_t r = 0; r < 6; ++r) {A[r * 6 + r] += lam * lam;}
  // rhs = J·τ
  Vec6 rhs{};
  for (std::size_t r = 0; r < 6; ++r) {
    double s = 0.0;
    for (std::size_t j = 0; j < J.cols; ++j) {s += J.at(r, j) * tau[j];}
    rhs[r] = s;
  }
  Vec6 y{};
  if (!solve6(A, rhs, y)) {return false;}    // 奇异 ⇒ 放弃本拍估计
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(y[i])) {return false;}
  }
  wrench_out = y;
  return true;
}

}  // namespace lite_move_controller
