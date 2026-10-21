// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// admittance.hpp —— 导纳控制律（无力传感器，外力由力矩残差反推）
//
// ===========================================================================
// 控制目标与"为什么是这个形式"
// ===========================================================================
//
// 导纳 = "被外力推动的顺从偏移"。用户推末端一个力 F，我们希望末端
// **让开一段位移 x_c**，而不是硬顶回去。用二阶虚拟动力学描述：
//
//     M_d · ẍ_c + D_x · ẋ_c + K_a · x_c = F_ext            (6 维，笛卡尔)
//
//   · M_d （虚拟质量）—— 决定"跟手"的惯性感，大 = 迟钝但稳；
//   · D_x （虚拟阻尼）—— 决定松手后停下来的快慢；
//   · K_a （虚拟刚度）—— **人推它让多少**：K_a = 0 ⇒ 自由拖动；
//                          K_a → ∞ ⇒ 近似刚体跟随（就退化成位置控制）。
//
// 解出的 x_c 叠加在期望轨迹上：`x_ref = x_d(t) + x_c`，再走 IK。
//
// ⚠️ **为什么用"导纳"而不是"阻抗"**：本层下发的 MIT 帧里位置目标是
//    电机内部位置环的输入。若直接给"力"，就变成阻抗；给"让开后的位置"，
//    就是导纳。两者都在本包里实现，由**启动期参数**选择（见 docs/设计.md §6）。
//
// ===========================================================================
// 无力传感器：F_ext 从哪来
// ===========================================================================
//
//     τ_ext = τ_measured − τ_model              （τ_model = g + C·q̇ + 摩擦 + 我们自己的前馈）
//     F_ext = J(q)^{+T} · τ_ext                 （伪逆转置，只保留"能产生末端运动"的分量）
//
// ⚠️ 两个已知局限（必须让上游知道，见 move_msgs/InteractionEvent.msg 的注释）：
//   ① 沿手臂轴线的纯压缩力、以及零空间内的力，**在 J 的零空间里，看不见**；
//   ② τ_measured 含减速箱摩擦与温漂 ⇒ F_ext 只适合**阈值事件检测**，不是力测量。
//
// ===========================================================================
// 三条安全机制（都有单测）
// ===========================================================================
//
// ① **偏移限幅 + 条件积分（anti-windup）**：
//    若 x_c 已经顶到 x_c_max 而误差还在同向累积，纯积分会"充爆"内部状态，
//    松手后弹飞。这里在饱和时**冻结**该轴的积分（conditional integration）。
//
// ② **外力低通（默认 10 Hz）**：
//    τ_meas 有量化噪声，不滤会让 x_c 抖动。低通放在**力**这一级
//    （而不是 x_c 那一级），因为虚拟动力学本身已经在滤动。
//
// ③ **dt 防护**：dt ≤ 0 / 非有限 → **不推进**（返回当前状态）。
//    时钟回跳不该让虚拟质量被"倒着积分"。
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "lite_move_controller/butterworth.hpp"

namespace lite_move_controller
{

/// 6 维笛卡尔向量（先 3 个力/平移，后 3 个力矩/旋转）。
using Vec6 = std::array<double, 6>;
/// 6×6 行主序矩阵。
using Mat6 = std::array<double, 36>;

inline Vec6 vec6_zero() {return Vec6{0, 0, 0, 0, 0, 0};}

inline bool vec6_finite(const Vec6 & v) noexcept
{
  for (double x : v) {if (!std::isfinite(x)) {return false;}}
  return true;
}

inline double vec6_norm(const Vec6 & v) noexcept
{
  double s = 0.0;
  for (double x : v) {s += x * x;}
  return std::sqrt(s);
}

inline Mat6 mat6_identity() noexcept
{
  Mat6 m{};
  for (int i = 0; i < 6; ++i) {m[static_cast<std::size_t>(i) * 6 + i] = 1.0;}
  return m;
}

/// 对角矩阵（本层所有 6×6 参数都是对角的 —— 各轴解耦）。
inline Mat6 mat6_diag(const Vec6 & d) noexcept
{
  Mat6 m{};
  for (int i = 0; i < 6; ++i) {m[static_cast<std::size_t>(i) * 6 + i] = d[static_cast<std::size_t>(i)];}
  return m;
}

inline Vec6 mat6_vec(const Mat6 & m, const Vec6 & v) noexcept
{
  Vec6 out{};
  for (int i = 0; i < 6; ++i) {
    double s = 0.0;
    for (int j = 0; j < 6; ++j) {
      s += m[static_cast<std::size_t>(i) * 6 + j] * v[static_cast<std::size_t>(j)];
    }
    out[static_cast<std::size_t>(i)] = s;
  }
  return out;
}

/// 解 `A x = b`（6×6，**部分主元**高斯消元）。
///
/// ⚠️ 为什么不用求逆：M_d 对角时除法就够；但设计上允许非对角（惯量整形），
///    所以留一个通用求解器。奇异时返回 false，**调用方必须处理**
///    （不要拿 NaN 继续算 —— 那会静默污染整条链路）。
inline bool solve6(Mat6 A, Vec6 b, Vec6 & x) noexcept
{
  for (int col = 0; col < 6; ++col) {
    // 选主元
    int piv = col;
    double best = std::fabs(A[static_cast<std::size_t>(col) * 6 + col]);
    for (int r = col + 1; r < 6; ++r) {
      const double v = std::fabs(A[static_cast<std::size_t>(r) * 6 + col]);
      if (v > best) {best = v; piv = r;}
    }
    if (!(best > 1e-12)) {return false;}
    if (piv != col) {
      for (int c = 0; c < 6; ++c) {
        std::swap(A[static_cast<std::size_t>(col) * 6 + c],
                  A[static_cast<std::size_t>(piv) * 6 + c]);
      }
      std::swap(b[static_cast<std::size_t>(col)], b[static_cast<std::size_t>(piv)]);
    }
    const double d = A[static_cast<std::size_t>(col) * 6 + col];
    for (int r = col + 1; r < 6; ++r) {
      const double f = A[static_cast<std::size_t>(r) * 6 + col] / d;
      if (f == 0.0) {continue;}
      for (int c = col; c < 6; ++c) {
        A[static_cast<std::size_t>(r) * 6 + c] -=
          f * A[static_cast<std::size_t>(col) * 6 + c];
      }
      b[static_cast<std::size_t>(r)] -= f * b[static_cast<std::size_t>(col)];
    }
  }
  for (int i = 5; i >= 0; --i) {
    double s = b[static_cast<std::size_t>(i)];
    for (int j = i + 1; j < 6; ++j) {
      s -= A[static_cast<std::size_t>(i) * 6 + j] * x[static_cast<std::size_t>(j)];
    }
    x[static_cast<std::size_t>(i)] = s / A[static_cast<std::size_t>(i) * 6 + i];
  }
  for (double v : x) {if (!std::isfinite(v)) {return false;}}
  return true;
}

/// 给定任务空间刚度 K 与"有效惯量" Λ，按临界阻尼比 ζ 推阻尼：
///     D_ii = 2·ζ·√(K_ii · Λ_ii)
/// ⚠️ K_ii 或 Λ_ii ≤ 0 时该轴阻尼取 0（不产生"负阻尼" —— 那是正反馈）。
inline Vec6 damping_from_stiffness(const Vec6 & K, const Vec6 & Lambda,
                                   double zeta) noexcept
{
  Vec6 D{};
  for (std::size_t i = 0; i < 6; ++i) {
    const double kl = K[i] * Lambda[i];
    D[i] = (kl > 0.0 && std::isfinite(kl)) ? 2.0 * zeta * std::sqrt(kl) : 0.0;
  }
  return D;
}

/// 导纳控制的配置。
struct AdmittanceConfig
{
  /// 虚拟质量 M_d（kg, kg·m²）。**必须 > 0**（=0 会变成"无穷快"，数值上发散）。
  Vec6 mass{1.0, 1.0, 1.0, 0.05, 0.05, 0.05};
  /// 虚拟刚度 K_a（N/m, N·m/rad）。0 = 自由拖动（纯导纳）。
  Vec6 stiffness{100.0, 100.0, 100.0, 2.0, 2.0, 2.0};
  /// 虚拟阻尼 D_x。为 0 时用 `damping_from_stiffness` 半自动推（推荐）。
  Vec6 damping{0, 0, 0, 0, 0, 0};
  /// 半自动推阻尼时的阻尼比（0.7~1.0 常用；< 0.5 会看到振荡）。
  double zeta{0.9};
  /// 顺从偏移上限（m, rad）。**必须有限** —— 它的作用是防"有人长期压住 → 状态饱和"。
  Vec6 max_offset{0.10, 0.10, 0.10, 0.30, 0.30, 0.30};
  /// 有效惯量 Λ（用于推阻尼）。缺省用 M_d 本身（通常够用）。
  Vec6 lambda_from_mass{true, true, true, true, true, true};
  /// 外力估计低通截止频率（Hz）。<=0 = 不滤。
  double wrench_lpf_hz{10.0};
};

/// 导纳控制律（纯逻辑，无 ROS、无 Eigen）。
class AdmittanceLaw
{
public:
  struct Status
  {
    bool saturated{false};        ///< 本拍有轴顶到限幅
    bool solve_failed{false};     ///< M_d 奇异（配置错误）
    bool advanced{false};         ///< 本拍是否真的推进了虚拟动力学
    std::size_t saturated_axes{0};
    double dt_used{0.0};
  };

  /// 配置。返回 false 表示参数不自洽（负质量 / 负刚度 / 非有限），**调用方必须拒绝启动**。
  bool configure(const AdmittanceConfig & cfg) noexcept
  {
    cfg_ = cfg;
    for (std::size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(cfg_.mass[i]) || cfg_.mass[i] <= 0.0) {return false;}
      if (!std::isfinite(cfg_.stiffness[i]) || cfg_.stiffness[i] < 0.0) {return false;}
      if (!std::isfinite(cfg_.max_offset[i]) || cfg_.max_offset[i] <= 0.0) {return false;}
      if (!std::isfinite(cfg_.damping[i]) || cfg_.damping[i] < 0.0) {return false;}
    }
    if (!std::isfinite(cfg_.zeta) || cfg_.zeta <= 0.0) {return false;}
    d_ = cfg_.damping;
    // 阻尼未显式给（全 0）⇒ 按 ζ 与"有效惯量 = 虚拟质量"半自动推。
    // ⚠️ 用显式的逐项判断而不是数组比较：std::array<bool,6> 的 operator==
    //    在 C++17 下不保证（C++20 才补上），会让编译依赖标准版本。
    bool damping_given = false;
    for (double v : d_) {if (v > 0.0) {damping_given = true; break;}}
    if (!damping_given) {
      d_ = damping_from_stiffness(cfg_.stiffness, cfg_.mass, cfg_.zeta);
    }
    configured_ = true;
    reset();
    return true;
  }

  /// 复位（激活时、故障恢复时调用）。偏移与速度清零，外力滤波器复位。
  void reset() noexcept
  {
    x_ = vec6_zero();
    xd_ = vec6_zero();
    wrench_filt_ = vec6_zero();
    for (auto & f : lpf_) {f.reset(0.0);}
    lpf_ok_ = false;
    frames_ = 0;
  }

  /// 每拍推进一步。
  ///
  /// @param wrench_ext  估计外力（N, N·m）。非有限则本拍不推进（返回上一状态）。
  /// @param dt          控制周期（秒）。<=0 / 非有限 → 不推进。
  /// @return 顺从偏移（m, rad），即 `x_c`。
  Vec6 step(const Vec6 & wrench_ext, double dt) noexcept
  {
    status_ = Status{};
    status_.dt_used = dt;
    if (!configured_ || !vec6_finite(wrench_ext) || !(dt > 0.0) || !std::isfinite(dt)) {
      return x_;
    }
    // ① 力低通（在力这一级滤，$x_c$ 由虚拟动力学自然平滑）
    if (!lpf_ok_) {
      for (std::size_t i = 0; i < 6; ++i) {lpf_[i].reset(wrench_ext[i]);}
      lpf_ok_ = true;
    }
    Vec6 f{};
    for (std::size_t i = 0; i < 6; ++i) {f[i] = lpf_[i].step(wrench_ext[i]);}

    // ② 半隐式欧拉：先把阻尼/刚度项算成"广义力"，再解 M_d·ẍ = F
    Vec6 rhs{};
    for (std::size_t i = 0; i < 6; ++i) {
      rhs[i] = f[i] - d_[i] * xd_[i] - cfg_.stiffness[i] * x_[i];
    }
    Vec6 xdd{};
    // M_d 在本层是**对角**配置（各轴解耦），所以直接除即可。
    // 逐轴相除时**最后一个轴用解算器**没有必要：非对角情况由 ImpedanceLaw 的
    // 任务空间力矩承担，导纳的虚拟质量保持对角是有意的简化
    //（设计文档 §6：M_d 是"调参量"，不是辨识量）。
    for (std::size_t i = 0; i < 6; ++i) {
      xdd[i] = rhs[i] / cfg_.mass[i];
      if (!std::isfinite(xdd[i])) {status_.solve_failed = true; return x_;}
    }

    // ③ 条件积分（anti-windup）：饱和且误差同向时**冻结该轴**
    for (std::size_t i = 0; i < 6; ++i) {
      const double x_new = x_[i] + xd_[i] * dt + 0.5 * xdd[i] * dt * dt;
      const double xd_new = xd_[i] + xdd[i] * dt;
      if (std::fabs(x_new) > cfg_.max_offset[i]) {
        // 顶到限幅：位置钉住，速度清零，**不再累积**
        x_[i] = std::copysign(cfg_.max_offset[i], x_new);
        xd_[i] = 0.0;
        status_.saturated = true;
        ++status_.saturated_axes;
      } else {
        x_[i] = x_new;
        xd_[i] = xd_new;
      }
    }
    status_.advanced = true;
    ++frames_;
    return x_;
  }

  const Vec6 & offset() const noexcept {return x_;}
  const Vec6 & offset_rate() const noexcept {return xd_;}
  const AdmittanceConfig & config() const noexcept {return cfg_;}
  const Status & status() const noexcept {return status_;}
  std::uint64_t frames() const noexcept {return frames_;}

  /// 外力是否超过给定阈值（供 InteractionEvent 用）。
  bool wrenchExceeds(const Vec6 & w, double threshold) const noexcept
  {
    return vec6_norm(w) > threshold;
  }

private:
  AdmittanceConfig cfg_{};
  Vec6 d_{};
  Vec6 x_{}, xd_{}, wrench_filt_{};
  std::array<Butterworth, 6> lpf_{};
  bool lpf_ok_{false};
  bool configured_{false};
  Status status_{};
  std::uint64_t frames_{0};
};

}  // namespace lite_move_controller
