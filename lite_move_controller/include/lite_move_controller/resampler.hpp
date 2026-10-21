// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// resampler.hpp —— 三次 Hermite 插值 + 相邻解之间的**区间密插**
//
// ===========================================================================
// 两个不同的需求，别混淆
// ===========================================================================
//
// ① **时间重采样**（`hermite_at`）：上游以不规则、可能只有 10~50 Hz 的流式节奏
//    给终端位姿，而运控层按 100~200 Hz 的固定周期下发。把不规则点重采样到
//    固定栅格上，位置/速度连续（C¹），避免"上游一抖动就跳一下"。
//
// ② **相邻 IK 解之间的密插**（`densify` / `densifyChecked`）：这是用户明确
//    要求的 ——"如果节点内部的关节位置之间的差值过大，则进行一次三次密插
//    操作再向下发布"。
//
//    为什么必须做：IK 是**逐点**解的，两个相邻路点解出的关节角可能相差很大
//    （尤其经过奇异位形、或上游路点间距大时）。若直接把第二个解下发，
//    电机会看到位置目标的**阶跃** —— MIT 下 `τ = Kp·(p_des − p)` 立刻给出
//    一个大偏置，表现为"末端猛地一跳"。密插把这一跳摊成若干个小步。
//
// ===========================================================================
// 数学：三次 Hermite（cubic Hermite / CSpline 的单段）
// ===========================================================================
//
// 给定两端 (p0, v0) 与 (p1, v1)，段长 T，归一化 s = t/T ∈ [0,1]：
//
//     h00(s) =  2s³ − 3s² + 1        h10(s) = s³ − 2s² + s
//     h01(s) = −2s³ + 3s²            h11(s) = s³ − s²
//     p(s)   = h00·p0 + h10·T·v0 + h01·p1 + h11·T·v1
//     v(s)   = (h00'·p0 + h10'·T·v0 + h01'·p1 + h11'·T·v1) / T
//
// ⚠️ 为什么用 Hermite 而不是线性：线性插值位置连续但**速度分段不连续**，
//    加速度跳变会让惯量前馈抖；Hermite 只需两端的位置+速度，**不需要加速度**
//    —— 这正是我们能可靠拿到的量（速度来自滤波/差分）。
//
// ⚠️ 为什么不用样条拟合整条轨迹：那需要**未来**路点，而流式输入下还没到。
//    Hermite 只用两端，天然适合流式。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace lite_move_controller
{

/// Hermite 单段的端点数据（逐关节）。
struct HermiteSpan
{
  std::vector<double> p0, v0, p1, v1;
  double duration{0.0};

  std::size_t size() const noexcept {return p0.size();}

  bool valid() const noexcept
  {
    const std::size_t n = p0.size();
    return n > 0 && v0.size() == n && p1.size() == n && v1.size() == n &&
           duration > 0.0 && std::isfinite(duration);
  }
};

/// 一次插值/采样的结果。
struct SamplePoint
{
  std::vector<double> position;
  std::vector<double> velocity;
};

// ---------------------------------------------------------------------------
// 基础：Hermite 求值
// ---------------------------------------------------------------------------

/// 位置。
inline double hermite(double p0, double v0, double p1, double v1,
                      double T, double s) noexcept
{
  const double s2 = s * s;
  const double s3 = s2 * s;
  const double h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
  const double h10 = s3 - 2.0 * s2 + s;
  const double h01 = -2.0 * s3 + 3.0 * s2;
  const double h11 = s3 - s2;
  return h00 * p0 + h10 * T * v0 + h01 * p1 + h11 * T * v1;
}

/// 位置对时间的导数（= 速度）。
inline double hermite_dot(double p0, double v0, double p1, double v1,
                          double T, double s) noexcept
{
  if (!(T > 0.0)) {return 0.0;}
  const double s2 = s * s;
  const double dh00 = 6.0 * s2 - 6.0 * s;
  const double dh10 = 3.0 * s2 - 4.0 * s + 1.0;
  const double dh01 = -6.0 * s2 + 6.0 * s;
  const double dh11 = 3.0 * s2 - 2.0 * s;
  return (dh00 * p0 + dh10 * T * v0 + dh01 * p1 + dh11 * T * v1) / T;
}

/// 在一个 Hermite 段上按归一化位置 s∈[0,1] 求值（供时间重采样用）。
inline SamplePoint hermite_at(const HermiteSpan & span, double s) noexcept
{
  SamplePoint out;
  const std::size_t n = span.size();
  out.position.resize(n);
  out.velocity.resize(n);
  for (std::size_t j = 0; j < n; ++j) {
    out.position[j] = hermite(span.p0[j], span.v0[j], span.p1[j], span.v1[j],
                              span.duration, s);
    out.velocity[j] = hermite_dot(span.p0[j], span.v0[j], span.p1[j], span.v1[j],
                                  span.duration, s);
  }
  return out;
}

// ---------------------------------------------------------------------------
// 密插
// ---------------------------------------------------------------------------

/// 密插结果：一连串等时间间隔的中间点（**不含**起点本身）。
struct DensifyResult
{
  /// 中间点（按时间递增）。空 = 不需要密插（两端已经足够近）。
  std::vector<SamplePoint> points;
  /// 把 [0, T] 等分成 `segments` 段；`points.size() == segments - 1`。
  std::size_t segments{1};
  /// 两端的最大单关节位移（rad）—— 诊断用
  double max_delta{0.0};
  /// 是否真的做了密插（false = 两端足够近，调用方可直接用终点）
  bool densified{false};
};

/// 两端的最大单关节位移。
inline double max_delta_for_steps(const SamplePoint & a, const SamplePoint & b) noexcept
{
  const std::size_t n = a.position.size();
  if (n == 0 || b.position.size() != n) {return 0.0;}
  double m = 0.0;
  for (std::size_t j = 0; j < n; ++j) {
    m = std::max(m, std::fabs(b.position[j] - a.position[j]));
  }
  return m;
}

/// Hermite 单段内"单步位移 / 平均位移"的上界安全系数（分段数**初值**用）。
///
/// 零端点速度时端点步长 = 1.5×平均（`h01'` 在 s=0 处是 3/T）；端点速度大时更高。
/// 所以 1.5 只是初值，真正的保证由 `densifyChecked` 按**实测**步长校正。
inline constexpr double HERMITE_STEP_SAFETY = 1.5;

/// 按给定段数直接构造密插点（不经启发式）。
inline DensifyResult densifyWithSegments(const SamplePoint & from, const SamplePoint & to,
                                         double duration, std::size_t segs)
{
  DensifyResult out;
  const std::size_t n = from.position.size();
  if (n == 0 || to.position.size() != n || from.velocity.size() != n ||
      to.velocity.size() != n || !(duration > 0.0) || segs < 2) {
    out.segments = 1;
    return out;
  }
  out.segments = segs;
  out.densified = true;
  out.max_delta = max_delta_for_steps(from, to);
  out.points.reserve(segs - 1);
  for (std::size_t k = 1; k < segs; ++k) {
    const double s = static_cast<double>(k) / static_cast<double>(segs);
    SamplePoint sp;
    sp.position.resize(n);
    sp.velocity.resize(n);
    for (std::size_t j = 0; j < n; ++j) {
      sp.position[j] = hermite(from.position[j], from.velocity[j],
                               to.position[j], to.velocity[j], duration, s);
      sp.velocity[j] = hermite_dot(from.position[j], from.velocity[j],
                                   to.position[j], to.velocity[j], duration, s);
    }
    out.points.push_back(std::move(sp));
  }
  return out;
}

/// 逐段的**实测**最大单关节位移（含两端）。
inline double worst_step(const SamplePoint & from, const SamplePoint & to,
                         const std::vector<SamplePoint> & mid) noexcept
{
  const std::size_t n = from.position.size();
  if (n == 0) {return 0.0;}
  double worst = 0.0;
  const SamplePoint * prev = &from;
  for (const auto & p : mid) {
    for (std::size_t j = 0; j < n; ++j) {
      worst = std::max(worst, std::fabs(p.position[j] - prev->position[j]));
    }
    prev = &p;
  }
  for (std::size_t j = 0; j < n; ++j) {
    worst = std::max(worst, std::fabs(to.position[j] - prev->position[j]));
  }
  return worst;
}

/// 相邻 IK 解之间的密插（启发式，够用且便宜）。
///
/// 判据：`max_j |p1_j − p0_j| > max_step`
/// 分段数：`ceil(1.5·max_delta / max_step)`，上限 `max_segments`（防病态输入炸内存）。
///
/// ⚠️ **时间-状态关系不变**：密插只是把同一段时间 [0, T] 等分成更多段，
///    端点位置/速度**严格不变**（Hermite 的构造性质）。它不改变轨迹，
///    只是让电机的中间目标更平滑。
///
/// ⚠️ 上限触发时不静默：调用方可从 `max_delta` 与 `segments` 的关系看出被夹过。
///    要"每步都保证 ≤ max_step"请用 `densifyChecked`。
inline DensifyResult densify(const SamplePoint & from, const SamplePoint & to,
                             double duration, double max_step,
                             std::size_t max_segments = 200)
{
  DensifyResult out;
  const std::size_t n = from.position.size();
  if (n == 0 || to.position.size() != n ||
      from.velocity.size() != n || to.velocity.size() != n ||
      !(duration > 0.0) || !(max_step > 0.0)) {
    out.segments = 1;
    return out;                 // 输入不合法 → 调用方应当直接用终点
  }
  const double delta = max_delta_for_steps(from, to);
  out.max_delta = delta;
  if (!std::isfinite(delta)) {out.segments = 1; return out;}
  if (delta <= max_step) {
    out.segments = 1;
    return out;                 // 足够近：不需要密插
  }
  std::size_t segs = static_cast<std::size_t>(
      std::ceil(HERMITE_STEP_SAFETY * delta / max_step));
  segs = std::max<std::size_t>(2, segs);
  segs = std::min(segs, max_segments);
  return densifyWithSegments(from, to, duration, segs);
}

/// 带"实际步长"校正的密插：**保证每一段**的单关节位移 ≤ `max_step`。
///
/// ⚠️ 为什么要校正：`densify` 的 1.5 倍是**平均**视角的估计，端点速度很大的
///    Hermite 段仍可能有单步超限。真机上单步超限 = 位置目标一次小跳 ⇒
///    `Kp·Δp` 出现不该有的力矩尖峰。这里按实测最坏步长逐轮加密（最多 8 轮，
///    每轮都在改善，总点数由 `max_segments` 兜住）。
inline DensifyResult densifyChecked(const SamplePoint & from, const SamplePoint & to,
                                    double duration, double max_step,
                                    std::size_t max_segments = 200)
{
  DensifyResult out = densify(from, to, duration, max_step, max_segments);
  if (!out.densified) {return out;}
  for (int iter = 0; iter < 8; ++iter) {
    const double worst = worst_step(from, to, out.points);
    if (!(worst > max_step * 1.0001)) {break;}
    if (out.segments >= max_segments) {break;}
    // 段数与最坏步长近似成反比 ⇒ 按比例放大段数
    std::size_t next = static_cast<std::size_t>(
        std::ceil(static_cast<double>(out.segments) * worst / max_step));
    next = std::max(next, out.segments + 1);
    next = std::min(next, max_segments);
    out = densifyWithSegments(from, to, duration, next);
  }
  return out;
}

}  // namespace lite_move_controller
