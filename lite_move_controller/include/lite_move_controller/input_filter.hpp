// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// input_filter.hpp —— 上游终端位姿的滤波与重采样（**纯逻辑，零 ROS**）
//
// ===========================================================================
// 为什么需要它（用户第 5 条：滤波以保证信息尽可能准确）
// ===========================================================================
//
// 上游是**流式**的（10~200 Hz，不规则到达），而运控层按固定周期下发。
// 直接把上游点当目标用会有三类问题：
//
//   ① **姿态表示不连续**：四元数有双覆盖（q 与 −q 表示同一姿态）。
//      上游若在某个点发生符号翻转，逐分量滤波会得到"穿过 0 插值"的结果 ——
//      姿态瞬移 180°。必须先做**符号对齐**。
//   ② **抖动**：遥操/视觉的位姿噪声会让 IK 解抖，进而让关节目标抖
//      （MIT 下直接变成力矩噪声）。需要低通。
//   ③ **节奏不匹配**：上游 37 Hz、下发 100 Hz，直接用会让"有的周期没有新点"。
//      需要重采样到固定栅格。
//
// ===========================================================================
// 处理链（顺序不可换）
// ===========================================================================
//
//   四元数符号对齐 ──▶ 转旋转向量 ──▶ 逐轴 Butterworth ──▶ 固定栅格重采样
//        (①)              (②)              (③)                   (④)
//
// ① 符号对齐：`dot(q, q_prev) < 0 ⇒ q = −q`。**必须在转旋转向量之前**，
//    否则 log3(R) 会在 ±π 附近跳变。
//
// ② 旋转向量 `log3(R)`：姿态的**最小表示**，在"小幅运动"下是一个线性空间，
//    可以逐分量滤波而不会引入伪旋转。⚠️ 欧拉角不行（万向锁 + 非唯一），
//    四元数逐分量滤波也不行（会破坏单位模长且不是最小表示）。
//
// ③ 逐轴二阶 Butterworth：位置 3 轴 + 旋转向量 3 轴 = **6 个独立实例**。
//    ⚠️ 绝不共用实例 —— 状态会互相污染，症状是"某个方向的动作串到另一个方向"。
//
// ④ 重采样：用上游时间戳建时间轴，插值到固定控制周期。位置用三次 Hermite
//    （**用滤波后的速度**），旋转向量用线性（旋转向量在小幅下线性够用）。
//
// ===========================================================================
// 速度从哪来
// ===========================================================================
//
// ⚠️ **不用上游给的速度**：上游的速度常常不可信（不同源、不同滤波、甚至没给）。
// 本层用**滤波后位置的中心差分**重建速度，再做一次轻低通（默认 25 Hz）。
// 上游若给了速度，只用来做**一致性校验**（偏差 > 30% 打一次性 WARN）。
//
// ===========================================================================
// 时间戳语义（与控制器同一教训）
// ===========================================================================
//
// ⚠️ 到达时刻用**调用方传入的单调毫秒**，不用 `header.stamp` 做主时基 ——
//    `use_sim_time` 不一致会把每一帧都判成陈旧（`lite_controller` 实测踩过）。
//    上游 stamp 只用来：(a) 检测时钟不一致（未来戳 > 200 ms ⇒ 拒该点），
//    (b) 在 `push()` 时算"上游自己的时间间隔"做前馈。
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "lite_ik_node/kinematics_types.hpp"
#include "lite_move_controller/butterworth.hpp"

namespace lite_move_controller
{

/// 六维终端状态：位置（m）+ 旋转向量（rad），以及它们的速度。
struct Pose6
{
  Vec3 position{0, 0, 0};
  Vec3 rotation{0, 0, 0};          ///< 旋转向量（axis·angle, rad）
  Vec3 linear_velocity{0, 0, 0};   ///< m/s
  Vec3 angular_velocity{0, 0, 0};  ///< rad/s

  bool finite() const noexcept
  {
    return vec3_finite(position) && vec3_finite(rotation) &&
           vec3_finite(linear_velocity) && vec3_finite(angular_velocity);
  }
  void zeroVelocities() noexcept
  {
    linear_velocity = vec3_zero();
    angular_velocity = vec3_zero();
  }
};

/// 上游一帧（已解析成纯数据，便于单测）。
struct RawPoseSample
{
  /// **单调毫秒**的到达时刻（调用方给）。⚠️ 不是 header.stamp。
  /// 新鲜度（`ageMs()`）、排序、速度差分**全部**用它。
  std::int64_t recv_ms{0};
  /// **系统时钟**的到达时刻（毫秒，epoch 基准）。**只**用于和 `stamp_ms` 比较，
  /// 判断上游时间戳是否"超前"（时钟不一致）。0 = 调用方没给（跳过该判据）。
  /// ⚠️ 必须是系统时钟：`stamp_ms` 来自 `header.stamp`（也是 epoch 基准），
  ///    拿单调时钟去减它是没有意义的。
  std::int64_t recv_sys_ms{0};
  /// 上游时间戳（毫秒）；仅用于时钟一致性校验，不参与主时基。
  std::int64_t stamp_ms{0};
  bool has_stamp{false};

  Vec3 position{0, 0, 0};
  Quat orientation{0, 0, 0, 1};
  /// 上游给的速度（可选）。只用做一致性校验，**不参与**重建。
  bool has_velocity{false};
  Vec3 linear_velocity{0, 0, 0};
  Vec3 angular_velocity{0, 0, 0};
};

/// 拒绝原因（用于日志与 `MoveStatus.filter_resets` 计数）。
enum class FilterReject : std::uint8_t
{
  None = 0,
  NonFinite = 1,          ///< NaN / Inf
  BadQuaternion = 2,      ///< 模长不可用或偏离 1 太多
  FutureStamp = 3,        ///< 上游时间戳超前（时钟不一致）
  Stale = 4,              ///< 到达时刻倒退（时钟回跳）
  OutOfOrder = 5,         ///< 上游时间戳倒退（乱序）
};

inline const char * filterRejectName(FilterReject r) noexcept
{
  switch (r) {
    case FilterReject::None:          return "NONE";
    case FilterReject::NonFinite:     return "NON_FINITE";
    case FilterReject::BadQuaternion: return "BAD_QUATERNION";
    case FilterReject::FutureStamp:   return "FUTURE_STAMP";
    case FilterReject::Stale:         return "STALE";
    case FilterReject::OutOfOrder:    return "OUT_OF_ORDER";
  }
  return "UNKNOWN";
}

/// 滤波配置。
struct InputFilterConfig
{
  /// 位姿低通截止频率（Hz）。0 = 旁路（不滤）。
  ///
  /// ⚠️ 低通是**按上游采样率**工作的（`push()` 每来一帧跑一次），所以截止频率
  ///    必须相对**上游实测频率**校验，不是相对控制频率：
  ///      上游 10 Hz ⇒ Nyquist 5 Hz，而 15 Hz 是它的 3 倍 ⇒ 那个"低通"实际是
  ///      **放大/混叠**。`push()` 会用实测到达间隔把截止频率夹到
  ///      `0.45 × 上游频率` 并计数（`lpfClampCount()`），不会静默失效。
  double pose_lpf_hz{15.0};
  /// 速度低通截止频率（Hz）。0 = 旁路。⚠️ 比位姿高（速度是差分出来的，已较糙）
  double velocity_lpf_hz{25.0};
  /// 控制周期（秒）。重采样栅格 = 1/control_rate_hz。
  double control_rate_hz{100.0};
  /// 上游时间戳**超前接收方系统时钟**多少毫秒判为"时钟不一致"。
  ///
  /// ⚠️ 语义必须是**绝对**的：`stamp_ms - recv_sys_ms > tol ⇒ 拒绝`。
  ///    2026-10 真机事故：这条曾写成"相对**上一帧**戳"（`stamp > last_stamp + 200`），
  ///    于是上游停流/重启超过 200 ms 之后，第一帧就"永远超前" ⇒ 被拒 ⇒
  ///    `last_stamp_ms_` 只在**接受**路径更新 ⇒ 之后每一帧都继续被拒 ⇒
  ///    **永久锁死**（现场症状：目标流 100 Hz 正常，但一帧都不被接受，
  ///    目标年龄线性增长，状态只能 DWELL）。绝对判据不会锁死。
  ///
  /// 取值 = 一个控制周期 + 明确余量（50 Hz ⇒ 20 ms + 20 ms = 40 ms）。
  /// 正常的同机发布者"戳 − 到达"只有 0~2 ms（传输 + 线程排队），40 ms 已是
  /// 20 倍余量；同时"故意 +50 ms 的伪造未来戳"仍然会被拒（fail-safe 不退化成
  /// 全盘接受）。
  double future_stamp_tolerance_ms{40.0};
  /// `sampleAt()` 的外推时间上限（ms）。防止上游断流后把目标射到天边。
  /// 取 2 个 10 Hz 周期（= 200 ms）+ 一点余量，够低帧率用，也不至于失控。
  double predict_max_ms{250.0};
  /// 上游速度与重建速度的相对偏差超过多少时打一致性告警
  double velocity_mismatch_ratio{0.30};
};

/// 滤波管线（**纯逻辑**：不依赖 rclcpp、不依赖 Eigen）。
///
/// 生命周期：
///   `configure()` 一次 → 每来一帧 `push()` → 每拍 `sample()` 取当前目标。
class InputFilter
{
public:
  /// 配置。返回 false ⇒ 参数不可用（`control_rate_hz <= 0` 等），调用方必须拒绝启动。
  bool configure(const InputFilterConfig & cfg) noexcept
  {
    cfg_ = cfg;
    if (!(cfg_.control_rate_hz > 0.0) || !std::isfinite(cfg_.control_rate_hz)) {return false;}
    period_ms_ = 1000.0 / cfg_.control_rate_hz;
    if (!std::isfinite(period_ms_) || period_ms_ <= 0.0) {return false;}
    for (auto & f : pose_lpf_) {
      if (!f.configure(cfg_.pose_lpf_hz, cfg_.control_rate_hz)) {return false;}
    }
    for (auto & f : vel_lpf_) {
      if (!f.configure(cfg_.velocity_lpf_hz, cfg_.control_rate_hz)) {return false;}
    }
    reset();
    configured_ = true;
    return true;
  }

  void reset() noexcept
  {
    have_prev_ = false;
    have_last_accepted_ = false;
    have_last_stamp_ = false;
    last_pose_ = Pose6{};
    last_accepted_ = Pose6{};
    last_stamp_ms_ = 0;
    reject_ = FilterReject::None;
    reject_count_ = 0;
    accepted_ = 0;
    velocity_mismatch_count_ = 0;
    has_velocity_ = false;
    prev_position_ = Vec3{0, 0, 0};
    prev_rotation_ = Vec3{0, 0, 0};
    have_prev_filtered_ = false;
  }

  /// 推入一帧上游样本。返回 false ⇒ 该帧被拒（原因见 `lastReject()`）。
  ///
  /// ⚠️ 被拒的帧**不推进任何内部状态**（不更新滤波器、不更新"上一帧"）——
  ///    否则一个坏帧会通过滤波器污染后续好几拍。
  bool push(const RawPoseSample & raw) noexcept
  {
    if (!configured_) {return false;}
    // ---- ① 数值合法性 ----
    if (!vec3_finite(raw.position) ||
        !std::isfinite(raw.orientation[0]) || !std::isfinite(raw.orientation[1]) ||
        !std::isfinite(raw.orientation[2]) || !std::isfinite(raw.orientation[3])) {
      return reject(FilterReject::NonFinite);
    }
    const double qn = quat_norm(raw.orientation);
    if (!(qn > 1e-9) || !std::isfinite(qn) || std::fabs(qn - 1.0) > 1e-3) {
      return reject(FilterReject::BadQuaternion);
    }
    // ---- ② 时间戳一致性（**绝对**判据，不会锁死）----
    //
    // 三件事分开：
    //   ① 未来戳：`stamp - recv_sys > tol` ⇒ 上游时钟和我们对不上（拒绝）。
    //      这是**绝对**比较，跟上一帧无关 ⇒ 上游停流/重启不会让它变坏。
    //   ② 小步倒退（≤ tol）⇒ 真乱序（拒绝）。
    //   ③ 大步倒退（> tol）⇒ 上游时钟被重置/换源 ⇒ **重新对齐**（接受），
    //      否则又是一条锁死的路。
    if (raw.has_stamp) {
      if (raw.recv_sys_ms > 0) {
        const std::int64_t ahead = raw.stamp_ms - raw.recv_sys_ms;
        if (ahead > stamp_future_max_ms_) {stamp_future_max_ms_ = ahead;}
        if (ahead > static_cast<std::int64_t>(cfg_.future_stamp_tolerance_ms)) {
          ++future_stamp_rejects_;
          return reject(FilterReject::FutureStamp);
        }
      }
      if (have_last_stamp_) {
        const std::int64_t back = last_stamp_ms_ - raw.stamp_ms;
        if (back > static_cast<std::int64_t>(cfg_.future_stamp_tolerance_ms)) {
          ++stamp_resync_count_;          // 大步倒退：重新对齐，不拒
        } else if (back > 0) {
          ++out_of_order_count_;
          return reject(FilterReject::OutOfOrder);
        }
      }
    }
    if (have_prev_ && raw.recv_ms < prev_recv_ms_) {
      return reject(FilterReject::Stale);
    }

    // ---- ③ 四元数符号对齐（必须在转旋转向量之前）----
    Quat q = raw.orientation;
    if (have_prev_) {
      if (quat_dot(q, prev_quat_) < 0.0) {
        q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3];
      }
    }
    const Vec3 rv = quat_to_rotvec(q);

    // ---- ③.5 上游频率跟踪 + 低通自适应 ----
    //
    // ⚠️ 低通是**按上游采样率**跑的（push 每来一帧一次）。截止频率必须低于
    //    上游 Nyquist，否则"低通"会变成放大/混叠：上游 10 Hz（Nyquist 5 Hz）
    //    配 pose_lpf_hz=15 就是典型反例。
    //    做法：用到达间隔的 EMA 估上游频率，把截止频率夹到 0.45·f_up；
    //    只有频率显著变化（>10%）时才重新 configure（configure 会清状态）。
    if (have_prev_filtered_ && raw.recv_ms > prev_recv_ms_) {
      const double dt_up = static_cast<double>(raw.recv_ms - prev_recv_ms_);
      up_dt_ema_ms_ = (up_dt_ema_ms_ > 1e-6)
                        ? (0.9 * up_dt_ema_ms_ + 0.1 * dt_up) : dt_up;
      const double f_up = (up_dt_ema_ms_ > 1e-6) ? (1000.0 / up_dt_ema_ms_) : 0.0;
      if (f_up > 1e-6) {
        const double f_cap = 0.45 * f_up;
        const double want_pose = (cfg_.pose_lpf_hz > 0.0) ? cfg_.pose_lpf_hz : 0.0;
        const double want_vel = (cfg_.velocity_lpf_hz > 0.0) ? cfg_.velocity_lpf_hz : 0.0;
        const double eff_pose = std::min(want_pose, f_cap);
        const double eff_vel = std::min(want_vel, f_cap);
        const bool need = (std::fabs(eff_pose - pose_lpf_eff_hz_) > 0.1 * std::max(1.0, eff_pose)) ||
                          (std::fabs(eff_vel - vel_lpf_eff_hz_) > 0.1 * std::max(1.0, eff_vel));
        if (need) {
          if (eff_pose > 0.0) {for (auto & f : pose_lpf_) {f.configure(eff_pose, f_up);}}
          if (eff_vel > 0.0) {for (auto & f : vel_lpf_) {f.configure(eff_vel, f_up);}}
          pose_lpf_eff_hz_ = eff_pose;
          vel_lpf_eff_hz_ = eff_vel;
          if (want_pose > eff_pose || want_vel > eff_vel) {++lpf_clamp_count_;}
        }
      }
    }

    // ---- ④ 逐轴低通 ----
    Vec3 pos{0, 0, 0}, rot{0, 0, 0};
    for (std::size_t i = 0; i < 3; ++i) {
      pos[i] = pose_lpf_[i].step(raw.position[i]);
      rot[i] = pose_lpf_[3 + i].step(rv[i]);
    }

    // ---- ⑤ 速度：滤波后位置的中心差分 + 轻低通（**不用上游速度**）----
    Vec3 lv = vec3_zero(), av = vec3_zero();
    if (have_prev_filtered_ && raw.recv_ms > prev_recv_ms_) {
      const double dt = static_cast<double>(raw.recv_ms - prev_recv_ms_) / 1000.0;
      if (dt > 1e-6 && std::isfinite(dt)) {
        for (std::size_t i = 0; i < 3; ++i) {
          lv[i] = (pos[i] - prev_position_[i]) / dt;
          av[i] = (rot[i] - prev_rotation_[i]) / dt;
        }
      }
    }
    for (std::size_t i = 0; i < 3; ++i) {
      lv[i] = vel_lpf_[i].step(lv[i]);
      av[i] = vel_lpf_[3 + i].step(av[i]);
    }

    // ---- ⑥ 上游速度一致性校验（只告警，不用它）----
    if (raw.has_velocity && have_prev_filtered_ &&
        vec3_finite(raw.linear_velocity) && vec3_finite(raw.angular_velocity)) {
      const double mag = std::sqrt(lv[0] * lv[0] + lv[1] * lv[1] + lv[2] * lv[2]);
      const double ref = std::sqrt(raw.linear_velocity[0] * raw.linear_velocity[0] +
                                   raw.linear_velocity[1] * raw.linear_velocity[1] +
                                   raw.linear_velocity[2] * raw.linear_velocity[2]);
      const double scale = std::max(mag, ref);
      if (scale > 1e-6) {
        const double diff = std::sqrt(
            (lv[0] - raw.linear_velocity[0]) * (lv[0] - raw.linear_velocity[0]) +
            (lv[1] - raw.linear_velocity[1]) * (lv[1] - raw.linear_velocity[1]) +
            (lv[2] - raw.linear_velocity[2]) * (lv[2] - raw.linear_velocity[2]));
        if (diff / scale > cfg_.velocity_mismatch_ratio) {++velocity_mismatch_count_;}
      }
    }

    // ---- ⑦ 落地 ----
    Pose6 p;
    p.position = pos;
    p.rotation = rot;
    p.linear_velocity = lv;
    p.angular_velocity = av;
    last_pose_ = p;
    last_accepted_ = p;
    have_prev_ = true;
    have_prev_filtered_ = true;
    have_last_accepted_ = true;
    prev_quat_ = q;
    prev_recv_ms_ = raw.recv_ms;
    prev_position_ = pos;
    prev_rotation_ = rot;
    if (raw.has_stamp) {last_stamp_ms_ = raw.stamp_ms; have_last_stamp_ = true;}
    has_velocity_ = true;
    reject_ = FilterReject::None;
    ++accepted_;
    return true;
  }

  /// 取当前目标（上一次被接受的姿态 + 重建的速度）。
  ///
  /// ⚠️ 没有接受过任何帧时返回 false —— **调用方不得把它当成"目标就是原点"**。
  bool sample(Pose6 & out) const noexcept
  {
    if (!have_last_accepted_) {return false;}
    out = last_accepted_;
    return true;
  }

  /// 取"**外推到 now_ms**"的目标（位置用滤波后的速度线性外推）。
  ///
  /// 为什么要外推：上游只有 10/20 Hz 时，`sample()` 返回的是**最近一次到达**
  /// 的目标（最老 100 ms）；直接把它当"当前目标"会让末端固定滞后 ~v·Δt
  /// （10 cm/s × 100 ms = 10 mm）。滤波管线本来就在算 `linear_velocity`
  /// （对滤波后位置做中心差分），用它外推即可把滞后补回来。
  ///
  /// `lead_ms` 额外再往前推一点：调用方用"一个 IK 周期"作为提前量，
  /// 因为新解要**经过一个插值周期才会真正被下发**。
  ///
  /// ⚠️ 外推**有上限**（`predict_max_ms`）：上游断了之后不能把目标射到天边。
  bool sampleAt(std::int64_t now_ms, Pose6 & out, double lead_ms = 0.0) const noexcept
  {
    if (!have_last_accepted_) {return false;}
    out = last_accepted_;
    double dt_ms = static_cast<double>(now_ms - prev_recv_ms_) + lead_ms;
    if (dt_ms <= 0.0) {return true;}
    if (cfg_.predict_max_ms > 0.0) {dt_ms = std::min(dt_ms, cfg_.predict_max_ms);}
    const double dt = dt_ms / 1000.0;
    for (std::size_t i = 0; i < 3; ++i) {
      out.position[i] += last_accepted_.linear_velocity[i] * dt;
      out.rotation[i] += last_accepted_.angular_velocity[i] * dt;
    }
    return true;
  }

  /// 距离上一次被接受帧过去了多少毫秒（调用方用它判"目标是否陈旧"）。
  std::int64_t ageMs(std::int64_t now_ms) const noexcept
  {
    if (!have_last_accepted_) {return -1;}
    return now_ms - prev_recv_ms_;
  }

  FilterReject lastReject() const noexcept {return reject_;}
  /// 上游实测频率（Hz，来自到达间隔的 EMA）。0 = 样本不足。
  double upstreamHz() const noexcept
  {
    return (up_dt_ema_ms_ > 1e-6) ? (1000.0 / up_dt_ema_ms_) : 0.0;
  }
  /// 上游到达间隔 EMA（ms）。
  double upstreamDtMs() const noexcept {return up_dt_ema_ms_;}
  /// 实际生效的位姿/速度低通截止频率（Hz，已按上游频率夹取；0 = 旁路）。
  double poseLpfEffectiveHz() const noexcept {return pose_lpf_eff_hz_;}
  double velocityLpfEffectiveHz() const noexcept {return vel_lpf_eff_hz_;}
  /// 因上游频率过低而把截止频率下调的次数。
  std::uint64_t lpfClampCount() const noexcept {return lpf_clamp_count_;}
  /// "未来戳"被拒的次数（时钟不一致诊断用）。
  std::uint64_t futureStampRejects() const noexcept {return future_stamp_rejects_;}
  /// 观测到的最大"戳 − 到达(系统钟)"（ms）。正数 = 上游戳超前。
  std::int64_t stampFutureMaxMs() const noexcept {return stamp_future_max_ms_;}
  /// 上游时钟大步倒退后重新对齐的次数（应很少；>0 说明上游重启过）。
  std::uint64_t stampResyncCount() const noexcept {return stamp_resync_count_;}
  /// 小步倒退（真乱序）被拒的次数。
  std::uint64_t outOfOrderCount() const noexcept {return out_of_order_count_;}
  std::uint64_t rejectCount() const noexcept {return reject_count_;}
  std::uint64_t acceptedCount() const noexcept {return accepted_;}
  std::uint64_t velocityMismatchCount() const noexcept {return velocity_mismatch_count_;}
  bool hasTarget() const noexcept {return have_last_accepted_;}
  double periodMs() const noexcept {return period_ms_;}
  const InputFilterConfig & config() const noexcept {return cfg_;}

private:
  bool reject(FilterReject r) noexcept
  {
    reject_ = r;
    ++reject_count_;
    return false;
  }

  InputFilterConfig cfg_{};
  std::array<Butterworth, 6> pose_lpf_{};
  std::array<Butterworth, 6> vel_lpf_{};
  double period_ms_{10.0};

  bool have_prev_{false};
  bool have_prev_filtered_{false};
  bool have_last_accepted_{false};
  bool have_last_stamp_{false};
  bool configured_{false};
  bool has_velocity_{false};

  Quat prev_quat_{0, 0, 0, 1};
  std::int64_t prev_recv_ms_{0};
  std::int64_t last_stamp_ms_{0};
  Vec3 prev_position_{0, 0, 0};
  Vec3 prev_rotation_{0, 0, 0};
  Pose6 last_pose_{};
  Pose6 last_accepted_{};

  FilterReject reject_{FilterReject::None};
  /// 上游到达间隔的 EMA（ms）；0 = 还没有两次到达。用于"上游频率"诊断与 LPF 夹取。
  double up_dt_ema_ms_{0.0};
  /// 实际生效的截止频率（夹取之后）；0 = 旁路。
  double pose_lpf_eff_hz_{0.0};
  double vel_lpf_eff_hz_{0.0};
  std::uint64_t lpf_clamp_count_{0};
  std::uint64_t future_stamp_rejects_{0};
  std::int64_t stamp_future_max_ms_{0};
  std::uint64_t stamp_resync_count_{0};
  std::uint64_t out_of_order_count_{0};
  std::uint64_t reject_count_{0};
  std::uint64_t accepted_{0};
  std::uint64_t velocity_mismatch_count_{0};
};

}  // namespace lite_move_controller
