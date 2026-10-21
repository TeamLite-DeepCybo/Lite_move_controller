// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// butterworth.hpp —— 二阶 Butterworth 低通（逐通道独立状态）
//
// ===========================================================================
// 它在本层解决什么问题
// ===========================================================================
//
// 运控层有两个**噪声源**，都需要"尽可能准确"地处理（用户第 5 条要求）：
//   · 上游的终端位姿：来自遥操/视觉/在线规划，抖动大、时间戳不规则；
//   · /joint_states 的关节量：来自 CAN 反馈，有量化噪声，velocity 常是差分估计
//     （比 position 更脏）。
//
// 本文件只提供"最底层的一个二阶低通"，把它做成**可单测的纯数学**，
// 而不是散落在节点里的几行系数计算 —— 因为滤波器系数写错的表现是
// "系统能跑但相位不对/发散"，极难在现场定位。
//
// ===========================================================================
// 为什么是二阶 Butterworth（而不是一阶 / 滑动平均 / 卡尔曼）
// ===========================================================================
//
// | 方案 | 通带平坦 | 相位延迟 | 参数个数 | 结论 |
// |---|---|---|---|---|
// | 一阶 RC | 否（通带就开始衰减） | 小 | 1 | 抑制不够 |
// | 滑动平均 | 否（旁瓣） | 大（N/2 拍） | 1 | 会把 100 Hz 的抖动延迟 5 ms |
// | **二阶 Butterworth** | **是（最大平坦）** | 中 | 2 | ★ 本层采用 |
// | 卡尔曼 | 是 | 最小 | 需噪声模型 | 本层**没有**可信的噪声模型，硬凑参数反而更差 |
//
// 结论：二阶 Butterworth 是"没有噪声统计信息时"的标准答案。
//
// ===========================================================================
// 离散化：双线性变换（Tustin） + 频率预畸变
// ===========================================================================
//
//      ωc = 2π·fc
//      K  = tan(ωc·T/2)                     ← 预畸变：保证数字截止频率 == fc
//      n  = 1 / (1 + K/Q + K²)              ← Q = 1/√2（Butterworth）
//      b0 = K²·n,  b1 = 2b0,  b2 = b0
//      a1 = 2(K² − 1)·n,  a2 = (1 − K/Q + K²)·n
//
//      直接 II 型（transposed direct form II）：
//      y  = b0·x + s1
//      s1 = b1·x − a1·y + s2
//      s2 = b2·x − a2·y
//
// ⚠️ 直接用 transposed DF-II（而不是"先算 b/a 再卷"）是为了**数值稳定**：
//    中间状态 s1/s2 的量级与输出同阶，不会在 Q 较大时放大。
//
// ===========================================================================
// 边界行为（都有单测）
// ===========================================================================
//
//   · fc <= 0 或 fc >= fs/2（奈奎斯特）→ **旁路**（直通）：不静默给一个错的截止频率；
//   · fs <= 0 → 构造失败（由调用方检查 `ready()`）；
//   · reset() 把状态清零并把输出初值设为给定值（避免"从 0 开始爬"的瞬态）。
#pragma once

#include <cmath>

namespace lite_move_controller
{

/// 二阶 Butterworth 低通滤波器（单通道）。
///
/// 用法：每个被滤的标量一个实例（位置 3 轴 + 旋转向量 3 轴 = 6 个）。
/// ⚠️ **不要**多个通道共用一个实例 —— 状态会互相污染，且症状是
///    "某个方向的动作串到另一个方向"，极难发现。
class Butterworth {
public:
  Butterworth() = default;

  /// 配置。`fc_hz <= 0` 或 `fc_hz >= 0.5*fs_hz` 时进入**旁路**模式（`ready()==true` 但直通）。
  /// 返回 false 表示 `fs_hz <= 0`（调用方必须检查，不要继续用）。
  bool configure(double fc_hz, double fs_hz)
  {
    fs_ = fs_hz;
    fc_ = fc_hz;
    s1_ = s2_ = 0.0;
    y_ = 0.0;
    if (!(fs_hz > 0.0) || !std::isfinite(fs_hz)) {
      ready_ = false;
      bypass_ = true;
      return false;
    }
    ready_ = true;
    if (!(fc_hz > 0.0) || !std::isfinite(fc_hz) || fc_hz >= 0.5 * fs_hz) {
      bypass_ = true;          // 旁路：直通，不假装滤过
      return true;
    }
    bypass_ = false;
    const double Q = 1.0 / std::sqrt(2.0);
    const double K = std::tan(M_PI * fc_hz / fs_hz);
    const double K2 = K * K;
    const double n = 1.0 / (1.0 + K / Q + K2);
    b0_ = K2 * n;
    b1_ = 2.0 * b0_;
    b2_ = b0_;
    a1_ = 2.0 * (K2 - 1.0) * n;
    a2_ = (1.0 - K / Q + K2) * n;
    return true;
  }

  bool ready() const noexcept {return ready_;}
  bool bypassed() const noexcept {return bypass_;}

  /// 复位：把内部状态置成"输入长期等于 `value`"的**稳态**。
  ///
  /// ⚠️ 为什么不能只把 s1/s2 清 0：那等于假设"输入在此之前一直是 0"，
  ///    于是复位后会有一段从 0 爬到 `value` 的瞬态（对 1 rad 的关节角就是
  ///    **瞬间一个 1 rad 的假运动**）。正确做法是解稳态方程：
  ///
  ///        y  = b0·x + s1
  ///        s1 = b1·x − a1·y + s2
  ///        s2 = b2·x − a2·y
  ///
  ///    代入 x = y = value 得 s1 = (1 − b0)·value，s2 = (a1 + a2)·value。
  void reset(double value = 0.0) noexcept
  {
    if (!std::isfinite(value)) {value = 0.0;}
    y_ = value;
    if (!ready_ || bypass_) {
      s1_ = s2_ = 0.0;
      return;
    }
    s1_ = (1.0 - b0_) * value;
    s2_ = (a1_ + a2_) * value;
  }

  double value() const noexcept {return y_;}

  /// 推进一步。
  double step(double x) noexcept
  {
    if (!ready_ || bypass_) {
      y_ = x;
      return y_;
    }
    if (!std::isfinite(x)) {
      // ⚠️ 非有限输入**不推进状态**并返回上一输出：
      //    让它传播会把整条控制链污染成 NaN，而故障源在上游。
      return y_;
    }
    const double y = b0_ * x + s1_;
    s1_ = b1_ * x - a1_ * y + s2_;
    s2_ = b2_ * x - a2_ * y;
    y_ = y;
    return y_;
  }

  // --- 只读系数（单测与诊断用）---
  double b0() const noexcept {return b0_;}
  double a1() const noexcept {return a1_;}
  double a2() const noexcept {return a2_;}

private:
  double fs_{0.0}, fc_{0.0};
  double b0_{1.0}, b1_{0.0}, b2_{0.0}, a1_{0.0}, a2_{0.0};
  double s1_{0.0}, s2_{0.0};
  double y_{0.0};
  bool ready_{false};
  bool bypass_{true};
};

}  // namespace lite_move_controller
