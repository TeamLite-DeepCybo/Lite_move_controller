// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// test_core.cpp —— lite_move_controller **纯 C++ 核心**的单测
//
// 为什么不用 gtest：本项目已有先例（lite_bridge 的 source_arbiter 测试
// 用 `g++ -I include test.cpp -o t && ./t` 裸跑）。核心逻辑零 ROS 依赖，
// 裸测能做到"改一行、跑 0.1 秒"，而不是走 colcon test 的完整链路。
//
// 覆盖：
//   ① Butterworth      —— 直流增益、截止频率衰减、复位无假运动、旁路/NaN
//   ② resampler        —— 端点严格相等、单调性、密插分段数、上限、非法输入
//   ③ admittance       —— 饱和限幅 + 条件积分、松手回复、dt/NaN 防护
//   ④ impedance        —— τ=JᵀF、负刚度拒绝、逐关节限幅
//   ⑤ limit_guard      —— 三道闸、NaN 整帧拒绝、饱和关节检测
//   ⑥ node_state       —— 迁移守卫、不发帧状态、SWITCHING 两步、FAULT 只能回 STANDBY

#include "lite_move_controller/admittance.hpp"
#include "lite_move_controller/butterworth.hpp"
#include "lite_move_controller/impedance.hpp"
#include "lite_move_controller/limit_guard.hpp"
#include "lite_move_controller/node_state.hpp"
#include "lite_move_controller/resampler.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace lite_move_controller;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                                     \
  do {                                                                       \
    ++g_checks;                                                              \
    if (!(cond)) {                                                           \
      ++g_failures;                                                          \
      std::printf("  FAIL %s:%d  ", __FILE__, __LINE__);                     \
      std::printf(__VA_ARGS__);                                              \
      std::printf("\n");                                                     \
    }                                                                        \
  } while (0)

static bool near(double a, double b, double tol = 1e-9)
{
  return std::fabs(a - b) <= tol;
}

// ---------------------------------------------------------------------------
// ① Butterworth
// ---------------------------------------------------------------------------

static void test_butterworth()
{
  std::printf("[butterworth]\n");
  {
    Butterworth f;
    CHECK(f.configure(-1.0, 100.0), "fc<=0 应当配置成功（进入旁路）");
    CHECK(f.bypassed(), "fc<=0 必须旁路");
    CHECK(near(f.step(3.5), 3.5), "旁路必须直通，得到 %.6f", f.step(3.5));
  }
  {
    Butterworth f;
    CHECK(f.configure(15.0, 100.0), "正常配置应成功");
    CHECK(!f.bypassed(), "15Hz/100Hz 不应旁路");
    // 直流增益必须 = 1（Weierstrass/Butterworth 的定义性质）
    f.reset(0.0);
    for (int i = 0; i < 4000; ++i) {f.step(1.0);}
    CHECK(near(f.value(), 1.0, 1e-6), "直流增益应=1，得到 %.9f", f.value());
  }
  {
    // 截止频率处幅度应约 1/√2（-3 dB），相位滞后 90°。
    // 用稳态正弦幅值比验证（足够长的瞬态衰减后再取峰值）。
    const double fs = 1000.0, fc = 20.0;
    Butterworth f;
    CHECK(f.configure(fc, fs), "configure");
    double peak = 0.0;
    for (int i = 0; i < 20000; ++i) {
      const double t = i / fs;
      const double y = f.step(std::sin(2.0 * M_PI * fc * t));
      if (i > 8000) {peak = std::max(peak, std::fabs(y));}
    }
    CHECK(near(peak, 1.0 / std::sqrt(2.0), 0.02),
          "截止频率幅值应≈0.7071，得到 %.4f", peak);
  }
  {
    // reset(value) 必须让"输入恒为 value"时**立刻**输出 value（无假运动）
    Butterworth f;
    CHECK(f.configure(5.0, 100.0), "configure");
    f.reset(1.234);
    CHECK(near(f.step(1.234), 1.234, 1e-9),
          "复位后第一拍应严格=初值（否则是一个 1.2 rad 的假运动），得到 %.9f",
          f.step(1.234));
  }
  {
    // NaN 不得污染状态
    Butterworth f;
    CHECK(f.configure(10.0, 100.0), "configure");
    f.reset(0.0);
    for (int i = 0; i < 500; ++i) {f.step(2.0);}
    const double before = f.value();
    const double y = f.step(std::nan(""));
    CHECK(near(y, before, 1e-12), "NaN 输入应返回上一输出，得到 %.6f", y);
    CHECK(std::isfinite(f.value()), "NaN 不得污染状态");
  }
  {
    Butterworth f;
    CHECK(!f.configure(10.0, 0.0), "fs<=0 必须失败（调用方应拒绝启动）");
    CHECK(!f.ready(), "失败后不得 ready");
  }
}

// ---------------------------------------------------------------------------
// ② resampler
// ---------------------------------------------------------------------------

static void test_resampler()
{
  std::printf("[resampler]\n");
  {
    // Hermite 端点必须严格命中（这是"密插不改变轨迹"的数学保证）
    HermiteSpan sp{{0.1, -0.2}, {0.5, 0.0}, {0.7, 0.4}, {-0.3, 0.1}, 0.25};
    auto a = hermite_at(sp, 0.0);
    auto b = hermite_at(sp, 1.0);
    CHECK(near(a.position[0], 0.1), "s=0 位置必须=p0，得到 %.9f", a.position[0]);
    CHECK(near(a.velocity[0], 0.5), "s=0 速度必须=v0，得到 %.9f", a.velocity[0]);
    CHECK(near(b.position[0], 0.7), "s=1 位置必须=p1，得到 %.9f", b.position[0]);
    CHECK(near(b.velocity[0], -0.3, 1e-9), "s=1 速度必须=v1（沿 T 归一），得到 %.9f",
          b.velocity[0]);
  }
  {
    // 不触发密插：两端足够近
    SamplePoint a{{0.0, 0.0}, {0.0, 0.0}};
    SamplePoint b{{0.001, -0.002}, {0.0, 0.0}};
    auto d = densify(a, b, 0.01, 0.05);
    CHECK(!d.densified, "0.002 rad < 0.05 不应密插");
    CHECK(d.points.empty(), "不应有点");
    CHECK(d.segments == 1, "segments 应为 1，得到 %zu", d.segments);
  }
  {
    // 触发密插：0.5 rad 步长，阈值 0.05 ⇒ 至少 10 段
    SamplePoint a{{0.0, 0.0}, {0.0, 0.0}};
    SamplePoint b{{0.5, 0.0}, {0.0, 0.0}};
    auto d = densify(a, b, 0.1, 0.05);
    CHECK(d.densified, "0.5 rad > 0.05 必须密插");
    CHECK(d.segments >= 10, "至少 10 段，得到 %zu", d.segments);
    CHECK(d.points.size() == d.segments - 1, "点数应=段数-1");
    // 端点严格不变
    CHECK(near(d.points.front().position[0],
               hermite(0, 0, 0.5, 0, 0.1, 1.0 / d.segments), 1e-12),
          "首个中间点必须落在 Hermite 上");
    // 单调递增（匀加速情形下位置单调）
    for (std::size_t i = 1; i < d.points.size(); ++i) {
      CHECK(d.points[i].position[0] >= d.points[i - 1].position[0] - 1e-12,
            "位置应单调不减");
    }
    // 相邻步长都不超过阈值
    std::vector<double> pos;
    pos.push_back(0.0);
    for (const auto & p : d.points) {pos.push_back(p.position[0]);}
    pos.push_back(0.5);
    for (std::size_t i = 1; i < pos.size(); ++i) {
      CHECK(std::fabs(pos[i] - pos[i - 1]) <= 0.05 + 1e-9,
            "每步应 <= max_step，得到 %.6f", std::fabs(pos[i] - pos[i - 1]));
    }
  }
  {
    // 上限生效：不炸内存
    SamplePoint a{{0.0}, {0.0}};
    SamplePoint b{{1e6}, {0.0}};
    auto d = densify(a, b, 0.1, 0.001, 32);
    CHECK(d.segments == 32, "上限应夹到 32，得到 %zu", d.segments);
    CHECK(d.points.size() == 31, "点数应=31，得到 %zu", d.points.size());
  }
  {
    // densifyChecked：保证**每一段** ≤ max_step（含端点速度很大的情形）
    SamplePoint a{{0.0, 0.0}, {0.0, 0.0}};
    SamplePoint b{{1.0, -0.5}, {4.0, 3.0}};      // 大端点速度
    const double step = 0.02;
    auto d = densifyChecked(a, b, 0.05, step, 400);
    CHECK(d.densified, "应触发密插");
    CHECK(d.points.size() == d.segments - 1, "点数应=段数-1");
    // 逐段实测
    std::vector<SamplePoint> seq;
    seq.push_back(a);
    for (const auto & q : d.points) {seq.push_back(q);}
    seq.push_back(b);
    for (std::size_t i = 1; i < seq.size(); ++i) {
      for (std::size_t j = 0; j < 2; ++j) {
        const double st = std::fabs(seq[i].position[j] - seq[i - 1].position[j]);
        CHECK(st <= step * 1.01, "每段应 <= max_step(%.4f)，得到 %.6f", step, st);
      }
    }
    // 端点严格不变
    CHECK(near(d.points.front().position[0],
               hermite(0, 0, 1.0, 4.0, 0.05, 1.0 / d.segments), 1e-12),
          "首点应在 Hermite 上");
  }
  {
    // 非法输入：不得产点、不得崩
    SamplePoint a{{0.0}, {0.0}};
    SamplePoint b{{1.0}, {0.0}};
    CHECK(densify(a, b, 0.0, 0.05).points.empty(), "duration<=0 不应产点");
    CHECK(densify(a, b, 0.1, 0.0).points.empty(), "max_step<=0 不应产点");
    CHECK(densify(a, b, 0.1, -1.0).points.empty(), "max_step<0 不应产点");
    SamplePoint bad{{std::nan("")}, {0.0}};
    CHECK(densify(a, bad, 0.1, 0.001).points.empty(), "NaN 端点不应产点");
  }
}

// ---------------------------------------------------------------------------
// ③ admittance
// ---------------------------------------------------------------------------

static void test_admittance()
{
  std::printf("[admittance]\n");
  {
    AdmittanceConfig bad;
    bad.mass[0] = 0.0;                 // 质量 0 = 无穷快，必须拒绝
    AdmittanceLaw law;
    CHECK(!law.configure(bad), "mass=0 必须拒绝配置");
  }
  {
    AdmittanceConfig bad;
    bad.stiffness[2] = -1.0;           // 负刚度 = 正反馈
    AdmittanceLaw law;
    CHECK(!law.configure(bad), "负刚度必须拒绝配置");
  }
  {
    AdmittanceConfig cfg;
    cfg.mass = {1, 1, 1, 0.05, 0.05, 0.05};
    cfg.stiffness = {100, 100, 100, 2, 2, 2};
    cfg.max_offset = {0.10, 0.10, 0.10, 0.30, 0.30, 0.30};
    cfg.wrench_lpf_hz = 0.0;           // 测试中不滤
    AdmittanceLaw law;
    CHECK(law.configure(cfg), "正常配置应成功");
    // 静态 100 N、K_a=100 ⇒ 无饱和时应 1.0 m；被 0.10 m 夹住
    Vec6 f{100, 0, 0, 0, 0, 0};
    for (int i = 0; i < 5000; ++i) {law.step(f, 0.01);}
    CHECK(near(law.offset()[0], 0.10, 1e-9), "应饱和在 0.10，得到 %.6f", law.offset()[0]);
    CHECK(law.status().saturated, "应报告 saturated");
    CHECK(near(law.offset_rate()[0], 0.0, 1e-12), "饱和后速度应清零（条件积分）");
    // 松手 ⇒ 弹性回复到 0
    for (int i = 0; i < 5000; ++i) {law.step(vec6_zero(), 0.01);}
    CHECK(std::fabs(law.offset()[0]) < 1e-4, "松手应回 0，得到 %.6f", law.offset()[0]);
  }
  {
    // 小力时**不应**饱和（验证限幅不是"总是夹住"）
    AdmittanceConfig cfg;
    cfg.mass = {1, 1, 1, 0.05, 0.05, 0.05};
    cfg.stiffness = {100, 100, 100, 2, 2, 2};
    AdmittanceLaw law;
    CHECK(law.configure(cfg), "configure");
    Vec6 f{5, 0, 0, 0, 0, 0};          // 5 N / 100 N/m = 0.05 m < 0.10
    for (int i = 0; i < 5000; ++i) {law.step(f, 0.01);}
    CHECK(near(law.offset()[0], 0.05, 5e-3), "5N 应稳态 0.05 m，得到 %.6f",
          law.offset()[0]);
    CHECK(!law.status().saturated, "不应饱和");
  }
  {
    // dt / NaN 防护
    AdmittanceConfig cfg;
    cfg.wrench_lpf_hz = 0.0;
    AdmittanceLaw law;
    CHECK(law.configure(cfg), "configure");
    Vec6 f{10, 0, 0, 0, 0, 0};
    for (int i = 0; i < 100; ++i) {law.step(f, 0.01);}
    const Vec6 before = law.offset();
    law.step(f, 0.0);
    CHECK(law.offset() == before, "dt=0 不应推进");
    law.step(f, -0.01);
    CHECK(law.offset() == before, "dt<0 不应推进");
    law.step(f, std::nan(""));
    CHECK(law.offset() == before, "dt=NaN 不应推进");
    Vec6 bad{std::nan(""), 0, 0, 0, 0, 0};
    law.step(bad, 0.01);
    CHECK(law.offset() == before, "NaN 外力不应推进状态");
    CHECK(!law.status().advanced, "未推进时应报告 advanced=false");
  }
  {
    // K_a = 0 ⇒ 纯导纳（自由拖动）：恒定力下应一直加速（位置持续增长，直到饱和）
    AdmittanceConfig cfg;
    cfg.stiffness = vec6_zero();
    cfg.max_offset = {1e6, 1e6, 1e6, 1e6, 1e6, 1e6};   // 放大上限，观察自由运动
    cfg.wrench_lpf_hz = 0.0;
    AdmittanceLaw law;
    CHECK(law.configure(cfg), "configure");
    Vec6 f{1, 0, 0, 0, 0, 0};
    for (int i = 0; i < 100; ++i) {law.step(f, 0.01);}
    const double x1 = law.offset()[0];
    for (int i = 0; i < 100; ++i) {law.step(f, 0.01);}
    CHECK(law.offset()[0] > x1 + 0.05, "K_a=0 时恒力应持续加速（x1=%.3f x2=%.3f）",
          x1, law.offset()[0]);
  }
}

// ---------------------------------------------------------------------------
// ④ impedance
// ---------------------------------------------------------------------------

static void test_impedance()
{
  std::printf("[impedance]\n");
  {
    ImpedanceConfig cfg;
    cfg.stiffness = {100, 100, 100, 10, 10, 10};
    cfg.damping = {0, 0, 0, 0, 0, 0};        // 自动推
    ImpedanceLaw law;
    CHECK(law.configure(cfg), "configure");
    CHECK(law.damping()[0] > 0.0, "阻尼应被自动推出：%.6f", law.damping()[0]);
  }
  {
    ImpedanceConfig cfg;
    cfg.stiffness[1] = -1.0;
    ImpedanceLaw law;
    CHECK(!law.configure(cfg), "负刚度必须拒绝");
  }
  {
    // τ = Jᵀ·F：单关节单方向
    ImpedanceConfig cfg;
    cfg.stiffness = {100, 100, 100, 10, 10, 10};
    cfg.damping = {1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9};
    ImpedanceLaw law;
    CHECK(law.configure(cfg), "configure");
    Jacobian J;
    J.resize(6, 7);
    J.at(0, 0) = 2.0;                        // x 方向对关节 0 的力臂
    Vec6 dx{0.01, 0, 0, 0, 0, 0};
    auto r = law.compute(J, dx, vec6_zero(), 7);
    CHECK(r.ok, "compute 应成功");
    // F = 100*0.01 = 1 N；τ0 = J(0,0)*F = 2*1 = 2 N·m
    CHECK(near(r.wrench[0], 1.0, 1e-9), "wrench=%.6f", r.wrench[0]);
    CHECK(near(r.tau[0], 2.0, 1e-9), "tau0=%.6f", r.tau[0]);
    CHECK(near(r.tau[1], 0.0), "tau1 应为 0");
  }
  {
    // 逐关节限幅
    ImpedanceConfig cfg;
    cfg.stiffness = {10000, 10000, 10000, 1000, 1000, 1000};
    cfg.damping = {1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9};
    cfg.torque_limit = {7.0, 7.0, 7.0, 4.0, 1.5, 1.8, 1.8};
    ImpedanceLaw law;
    CHECK(law.configure(cfg), "configure");
    Jacobian J;
    J.resize(6, 7);
    for (std::size_t j = 0; j < 7; ++j) {J.at(0, j) = 1.0;}
    Vec6 dx{0.5, 0, 0, 0, 0, 0};             // 会产生 5000 N → 远超限幅
    auto r = law.compute(J, dx, vec6_zero(), 7);
    CHECK(r.ok, "compute 应成功");
    CHECK(r.clamped_by_local_limit, "应触发本地限幅");
    CHECK(near(r.tau[0], 7.0, 1e-9), "关节0 应夹到 7.0，得到 %.6f", r.tau[0]);
    CHECK(near(r.tau[4], 1.5, 1e-9), "关节4 应夹到 1.5，得到 %.6f", r.tau[4]);
    CHECK(r.clamped_joints == 7, "7 个关节都被夹，得到 %zu", r.clamped_joints);
  }
  {
    // 非法输入：J 尺寸不符 / NaN 误差
    ImpedanceConfig cfg;
    ImpedanceLaw law;
    CHECK(law.configure(cfg), "configure");
    Jacobian J;
    J.resize(6, 7);
    Vec6 good{0, 0, 0, 0, 0, 0};
    CHECK(!law.compute(J, good, vec6_zero(), 6).ok, "关节数不符应失败");
    Vec6 bad{std::nan(""), 0, 0, 0, 0, 0};
    CHECK(!law.compute(J, bad, vec6_zero(), 7).ok, "NaN 误差应失败");
    Jacobian empty;
    CHECK(!law.compute(empty, good, vec6_zero(), 7).ok, "非法 J 应失败");
  }
}

// ---------------------------------------------------------------------------
// ⑤ limit_guard
// ---------------------------------------------------------------------------

static JointLimits make_limits(std::size_t n = 7)
{
  JointLimits L;
  L.position_min.assign(n, -3.0);
  L.position_max.assign(n, 3.0);
  L.velocity_max.assign(n, 1.0);
  L.torque_max.assign(n, 7.0);
  L.stiffness_max.assign(n, 50.0);
  L.damping_max.assign(n, 5.0);
  return L;
}

static void test_limit_guard()
{
  std::printf("[limit_guard]\n");
  {
    JointLimits bad = make_limits();
    bad.torque_max[3] = 0.0;            // 上限为 0 = 配错
    LimitGuard g;
    CHECK(!g.configure(bad), "上限<=0 必须拒绝配置");
  }
  {
    JointLimits bad = make_limits();
    bad.velocity_max.pop_back();        // 长度不一致
    LimitGuard g;
    CHECK(!g.configure(bad), "长度不一致必须拒绝配置");
  }
  {
    LimitGuard g;
    CHECK(g.configure(make_limits()), "configure");
    JointCommandFrame fr;
    fr.resize(7);
    fr.position[0] = 99.0;  fr.position[1] = -99.0;
    fr.velocity[0] = 9.0;   fr.velocity[1] = -9.0;
    fr.effort[0] = 99.0;    fr.effort[1] = -99.0;
    fr.stiffness[0] = -1.0; fr.stiffness[1] = 500.0;
    fr.damping[0] = 500.0;
    CHECK(g.apply(fr), "apply 应成功");
    CHECK(near(fr.position[0], 3.0), "pos0 应夹到 3，得到 %.3f", fr.position[0]);
    CHECK(near(fr.position[1], -3.0), "pos1 应夹到 -3");
    CHECK(near(fr.velocity[0], 1.0), "vel0 应夹到 1");
    CHECK(near(fr.effort[0], 7.0), "eff0 应夹到 7");
    CHECK(near(fr.effort[1], -7.0), "eff1 应夹到 -7");
    CHECK(near(fr.stiffness[0], 0.0), "负 Kp 应夹到 0");
    CHECK(near(fr.stiffness[1], 50.0), "Kp 应夹到 50");
    CHECK(near(fr.damping[0], 5.0), "Kd 应夹到 5");
    CHECK(g.stats().clamped_position == 2, "位置夹取计数=%llu",
          static_cast<unsigned long long>(g.stats().clamped_position));
    CHECK(g.stats().clamped_gain == 3, "增益夹取计数=%llu",
          static_cast<unsigned long long>(g.stats().clamped_gain));
  }
  {
    // NaN 必须**整帧拒绝**（不能"修好再发"）
    LimitGuard g;
    CHECK(g.configure(make_limits()), "configure");
    JointCommandFrame fr;
    fr.resize(7);
    fr.effort[2] = std::nan("");
    CHECK(!g.apply(fr), "NaN 必须整帧拒绝");
    CHECK(g.stats().nonfinite_rejected == 1, "应计数一次");
    // 拒绝时**不得**修改任何值（避免"半夹帧"流出去）
    CHECK(std::isnan(fr.effort[2]), "被拒的帧不应被就地修改");
  }
  {
    // 饱和关节检测
    LimitGuard g;
    CHECK(g.configure(make_limits()), "configure");
    std::vector<double> q(7, 0.0);
    q[2] = 2.99;                        // 离上限 3.0 只剩 0.01
    q[5] = -2.95;                       // 离下限 3.0 只剩 0.05
    auto sat = g.saturated_joints(q, 0.1);
    CHECK(sat.size() == 2, "应检出 2 个饱和关节，得到 %zu", sat.size());
    CHECK(sat[0] == 2 && sat[1] == 5, "关节下标应为 2 与 5");
    CHECK(!g.position_in_bounds(q, 0.1), "带余量时应判越界");
    CHECK(g.position_in_bounds(q, 0.001), "不带余量时应在界内");
  }
}

// ---------------------------------------------------------------------------
// ⑥ node_state
// ---------------------------------------------------------------------------

//: 测试里模拟"桥的回读"：桥状态是**粘性**的（外壳每拍从服务回读后填入）。
//: ⚠️ 这正是修掉那个 bug 后的语义：桥状态**只有一个来源**（入参），
//:    不再有"note*() 写成员 + 读入参"两条路可 以分叉。
static bool g_bridge_authorized = false;
static bool g_bridge_mode_ok = false;

static StateInputs base_inputs(std::int64_t t)
{
  StateInputs in;
  in.now_ms = t;
  in.load_ok = true;
  in.state_fresh = true;
  in.target_fresh = true;
  in.bridge_authorized = g_bridge_authorized;
  in.bridge_mode_ok = g_bridge_mode_ok;
  return in;
}

static void test_node_state()
{
  std::printf("[node_state]\n");
  g_bridge_authorized = false;
  g_bridge_mode_ok = false;
  {
    // 不发帧的状态集合必须精确
    CHECK(!statePublishes(NodeState::Standby), "STANDBY 不得发帧（会松劲）");
    CHECK(!statePublishes(NodeState::Fault), "FAULT 不得发帧");
    CHECK(!statePublishes(NodeState::Suspended), "SUSPENDED 不得发帧");
    CHECK(!statePublishes(NodeState::Arming), "ARMING 不得发帧");
    CHECK(statePublishes(NodeState::Running), "RUNNING 必须发帧");
    CHECK(statePublishes(NodeState::Dwell), "DWELL 必须发保持帧");
    CHECK(statePublishesFullRate(NodeState::Running), "RUNNING 全速");
    CHECK(!statePublishesFullRate(NodeState::Dwell), "DWELL 低频");
  }
  {
    // 全流程：IDLE→LOADING→STANDBY
    NodeStateMachine m(0);
    CHECK(m.state() == NodeState::Idle, "初始应为 IDLE");
    m.tick(base_inputs(1));
    CHECK(m.state() == NodeState::Loading, "load_ok=true 应进 LOADING，得到 %s",
          nodeStateName(m.state()));
    m.tick(base_inputs(2));
    CHECK(m.state() == NodeState::Standby, "应进 STANDBY，得到 %s",
          nodeStateName(m.state()));
  }
  {
    // enable 但状态不新鲜 ⇒ **必须拒绝**（拿不到实测角就不能接管）
    NodeStateMachine m(0);
    auto in = base_inputs(1);
    m.tick(in);
    m.tick(base_inputs(2));
    in = base_inputs(3);
    in.enable_requested = true;
    in.state_fresh = false;
    m.tick(in);
    CHECK(m.state() == NodeState::Standby, "状态不新鲜时 enable 必须被拒，得到 %s",
          nodeStateName(m.state()));
    CHECK(m.detail().find("不新鲜") != std::string::npos,
          "应说明原因，得到：%s", m.detail().c_str());
  }
  {
    // 完整接管链：ARMING → SWITCHING（两步：HOLD 再 TRACKING）→ RUNNING
    NodeStateMachine m(0);
    m.tick(base_inputs(1));
    m.tick(base_inputs(2));
    auto in = base_inputs(3);
    in.enable_requested = true;
    auto a = m.tick(in);
    CHECK(m.state() == NodeState::Arming, "应进 ARMING");
    CHECK(a.request_lock_source, "必须请求锁源");

    // 桥还没确认 ⇒ 停在 ARMING，不重复请求
    a = m.tick(base_inputs(4));
    CHECK(m.state() == NodeState::Arming, "未确认授权应停在 ARMING");
    CHECK(!a.request_lock_source, "不得重复请求锁源");

    // 桥确认 ⇒ 进 SWITCHING 并请求 HOLD
    g_bridge_authorized = true;
    a = m.tick(base_inputs(5));
    CHECK(m.state() == NodeState::Switching, "应进 SWITCHING，得到 %s",
          nodeStateName(m.state()));
    CHECK(a.request_mode_hold, "第一步必须请求 HOLD（迁移守则：→2 只能从 1 进）");
    CHECK(!a.request_mode_tracking, "第一步不得直接请求 TRACKING");

    // HOLD 回读 ok ⇒ 请求 TRACKING
    g_bridge_mode_ok = true;
    a = m.tick(base_inputs(6));
    CHECK(a.request_mode_tracking, "第二步必须请求 TRACKING");
    CHECK(m.state() == NodeState::Switching, "此刻还没到 RUNNING");

    // TRACKING 回读 ok ⇒ RUNNING
    g_bridge_authorized = true;
    g_bridge_mode_ok = true;
    a = m.tick(base_inputs(7));
    CHECK(m.state() == NodeState::Running, "应进 RUNNING，得到 %s",
          nodeStateName(m.state()));
  }
  {
    // RUNNING 中被抢占 ⇒ SUSPENDED 且**停发**
    NodeStateMachine m(0);
    m.tick(base_inputs(1));
    m.tick(base_inputs(2));
    auto in = base_inputs(3);
    in.enable_requested = true;
    m.tick(in);
    g_bridge_authorized = true;
    m.tick(base_inputs(4));
    g_bridge_mode_ok = true;
    m.tick(base_inputs(5));
    g_bridge_authorized = true;
    g_bridge_mode_ok = true;
    m.tick(base_inputs(6));
    CHECK(m.state() == NodeState::Running, "前置：应已 RUNNING");

    in = base_inputs(7);
    in.bridge_authorized = false;
    g_bridge_authorized = false;
    m.tick(in);
    CHECK(m.state() == NodeState::Suspended, "被抢占应进 SUSPENDED，得到 %s",
          nodeStateName(m.state()));
    CHECK(!statePublishes(m.state()), "SUSPENDED 必须停发");
  }
  {
    // 目标超时 ⇒ DWELL（继续发**低频保持帧**，不是停发）
    NodeStateMachine m(0);
    m.tick(base_inputs(1));
    m.tick(base_inputs(2));
    auto in = base_inputs(3);
    in.enable_requested = true;
    m.tick(in);
    g_bridge_authorized = true;
    m.tick(base_inputs(4));
    g_bridge_mode_ok = true;
    m.tick(base_inputs(5));
    g_bridge_authorized = true;
    g_bridge_mode_ok = true;
    m.tick(base_inputs(6));
    in = base_inputs(7);
    in.target_fresh = false;
    m.tick(in);
    CHECK(m.state() == NodeState::Dwell, "目标超时应进 DWELL，得到 %s",
          nodeStateName(m.state()));
    CHECK(statePublishes(m.state()), "DWELL 必须继续发保持帧（不能塌）");
    // 目标恢复 ⇒ 回 RUNNING
    g_bridge_authorized = true;
    m.tick(base_inputs(8));
    CHECK(m.state() == NodeState::Running, "目标恢复应回 RUNNING");
  }
  {
    // 违例 ⇒ FAULT，且要求交权；clear_fault 只能回 STANDBY（不能直接回 RUNNING）
    NodeStateMachine m(0);
    m.tick(base_inputs(1));
    m.tick(base_inputs(2));
    auto in = base_inputs(3);
    in.violation = "位置超差 0.5 rad 持续 0.2 s";
    auto a = m.tick(in);
    CHECK(m.state() == NodeState::Fault, "违例应进 FAULT，得到 %s",
          nodeStateName(m.state()));
    CHECK(a.request_release_source, "进 FAULT 必须交权");
    CHECK(a.log_error, "应报错");

    in = base_inputs(4);
    in.clear_fault_requested = true;
    in.state_fresh = false;
    m.tick(in);
    CHECK(m.state() == NodeState::Fault, "状态不新鲜时 clear_fault 必须被拒");

    in = base_inputs(5);
    in.clear_fault_requested = true;
    m.tick(in);
    CHECK(m.state() == NodeState::Standby, "clear_fault 应回 STANDBY，得到 %s",
          nodeStateName(m.state()));
    CHECK(m.detail().find("重新 enable") != std::string::npos,
          "应说明需要重新 enable，得到：%s", m.detail().c_str());
  }
  {
    // 运行中状态失鲜 ⇒ FAULT（不能"用过期数据继续出力"）
    NodeStateMachine m(0);
    m.tick(base_inputs(1));
    m.tick(base_inputs(2));
    auto in = base_inputs(3);
    in.enable_requested = true;
    m.tick(in);
    g_bridge_authorized = true;
    m.tick(base_inputs(4));
    g_bridge_mode_ok = true;
    m.tick(base_inputs(5));
    g_bridge_authorized = true;
    g_bridge_mode_ok = true;
    m.tick(base_inputs(6));
    CHECK(m.state() == NodeState::Running, "前置：RUNNING");
    in = base_inputs(7);
    in.state_fresh = false;
    m.tick(in);
    CHECK(m.state() == NodeState::Fault, "运行中失鲜应进 FAULT，得到 %s",
          nodeStateName(m.state()));
  }
  {
    // load 失败 ⇒ ERROR，且不可恢复
    NodeStateMachine m(0);
    auto in = base_inputs(1);
    in.load_ok = false;
    in.load_error = "IK 插件加载失败";
    m.tick(in);
    CHECK(m.state() == NodeState::Error, "load 失败应进 ERROR，得到 %s",
          nodeStateName(m.state()));
    m.tick(base_inputs(2));
    CHECK(m.state() == NodeState::Error, "ERROR 不应自行恢复");
  }
  {
    // RUNNING 中 disable ⇒ 先请求 HOLD（避免松劲）再交权，回 STANDBY
    NodeStateMachine m(0);
    m.tick(base_inputs(1));
    m.tick(base_inputs(2));
    auto in = base_inputs(3);
    in.enable_requested = true;
    m.tick(in);
    g_bridge_authorized = true;
    m.tick(base_inputs(4));
    g_bridge_mode_ok = true;
    m.tick(base_inputs(5));
    g_bridge_authorized = true;
    g_bridge_mode_ok = true;
    m.tick(base_inputs(6));
    in = base_inputs(7);
    in.disable_requested = true;
    auto a = m.tick(in);
    CHECK(m.state() == NodeState::Standby, "disable 应回 STANDBY");
    CHECK(a.request_mode_hold, "disable 应先请求 HOLD（否则 MIT 会松劲塌下来）");
    CHECK(a.request_release_source, "disable 应交权");
  }
}

int main()
{
  test_butterworth();
  test_resampler();
  test_admittance();
  test_impedance();
  test_limit_guard();
  test_node_state();
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
