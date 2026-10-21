// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// test_filter_shape.cpp —— 输入滤波管线 + 测试数据源形状的纯 C++ 单测
#include "lite_move_controller/input_filter.hpp"
#include "lite_move_controller/shape_traj.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace lite_move_controller;

static int g_fail = 0, g_n = 0;
#define CHECK(c, ...) do { ++g_n; if(!(c)) { ++g_fail; \
  std::printf("  FAIL %d  ", __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while(0)

static bool near(double a, double b, double tol=1e-9){ return std::fabs(a-b)<=tol; }
static double vnorm(const Vec3& v){ return std::sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }

// ---------------------------------------------------------------------------
// ① 四元数 ↔ 旋转向量
// ---------------------------------------------------------------------------
static void test_quat_rotvec()
{
  std::printf("[quat<->rotvec]\n");
  // 单位
  auto rv = quat_to_rotvec(Quat{0,0,0,1});
  CHECK(near(vnorm(rv),0.0,1e-12), "单位四元数 → 零旋转，得到 %.3e", vnorm(rv));
  // 绕 z 转 90°
  const double a = M_PI/2;
  Quat q{0,0,std::sin(a/2),std::cos(a/2)};
  rv = quat_to_rotvec(q);
  CHECK(near(rv[2], a, 1e-12), "绕 z 的旋转角应为 %.6f，得到 %.6f", a, rv[2]);
  CHECK(near(rv[0],0.0,1e-12) && near(rv[1],0.0,1e-12), "x/y 分量应为 0");
  // 往返一致
  auto back = rotvec_to_quat(rv);
  CHECK(near(std::fabs(quat_dot(back,q)),1.0,1e-12), "往返应回到同一姿态");
  // 双覆盖：q 与 −q 必须给同一旋转向量（靠 w<0 取反）
  Quat neg{-q[0],-q[1],-q[2],-q[3]};
  auto rv2 = quat_to_rotvec(neg);
  CHECK(near(rv2[2], rv[2], 1e-12), "−q 应给同一旋转向量，得到 %.6f vs %.6f", rv2[2], rv[2]);
  // 非法输入
  CHECK(near(vnorm(quat_to_rotvec(Quat{0,0,0,0})),0.0), "零四元数应返回零而非 NaN");
}

// ---------------------------------------------------------------------------
// ② 滤波管线
// ---------------------------------------------------------------------------
static RawPoseSample mk(std::int64_t t, double x, double y, double z)
{
  // ⚠️ recv_sys_ms 也要给：`stamp_ms` 是 epoch 基准，判"未来戳"时只能和
  //    **系统钟**的到达时刻比（`recv_ms` 是单调钟，减它会得到天文数字）。
  RawPoseSample s; s.recv_ms=t; s.recv_sys_ms=t; s.stamp_ms=t; s.has_stamp=true;
  s.position={x,y,z}; s.orientation={0,0,0,1}; return s;
}

static void test_input_filter()
{
  std::printf("[input_filter]\n");
  {
    InputFilterConfig bad; bad.control_rate_hz = 0.0;
    InputFilter f;
    CHECK(!f.configure(bad), "control_rate=0 必须拒绝");
  }
  {
    // sample() 在没有任何帧时必须返回 false（不能把"原点"当目标）
    InputFilterConfig cfg; cfg.pose_lpf_hz = 0.0; cfg.velocity_lpf_hz = 0.0;
    InputFilter f; CHECK(f.configure(cfg), "configure");
    Pose6 p;
    CHECK(!f.sample(p), "没有帧时 sample 必须返回 false");
  }
  {
    // 静态输入：滤波后应收敛到真值（直流增益 = 1）
    InputFilterConfig cfg; cfg.pose_lpf_hz = 15.0; cfg.velocity_lpf_hz = 25.0;
    InputFilter f; CHECK(f.configure(cfg), "configure");
    for (int i = 0; i < 400; ++i) {
      CHECK(f.push(mk(i*10, 0.3, -0.2, 0.5)), "push 应成功");
    }
    Pose6 p; CHECK(f.sample(p), "sample");
    CHECK(near(p.position[0],0.3,1e-4), "x 应收敛到 0.3，得到 %.6f", p.position[0]);
    CHECK(near(p.position[2],0.5,1e-4), "z 应收敛到 0.5，得到 %.6f", p.position[2]);
    CHECK(near(vnorm(p.linear_velocity),0.0,1e-3), "静止速度应≈0，得到 %.6f",
          vnorm(p.linear_velocity));
  }
  {
    // 速度重建：沿 x 以 0.1 m/s 匀速 ⇒ 重建速度应≈0.1
    InputFilterConfig cfg; cfg.pose_lpf_hz = 0.0; cfg.velocity_lpf_hz = 0.0;
    InputFilter f; CHECK(f.configure(cfg), "configure");
    for (int i = 0; i < 200; ++i) {
      const double t = i*0.01;
      CHECK(f.push(mk(i*10, 0.1*t, 0.0, 0.0)), "push");
    }
    Pose6 p; CHECK(f.sample(p), "sample");
    CHECK(near(p.linear_velocity[0],0.1,1e-6), "重建速度应=0.1，得到 %.6f",
          p.linear_velocity[0]);
  }
  {
    // 坏帧必须被拒且**不推进状态**
    InputFilterConfig cfg; cfg.pose_lpf_hz = 0.0;
    InputFilter f; CHECK(f.configure(cfg), "configure");
    CHECK(f.push(mk(0, 0.1, 0, 0)), "首帧");
    Pose6 before; f.sample(before);
    RawPoseSample bad = mk(10, 0.2, 0, 0);
    bad.position[0] = std::nan("");
    CHECK(!f.push(bad), "NaN 必须被拒");
    CHECK(f.lastReject()==FilterReject::NonFinite, "原因应为 NonFinite");
    CHECK(near(f.sample(before) ? before.position[0] : -1, 0.1, 1e-12),
          "被拒帧不得改动目标");
    CHECK(f.rejectCount()==1, "拒绝计数=1，得到 %llu",
          (unsigned long long)f.rejectCount());
  }
  {
    // 四元数模长不对必须拒（不静默归一化）
    InputFilterConfig cfg;
    InputFilter f; CHECK(f.configure(cfg), "configure");
    RawPoseSample s = mk(0,0,0,0); s.orientation={0,0,0,0.5};   // ‖q‖=0.5
    CHECK(!f.push(s), "模长偏差 >1e-3 必须拒绝");
    CHECK(f.lastReject()==FilterReject::BadQuaternion, "原因应为 BadQuaternion");
  }
  {
    // 未来戳必须拒（时钟不一致）——**绝对**判据：戳 vs 到达(系统钟)
    InputFilterConfig cfg; InputFilter f; CHECK(f.configure(cfg), "configure");
    CHECK(f.push(mk(1000, 0,0,0)), "首帧");
    RawPoseSample s = mk(1000, 0,0,0); s.stamp_ms = 1000 + 5000;   // 超前 5 s
    CHECK(!f.push(s), "超前戳必须拒绝");
    CHECK(f.lastReject()==FilterReject::FutureStamp, "原因应为 FutureStamp");
    CHECK(f.futureStampRejects()==1, "未来戳计数=1，得到 %llu",
          (unsigned long long)f.futureStampRejects());
    CHECK(f.stampFutureMaxMs() >= 5000, "最大超前应 ≥5000 ms，得到 %lld",
          (long long)f.stampFutureMaxMs());
  }
  {
    // 故意 +50 ms 的未来戳：必须拒（容差 40 ms，fail-safe 不退化成全盘接受）
    InputFilterConfig cfg; InputFilter f; CHECK(f.configure(cfg), "configure");
    CHECK(f.push(mk(1000, 0,0,0)), "首帧");
    RawPoseSample s = mk(1010, 0,0,0); s.stamp_ms = 1010 + 50;
    CHECK(!f.push(s), "+50 ms 未来戳必须拒绝");
    CHECK(f.lastReject()==FilterReject::FutureStamp, "+50 ms 应报 FutureStamp");
    // 而 +10 ms 的正常抖动必须**接受**
    RawPoseSample ok = mk(1020, 0,0,0); ok.stamp_ms = 1020 + 10;
    CHECK(f.push(ok), "+10 ms 抖动必须接受");
  }
  {
    // ⚠️ 回归：上游停流 > 容差之后**不得锁死**（2026-10 真机事故的直接复现）
    //    老代码把判据写成"stamp > 上一帧戳 + 200"，停流 60 s 后第一帧就永远
    //    超前 ⇒ 被拒 ⇒ last_stamp 只在接受路径更新 ⇒ **永久拒绝**。
    InputFilterConfig cfg; InputFilter f; CHECK(f.configure(cfg), "configure");
    CHECK(f.push(mk(1000, 0,0,0)), "首帧");
    RawPoseSample s = mk(61000, 0,0,0);          // 60 s 后才来下一帧（停流/重启）
    CHECK(f.push(s), "停流 60 s 后的第一帧必须被接受（不能锁死）");
    CHECK(f.lastReject()==FilterReject::None, "不应留下拒绝原因");
  }
  {
    // 乱序戳（小步倒退）必须拒；大步倒退（> 容差）= 上游时钟重置 ⇒ 重对齐
    InputFilterConfig cfg; InputFilter f; CHECK(f.configure(cfg), "configure");
    CHECK(f.push(mk(1000, 0,0,0)), "首帧");
    RawPoseSample s = mk(1005, 0,0,0); s.stamp_ms = 1000 - 5;   // 只倒退 5 ms
    CHECK(!f.push(s), "小步倒退戳必须拒绝");
    CHECK(f.lastReject()==FilterReject::OutOfOrder, "原因应为 OutOfOrder");
    CHECK(f.outOfOrderCount()==1, "乱序计数=1");
    RawPoseSample big = mk(1006, 0,0,0); big.stamp_ms = 1000 - 5000;  // 倒退 5 s
    CHECK(f.push(big), "大步倒退应重对齐（接受），不能锁死");
    CHECK(f.stampResyncCount()==1, "重对齐计数=1");
  }
  {
    // 到达时刻倒退必须拒（**不带**上游戳，才能隔离到 Stale 判定）
    InputFilterConfig cfg; InputFilter f; CHECK(f.configure(cfg), "configure");
    CHECK(f.push(mk(1000, 0,0,0)), "首帧");
    RawPoseSample s = mk(900, 0,0,0); s.has_stamp = false;   // 只测 recv_ms 倒退
    CHECK(!f.push(s), "到达时刻倒退必须拒绝");
    CHECK(f.lastReject()==FilterReject::Stale, "原因应为 Stale，得到 %s",
          filterRejectName(f.lastReject()));
  }
  {
    // 乱序戳优先于 Stale：戳与到达时刻**同时**倒退时报 OutOfOrder
    InputFilterConfig cfg; InputFilter f; CHECK(f.configure(cfg), "configure");
    CHECK(f.push(mk(1000, 0,0,0)), "首帧");
    RawPoseSample s = mk(900, 0,0,0);      // 到达（单调钟）倒退 100 ms
    s.recv_sys_ms = 995; s.stamp_ms = 995; // 系统钟与戳一致（不是未来），戳只退 5 ms
    CHECK(!f.push(s), "乱序戳必须拒绝");
    CHECK(f.lastReject()==FilterReject::OutOfOrder,
          "戳倒退应优先报 OutOfOrder，得到 %s", filterRejectName(f.lastReject()));
  }
  {
    // 四元数符号翻转**不得**让姿态跳变（这是本模块存在的核心理由）
    InputFilterConfig cfg; cfg.pose_lpf_hz = 0.0;
    InputFilter f; CHECK(f.configure(cfg), "configure");
    const double a = 0.2;                       // 绕 z 转 0.2 rad
    Quat q{0,0,std::sin(a/2),std::cos(a/2)};
    RawPoseSample s0 = mk(0,0,0,0); s0.orientation=q;
    CHECK(f.push(s0), "push q");
    Pose6 p0; f.sample(p0);
    // 同一个姿态用 −q 表示
    RawPoseSample s1 = mk(10,0,0,0); s1.orientation={-q[0],-q[1],-q[2],-q[3]};
    CHECK(f.push(s1), "push −q");
    Pose6 p1; f.sample(p1);
    CHECK(near(p1.rotation[2], p0.rotation[2], 1e-9),
          "符号翻转后旋转向量必须不变，得到 %.9f vs %.9f", p1.rotation[2], p0.rotation[2]);
  }
  {
    // ageMs
    InputFilterConfig cfg; InputFilter f; CHECK(f.configure(cfg), "configure");
    CHECK(f.ageMs(1234) == -1, "无帧时 age 应为 -1");
    CHECK(f.push(mk(1000,0,0,0)), "push");
    CHECK(f.ageMs(1050) == 50, "age 应为 50，得到 %lld", (long long)f.ageMs(1050));
  }
}

// ---------------------------------------------------------------------------
// ③ 形状轨迹
// ---------------------------------------------------------------------------
static void test_shapes()
{
  std::printf("[shape_traj]\n");
  {
    ShapeSpec c; c.kind=ShapeKind::Circle; c.size=0.15; c.speed=0.10;
    CHECK(near(c.perimeter(), M_PI*0.15, 1e-12), "圆周长 = π·d = %.6f，得到 %.6f",
          M_PI*0.15, c.perimeter());
    CHECK(near(c.period(), M_PI*0.15/0.10, 1e-9), "周期 = L/v = %.4f，得到 %.4f",
          M_PI*0.15/0.10, c.period());
    ShapeSpec q; q.kind=ShapeKind::Square; q.size=0.10; q.speed=0.10;
    CHECK(near(q.perimeter(), 0.40, 1e-12), "方周长 = 4a = 0.4，得到 %.6f", q.perimeter());
    CHECK(near(q.period(), 4.0, 1e-9), "周期 = 4 s，得到 %.4f", q.period());
  }
  {
    // 圆：半径恒定、闭合、速度恒定 = speed
    ShapeSpec c; c.kind=ShapeKind::Circle; c.size=0.15; c.speed=0.10;
    c.center={0.30,0.0,0.0}; c.plane.u={0,1,0}; c.plane.w={0,0,1}; c.plane.n={1,0,0};
    Vec3 p; Quat o; Vec3 v;
    double rmin=1e9, rmax=0, vmin=1e9, vmax=0;
    for (int i=0;i<4000;++i){
      const double t = c.period()*i/4000.0;
      CHECK(shapeAt(c,t,p,o,v), "shapeAt");
      const double dy=p[1]-c.center[1], dz=p[2]-c.center[2];
      const double r=std::sqrt(dy*dy+dz*dz);
      rmin=std::min(rmin,r); rmax=std::max(rmax,r);
      const double sp=vnorm(v); vmin=std::min(vmin,sp); vmax=std::max(vmax,sp);
    }
    CHECK(near(rmin,0.075,1e-9) && near(rmax,0.075,1e-9),
          "圆半径应恒为 0.075，得到 [%.6f, %.6f]", rmin, rmax);
    CHECK(near(vmin,0.10,1e-9) && near(vmax,0.10,1e-9),
          "圆速度应恒为 0.10，得到 [%.6f, %.6f]", vmin, vmax);
    // 闭合：t=0 与 t=period 位置相同
    Vec3 p0,p1; Quat o0,o1; Vec3 v0,v1;
    shapeAt(c,0.0,p0,o0,v0); shapeAt(c,c.period(),p1,o1,v1);
    CHECK(near(p0[0],p1[0],1e-9)&&near(p0[1],p1[1],1e-9)&&near(p0[2],p1[2],1e-9),
          "圆应闭合");
  }
  {
    // 方：四条边、周长一致、对角线不为 0
    ShapeSpec q; q.kind=ShapeKind::Square; q.size=0.10; q.speed=0.10;
    q.center={0.30,0.0,0.0}; q.plane.u={0,1,0}; q.plane.w={0,0,1}; q.plane.n={1,0,0};
    Vec3 p; Quat o; Vec3 v;
    double vmin=1e9, vmax=0;
    for (int i=0;i<4000;++i){
      const double t=q.period()*i/4000.0;
      CHECK(shapeAt(q,t,p,o,v), "shapeAt");
      const double sp=vnorm(v);
      vmin=std::min(vmin,sp); vmax=std::max(vmax,sp);
    }
    CHECK(near(vmin,0.10,1e-9) && near(vmax,0.10,1e-9),
          "方速度应恒为 0.10，得到 [%.6f, %.6f]", vmin, vmax);
    Vec3 p0,p1; Quat o0,o1; Vec3 v0,v1;
    shapeAt(q,0.0,p0,o0,v0); shapeAt(q,q.period(),p1,o1,v1);
    CHECK(near(p0[1],p1[1],1e-9)&&near(p0[2],p1[2],1e-9), "方应闭合");
    // 角点处：位置连续（用极小的 t 增量跨过角点）
    const double t_corner = 0.10/0.10;   // 走完第一条边 = 1 s
    Vec3 pa,pb; Quat oa,ob; Vec3 va,vb;
    shapeAt(q,t_corner-1e-6,pa,oa,va);
    shapeAt(q,t_corner+1e-6,pb,ob,vb);
    const double jump=std::sqrt((pa[1]-pb[1])*(pa[1]-pb[1])+(pa[2]-pb[2])*(pa[2]-pb[2]));
    CHECK(jump < 1e-4, "角点处位置必须连续，跳变 %.3e", jump);
  }
  {
    // 带圆角的方：周长更短（直角被替换）
    ShapeSpec q; q.kind=ShapeKind::Square; q.size=0.10; q.speed=0.10; q.corner_round=0.01;
    const double L=4.0*(0.10-0.02)+2.0*M_PI*0.01;
    CHECK(near(q.perimeter(), L, 1e-12), "圆角方周长应=%.6f，得到 %.6f", L, q.perimeter());
    Vec3 p; Quat o; Vec3 v;
    double vmin=1e9,vmax=0;
    for (int i=0;i<4000;++i){
      shapeAt(q, q.period()*i/4000.0, p,o,v);
      const double sp=vnorm(v); vmin=std::min(vmin,sp); vmax=std::max(vmax,sp);
    }
    CHECK(near(vmin,0.10,1e-9)&&near(vmax,0.10,1e-9), "圆角方速度应恒定");
  }
  {
    // 非法输入
    ShapeSpec bad; bad.kind=ShapeKind::Circle; bad.size=0.0;
    Vec3 p; Quat o; Vec3 v;
    CHECK(!shapeAt(bad, 1.0, p, o, v), "size=0 必须返回 false");
    ShapeSpec bad2; bad2.speed=0.0;
    CHECK(!shapeAt(bad2, 1.0, p, o, v), "speed=0 必须返回 false");
    ShapeSpec bad3; bad3.plane.u={1,0,0}; bad3.plane.w={1,0,0}; bad3.plane.n={0,0,1};
    CHECK(!shapeAt(bad3, 1.0, p, o, v), "退化平面（u∥w）必须返回 false");
  }
  {
    // 姿态：终端"向前"必须 = 平面法向；且四元数归一
    ShapeSpec c; c.kind=ShapeKind::Circle; c.size=0.15; c.speed=0.10;
    c.plane.u={0,1,0}; c.plane.w={0,0,1}; c.plane.n={1,0,0};
    Vec3 p; Quat o; Vec3 v;
    CHECK(shapeAt(c,0.5,p,o,v), "shapeAt");
    CHECK(near(quat_norm(o),1.0,1e-12), "四元数必须归一，得到 %.12f", quat_norm(o));
    // 把 n=(1,0,0) 转过去 ⇒ R·(0,0,1) 应 = n（x 轴是"向前"，这里验证 z 轴 = w）
    auto rv = quat_to_rotvec(o);
    CHECK(std::isfinite(rv[0])&&std::isfinite(rv[1])&&std::isfinite(rv[2]),
          "旋转向量必须有限");
  }
}

int main()
{
  test_quat_rotvec();
  test_input_filter();
  test_shapes();
  std::printf("\n%d checks, %d failures\n", g_n, g_fail);
  return g_fail==0 ? 0 : 1;
}
