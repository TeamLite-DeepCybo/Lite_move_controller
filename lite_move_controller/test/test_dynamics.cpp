// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// test_dynamics.cpp —— 动力学模块（**pinocchio 版**）的集成测试
//
// 需要真实 URDF 与一份 schema2 的 params.json：
//     LITE_ARM_URDF=<path> LITE_PARAMS_JSON=<path> ./test_dynamics
// 缺任一个就 SKIP（打印 SKIP 返回 0），**不假通过**。
//
// ⚠️ 本测试只做"结构 + 物理合理性 + 自洽性"检查。
//    **与辨识侧的逐点数值一致性**由 `test/tools/cmp_pin_vs_python.py` 做
//    （跨语言对拍，见该脚本），不在这里假装验证。
#include "lite_move_controller/dynamics_pin.hpp"
#include "lite_ik_node/ik_kdl.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace lite_move_controller;

static int g_fail = 0, g_n = 0;
#define CHECK(c, ...) do { ++g_n; if(!(c)) { ++g_fail; \
  std::printf("  FAIL %d  ", __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while(0)

static std::string readFile(const std::string & p)
{
  std::ifstream f(p);
  if (!f) {return {};}
  std::stringstream ss; ss << f.rdbuf();
  return ss.str();
}

int main()
{
  const char * e_urdf = std::getenv("LITE_ARM_URDF");
  const char * e_json = std::getenv("LITE_PARAMS_JSON");
  if (e_urdf == nullptr || e_json == nullptr) {
    std::printf("SKIP: 需要 LITE_ARM_URDF 与 LITE_PARAMS_JSON\n");
    return 0;
  }
  const std::string urdf = readFile(e_urdf);
  if (urdf.empty()) {std::printf("SKIP: 读不到 URDF\n"); return 0;}

  // ---- ① JSON 解析 ----
  DynamicsParams params;
  {
    MiniJson root;
    std::string err;
    CHECK(MiniJson::parseFile(e_json, root, err), "JSON 解析失败：%s", err.c_str());
    CHECK(parseDynamicsParams(root, params, err), "参数抽取失败：%s", err.c_str());
    params.source_path = e_json;
    CHECK(root.isObject(), "根应为对象");
    CHECK(root.find("schema") != nullptr, "应能取到 schema");
    CHECK(root.find("no_such_key_xyz") == nullptr, "不存在的键应返回 nullptr");
  }
  CHECK(params.schema == 2, "schema 应为 2，得到 %d", params.schema);
  CHECK(params.joints.size() == 14, "joints 应为 14，得到 %zu", params.joints.size());
  CHECK(params.urdf_sha256.size() == 64, "urdf_sha256 应为 64 位，得到 %zu",
        params.urdf_sha256.size());
  CHECK(!params.links.empty(), "应解析出连杆参数");
  CHECK(params.friction.available, "摩擦参数应可用");
  std::printf("  模型 = %s，连杆 %zu 个，pi_I %zu 个\n",
              params.model_kind.c_str(), params.links.size(), params.inertia_dynamic.size());

  // ---- ② Infinity / NaN 必须被接受（Python json.dump 会写出裸 Infinity）----
  {
    MiniJson probe;
    std::string err;
    CHECK(MiniJson::parse(R"({"a": Infinity, "b": -Infinity, "c": NaN})", probe, err),
          "应接受 Infinity/-Infinity/NaN：%s", err.c_str());
    const MiniJson * a = probe.find("a");
    CHECK(a != nullptr && std::isinf(a->asNumber(0.0)) && a->asNumber(0.0) > 0,
          "Infinity 应为 +inf");
    const MiniJson * c = probe.find("c");
    CHECK(c != nullptr && std::isnan(c->asNumber(0.0)), "NaN 应为 nan");
  }

  // ---- ③ 解析必须**幂等**（同一个输出对象调多次不得累加）----
  {
    MiniJson root2;
    std::string err2;
    CHECK(MiniJson::parseFile(e_json, root2, err2), "re-parse");
    DynamicsParams again;
    CHECK(parseDynamicsParams(root2, again, err2), "second parse");
    const std::size_t n1 = again.joints.size(), i1 = again.inertia_dynamic.size();
    for (int rep = 0; rep < 3; ++rep) {CHECK(parseDynamicsParams(root2, again, err2), "repeat");}
    CHECK(again.joints.size() == n1, "joints 不得累加：%zu → %zu", n1, again.joints.size());
    CHECK(again.inertia_dynamic.size() == i1, "pi_I 不得累加：%zu → %zu",
          i1, again.inertia_dynamic.size());
    CHECK(again.links.size() == params.links.size(), "links 不得累加");
  }

  // ---- ④ schema1 必须被拒 ----
  {
    MiniJson old;
    std::string err;
    CHECK(MiniJson::parse(R"({"schema":1})", old, err), "解析 schema1 探针");
    DynamicsParams p2;
    CHECK(!parseDynamicsParams(old, p2, err), "schema1 必须被拒绝");
  }

  // ---- ⑤ 库仑摩擦：零偏置 + 精确奇的对称化版本 ----
  {
    const double eps = 0.01;
    CHECK(frictionTorque(0.08, -0.03, 0.12, 0.0, eps) == 0.0, "零速摩擦必须为 0");
    // τ(−v)+τ(v) = (c1p−c1n)·s² —— 模型的**固有**性质（两侧不对称时必然带偶次项）
    const double c1p = 0.08, c1n = -0.03, c2 = 0.12;
    for (double v : {0.001, 0.05, 0.5, -0.002, -0.3}) {
      const double a = frictionTorque(c1p, c1n, c2, v, eps);
      const double b = frictionTorque(c1p, c1n, c2, -v, eps);
      const double s2 = std::tanh(v / eps) * std::tanh(v / eps);
      CHECK(std::fabs((a + b) - (c1p - c1n) * s2) < 1e-12,
            "τ(−v)+τ(v) 应 = (c1p−c1n)·s²（v=%.4f）", v);
    }
    // 对称化版本必须**精确奇对称**（零力拖动/导纳需要）
    for (double v : {0.001, 0.5, -0.003, -0.25}) {
      const double a = frictionTorqueSymmetric(0.08, -0.03, 0.12, v, eps, 0);
      const double b = frictionTorqueSymmetric(0.08, -0.03, 0.12, -v, eps, 0);
      CHECK(std::fabs(a + b) < 1e-15, "对称化版本必须严格奇对称（v=%.4f）", v);
    }
    // |τ| 随 |v| 单调不减
    double prev = 0.0;
    bool mono = true;
    for (int i = 1; i <= 500; ++i) {
      const double t = std::fabs(frictionTorque(0.5, 0.5, 0.2, i * 1e-4, eps));
      if (t < prev - 1e-12) {mono = false; break;}
      prev = t;
    }
    CHECK(mono, "|τ(v)| 应单调不减");
  }

  // ---- ⑥ pinocchio 建模型 + 注入 ----
  const std::vector<std::string> L = {
      "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
      "left_elbow_pitch", "left_wrist_yaw", "left_wrist_roll", "left_wrist_pitch"};
  PinDynamics dyn;
  const double gravity[3] = {0.0, 0.0, -9.81};
  std::string err;
  CHECK(dyn.load(urdf, L, "left_gripper_tip_middle_link", params, gravity, err),
        "pinocchio 加载失败：%s", err.c_str());
  if (!dyn.ready()) {std::printf("\n%d checks, %d failures\n", g_n, g_fail); return 1;}
  CHECK(dyn.dof() == 7, "自由度应为 7，得到 %zu", dyn.dof());
  if (const char * dbg = std::getenv("LMC_DBG")) {
    (void)dbg;
    std::printf("  [dbg] friction.available=%d c1_pos=%zu c2=%zu\n",
                (int)params.friction.available, params.friction.c1_pos.size(),
                params.friction.c2.size());
    std::printf("  [dbg] params.joints=%zu inertia_dynamic=%zu gravity_params=%zu\n",
                params.joints.size(), params.inertia_dynamic.size(),
                params.gravity_params.size());
  }
  std::printf("  未匹配连杆 %zu 个（夹爪/末端属正常）\n", dyn.unmatchedLinks().size());

  // ---- ⑦ 重力矩：有限、随姿态变化、量级物理合理 ----
  {
    std::vector<double> q1(7, 0.0), t1, t2;
    CHECK(dyn.gravityTorque(q1, t1), "重力矩应成功");
    CHECK(t1.size() == 7, "长度 7");
    for (double t : t1) {CHECK(std::isfinite(t), "重力矩必须有限");}
    std::vector<double> q2 = {0.8, -0.6, 0.4, -0.9, 0.3, 0.2, -0.1};
    CHECK(dyn.gravityTorque(q2, t2), "重力矩应成功");
    double md = 0.0, mx = 0.0;
    for (std::size_t i = 0; i < 7; ++i) {
      md = std::max(md, std::fabs(t2[i] - t1[i]));
      mx = std::max(mx, std::fabs(t2[i]));
      CHECK(std::fabs(t2[i]) < 50.0, "重力矩量级不合理：%.3f", t2[i]);
    }
    CHECK(md > 0.1, "换姿态重力矩应变化（>0.1），得到 %.4f", md);
    std::printf("  τ_g(q=0) 前3 = (%.4f, %.4f, %.4f)  max|τ_g| = %.3f N·m\n",
                t1[0], t1[1], t1[2], mx);
  }

  // ---- ⑧ 统一 modelTorque：三种模式的口径 ----
  {
    std::vector<double> q = {0.3, -0.2, 0.1, -0.4, 0.05, 0.1, -0.1};
    std::vector<double> v(7, 0.0), a(7, 0.0);
    std::vector<double> t_nofric, t_g;
    CHECK(dyn.gravityTorque(q, t_g), "重力");
    // v=0, a=0, 不带惯性 ⇒ 应与重力一致（摩擦在 v=0 处为 0）
    CHECK(dyn.modelTorque(q, v, a, false, t_nofric), "modelTorque(不含惯性)");
    for (std::size_t i = 0; i < 7; ++i) {
      CHECK(std::fabs(t_nofric[i] - t_g[i]) < 1e-9,
            "v=0,a=0 且不含惯性时应等于重力（关节 %zu：%.9f vs %.9f）",
            i, t_nofric[i], t_g[i]);
    }
    // v≠0 ⇒ 摩擦出现
    std::vector<double> v2(7, 0.3), t_fric;
    CHECK(dyn.modelTorque(q, v2, a, false, t_fric), "modelTorque(带摩擦)");
    double df = 0.0;
    for (std::size_t i = 0; i < 7; ++i) {df = std::max(df, std::fabs(t_fric[i] - t_g[i]));}
    CHECK(df > 1e-6, "v≠0 时摩擦项应出现，得到 %.3e", df);
    // 带惯性且 a≠0 ⇒ 应与"不含惯性"不同
    std::vector<double> a2(7, 1.0), t_in;
    if (dyn.hasInertiaTerm()) {
      CHECK(dyn.modelTorque(q, v2, a2, true, t_in), "modelTorque(带惯性)");
      double di = 0.0;
      for (std::size_t i = 0; i < 7; ++i) {di = std::max(di, std::fabs(t_in[i] - t_fric[i]));}
      CHECK(di > 1e-9, "a≠0 时惯性项应出现，得到 %.3e", di);
      std::printf("  惯量项使力矩最大变化 %.4f N·m（a=1 rad/s²）\n", di);
    } else {
      std::printf("  （无 pi_I ⇒ 惯性项不可用，阻抗/导纳本来也不用）\n");
    }
    // a=0 且 v≠0、带惯性 ⇒ 只有科氏项
    std::vector<double> t_cor;
    if (dyn.hasInertiaTerm()) {
      CHECK(dyn.modelTorque(q, v2, a, true, t_cor), "modelTorque(a=0,带惯性)");
    }
  }

  // ---- ⑨ 单次耗时（判断是否需要降频缓存）----
  {
    std::vector<double> q = {0.3, -0.2, 0.1, -0.4, 0.05, 0.1, -0.1};
    std::vector<double> v(7, 0.2), a(7, 0.5), t;
    double worst = 0.0, sum = 0.0;
    const int N = 200;
    for (int i = 0; i < N; ++i) {
      dyn.modelTorque(q, v, a, true, t);
      const double us = dyn.lastCostUs();
      worst = std::max(worst, us);
      sum += us;
    }
    std::printf("  modelTorque 耗时：均值 %.2f µs，最大 %.2f µs（100 Hz 周期 10000 µs）\n",
                sum / N, worst);
    CHECK(worst < 2000.0, "单次耗时应远小于控制周期（最大 %.1f µs）", worst);
  }

  // ---- ⑩ 非法输入不得崩、不得给 NaN ----
  {
    std::vector<double> out;
    std::vector<double> qbad(7, std::nan(""));
    CHECK(!dyn.gravityTorque(qbad, out), "NaN 关节角应 false");
    std::vector<double> t;
    CHECK(!dyn.modelTorque(qbad, std::vector<double>(7, 0.0), std::vector<double>(7, 0.0),
                           false, t), "NaN 关节角应 false");
    std::vector<double> qshort{0.0, 0.0};
    CHECK(!dyn.gravityTorque(qshort, out), "长度不符应 false");
    std::vector<double> q = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> vbad(7, std::nan(""));
    CHECK(!dyn.modelTorque(q, vbad, std::vector<double>(7, 0.0), false, t), "NaN 速度应 false");
  }

  std::printf("\n%d checks, %d failures\n", g_n, g_fail);
  return g_fail == 0 ? 0 : 1;
}
