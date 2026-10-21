// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// dynamics_pin.cpp —— 见头文件的"为什么用 pinocchio"与"单点动力学模型"
#include "lite_move_controller/dynamics_pin.hpp"

#include <urdf_parser/urdf_parser.h>         // ::urdf::parseURDF

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/joint/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/inertia.hpp>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>

namespace lite_move_controller
{

using PinModel = pinocchio::Model;
using PinData = pinocchio::Data;

namespace
{
/// 把一个连杆的 10 个动力学参数打包成 pinocchio 的 `Vector10`。
///
/// ⚠️ 顺序必须与 `computeJointTorqueRegressor` / `toDynamicParameters()` 一致：
///     `[m, m·cx, m·cy, m·cz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`
///
/// ⚠️ `params.json` 里 (m, μ) 在 `gravity.params`，6 个惯量分量在
///     `inertia.raw_fit.pi_I` —— 分两处存，必须**拼起来**才是完整刚体。
pinocchio::Inertia::Vector10 packLinkParams(const LinkDynamics & ld,
                                            const std::array<double, 6> & I6,
                                            bool has_gravity)
{
  pinocchio::Inertia::Vector10 v = pinocchio::Inertia::Vector10::Zero();
  // (m, μ)
  double m = ld.mass;
  double mu[3] = {ld.com[0], ld.com[1], ld.com[2]};
  if (!has_gravity) {m = 0.0; mu[0] = mu[1] = mu[2] = 0.0;}
  if (!std::isfinite(m)) {m = 0.0;}
  for (double & x : mu) {if (!std::isfinite(x)) {x = 0.0;}}
  v[0] = m;
  v[1] = mu[0];
  v[2] = mu[1];
  v[3] = mu[2];
  // 6 个惯量分量（顺序 = pinocchio 的 Ixx, Ixy, Iyy, Ixz, Iyz, Izz）
  for (std::size_t i = 0; i < 6; ++i) {
    v[4 + static_cast<Eigen::Index>(i)] = std::isfinite(I6[i]) ? I6[i] : 0.0;
  }
  return v;
}
}  // namespace

struct PinDynamics::Impl
{
  PinModel model;                 ///< 完整模型（每连杆 10 参数）
  PinModel gmodel;                ///< 只含 (m, μ) 的重力模型
  mutable PinData data;
  mutable PinData gdata;
  std::vector<int> joint_ids;     ///< 本臂 7 个关节在 model 里的 id（**输出顺序**）
  std::vector<int> idx_v;         ///< 每个关节的 v 下标
  int tip_frame_id{-1};
  Eigen::VectorXd q_full, v_full, a_full;
  Eigen::VectorXd q_g;
  FrictionDynamics friction;   ///< 摩擦参数（来自 params.json）
  std::vector<std::size_t> friction_index;  ///< 本臂关节 → params.joints 下标
  bool ready{false};
};

PinDynamics::PinDynamics() : impl_(std::make_unique<Impl>()) {}
PinDynamics::~PinDynamics() = default;
PinDynamics::PinDynamics(PinDynamics &&) noexcept = default;
PinDynamics & PinDynamics::operator=(PinDynamics &&) noexcept = default;

bool PinDynamics::load(const std::string & urdf_text,
                       const std::vector<std::string> & joint_names,
                       const std::string & tip_frame,
                       const DynamicsParams & params,
                       const double gravity[3],
                       std::string & error_out)
{
  ready_ = false;
  has_inertia_term_ = false;
  unmatched_.clear();
  joints_ = joint_names;
  dof_ = joint_names.size();
  if (dof_ == 0) {error_out = "关节列表为空"; return false;}

  try {
    // ---- ① 建模型 ----
    // ⚠️ 用**默认** `buildModelFromXML`（带 free-flyer 根关节，nq=nv=24）。
    //    这与 `lite_dyid/fastsolve.py` 的做法**完全一致**：
    //        pm = pin.buildModelFromUrdf(tmp)
    //        v_index = {n: pm.joints[jid[n]].idx_v for n in pm.names[1:]}
    //        arm_v  = [v_index[j] for j in ARM_JOINTS]
    //    free-flyer 的 6 个自由度我们**始终置零**（基座固定），
    //    它的力矩输出我们不取，所以不影响臂关节的 τ。
    //    （另一条路是传自定义 rootJoint，但那些重载只吃文件名/urdf::Model，
    //      徒增一个临时文件与一次重复解析，没有收益。）
    pinocchio::urdf::buildModelFromXML(urdf_text, impl_->model);
    impl_->gmodel = impl_->model;

    const PinModel & pm = impl_->model;
    // ---- ② 抽 pi_I（6 分量 × n_links）----
    {
      std::size_t nlinks = 0;
      for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(pm.njoints); ++j) {
        if (pm.njoints > 0) {++nlinks;}
      }
      const std::size_t need = 6 * nlinks;
      if (params.inertia_dynamic.size() == need) {
        has_inertia_term_ = true;
      } else if (!params.inertia_dynamic.empty()) {
        // 维数不符：**不猜**，当作没有惯量项（阻抗/导纳本来也不用）
        error_out = "pi_I 维数(" + std::to_string(params.inertia_dynamic.size()) +
                    ") 与 6×(njoints-1)(" + std::to_string(need) + ") 不符 → 惯性项不可用";
      }
    }

    // ---- ③ 逐连杆注入：full = (m,μ) + pi_I；gmodel = (m,μ) only ----
    // ⚠️⚠️ 名字映射：`pm.names[j]` 是**关节名**（`left_shoulder_pitch`），
    //    而 `params.links` 的键是**连杆名**（`left_shoulder_pitch_link`）。
    //    用关节名去查 links **一定全部落空** → 力矩恒为 0（实测踩过：18/18 未匹配）。
    //    正确映射：pinocchio 里每个关节 j 有一个 `BODY` frame，其
    //    `parentJoint == j`，那个 frame 的名字就是 URDF 的连杆名。
    std::vector<std::string> link_of_joint(static_cast<std::size_t>(pm.njoints));
    for (const auto & fr : pm.frames) {
      if (fr.type != pinocchio::BODY) {continue;}
      const auto pj = static_cast<std::size_t>(fr.parentJoint);
      if (pj < link_of_joint.size() && link_of_joint[pj].empty()) {
        link_of_joint[pj] = fr.name;
      }
    }

    std::size_t injected = 0;
    for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(pm.njoints); ++j) {
      const std::string & link_name = link_of_joint[static_cast<std::size_t>(j)];
      const auto it = params.links.find(link_name);
      if (it == params.links.end()) {
        // 参数里没有这个连杆（例如夹爪/末端）⇒ **不静默**：记下来并置零
        unmatched_.push_back(link_name.empty() ? pm.names[j] : link_name);
        impl_->model.inertias[j] = pinocchio::Inertia::Zero();
        impl_->gmodel.inertias[j] = pinocchio::Inertia::Zero();
        continue;
      }
      std::array<double, 6> I6{0, 0, 0, 0, 0, 0};
      if (has_inertia_term_) {
        // 块下标 k = joint id - 1（与 fastsolve 的 `gcols/icols = 10*k + ...`
        // 同一约定：k 从 0 起遍历 `range(1, njoints)`）
        const std::size_t k = static_cast<std::size_t>(j) - 1;
        for (std::size_t c = 0; c < 6; ++c) {I6[c] = params.inertia_dynamic[6 * k + c];}
      }
      // ⚠️⚠️ (m, μ) 的**权威来源是 `gravity.params`**，不是 `inertia.links`。
      //    实测两者不等价：`inertia.links[1].mass = -1.0`（哨兵），而
      //    `gravity.params[4] = 0.0`。用错来源会让重力矩从 5.6 N·m 变成 382 N·m。
      //
      // ⚠️⚠️⚠️ `gravity.params` 的 4 个分量**不是** [m, m·cx, m·cy, m·cz] 直读：
      //    lite_dyid 的回归基是（model.py::_build_basis_models）
      //        b=0: Inertia(1, 0)        → 质量 1 在原点
      //        b=1..3: Inertia(1, e_a)   → 质量 1 + 一阶矩 e_a
      //    线性合并成单个刚体时：
      //        等效质量 = p0 + p1 + p2 + p3
      //        一阶矩   = [p1, p2, p3]
      //    （lite_dyid 的 fast_gravity_evaluator 就是这么做的。）
      //    直接拿 p0 当质量会少掉 (p1+p2+p3)·A(q) 这一项：本机实测逐位形偏差
      //    0.15~0.29 N·m，与辨识残差（0.14 N·m）同量级 —— 前馈"总差一点"。
      const std::size_t k = static_cast<std::size_t>(j) - 1;
      const bool have_gp = params.gravity_params.size() >= 4 * (k + 1);
      double m_raw, mu3[3];
      if (have_gp) {
        const double p0 = params.gravity_params[4 * k];
        mu3[0] = params.gravity_params[4 * k + 1];
        mu3[1] = params.gravity_params[4 * k + 2];
        mu3[2] = params.gravity_params[4 * k + 3];
        m_raw = p0 + mu3[0] + mu3[1] + mu3[2];
      } else {
        m_raw = it->second.mass;
        mu3[0] = it->second.com[0];
        mu3[1] = it->second.com[1];
        mu3[2] = it->second.com[2];
      }
      if (!std::isfinite(m_raw)) {m_raw = 0.0;}
      for (double & x : mu3) {if (!std::isfinite(x)) {x = 0.0;}}
      pinocchio::Inertia::Vector10 v10 = pinocchio::Inertia::Vector10::Zero();
      v10[0] = m_raw;
      v10[1] = mu3[0];
      v10[2] = mu3[1];
      v10[3] = mu3[2];
      for (std::size_t c = 0; c < 6; ++c) {v10[4 + static_cast<Eigen::Index>(c)] = I6[c];}
      impl_->model.inertias[j] = pinocchio::Inertia::FromDynamicParameters(v10);

      // 重力模型：只含 (m, μ)。
      // ⚠️ 辨识解里存在 `m≈0 但 μ≠0` 的连杆（物理不可实现）。
      //    `fastsolve.py` 用 `mass=eps、lever=μ/eps` 的等价代换保住 μ
      //    （注释原文："若直接 max(m,0) 会丢掉 μ，实测 τ_g 偏 0.16 N·m"）。
      //    这里**逐字照做**，否则重力会偏 0.16 N·m。
      constexpr double eps_m = 1e-9;
      const double m_rep = (m_raw > eps_m) ? m_raw : eps_m;
      const Eigen::Vector3d mu(mu3[0], mu3[1], mu3[2]);
      impl_->gmodel.inertias[j] =
          pinocchio::Inertia(m_rep, mu / m_rep, Eigen::Matrix3d::Zero());
      ++injected;
    }
    (void)gravity;

    // ---- 摩擦参数（**必须在 load 里就拷进来**）----
    // ⚠️ 这段一度"漏写"（替换脚本没匹配上），后果是 `impl_->friction.available`
    //    恒为 0 ⇒ **摩擦前馈恒为 0**，而 `params.friction` 一切正常、
    //    测试只报"摩擦项没出现"。这类"参数没搬过来"的错没有任何报错。
    impl_->friction = params.friction;
    // 本臂关节 → `params.joints`（canonical 顺序）下标，**按名字匹配**：
    // 用位置下标 i 会让左右臂串号，且模型里 free-flyer 的存在会引起整体偏移。
    impl_->friction_index.assign(dof_, static_cast<std::size_t>(-1));
    for (std::size_t i = 0; i < dof_; ++i) {
      for (std::size_t c = 0; c < params.joints.size(); ++c) {
        if (params.joints[c] == joint_names[i]) {impl_->friction_index[i] = c; break;}
      }
    }

    impl_->data = PinData(impl_->model);
    impl_->gdata = PinData(impl_->gmodel);
    impl_->q_full = Eigen::VectorXd::Zero(impl_->model.nq);
    impl_->v_full = Eigen::VectorXd::Zero(impl_->model.nv);
    impl_->a_full = Eigen::VectorXd::Zero(impl_->model.nv);
    impl_->q_g = Eigen::VectorXd::Zero(impl_->gmodel.nq);

    // ---- ④ 关节索引 ----
    impl_->joint_ids.clear();
    impl_->idx_v.clear();
    for (const auto & jn : joint_names) {
      const auto it = std::find(pm.names.begin(), pm.names.end(), jn);
      if (it == pm.names.end()) {
        error_out = "模型里找不到关节 '" + jn + "'";
        return false;
      }
      const int jid = static_cast<int>(std::distance(pm.names.begin(), it));
      impl_->joint_ids.push_back(jid);
      impl_->idx_v.push_back(static_cast<int>(pm.joints[jid].idx_v()));
    }

    // ---- ⑤ 末端 frame ----
    if (!tip_frame.empty()) {
      for (std::size_t f = 0; f < pm.frames.size(); ++f) {
        if (pm.frames[f].name == tip_frame) {impl_->tip_frame_id = static_cast<int>(f); break;}
      }
      if (impl_->tip_frame_id < 0) {
        // 末端 frame 找不到 ⇒ 雅可比不可用。**不静默**：报给调用方。
        std::fprintf(stderr, "[dynamics_pin] 末端 frame '%s' 不在模型里\n", tip_frame.c_str());
      }
    }

    impl_->ready = true;
    ready_ = true;
    if (injected == 0) {error_out = "没有任何连杆被注入（名字全对不上）"; return false;}
    return true;
  } catch (const std::exception & e) {
    error_out = std::string("pinocchio 建模型/注入失败：") + e.what();
    ready_ = false;
    return false;
  }
}

bool PinDynamics::gravityTorque(const std::vector<double> & q,
                                std::vector<double> & tau_g) const
{
  if (!ready_ || q.size() != dof_) {return false;}
  try {
    impl_->q_g.setZero();
    for (std::size_t i = 0; i < dof_; ++i) {
      if (!std::isfinite(q[i])) {return false;}
      impl_->q_g[impl_->gmodel.joints[impl_->joint_ids[i]].idx_q()] = q[i];
    }
    const Eigen::VectorXd & res =
        pinocchio::computeGeneralizedGravity(impl_->gmodel, impl_->gdata, impl_->q_g);
    tau_g.assign(dof_, 0.0);
    for (std::size_t i = 0; i < dof_; ++i) {
      tau_g[i] = res[impl_->idx_v[i]];
      if (!std::isfinite(tau_g[i])) {return false;}
    }
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool PinDynamics::modelTorque(const std::vector<double> & q,
                              const std::vector<double> & v,
                              const std::vector<double> & a,
                              bool with_inertia,
                              std::vector<double> & tau_out) const
{
  if (!ready_ || q.size() != dof_) {return false;}
  const auto t0 = std::chrono::steady_clock::now();
  try {
    // 重力（始终有）
    std::vector<double> tg;
    if (!gravityTorque(q, tg)) {return false;}
    tau_out = tg;

    // 摩擦（始终有）
    //
    // ⚠️ 摩擦数组是按 **`params.joints`（canonical 顺序）** 排的，不能直接用
    //    本臂关节的位置下标 `i` 去索引 —— 那样左右臂会串号（左臂用到右臂的
    //    摩擦系数），而且模型里 free-flyer 的存在会让下标整体偏移。
    //    这里按**关节名**在 `params.joints` 里定位。
    if (std::getenv("LMC_DBG")) {
      std::fprintf(stderr, "[dbg] friction.available=%d dof=%zu v.size=%zu c1p=%zu fidx0=%zd\n",
                   (int)impl_->friction.available, dof_, v.size(),
                   impl_->friction.c1_pos.size(),
                   impl_->friction_index.empty() ? -1L
                   : static_cast<long>(impl_->friction_index[0]));
    }
    if (impl_->friction.available) {
      if (v.size() != dof_) {return false;}
      for (std::size_t i = 0; i < dof_; ++i) {
        if (!std::isfinite(v[i])) {return false;}
        std::size_t gi = impl_->friction_index[i];
        if (gi == static_cast<std::size_t>(-1)) {continue;}   // 该关节没有摩擦参数
        tau_out[i] += frictionTorqueSymmetric(impl_->friction.c1_pos[gi],
                                              impl_->friction.c1_neg[gi],
                                              impl_->friction.c2[gi], v[i]);
      }
    }

    // 惯性 + 科氏 + 离心（可选）
    if (with_inertia && has_inertia_term_) {
      if (v.size() != dof_ || a.size() != dof_) {return false;}
      impl_->q_full.setZero();
      impl_->v_full.setZero();
      impl_->a_full.setZero();
      for (std::size_t i = 0; i < dof_; ++i) {
        if (!std::isfinite(v[i]) || !std::isfinite(a[i])) {return false;}
        const auto & jm = impl_->model.joints[impl_->joint_ids[i]];
        impl_->q_full[jm.idx_q()] = q[i];
        impl_->v_full[jm.idx_v()] = v[i];
        impl_->a_full[jm.idx_v()] = a[i];
      }
      // τ_I = rnea(full, q,v,a) − rnea(full, q,0,0)
      // ⚠️ 必须减掉 a=0,v=0 的那一份，否则重力会被**重复计入**（full 模型里
      //    已经含 (m,μ)，rnea 本身就带重力）。这与 fastsolve 的
      //    `gdblk = Y[:,gcols]@pi_g`（把重力分离出去）是同一件事。
      impl_->v_full.setZero();   // 先算 a=0,v=0 的纯重力项
      impl_->a_full.setZero();
      const Eigen::VectorXd base = pinocchio::rnea(
          impl_->model, impl_->data, impl_->q_full, impl_->v_full, impl_->a_full);
      for (std::size_t i = 0; i < dof_; ++i) {
        const auto & jm = impl_->model.joints[impl_->joint_ids[i]];
        impl_->v_full[jm.idx_v()] = v[i];
        impl_->a_full[jm.idx_v()] = a[i];
      }
      const Eigen::VectorXd full = pinocchio::rnea(
          impl_->model, impl_->data, impl_->q_full, impl_->v_full, impl_->a_full);
      for (std::size_t i = 0; i < dof_; ++i) {
        const double d = full[impl_->idx_v[i]] - base[impl_->idx_v[i]];
        if (!std::isfinite(d)) {return false;}
        tau_out[i] += d;
      }
    }

    const auto t1 = std::chrono::steady_clock::now();
    last_cost_us_ = std::chrono::duration<double, std::micro>(t1 - t0).count();
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool PinDynamics::jacobian(const std::vector<double> & q, Jacobian & J) const
{
  // TODO(下一轮)：用 pinocchio::computeJointJacobians + getJointJacobian(WORLD)
  // 复现已验证的 base 系约定。当前由 `ik_kdl` 的 KDL 链提供雅可比
  // （已逐列验证与数值微分一致到 2.6e-8），不在这一步改动。
  (void)q; (void)J;
  return false;
}

}  // namespace lite_move_controller
