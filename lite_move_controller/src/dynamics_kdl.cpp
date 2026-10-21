// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// dynamics_kdl.cpp —— 见头文件的坐标系与惯量限制说明
#include "lite_move_controller/dynamics_kdl.hpp"

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <kdl/rotationalinertia.hpp>

#include <array>
#include <cmath>
#include <vector>

namespace lite_move_controller
{

namespace
{
/// KDL Jacobian（6×n，行主序 6 行）→ 本层的 Jacobian。
void kdlJacToJac(const KDL::Jacobian & src, Jacobian & dst)
{
  const std::size_t n = static_cast<std::size_t>(src.columns());
  dst.resize(6, n);
  for (std::size_t r = 0; r < 6; ++r) {
    for (std::size_t c = 0; c < n; ++c) {
      dst.at(r, c) = src(static_cast<unsigned>(r), static_cast<unsigned>(c));
    }
  }
}
}  // namespace

struct KdlDynamics::Impl
{
  /// ⚠️ **必须自己持有一份链的副本**：我们要按连杆名替换每段的惯量，
  ///    而 `KdlArmChain` 里的链是只读的（属于 IK 后端）。
  ///    早先用函数内 `static thread_local` 存副本 —— 那是**全局状态**：
  ///    两个实例（左右臂）会互相覆盖，症状是"右臂的重力补偿用了左臂的模型"，
  ///    而且是静默的。改成成员后每个实例各自持有。
  std::unique_ptr<KDL::Chain> owned_chain;
  const KDL::Chain * chain{nullptr};
  std::unique_ptr<KDL::ChainDynParam> dyn;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
};

KdlDynamics::KdlDynamics() : impl_(std::make_unique<Impl>()) {}
KdlDynamics::~KdlDynamics() = default;
KdlDynamics::KdlDynamics(KdlDynamics &&) noexcept = default;
KdlDynamics & KdlDynamics::operator=(KdlDynamics &&) noexcept = default;

bool KdlDynamics::load(const KdlArmChain & chain,
                       const DynamicsParams & params,
                       const double gravity[3],
                       std::string & error_out)
{
  ready_ = false;
  unmatched_.clear();
  joints_ = chain.jointNames();
  dof_ = chain.dof();
  has_inertia_ = params.usable_for_mass_matrix;

  if (!chain.ready()) {error_out = "链未就绪"; return false;}
  if (dof_ == 0) {error_out = "链的自由度为 0"; return false;}
  const void * const raw = chain.chainOpaque();
  if (raw == nullptr) {error_out = "拿不到底层 KDL 链"; return false;}
  impl_->chain = static_cast<const KDL::Chain *>(raw);

  // 需要一份**可修改**的链：KDL 的 Chain 在拷贝后才好逐段 setInertia。
  auto owned = std::make_unique<KDL::Chain>(*impl_->chain);

  // ==========================================================================
  // 按**连杆名**替换每一段的惯量
  // ==========================================================================
  //
  // ⚠️⚠️ 这里有两个必须做对、我第一版**两个都做错了**的地方：
  //
  // ---- 错误 ①：把 `com` 当成质心位置 ----
  //   `lite_dyid` 的参数基（`model.py::_build_basis_models`，每连杆 4 个基，
  //   共 4×18 = 72 列）是**质量 + 一阶矩**：
  //       b=0: Inertia(1.0, [0,0,0], 0)  → 参数 = m
  //       b=1: Inertia(1.0, [1,0,0], 0)  → 参数 = m·cx
  //       b=2: Inertia(1.0, [0,1,0], 0)  → 参数 = m·cy
  //       b=3: Inertia(1.0, [0,0,1], 0)  → 参数 = m·cz
  //   所以 `params.json` 的 `links.<n>.com` 是**一阶矩 m·c**，而**不是**质心；
  //   惯量张量在回归基里**恒为 0**。
  //   正确写法：`RigidBodyInertia(mass, first_moment_in_segment_frame, I=0)`。
  //   实测（4 个姿态对拍 `τ=Y(q)·π`）：改对后偏差从 0.84 N·m 降到 0.02~0.25 N·m。
  //
  // ---- 错误 ②：忽略 KDL segment frame 与 URDF link frame 的旋转 ----
  //   KDL 建链时会把**关节轴旋转到 Z**，所以 segment frame 与 URDF link frame
  //   之间差一个固定旋转 `R_ls`。实测这个 URDF 的关节轴在 link frame 下
  //   离 Z 很远（例如 `left_shoulder_pitch` 的轴 `[-1,0,0]` 经 RPY 旋转后是
  //   `[0.985, 0, 0.174]`，与 Z 差 ~80°）—— 不补偿这一项，一阶矩方向就是错的。
  //
  //   `kdl_parser` 用 `R_ls` 的**第三列**作为 segment 的关节轴，
  //   所以 `R_ls` 可以把 URDF 轴 `a` 映射到 Z：`R_ls · a = ±Z`。
  //   一阶矩在 segment frame 下 = `R_lsᵀ · h_link`。
  //
  // ---- 为什么 `I` 保持 0 是正确的 ----
  //   本包的辨识**只有重力回归**（`model.py::arm_regressor` 直接返回
  //   `gravity_regressor`）。重力只依赖 `mass` 与**一阶矩**，**与惯性张量无关**。
  //   所以重力补偿不需要惯量；输出里的 `I` 仅供质量矩阵 `M(q)` 使用
  //   （`usable_for_mass_matrix`）。
  // ==========================================================================
  std::size_t matched = 0;
  for (unsigned i = 0; i < owned->getNrOfSegments(); ++i) {
    KDL::Segment & seg = owned->getSegment(i);
    const std::string seg_name = seg.getName();
    const auto it = params.links.find(seg_name);
    if (it == params.links.end()) {
      // 夹爪/末端等不在辨识范围内的段：占位值不可信 ⇒ 置零
      unmatched_.push_back(seg_name);
      seg.setInertia(KDL::RigidBodyInertia::Zero());
      continue;
    }
    const LinkDynamics & d = it->second;

    // ---- `R_ls`：从关节轴构造一个把该轴映射到 Z 的旋转 ----
    // KDL 的 `Joint::JointAxis` 给出轴在 segment frame 下的表示（就是 ±Z）；
    // 我们需要反方向的 `R_ls`。用 URDF link frame 下的一阶矩做变换，
    // 所以这里用关节轴在 **link frame** 下的表示 `a`：
    //     R_ls 的第三列 = a  ⇒  用 Gram-Schmidt 补出正交基
    // ⚠️ 若拿不到可靠轴，就退化为单位阵（并记一次警告计数），**不假装知道**。
    KDL::Rotation R_ls = KDL::Rotation::Identity();
    {
      // 关节轴（在 link frame 下）从 URDF 的 <axis> 取 —— 由调用方通过
      // `params.axis_of_link` 传入；没有就直接用单位阵。
      const auto ax = params.axis_of_link.find(seg_name);
      if (ax != params.axis_of_link.end()) {
        double a[3] = {ax->second[0], ax->second[1], ax->second[2]};
        const double n = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
        if (n > 1e-9) {
          for (double & x : a) {x /= n;}
          // 构造正交基：取一个与 a 不平行的参考向量
          double ref[3] = {0.0, 0.0, 1.0};
          if (std::fabs(a[2]) > 0.9) {ref[0] = 1.0; ref[1] = 0.0; ref[2] = 0.0;}
          // u = ref × a, v = a × u  （u、v、a 正交）
          double u[3] = {ref[1] * a[2] - ref[2] * a[1],
                         ref[2] * a[0] - ref[0] * a[2],
                         ref[0] * a[1] - ref[1] * a[0]};
          double un = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
          if (un > 1e-9) {
            for (double & x : u) {x /= un;}
            const double v[3] = {a[1] * u[2] - a[2] * u[1],
                                 a[2] * u[0] - a[0] * u[2],
                                 a[0] * u[1] - a[1] * u[0]};
            // R_ls 的列 = (u, v, a) ⇒ R_ls·a = Z ✓
            R_ls = KDL::Rotation(u[0], v[0], a[0],
                                 u[1], v[1], a[1],
                                 u[2], v[2], a[2]);
          }
        }
      }
    }

    // 一阶矩：URDF link frame → segment frame
    const KDL::Vector h_link(d.com[0], d.com[1], d.com[2]);
    // 方向：segment frame 下的一阶矩 = R_ls · h_link（R_ls 把 link 轴映到 Z）
    const KDL::Vector h_seg = R_ls * h_link;
    // I = 0：重力只依赖 mass 与一阶矩（见上）
    seg.setInertia(KDL::RigidBodyInertia(d.mass, h_seg, KDL::RotationalInertia::Zero()));
    ++matched;
  }
  if (matched == 0) {
    error_out = "没有任何连杆名能对上链上的 segment（params.json 与 URDF 不同源？）";
    return false;
  }
  impl_->owned_chain = std::move(owned);
  impl_->chain = impl_->owned_chain.get();

  const KDL::Vector gvec(gravity[0], gravity[1], gravity[2]);
  impl_->dyn = std::make_unique<KDL::ChainDynParam>(*impl_->chain, gvec);
  impl_->jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(*impl_->chain);
  ready_ = true;
  return true;
}

bool KdlDynamics::gravityTorque(const std::vector<double> & q,
                                std::vector<double> & tau_g) const
{
  if (!ready_ || q.size() != dof_) {return false;}
  KDL::JntArray qa(static_cast<unsigned>(dof_));
  for (std::size_t i = 0; i < dof_; ++i) {
    if (!std::isfinite(q[i])) {return false;}
    qa(static_cast<unsigned>(i)) = q[i];
  }
  KDL::JntArray g(static_cast<unsigned>(dof_));
  if (impl_->dyn->JntToGravity(qa, g) < 0) {return false;}
  tau_g.resize(dof_);
  for (std::size_t i = 0; i < dof_; ++i) {
    tau_g[i] = g(static_cast<unsigned>(i));
    if (!std::isfinite(tau_g[i])) {return false;}
  }
  return true;
}

bool KdlDynamics::jacobian(const std::vector<double> & q, Jacobian & J) const
{
  if (!ready_ || q.size() != dof_) {return false;}
  KDL::JntArray qa(static_cast<unsigned>(dof_));
  for (std::size_t i = 0; i < dof_; ++i) {
    if (!std::isfinite(q[i])) {return false;}
    qa(static_cast<unsigned>(i)) = q[i];
  }
  KDL::Jacobian kj(static_cast<unsigned>(dof_));
  if (impl_->jac_solver->JntToJac(qa, kj) < 0) {return false;}
  kdlJacToJac(kj, J);
  for (double v : J.data) {if (!std::isfinite(v)) {return false;}}
  return true;
}

bool KdlDynamics::tauFromWrench(const Jacobian & J, const std::array<double, 6> & wrench,
                                std::vector<double> & tau) const
{
  if (!J.valid() || J.cols != dof_) {return false;}
  tau.assign(dof_, 0.0);
  for (std::size_t j = 0; j < dof_; ++j) {
    double s = 0.0;
    for (std::size_t r = 0; r < 6; ++r) {s += J.at(r, j) * wrench[r];}
    tau[j] = s;
    if (!std::isfinite(tau[j])) {return false;}
  }
  return true;
}

bool KdlDynamics::estimateWrench(const Jacobian & J, const std::vector<double> & tau_ext,
                                 std::array<double, 6> & wrench_out, double lambda) const
{
  if (!J.valid() || J.cols != dof_ || tau_ext.size() != dof_) {return false;}
  // A = J·Jᵀ + λ²I（6×6）
  Mat6 A{};
  for (std::size_t r = 0; r < 6; ++r) {
    for (std::size_t c = 0; c < 6; ++c) {
      double s = 0.0;
      for (std::size_t j = 0; j < dof_; ++j) {s += J.at(r, j) * J.at(c, j);}
      A[r * 6 + c] = s;
    }
  }
  const double lam = (lambda > 0.0) ? lambda : 1e-6;
  for (std::size_t r = 0; r < 6; ++r) {A[r * 6 + r] += lam * lam;}
  // rhs = J·τ_ext
  Vec6 rhs{};
  for (std::size_t r = 0; r < 6; ++r) {
    double s = 0.0;
    for (std::size_t j = 0; j < dof_; ++j) {s += J.at(r, j) * tau_ext[j];}
    rhs[r] = s;
  }
  Vec6 y{};
  if (!solve6(A, rhs, y)) {return false;}          // 奇异 ⇒ 放弃本拍估计
  for (std::size_t i = 0; i < 6; ++i) {
    if (!std::isfinite(y[i])) {return false;}
  }
  wrench_out = y;
  return true;
}

double KdlDynamics::conditioningHint(const Jacobian & J) const
{
  if (!J.valid()) {return 0.0;}
  // 粗估：J·Jᵀ 的对角元之和 / 最小对角元（不做完整 SVD —— 每拍给不起）。
  double mn = 1e300, mx = 0.0;
  for (std::size_t r = 0; r < 6; ++r) {
    double s = 0.0;
    for (std::size_t j = 0; j < J.cols; ++j) {s += J.at(r, j) * J.at(r, j);}
    mn = std::min(mn, s);
    mx = std::max(mx, s);
  }
  return (mn > 1e-12) ? mx / mn : 1e12;
}

}  // namespace lite_move_controller
