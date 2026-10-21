// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// shape_traj.hpp —— **测试数据源**的轨迹生成（圆 / 正方形 在任意平面内）
//
// ===========================================================================
// 为什么它是运控层的一部分（而不是随手写个脚本）
// ===========================================================================
//
// 常规 / 导纳 / 阻抗三种控制律要能比较，前提是**输入完全可比**：
// 同一段时间-状态关系、同一平面、同一速度、同一姿态。随手写的脚本做不到
// （每个人参数不一样、还带随机），于是"导纳比常规抖"这种结论无法复现。
//
// 所以这里把形状生成做成**纯逻辑**（可单测：周长、速度、闭合性、离散精度），
// 由 `shape_source_node` 包成 ROS 节点。
//
// ===========================================================================
// 两种形状的参数化（按**弧长**匀速，而不是按角度匀速）
// ===========================================================================
//
// 用户要求"平移速度 10 cm/s"。按弧长参数化才能保证三种形状下速度一致：
//
//   圆：周长 L = 2πr，走 L 用 L/v 秒 ⇒ 角速度 ω = v/r（不是固定 Hz！）
//   方：周长 L = 4a，四边逐边推进；**角点处不停顿**（速度连续，位置不连续于角点
//       的方向，但位置本身连续 —— 这正是我们想测的：角点会不会激起振荡）
//
// ⚠️ **角点处位置连续、速度方向突变**：这是刻意的。方形测试的价值之一就是
//    看控制律在"速度不连续"处的表现（导纳会滞后、阻抗会顶一下）。
//    若要平滑版本，另加圆角（`corner_round`）。
//
// ===========================================================================
// 姿态：终端保持"水平向前"
// ===========================================================================
//
// 三个形状都在**同一个平面**内，终端姿态在该平面内**固定**（保持水平向前）。
// 平面由三个正交轴给出：
//     u（平面内 x）、w（平面内 y）、n（平面法向 = 终端"向前"方向）
// 位置 = center + p_u·u + p_w·w，姿态 = 固定四元数（由 n 与"向上"定义）。
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

#include "lite_move_controller/input_filter.hpp"   // Vec3 / Quat

namespace lite_move_controller
{

/// 形状种类。
enum class ShapeKind : std::uint8_t
{
  Circle = 0,
  Square = 1,
};

inline const char * shapeKindName(ShapeKind k) noexcept
{
  return (k == ShapeKind::Circle) ? "circle" : "square";
}

/// 平面定义：`u` 与 `w` 张成绘图平面，`n` 是平面法向（= 终端"向前"）。
struct PlaneFrame
{
  Vec3 u{1.0, 0.0, 0.0};
  Vec3 w{0.0, 1.0, 0.0};
  Vec3 n{0.0, 0.0, 1.0};

  /// 正交归一化（Gram-Schmidt）。返回 false ⇒ 退化（u 与 w 共线），调用方必须拒绝。
  bool orthonormalize() noexcept
  {
    if (!vec3_finite(u) || !vec3_finite(w) || !vec3_finite(n)) {return false;}
    if (!normalize(u)) {return false;}
    // w 去掉 u 分量
    const double d = dot(w, u);
    for (int i = 0; i < 3; ++i) {w[static_cast<std::size_t>(i)] -= d * u[static_cast<std::size_t>(i)];}
    if (!normalize(w)) {return false;}          // u 与 w 共线 → 退化
    // n 用 u × w 定（保证右手系；调用方给的 n 只决定正负）
    const Vec3 cross{u[1] * w[2] - u[2] * w[1],
                     u[2] * w[0] - u[0] * w[2],
                     u[0] * w[1] - u[1] * w[0]};
    const double sgn = (dot(n, cross) >= 0.0) ? 1.0 : -1.0;
    for (int i = 0; i < 3; ++i) {n[static_cast<std::size_t>(i)] = sgn * cross[static_cast<std::size_t>(i)];}
    return true;
  }

  static double dot(const Vec3 & a, const Vec3 & b) noexcept
  {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  }
  static bool normalize(Vec3 & v) noexcept
  {
    const double nrm = std::sqrt(dot(v, v));
    if (!(nrm > 1e-9) || !std::isfinite(nrm)) {return false;}
    for (double & x : v) {x /= nrm;}
    return true;
  }
};

/// 形状参数。
struct ShapeSpec
{
  ShapeKind kind{ShapeKind::Circle};
  /// 绘图平面中心（m，在目标 frame 下）
  Vec3 center{0.30, 0.0, 0.0};
  PlaneFrame plane{};
  /// 圆：直径（m）；方：边长（m）
  double size{0.15};
  /// 弧长速度（m/s）
  double speed{0.10};
  /// 起始相位（0 = 从 u 轴正方向的点开始）
  double phase0{0.0};
  /// 方的圆角半径（m，0 = 尖角）
  double corner_round{0.0};
  /// 是否在末尾回到起点（闭合）。true = 周期形状，可以无限循环。
  bool loop{true};

  // 姿态：哪根**工具轴**与平面法向 n 对齐（= "垂直指向平面/向前"）。
  //
  // ⚠️ 这不是一个可以随便选的约定：同一个竖直平面，法向由工具 X / Y / Z 承担
  //    是三种**相差 90° 滚转**的姿态，而本机 7 轴的腕部滚转范围只有 ±0.70 rad，
  //    实测差别是"能不能解"级别的：
  //      法向=工具X（旧约定）  左臂 2/5、右臂 0/5（圆心就差 76~80 mm）
  //      法向=工具Z            左臂 5/5、右臂 5/5（本机最优）
  //      法向=工具Y            左臂 1/5、右臂 5/5 但最差 182 mm
  //    所以必须可配，而不是硬绑 X。
  //
  //   'x' → (X,Y,Z) = (n, u, w)  工具 X 向前（**旧行为**，向后兼容）
  //   'z' → (X,Y,Z) = (u, w, n)  工具 Z 向前（本机实测最可达）
  //   'y' → (X,Y,Z) = (w, n, u)  工具 Y 向前
  char approach_axis{'x'};

  /// 形状周长（m）。
  double perimeter() const noexcept
  {
    if (kind == ShapeKind::Circle) {
      return M_PI * size;                       // 2πr, r = size/2
    }
    // 方形：四边 + （若圆角）把 4 个直角换成四段 1/4 圆弧
    const double straight = 4.0 * std::max(0.0, size - 2.0 * corner_round);
    const double arc = 2.0 * M_PI * std::max(0.0, corner_round);
    return straight + arc;
  }

  /// 走完一周需要的时间（s）。
  double period() const noexcept
  {
    const double L = perimeter();
    return (speed > 1e-9) ? L / speed : 0.0;
  }
};

/// 形状上的一点：平面内坐标 + 切向速度方向。
struct ShapePoint
{
  double s{0.0};          ///< 平面内 u 方向的偏移（m）
  double t{0.0};          ///< 平面内 w 方向的偏移（m）
  double ds{0.0};         ///< 速度的 u 分量（m/s）
  double dt{0.0};         ///< 速度的 w 分量（m/s）
};

/// 圆上按弧长参数化的点。
inline ShapePoint circleAt(const ShapeSpec & spec, double arc) noexcept
{
  const double r = 0.5 * spec.size;
  if (!(r > 1e-9)) {return ShapePoint{};}
  const double L = 2.0 * M_PI * r;
  double a = std::fmod(arc, L);
  if (a < 0.0) {a += L;}
  const double th = spec.phase0 + a / r;        // 弧长 → 角度
  const double v = spec.speed;
  ShapePoint p;
  p.s = r * std::cos(th);
  p.t = r * std::sin(th);
  // d/darc · v = 切向单位向量 × v
  p.ds = -std::sin(th) * v;
  p.dt = std::cos(th) * v;
  return p;
}

/// 方（可带圆角）上按弧长参数化的点。
///
/// 直边按 u/w 方向直线推进，角点用 1/4 圆弧过渡（`corner_round > 0`）；
/// `corner_round == 0` 时角点是**尖的**（位置连续、速度方向突变）。
inline ShapePoint squareAt(const ShapeSpec & spec, double arc) noexcept
{
  const double a = std::max(0.0, spec.size);
  const double cr = std::min(std::max(0.0, spec.corner_round), 0.5 * a);
  const double half = 0.5 * a;
  const double L = spec.perimeter();
  if (!(L > 1e-9)) {return ShapePoint{};}
  double s_arc = std::fmod(arc, L);
  if (s_arc < 0.0) {s_arc += L;}
  const double v = spec.speed;
  ShapePoint p;

  if (cr <= 1e-9) {
    // 尖角：四段直线，每段长 a
    const int seg = static_cast<int>(std::floor(s_arc / a)) % 4;
    const double local = s_arc - static_cast<double>(seg) * a;
    // 顶点顺序（逆时针，从 (+half, -half) 起）：右下 → 右上 → 左上 → 左下
    const double x0[4] = {half, half, -half, -half};
    const double y0[4] = {-half, half, half, -half};
    const double dx[4] = {0.0, -1.0, 0.0, 1.0};
    const double dy[4] = {1.0, 0.0, -1.0, 0.0};
    p.s = x0[seg] + dx[seg] * local;
    p.t = y0[seg] + dy[seg] * local;
    // 加 phase0 的旋转（绕平面法向）
    const double c = std::cos(spec.phase0), sn = std::sin(spec.phase0);
    const double s0 = p.s, t0 = p.t;
    p.s = c * s0 - sn * t0;
    p.t = sn * s0 + c * t0;
    p.ds = v * (c * dx[seg] - sn * dy[seg]);
    p.dt = v * (sn * dx[seg] + c * dy[seg]);
    return p;
  }
  // 圆角：每边一段直线（长 a − 2cr）+ 每个角一段 1/4 圆弧（长 π·cr/2）
  const double straight = a - 2.0 * cr;
  const double arc_len = 0.5 * M_PI * cr;
  double rest = s_arc;
  // 四个角（逆时针）：右下、右上、左上、左下
  const double cx[4] = {half - cr, half - cr, -(half - cr), -(half - cr)};
  const double cy[4] = {-(half - cr), half - cr, half - cr, -(half - cr)};
  const double start_ang[4] = {-0.5 * M_PI, 0.0, 0.5 * M_PI, M_PI};
  for (int k = 0; k < 4; ++k) {
    if (rest < straight) {
      // 第 k 条直边：从角 k 的圆弧终点沿 u 或 w 方向走 straight
      const double dirx[4] = {0.0, -1.0, 0.0, 1.0};
      const double diry[4] = {1.0, 0.0, -1.0, 0.0};
      const double end_ang = start_ang[k] + 0.5 * M_PI;
      const double sx = cx[k] + cr * std::cos(end_ang);
      const double sy = cy[k] + cr * std::sin(end_ang);
      p.s = sx + dirx[k] * rest;
      p.t = sy + diry[k] * rest;
      p.ds = v * dirx[k];
      p.dt = v * diry[k];
      const double c = std::cos(spec.phase0), sn = std::sin(spec.phase0);
      const double s0 = p.s, t0 = p.t, ds0 = p.ds, dt0 = p.dt;
      p.s = c * s0 - sn * t0;
      p.t = sn * s0 + c * t0;
      p.ds = c * ds0 - sn * dt0;
      p.dt = sn * ds0 + c * dt0;
      return p;
    }
    rest -= straight;
    if (rest < arc_len) {
      const double ang = start_ang[k] + rest / cr;
      p.s = cx[k] + cr * std::cos(ang);
      p.t = cy[k] + cr * std::sin(ang);
      p.ds = -std::sin(ang) * v;
      p.dt = std::cos(ang) * v;
      const double c = std::cos(spec.phase0), sn = std::sin(spec.phase0);
      const double s0 = p.s, t0 = p.t, ds0 = p.ds, dt0 = p.dt;
      p.s = c * s0 - sn * t0;
      p.t = sn * s0 + c * t0;
      p.ds = c * ds0 - sn * dt0;
      p.dt = sn * ds0 + c * dt0;
      return p;
    }
    rest -= arc_len;
  }
  return p;                                     // 数值兜底（不应到达）
}

/// 生成一帧：给定时间 t（秒，从形状起点算），返回目标位姿（frame 下的位置 + 固定姿态）。
///
/// ⚠️ `loop == false` 时超过一个周期**停在终点**（不绕回），便于"画完就停"。
inline bool shapeAt(const ShapeSpec & spec, double t,
                    Vec3 & position_out, Quat & orientation_out,
                    Vec3 & linear_vel_out) noexcept
{
  if (!(spec.speed > 1e-9) || !(spec.size > 1e-9)) {return false;}
  PlaneFrame pl = spec.plane;
  if (!pl.orthonormalize()) {return false;}
  const double L = spec.perimeter();
  double arc = spec.speed * std::max(0.0, t);
  if (!spec.loop && arc > L) {arc = L;}

  const ShapePoint sp = (spec.kind == ShapeKind::Circle) ? circleAt(spec, arc)
                                                         : squareAt(spec, arc);
  // 平面内坐标 → 三维位置
  for (int i = 0; i < 3; ++i) {
    position_out[static_cast<std::size_t>(i)] =
        spec.center[static_cast<std::size_t>(i)] +
        sp.s * pl.u[static_cast<std::size_t>(i)] +
        sp.t * pl.w[static_cast<std::size_t>(i)];
    linear_vel_out[static_cast<std::size_t>(i)] =
        sp.ds * pl.u[static_cast<std::size_t>(i)] +
        sp.dt * pl.w[static_cast<std::size_t>(i)];
  }
  // 姿态：由 `spec.approach_axis` 决定哪根工具轴与平面法向对齐。
  //   只需给出 (x_axis, z_axis)，y = z×x 与再正交化在下面统一做：
  //     'x' → (n, w)   'z' → (u, n)   'y' → (w, u)
  Vec3 x_axis = pl.n;                 // 向前
  Vec3 z_axis = pl.w;                 // 向上
  switch (spec.approach_axis) {
    case 'z': x_axis = pl.u; z_axis = pl.n; break;
    case 'y': x_axis = pl.w; z_axis = pl.u; break;
    default:  break;                  // 'x' = 旧行为
  }
  // y = z × x（右手系：x 向前, z 向上 ⇒ y 向左）
  Vec3 y_axis{z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
              z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
              z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0]};
  if (!PlaneFrame::normalize(y_axis)) {return false;}
  // 重新正交化 z = x × y
  z_axis = Vec3{x_axis[1] * y_axis[2] - x_axis[2] * y_axis[1],
                x_axis[2] * y_axis[0] - x_axis[0] * y_axis[2],
                x_axis[0] * y_axis[1] - x_axis[1] * y_axis[0]};
  // 旋转矩阵 R = [x y z] → 四元数（Shepperd 法）
  const double m00 = x_axis[0], m01 = y_axis[0], m02 = z_axis[0];
  const double m10 = x_axis[1], m11 = y_axis[1], m12 = z_axis[1];
  const double m20 = x_axis[2], m21 = y_axis[2], m22 = z_axis[2];
  const double tr = m00 + m11 + m22;
  Quat q{0, 0, 0, 1};
  if (tr > 0.0) {
    const double s = std::sqrt(tr + 1.0) * 2.0;
    q[3] = 0.25 * s;
    q[0] = (m21 - m12) / s;
    q[1] = (m02 - m20) / s;
    q[2] = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;
    q[3] = (m21 - m12) / s;
    q[0] = 0.25 * s;
    q[1] = (m01 + m10) / s;
    q[2] = (m02 + m20) / s;
  } else if (m11 > m22) {
    const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;
    q[3] = (m02 - m20) / s;
    q[0] = (m01 + m10) / s;
    q[1] = 0.25 * s;
    q[2] = (m12 + m21) / s;
  } else {
    const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;
    q[3] = (m10 - m01) / s;
    q[0] = (m02 + m20) / s;
    q[1] = (m12 + m21) / s;
    q[2] = 0.25 * s;
  }
  orientation_out = q;
  return quat_normalize(orientation_out);
}

}  // namespace lite_move_controller
