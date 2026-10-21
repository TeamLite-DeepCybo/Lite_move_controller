// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// shape_source_node.cpp —— **测试数据源**：给运控层喂"圆 / 正方形"终端轨迹
//
// ===========================================================================
// 为什么它是运控层的一部分（而不是随手一个脚本）
// ===========================================================================
//
// 常规 / 导纳 / 阻抗三种控制律要能**比较**，前提是输入完全可比：
// 同一段时间-状态关系、同一平面、同一速度、同一姿态。随手脚本做不到
// （每人参数不同、还带随机），于是"导纳比常规抖"这种结论无法复现。
//
// 所以形状生成在 `shape_traj.hpp`（纯逻辑、有单测：周长/周期/弧长匀速/闭合），
// 本文件只做"按控制周期把它发出去"。
//
// ===========================================================================
// 默认形状（用户要求）
// ===========================================================================
//
//   左手：直径 **15 cm** 的圆
//   右手：边长 **10 cm** 的正方形
//   平移速度 **10 cm/s**
//   姿态：在垂直平面内保持**水平向前**（终端姿态固定）
//   平面：默认 Y-Z 平面（`u = +Y`、`w = +Z`、法向 `n = +X`）
//         ⇒ 终端"向前"指向 +X，"向上"为 +Z，即在**竖直平面里**画图。
//
// ⚠️ 中心点必须显式给（默认 `0.30, 0.0, 0.0` 只是"离基座 30 cm 的正前方"）。
//    真机上必须按实际可达空间调整 —— 给错中心点的表现是 IK 一直返回残差解。
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <move_msgs/msg/move_target.hpp>
#include <move_msgs/msg/move_target_array.hpp>

#include "lite_move_controller/shape_traj.hpp"

using namespace lite_move_controller;   // NOLINT

namespace
{
std::int64_t steadyNowMs()
{
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

class ShapeSourceNode : public rclcpp::Node
{
public:
  ShapeSourceNode() : Node("move_shape_source")
  {
    declare_parameter<std::string>("topic", "/lite_move_controller/task_target");
    declare_parameter<double>("rate_hz", 100.0);
    declare_parameter<double>("circle_diameter_m", 0.15);     // 左手
    declare_parameter<double>("square_side_m", 0.10);         // 右手
    declare_parameter<double>("speed_mps", 0.10);             // 10 cm/s
    declare_parameter<std::vector<double>>("center", {0.30, 0.0, 0.0});
    // ⚠️ 左右**各自**的圆心：两臂不能共用同一个中心（实测过：x 都写 +0.30 时，
    //    右臂被命令横跨身体 0.35 m，左右目标间距最小 6.8 mm → 两个夹爪会互撞）。
    //    为空则回退到 `center`（旧行为）。
    declare_parameter<std::vector<double>>("center_left", {});
    declare_parameter<std::vector<double>>("center_right", {});
    // 右臂画什么：circle / square（旧行为是 square）。左臂固定画圆。
    declare_parameter<std::string>("right_kind", "square");
    // 右臂画圆时的直径（<=0 表示跟随 circle_diameter_m）
    declare_parameter<double>("right_diameter_m", 0.0);
    // 哪根工具轴垂直指向平面：x（旧行为）/ y / z。本机实测 z 最可达。
    declare_parameter<std::string>("approach_axis", "x");
    declare_parameter<std::vector<double>>("plane_u", {0.0, 1.0, 0.0});
    declare_parameter<std::vector<double>>("plane_w", {0.0, 0.0, 1.0});
    declare_parameter<std::vector<double>>("plane_n", {1.0, 0.0, 0.0});
    declare_parameter<std::string>("frame_id", "world_root");
    declare_parameter<bool>("loop", true);
    declare_parameter<double>("duration_sec", 0.0);  // 0 = 一直发
    declare_parameter<double>("restart_pause_sec", 1.0);
    // 可选：**逐臂固定姿态**（四元数 x,y,z,w）。空 = 按 plane/approach_axis 推。
    //
    // ⚠️ 为什么需要：两臂当前姿态可能相差很大（本机实测 115°），而 plane 是
    //    两支臂**共用**的 ⇒ 共用推导姿态会把其中一条臂的腕部扭到限位附近。
    //    做"保持当前姿态、只平移画图"的测试时，把各自的当前姿态传进来即可。
    declare_parameter<std::vector<double>>("orientation_left", {});
    declare_parameter<std::vector<double>>("orientation_right", {});

    const double rate = get_parameter("rate_hz").as_double();
    if (!(rate > 0.0)) {
      RCLCPP_FATAL(get_logger(), "rate_hz 必须为正");
      throw std::runtime_error("rate_hz");
    }
    const auto center = get_parameter("center").as_double_array();
    auto cl = get_parameter("center_left").as_double_array();
    auto cr = get_parameter("center_right").as_double_array();
    if (cl.size() != 3) {cl = center;}
    if (cr.size() != 3) {cr = center;}
    const auto pu = get_parameter("plane_u").as_double_array();
    const auto pw = get_parameter("plane_w").as_double_array();
    const auto pn = get_parameter("plane_n").as_double_array();
    if (center.size() != 3 || pu.size() != 3 || pw.size() != 3 || pn.size() != 3) {
      RCLCPP_FATAL(get_logger(), "center / plane_u / plane_w / plane_n 都必须是 3 个数");
      throw std::runtime_error("shape params");
    }

    circle_.kind = ShapeKind::Circle;
    circle_.size = get_parameter("circle_diameter_m").as_double();
    circle_.speed = get_parameter("speed_mps").as_double();
    circle_.loop = get_parameter("loop").as_bool();
    circle_.center = Vec3{cl[0], cl[1], cl[2]};          // 左：自己的圆心
    circle_.plane.u = Vec3{pu[0], pu[1], pu[2]};
    circle_.plane.w = Vec3{pw[0], pw[1], pw[2]};
    circle_.plane.n = Vec3{pn[0], pn[1], pn[2]};

    square_ = circle_;
    square_.center = Vec3{cr[0], cr[1], cr[2]};          // 右：自己的圆心
    {
      const std::string rk = get_parameter("right_kind").as_string();
      square_.kind = (rk == "circle") ? ShapeKind::Circle : ShapeKind::Square;
      if (square_.kind == ShapeKind::Circle) {
        const double rd = get_parameter("right_diameter_m").as_double();
        square_.size = (rd > 0.0) ? rd : get_parameter("circle_diameter_m").as_double();
      } else {
        square_.size = get_parameter("square_side_m").as_double();
      }
    }
    // 工具"向前"轴（x/y/z）：两臂一致
    {
      std::string ax = get_parameter("approach_axis").as_string();
      std::transform(ax.begin(), ax.end(), ax.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c)); });
      const char a = ax.empty() ? 'x' : ax[0];
      circle_.approach_axis = a;
      square_.approach_axis = a;
    }

    frame_id_ = get_parameter("frame_id").as_string();
    duration_sec_ = get_parameter("duration_sec").as_double();
    restart_pause_sec_ = get_parameter("restart_pause_sec").as_double();

    // 自检：两个形状都必须能生成（退化平面等会被这里拦下）
    Vec3 p, v; Quat q;
    if (!shapeAt(circle_, 0.0, p, q, v) || !shapeAt(square_, 0.0, p, q, v)) {
      RCLCPP_FATAL(get_logger(), "形状参数非法（平面退化 / size=0 / speed=0）");
      throw std::runtime_error("shape invalid");
    }
    const char * knL = (circle_.kind == ShapeKind::Circle) ? "圆" : "方";
    const char * knR = (square_.kind == ShapeKind::Circle) ? "圆" : "方";
    RCLCPP_INFO(get_logger(),
                "测试数据源：左手 %s 尺寸 %.1f cm（周长 %.3f m，周期 %.2f s，圆心 (%.3f,%.3f,%.3f)）；"
                "右手 %s 尺寸 %.1f cm（周长 %.3f m，周期 %.2f s，圆心 (%.3f,%.3f,%.3f)）；"
                "速度 %.1f cm/s，approach_axis=%c",
                knL, circle_.size * 100, circle_.perimeter(), circle_.period(),
                circle_.center[0], circle_.center[1], circle_.center[2],
                knR, square_.size * 100, square_.perimeter(), square_.period(),
                square_.center[0], square_.center[1], square_.center[2],
                circle_.speed * 100, circle_.approach_axis);
    RCLCPP_INFO(get_logger(), "frame=%s 话题=%s",
                frame_id_.c_str(), get_parameter("topic").as_string().c_str());

    auto ql = get_parameter("orientation_left").as_double_array();
    auto qr = get_parameter("orientation_right").as_double_array();
    if (ql.size() == 4) {orient_left_ = Quat{ql[0], ql[1], ql[2], ql[3]}; has_orient_left_ = true;}
    if (qr.size() == 4) {orient_right_ = Quat{qr[0], qr[1], qr[2], qr[3]}; has_orient_right_ = true;}
    RCLCPP_INFO(get_logger(), "固定姿态：left=%s right=%s",
                has_orient_left_ ? "yes" : "no（按 plane 推）",
                has_orient_right_ ? "yes" : "no（按 plane 推）");

    pub_ = create_publisher<move_msgs::msg::MoveTargetArray>(
        get_parameter("topic").as_string(), rclcpp::QoS(20).reliable());
    t0_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate)),
        [this]() {onTick();});
  }

private:
  void onTick()
  {
    const double t = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - t0_).count();
    // 可选总时长：到点后**保持**（不突然停发 —— 停发会让运控层进 DWELL，
    // 那是"目标超时"的语义，用它来当"测完"会掩盖真实行为）。
    const double te = (duration_sec_ > 0.0) ? std::min(t, duration_sec_) : t;

    move_msgs::msg::MoveTargetArray arr;
    arr.targets.push_back(makeTarget("left", circle_, te));
    arr.targets.push_back(makeTarget("right", square_, te));
    pub_->publish(arr);
    ++sent_;
    if (sent_ % 200 == 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                           "已发 %llu 帧（t=%.1f s）",
                           static_cast<unsigned long long>(sent_), te);
    }
  }

  move_msgs::msg::MoveTarget makeTarget(const std::string & arm, const ShapeSpec & spec,
                                        double t)
  {
    Vec3 p{0, 0, 0}, v{0, 0, 0};
    Quat q{0, 0, 0, 1};
    if (!shapeAt(spec, t, p, q, v)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "形状生成失败（臂 %s）", arm.c_str());
    }
    // 逐臂固定姿态优先（见参数说明）：保持当前腕姿态，只做平移画图。
    if (arm == "left" && has_orient_left_) {q = orient_left_;}
    if (arm == "right" && has_orient_right_) {q = orient_right_;}
    move_msgs::msg::MoveTarget m;
    m.header.stamp = now();
    m.header.frame_id = frame_id_;
    m.arm = arm;
    m.frame_id = frame_id_;
    m.position.x = p[0];
    m.position.y = p[1];
    m.position.z = p[2];
    m.orientation.x = q[0];
    m.orientation.y = q[1];
    m.orientation.z = q[2];
    m.orientation.w = q[3];
    m.params_valid = false;        // 用运控层的启动期配置（本版不支持逐帧覆盖）
    m.sequence = ++seq_;
    return m;
  }

  ShapeSpec circle_, square_;
  Quat orient_left_{0, 0, 0, 1}, orient_right_{0, 0, 0, 1};
  bool has_orient_left_{false}, has_orient_right_{false};
  std::string frame_id_;
  double duration_sec_{0.0}, restart_pause_sec_{1.0};
  rclcpp::Publisher<move_msgs::msg::MoveTargetArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point t0_;
  std::uint64_t sent_{0}, seq_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ShapeSourceNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("move_shape_source"), "启动失败：%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
