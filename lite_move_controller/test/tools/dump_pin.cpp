// dump_pin.cpp —— 从 stdin 读 (q,v,a)、输出 C++ 侧的 τ_g / τ_friction / τ_total
// 用于与 Python（lite_dyid 辨识侧）逐点对拍。见 cmp_pin_vs_python.py
#include "lite_move_controller/dynamics_pin.hpp"
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>
using namespace lite_move_controller;
static std::string rd(const char * p)
{
  std::ifstream f(p);
  std::stringstream s;
  s << f.rdbuf();
  return s.str();
}
int main(int argc, char ** argv)
{
  if (argc < 3) {std::fprintf(stderr, "用法: dump_pin <urdf> <params.json>\n"); return 2;}
  MiniJson root;
  std::string err;
  if (!MiniJson::parseFile(argv[2], root, err)) {
    std::fprintf(stderr, "JSON: %s\n", err.c_str()); return 2;
  }
  DynamicsParams dp;
  if (!parseDynamicsParams(root, dp, err)) {std::fprintf(stderr, "params: %s\n", err.c_str()); return 2;}
  std::vector<std::string> L = {
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
    "left_elbow_pitch", "left_wrist_yaw", "left_wrist_roll", "left_wrist_pitch"};
  PinDynamics dyn;
  const double g[3] = {0.0, 0.0, -9.81};
  if (!dyn.load(rd(argv[1]), L, "left_gripper_tip_middle_link", dp, g, err)) {
    std::fprintf(stderr, "load: %s\n", err.c_str()); return 2;
  }
  double q[7], v[7], a[7];
  while (std::scanf("%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                    &q[0], &q[1], &q[2], &q[3], &q[4], &q[5], &q[6],
                    &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6],
                    &a[0], &a[1], &a[2], &a[3], &a[4], &a[5], &a[6]) == 21) {
    std::vector<double> qq(q, q + 7), vv(v, v + 7), aa(a, a + 7), tg, tf, tt;
    if (!dyn.gravityTorque(qq, tg)) {std::printf("FAIL\n"); continue;}
    // 摩擦 = modelTorque(含惯性=false, v) − τ_g
    if (!dyn.modelTorque(qq, vv, aa, false, tt)) {std::printf("FAIL\n"); continue;}
    tf.resize(7);
    for (int i = 0; i < 7; ++i) {tf[i] = tt[i] - tg[i];}
    std::vector<double> tall;
    if (!dyn.modelTorque(qq, vv, aa, true, tall)) {std::printf("FAIL\n"); continue;}
    for (int i = 0; i < 7; ++i) {std::printf("%.12g ", tg[i]);}
    for (int i = 0; i < 7; ++i) {std::printf("%.12g ", tf[i]);}
    for (int i = 0; i < 7; ++i) {std::printf("%.12g%c", tall[i], i == 6 ? '\n' : ' ');}
  }
  return 0;
}
