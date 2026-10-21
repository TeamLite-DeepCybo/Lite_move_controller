// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// gravity_yaml.hpp —— 从 `lite_controller/config/gravity/inertia_params.yaml`
// （`dynamics_calib identify` 的正式结果，HoldController 一直在用）读出
// **重力前馈**参数。
//
// ===========================================================================
// 为什么只读这个文件就够（以及为什么不瞎补）
// ===========================================================================
//
// 准静态下**只有一阶矩 m·c 必然可辨识**：
//     U(q) = Σ_k m_k·g·(o_k + R_k·c_k)_z
//   · 只依赖一阶矩的项 —— 任何位形都可辨识；
//   · 依赖 m_k 的项 —— 只有该连杆原点随近端关节垂直移动时才随 q 变。
// 所以 yaml 里 `mass: null` 的连杆（如肩 pitch）**不是数据缺失的意外**，
// 而是"这个量在重力回归里根本不可辨识"。本文件的处理与
// `lite_controller::GravityCompensator` 完全一致：**mass=1.0、
// com=first_moment**（只保证乘积 m·c 正确，τ_g 因此正确；质量矩阵本来也不可用）。
//
// ⚠️ `offsets_nm` 是"力矩读数零偏 + 常摩擦"的经验项，**不是**刚体参数：
//    运行时按 `tau_ff = τ_g(q) + offset` 直接相加（限幅之前）。
//
// ⚠️ 本文件**只读**：不改任何别包的文件，只按路径读取。
#pragma once

#include <map>
#include <string>

#include "lite_move_controller/dynamics.hpp"

namespace lite_move_controller
{

/// `inertia_params.yaml` 的解析结果。
struct GravityYaml
{
  DynamicsParams params;                    ///< links（mass + com=一阶矩）
  std::map<std::string, double> offsets;    ///< 关节名 → 常数偏置（N·m）
  std::string path;                         ///< 实际读取的文件
  int schema{0};
  std::size_t links{0};
  std::size_t mass_unknown{0};              ///< mass=null（按 1.0 处理）的连杆数
};

/// 解析一个 `inertia_params.yaml`。
///
/// @param path        文件路径
/// @param out         结果（成功时填好 params.links / offsets）
/// @param error       失败原因（**总是**写清，调用方要打 WARN）
bool loadGravityYaml(const std::string & path, GravityYaml & out, std::string & error);

/// 按优先级找一个可用的 `inertia_params.yaml`：
///   ① 参数 `gravity_model_yaml`（非空则**只用**它，找不到就失败，不静默回退）
///   ② `<install 前缀>/share/lite_controller/config/gravity/inertia_params.yaml`
///   ③ 源码树 `src/lite_controller/config/gravity/inertia_params.yaml`
///      （从当前工作目录与可执行文件所在目录各自向上找）
/// @return 找到的路径；空串 = 都没找到（`tried` 里是试过的路径，用于 WARN）
std::string resolveGravityYaml(const std::string & explicit_path,
                               std::vector<std::string> & tried);

}  // namespace lite_move_controller
