// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// gravity_yaml.cpp —— 见 gravity_yaml.hpp 的设计说明。
#include "lite_move_controller/gravity_yaml.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace lite_move_controller
{
namespace
{

/// 从 `path` 向上最多 6 层找 `src/lite_controller/config/gravity/inertia_params.yaml`。
std::string findInSourceTree(const std::filesystem::path & start)
{
  std::error_code ec;
  std::filesystem::path p = std::filesystem::absolute(start, ec);
  if (ec) {return {};}
  for (int i = 0; i < 6; ++i) {
    const auto cand = p / "src" / "lite_controller" / "config" / "gravity" /
                      "inertia_params.yaml";
    if (std::filesystem::exists(cand, ec) && !ec) {return cand.string();}
    if (!p.has_parent_path() || p.parent_path() == p) {break;}
    p = p.parent_path();
  }
  return {};
}

}  // namespace

bool loadGravityYaml(const std::string & path, GravityYaml & out, std::string & error)
{
  out = GravityYaml{};
  out.path = path;
  error.clear();
  if (path.empty()) {error = "路径为空"; return false;}

  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    error = std::string("YAML 解析失败（") + e.what() + "）";
    return false;
  }
  if (!root.IsMap()) {error = "根节点不是 map"; return false;}

  // ⚠️ schema 必须是 1：schema 2 是 lite_dyid 的 params.json（有完整惯量），
  //    走另一条通路（params_json 优先）。混用会让"有没有惯量"这件事说不清。
  out.schema = root["schema"] ? root["schema"].as<int>(0) : 0;
  if (out.schema != 1) {
    error = "schema=" + std::to_string(out.schema) +
            "（本文件要求 1；schema 2 请走 params_json 通路）";
    return false;
  }
  const auto links = root["links"];
  if (!links || !links.IsMap()) {error = "缺少 links 段"; return false;}

  out.params.schema = 1;
  // ⚠️ yaml 只有质量与一阶矩（惯量张量在回归基里恒为 0）⇒ **不可**用于质量矩阵。
  //    这与 params.json 的 `usable_for_mass_matrix` 语义一致，都是 false。
  out.params.usable_for_mass_matrix = false;
  out.params.model_kind = "gravity_first_moment";

  for (const auto & kv : links) {
    const std::string name = kv.first.as<std::string>();
    const auto node = kv.second;
    if (!node.IsMap()) {continue;}
    const auto fm = node["first_moment"];
    if (!fm || !fm.IsSequence() || fm.size() != 3) {
      error = "links." + name + ".first_moment 缺失或不是 3 个数";
      return false;
    }
    LinkDynamics d;
    d.name = name;
    for (std::size_t i = 0; i < 3; ++i) {
      d.com[i] = fm[i].as<double>();
      if (!std::isfinite(d.com[i])) {
        error = "links." + name + ".first_moment[" + std::to_string(i) + "] 不是有限数";
        return false;
      }
    }
    // mass: 可为 null（不可辨识）。null ⇒ 与 GravityCompensator 一致：mass=1.0、
    // com=first_moment（只保证乘积正确；τ_g 只依赖乘积）。
    const auto ms = node["mass"];
    if (ms && !ms.IsNull()) {
      d.mass = ms.as<double>();
      if (!std::isfinite(d.mass) || d.mass <= 0.0) {
        error = "links." + name + ".mass 非法（" + std::to_string(d.mass) + "）";
        return false;
      }
      d.mass_identified = true;
    } else {
      d.mass = 1.0;
      d.mass_identified = false;
      ++out.mass_unknown;
    }
    const auto idf = node["identified"];
    if (idf && idf.IsSequence()) {
      for (const auto & s : idf) {d.identified.push_back(s.as<std::string>());}
    }
    d.has_inertia = false;       // 重心回归不含惯量张量（见头文件）
    out.params.links[name] = d;
    ++out.links;
  }
  if (out.links == 0) {error = "links 段里没有任何可用连杆"; return false;}

  const auto offs = root["offsets_nm"];
  if (offs && offs.IsMap()) {
    for (const auto & kv : offs) {
      const double v = kv.second.as<double>();
      if (!std::isfinite(v)) {
        error = "offsets_nm." + kv.first.as<std::string>() + " 不是有限数";
        return false;
      }
      out.offsets[kv.first.as<std::string>()] = v;
    }
  }
  return true;
}

std::string resolveGravityYaml(const std::string & explicit_path,
                               std::vector<std::string> & tried)
{
  tried.clear();
  std::error_code ec;

  // ① 显式参数：**只用它**（给了却找不到 = 配置错，必须让调用方看到 WARN，
  //    而不是悄悄换成别的文件 —— 那样"用的哪份标定"就说不清了）。
  if (!explicit_path.empty()) {
    tried.push_back(explicit_path + "（参数 gravity_model_yaml）");
    if (std::filesystem::exists(explicit_path, ec) && !ec) {return explicit_path;}
    return {};
  }

  // ② 安装空间：<prefix>/share/lite_controller/config/gravity/inertia_params.yaml
  //    从可执行文件路径（.../install/lite_move_controller/lib/lite_move_controller/）
  //    向上找 install 前缀。
  try {
    const auto exe = std::filesystem::read_symlink("/proc/self/exe", ec);
    if (!ec) {
      std::filesystem::path p = exe.parent_path();
      for (int i = 0; i < 8; ++i) {
        const auto cand = p / "share" / "lite_controller" / "config" / "gravity" /
                          "inertia_params.yaml";
        tried.push_back(cand.string());
        if (std::filesystem::exists(cand, ec) && !ec) {return cand.string();}
        if (!p.has_parent_path() || p.parent_path() == p) {break;}
        p = p.parent_path();
      }
    }
  } catch (const std::exception &) {
    // 忽略：下面的源码树回退还会试
  }

  // ③ 源码树回退（从当前目录 / 可执行文件目录各自向上找）
  std::vector<std::filesystem::path> starts;
  if (std::filesystem::exists(std::filesystem::current_path(ec), ec) && !ec) {
    starts.push_back(std::filesystem::current_path());
  }
  const auto exe2 = std::filesystem::read_symlink("/proc/self/exe", ec);
  if (!ec) {starts.push_back(exe2.parent_path());}
  for (const auto & s : starts) {
    const auto found = findInSourceTree(s);
    if (!found.empty()) {
      tried.push_back(found);
      return found;
    }
    tried.push_back((s / "src/lite_controller/config/gravity/inertia_params.yaml").string());
  }
  return {};
}

}  // namespace lite_move_controller
