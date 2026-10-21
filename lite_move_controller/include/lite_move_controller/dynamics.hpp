// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// dynamics.hpp —— 从 `lite_dyid` 的 `params.json`（schema 2）读动力学参数，
//                算前馈力矩（重力 + 摩擦）与外力估计
//
// ===========================================================================
// 为什么本层要自己读 JSON（而不是 import lite_dyid 的代码）
// ===========================================================================
//
// 用户要求："本包只依赖于 lite_dyid 的**产物**，但是不要对代码产生依赖关系；
//            有关动力学参数的选择默认选择当前 lite_dyid 输出目录中结果。"
//
// 所以本文件直接读 `params.json`（schema 2），**不** import 它的 Python 模块、
// 不在 CMake 里依赖它。schema 2 的语义（由 lite_dyid 保证）：
//   · `links.<name>.mass` 与 `.com`：**必用**（重力矩 τ_g = Σ Jᵀ(m·g) 全靠它）
//   · `links.<name>.I`：静态+摩擦模型下**全为 0**（未辨识），此时
//     `usable_for_mass_matrix == false` ⇒ 本层**拒绝**用它算 M(q)，
//     导纳的惯性整形/阻抗的 D 推导自动退化为名义质量
//   · `gravity.offsets_nm` 不在 schema 2 顶层 —— 见下方 `offsets` 说明
//
// ===========================================================================
// 三条安全规则
// ===========================================================================
//
// ① **模型与参数必须同源**：`params.json` 里有 `robot.urdf_sha256`。
//    本层把它与当前 URDF 的 sha256 对拍，**不一致就拒绝启动**（fail-closed）——
//    用错模型的症状是"力矩方向对但大小不对"，实测极难发现。
//
// ② **质量不可辨识（`mass: null`）取 1.0、com = 一阶矩/1.0** —— 与
//    `lite_controller` 的 GravityCompensator 同一条铁律（成对使用才有物理意义）。
//
// ③ **逐关节力矩钳位**：MIT 通路驱动只夹到型号峰值（rs-00 17 N·m，
//    而本机额定只有 7 N·m）。保守上限必须本层给。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace lite_move_controller
{

/// 单个连杆的辨识参数。
struct LinkDynamics
{
  std::string name;
  /// `false` = `mass: null`（该连杆质量不可辨识）。此时本层按
  /// `mass = 1.0`、`com = first_moment / 1.0` 使用（与 GravityCompensator 一致）。
  bool mass_identified{false};
  double mass{0.0};
  double com[3]{0.0, 0.0, 0.0};
  /// 3×3 惯量张量（**连杆坐标系**）。`has_inertia == false` 时全 0。
  double inertia[3][3]{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  bool has_inertia{false};
  /// 哪些分量是数据真正定出来的（其余是正则/边界给的）
  std::vector<std::string> identified;
};

/// 摩擦参数（库仑分速度符号 + 粘滞）。
struct FrictionDynamics
{
  std::vector<double> c1_pos, c1_neg, c2;
  bool available{false};
};

/// schema 2 的 `params.json` 内容（本层用到的部分）。
struct DynamicsParams
{
  int schema{0};
  std::string model_kind;                 ///< static_friction_only / static_friction_inertia
  bool usable_for_mass_matrix{false};     ///< 只有它 true 才允许算 M(q)
  std::string urdf_sha256;                ///< 与当前 URDF 对拍
  std::vector<std::string> joints;        ///< canonical 顺序
  std::map<std::string, LinkDynamics> links;
  std::vector<double> gravity_params;     ///< gravity.params（4·n_links 的自在参数）
  /// `inertia.raw_fit.pi_I`：**6 个惯量分量 × n_links**（pinocchio 的
  /// `computeJointTorqueRegressor` 的 6 个惯量列对应的系数）。
  ///
  /// ⚠️ 顺序是 `[Ixx, Ixy, Iyy, Ixz, Iyz, Izz]`（pinocchio 的
  ///    `toDynamicParameters()` 约定），**每连杆 6 个连续**。
  /// ⚠️ 数值秩亏（本机 108 维里秩 62）⇒ 逐分量**不可单独解读**；
  ///    必须**整体写回模型**后用 rnea 使用（params.json 的 note 也这么写）。
  std::vector<double> inertia_dynamic;

  /// 连杆名 → 该关节轴在 **URDF link frame** 下的表示（单位向量）。
  ///
  /// ⚠️ 为什么需要它：KDL 建链时把关节轴旋转到 Z，segment frame 与 URDF
  ///    link frame 差一个固定旋转 `R_ls`；而辨识出的**一阶矩**是在 link frame
  ///    下表达的。要写进 KDL 模型就必须先转过去，而构造 `R_ls` 需要轴向量。
  ///    缺失时退化为单位阵（并计数），**不假装知道**。
  std::map<std::string, std::array<double, 3>> axis_of_link;
  FrictionDynamics friction;
  std::string source_path;
};

/// 极简 JSON 读取（只支持本文件需要的形状：对象/数组/数字/字符串/bool/null）。
///
/// ⚠️ 为什么自己写而不是引入 nlohmann/json：本层的**核心**是安全关键逻辑，
///    必须能 `g++ -I include test.cpp` 裸跑单测。引入一个 header-only 的
///    第三方 JSON 库会让"裸跑"变成"先装依赖"，而这里只需要读一个**自家格式**
///    的文件（字段名与层级由 lite_dyid 的 schema 2 固定）。
///    代价：不支持注释/科学计数法的全部变体/转义 —— 对本用途足够。
class MiniJson
{
public:
  enum class Kind { Null, Bool, Number, String, Array, Object };

  Kind kind{Kind::Null};
  bool b{false};
  double num{0.0};
  std::string str;
  std::vector<MiniJson> arr;
  std::map<std::string, MiniJson> obj;

  bool isObject() const noexcept {return kind == Kind::Object;}
  bool isArray() const noexcept {return kind == Kind::Array;}
  bool isNumber() const noexcept {return kind == Kind::Number;}
  bool isNull() const noexcept {return kind == Kind::Null;}

  /// 取对象成员；不存在返回 nullptr。
  const MiniJson * find(const std::string & key) const noexcept
  {
    if (kind != Kind::Object) {return nullptr;}
    const auto it = obj.find(key);
    return (it == obj.end()) ? nullptr : &it->second;
  }

  /// 数值（非数值返回 fallback）。
  double asNumber(double fallback = 0.0) const noexcept
  {
    return (kind == Kind::Number) ? num : fallback;
  }
  std::string asString(const std::string & fallback = "") const noexcept
  {
    return (kind == Kind::String) ? str : fallback;
  }
  bool asBool(bool fallback = false) const noexcept
  {
    return (kind == Kind::Bool) ? b : fallback;
  }

  /// 解析。失败返回 false 并给出原因。
  static bool parse(const std::string & text, MiniJson & out, std::string & error);

  /// 从文件读并解析。
  static bool parseFile(const std::string & path, MiniJson & out, std::string & error);

private:
  struct Parser
  {
    const std::string & s;
    std::size_t i{0};
    std::string err;

    explicit Parser(const std::string & text) : s(text) {}

    void skipWs()
    {
      while (i < s.size()) {
        const char c = s[i];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {++i; continue;}
        // 支持 // 行注释（便于调试；schema 2 本身不含注释）
        if (c == '/' && i + 1 < s.size() && s[i + 1] == '/') {
          while (i < s.size() && s[i] != '\n') {++i;}
          continue;
        }
        break;
      }
    }
    bool fail(const std::string & why)
    {
      if (err.empty()) {err = why + "（位置 " + std::to_string(i) + "）";}
      return false;
    }
    bool parseValue(MiniJson & v)
    {
      skipWs();
      if (i >= s.size()) {return fail("意外的文件结尾");}
      const char c = s[i];
      if (c == '{') {return parseObject(v);}
      if (c == '[') {return parseArray(v);}
      if (c == '"') {
        v.kind = Kind::String;
        return parseString(v.str);
      }
      if (c == 't' || c == 'f') {return parseBool(v);}
      if (c == 'n') {return parseNull(v);}
      // ⚠️ Python 的 `json.dump(..., allow_nan=True)`（**默认**）会把
      //    `float('inf')` / `nan` 写成裸的 `Infinity` / `NaN` —— 那**不是合法
      //    JSON**，但 lite_dyid 的输出里确实会有它（实测：`gravity.cond = Infinity`，
      //    因为奇异矩阵的条件数在数值上就是无穷）。
      //    严格解析器会因此**拒绝整个文件**，症状是"参数明明是好的却加载不上"。
      //    这里显式接受三个非标准 token，并**保留非有限语义**（不悄悄变成 0）：
      //    调用方对**用到的**字段（mass/com/I）另有有限性检查。
      if (s.compare(i, 8, "Infinity") == 0) {v.kind = Kind::Number; v.num = INFINITY; i += 8; return true;}
      if (s.compare(i, 9, "-Infinity") == 0) {v.kind = Kind::Number; v.num = -INFINITY; i += 9; return true;}
      if (s.compare(i, 3, "NaN") == 0) {v.kind = Kind::Number; v.num = NAN; i += 3; return true;}
      return parseNumber(v);
    }
    bool parseObject(MiniJson & v)
    {
      v.kind = Kind::Object;
      ++i;                                  // '{'
      skipWs();
      if (i < s.size() && s[i] == '}') {++i; return true;}
      while (true) {
        skipWs();
        std::string key;
        if (!parseString(key)) {return false;}
        skipWs();
        if (i >= s.size() || s[i] != ':') {return fail("对象成员缺少 ':'");}
        ++i;
        MiniJson child;
        if (!parseValue(child)) {return false;}
        v.obj[key] = std::move(child);
        skipWs();
        if (i < s.size() && s[i] == ',') {++i; continue;}
        if (i < s.size() && s[i] == '}') {++i; return true;}
        return fail("对象缺少 ',' 或 '}'");
      }
    }
    bool parseArray(MiniJson & v)
    {
      v.kind = Kind::Array;
      ++i;                                  // '['
      skipWs();
      if (i < s.size() && s[i] == ']') {++i; return true;}
      while (true) {
        MiniJson child;
        if (!parseValue(child)) {return false;}
        v.arr.push_back(std::move(child));
        skipWs();
        if (i < s.size() && s[i] == ',') {++i; continue;}
        if (i < s.size() && s[i] == ']') {++i; return true;}
        return fail("数组缺少 ',' 或 ']'");
      }
    }
    bool parseString(std::string & out)
    {
      skipWs();
      if (i >= s.size() || s[i] != '"') {return fail("期望字符串");}
      ++i;
      out.clear();
      while (i < s.size()) {
        const char c = s[i++];
        if (c == '"') {return true;}
        if (c == '\\') {
          if (i >= s.size()) {return fail("字符串转义不完整");}
          const char e = s[i++];
          switch (e) {
            case 'n': out.push_back('\n'); break;
            case 't': out.push_back('\t'); break;
            case 'r': out.push_back('\r'); break;
            case '"': out.push_back('"'); break;
            case '\\': out.push_back('\\'); break;
            case '/': out.push_back('/'); break;
            default: out.push_back(e); break;   // \uXXXX 不完整支持（本用途不需要）
          }
          continue;
        }
        out.push_back(c);
      }
      return fail("字符串没有结束引号");
    }
    bool parseBool(MiniJson & v)
    {
      if (s.compare(i, 4, "true") == 0) {v.kind = Kind::Bool; v.b = true; i += 4; return true;}
      if (s.compare(i, 5, "false") == 0) {v.kind = Kind::Bool; v.b = false; i += 5; return true;}
      return fail("期望 true/false");
    }
    bool parseNull(MiniJson & v)
    {
      if (s.compare(i, 4, "null") == 0) {v.kind = Kind::Null; i += 4; return true;}
      return fail("期望 null");
    }
    bool parseNumber(MiniJson & v)
    {
      const std::size_t start = i;
      if (i < s.size() && (s[i] == '-' || s[i] == '+')) {++i;}
      while (i < s.size() &&
             (std::isdigit(static_cast<unsigned char>(s[i])) || s[i] == '.' ||
              s[i] == 'e' || s[i] == 'E' || s[i] == '-' || s[i] == '+')) {
        ++i;
      }
      if (i == start) {return fail("期望数值");}
      try {
        v.kind = Kind::Number;
        v.num = std::stod(s.substr(start, i - start));
        return true;
      } catch (...) {
        return fail("数值解析失败");
      }
    }
  };
};

inline bool MiniJson::parse(const std::string & text, MiniJson & out, std::string & error)
{
  Parser p(text);
  if (!p.parseValue(out)) {error = p.err.empty() ? "JSON 解析失败" : p.err; return false;}
  return true;
}

inline bool MiniJson::parseFile(const std::string & path, MiniJson & out, std::string & error)
{
  std::ifstream f(path);
  if (!f) {error = "打不开文件：" + path; return false;}
  std::stringstream ss;
  ss << f.rdbuf();
  return parse(ss.str(), out, error);
}

/// 从 schema 2 的 JSON 抽出动力学参数（纯逻辑，可单测）。
inline bool parseDynamicsParams(const MiniJson & root, DynamicsParams & out,
                                std::string & error)
{
  // ⚠️⚠️ **入口必须清空 `out`**：本函数是"追加式"填充（push_back 各数组）。
  //    若调用方复用一个对象（例如"同一个 params.json 给两条臂各解析一次"），
  //    第二次会把 `joints` 追加成 28 个 —— 于是下游的
  //    "joints 数 != canonical 数" 检查**误报**，右臂的前馈被静默关掉。
  //    实测踩过：左臂 14、右臂 28。
  //    修法有两个：入口清空（这里）**以及**调用方不复用（两边都做，双保险）。
  out = DynamicsParams{};
  if (!root.isObject()) {error = "params.json 根不是对象"; return false;}
  out.schema = static_cast<int>(root.find("schema") ? root.find("schema")->asNumber(0) : 0);
  if (out.schema != 2) {
    // ⚠️ schema 1 没有 inertia/robot 段 ⇒ 无法做"模型同源"校验，
    //    也拿不到 `usable_for_mass_matrix`。**拒绝**而不是"尽力而为"。
    error = "params.json 的 schema=" + std::to_string(out.schema) +
            "（本层要求 schema 2；请用新版 lite_dyid 重新输出）";
    return false;
  }
  if (const auto * src = root.find("source")) {
    out.model_kind = src->find("model") ? src->find("model")->asString() : "";
  }
  if (const auto * rob = root.find("robot")) {
    out.urdf_sha256 = rob->find("urdf_sha256") ? rob->find("urdf_sha256")->asString() : "";
    if (const auto * js = rob->find("joints")) {
      for (const auto & j : js->arr) {out.joints.push_back(j.asString());}
    }
  }
  const auto * inertia = root.find("inertia");
  if (inertia == nullptr) {error = "params.json 缺少 inertia 段（schema 2 必有）"; return false;}
  out.usable_for_mass_matrix =
      inertia->find("usable_for_mass_matrix") &&
      inertia->find("usable_for_mass_matrix")->asBool(false);
  const auto * links = inertia->find("links");
  if (links == nullptr || !links->isObject()) {
    error = "params.json 的 inertia.links 缺失或不是对象";
    return false;
  }
  for (const auto & kv : links->obj) {
    const MiniJson & ln = kv.second;
    LinkDynamics d;
    d.name = kv.first;
    const auto * mass = ln.find("mass");
    if (mass == nullptr || mass->isNull()) {
      // 质量不可辨识：取 mass=1.0、com=一阶矩/1.0（与 GravityCompensator 同一条铁律）
      d.mass_identified = false;
      d.mass = 1.0;
    } else {
      d.mass_identified = true;
      d.mass = mass->asNumber(0.0);
    }
    if (const auto * com = ln.find("com")) {
      for (std::size_t k = 0; k < 3 && k < com->arr.size(); ++k) {
        d.com[k] = com->arr[k].asNumber(0.0);
      }
    }
    if (const auto * ident = ln.find("identified")) {
      for (const auto & s : ident->arr) {d.identified.push_back(s.asString());}
    }
    if (const auto * I = ln.find("I")) {
      bool nonzero = false;
      for (std::size_t r = 0; r < 3 && r < I->arr.size(); ++r) {
        for (std::size_t c = 0; c < 3 && c < I->arr[r].arr.size(); ++c) {
          d.inertia[r][c] = I->arr[r].arr[c].asNumber(0.0);
          if (d.inertia[r][c] != 0.0) {nonzero = true;}
        }
      }
      d.has_inertia = nonzero;
    }
    out.links[d.name] = std::move(d);
  }
  // pi_I：6 × n_links 的惯量系数
  if (inertia != nullptr) {
    if (const auto * rf = inertia->find("raw_fit")) {
      if (const auto * pI = rf->find("pi_I")) {
        for (const auto & x : pI->arr) {out.inertia_dynamic.push_back(x.asNumber(0.0));}
      }
    }
  }
  if (const auto * g = root.find("gravity")) {
    if (const auto * p = g->find("params")) {
      for (const auto & x : p->arr) {out.gravity_params.push_back(x.asNumber(0.0));}
    }
  }
  if (const auto * fr = root.find("friction")) {
    auto fill = [&fr](const char * key, std::vector<double> & dst) {
      const auto * a = fr->find(key);
      if (a == nullptr) {return;}
      for (const auto & x : a->arr) {dst.push_back(x.asNumber(0.0));}
    };
    fill("c1_pos", out.friction.c1_pos);
    fill("c1_neg", out.friction.c1_neg);
    fill("c2", out.friction.c2);
    out.friction.available = !out.friction.c1_pos.empty() &&
                             out.friction.c1_pos.size() == out.friction.c1_neg.size() &&
                             out.friction.c1_pos.size() == out.friction.c2.size();
  }
  if (out.links.empty()) {error = "params.json 里没有任何连杆参数"; return false;}
  return true;
}

/// 库仑 + 粘滞摩擦前馈。
///
/// ===========================================================================
/// 形式推导（这次推对了，前面错了四次，把结论钉在这里）
/// ===========================================================================
///
/// 库仑项是**多值**的：`τ_c = c1_pos`（v>0）/ `c1_neg`（v<0），在 v=0 处不连续。
/// 要用一个解析式平滑它，标准做法是把"符号"和"幅值"分开：
///
///     s        = tanh(v/eps)                              （奇的符号）
///     c1(v)    = (c1_pos + c1_neg)/2
///                + (c1_pos − c1_neg)/2 · s                 （**偶**的幅值）
///     τ        = c1(v) · s + c2·v
///              = mid·s + half·s² + c2·v
///
/// **奇偶性检查**（这是关键，前面四次都栽在这里）：
///     τ(−v) + τ(v) = [mid·(−s) + half·s²] + [mid·s + half·s²]
///                  = 2·half·s² = (c1_pos − c1_neg)·s²
///
/// ⇒ **只有当 `c1_pos == c1_neg` 时 τ 才严格奇对称**。
///    两侧不对称时必然带一个**偶的** `half·s²` 项（它代表"死区幅值随速度方向
///    从一侧过渡到另一侧"），这是模型的**固有性质**，不是 bug。
///
/// 所以本函数**如实声明**两条性质：
///   ✅ `τ(0) == 0`（无偏置）
///   ✅ `|v|` 连续、`|τ|` 随 `|v|` 单调不减（tanh 平滑，无极限环）
///   ⚠️ `τ(−v) == −τ(v)` **仅当 `|c1_pos| == |c1_neg|`**；不对称时偏差为
///      `(c1_pos − c1_neg)·s²`，量级 `≤ |c1_pos − c1_neg|`（本机辨识值
///      0.01~0.03 N·m 量级，远小于静摩擦本身）。
///      **调用方若需要严格奇对称**，请先把两侧系数对称化（取平均或取较大者）——
///      把选择权留给调用方，而不是在这里偷偷改掉辨识结果。
inline double frictionTorque(double c1_pos, double c1_neg, double c2, double v,
                             double eps = 0.01) noexcept
{
  if (!std::isfinite(v)) {return 0.0;}
  const double e = (eps > 1e-9) ? eps : 1e-9;
  const double s = std::tanh(v / e);                 // 奇
  const double mid = 0.5 * (c1_pos + c1_neg);
  const double half = 0.5 * (c1_pos - c1_neg);
  return (mid + half * s) * s + c2 * v;
}

/// `frictionTorque` 的**严格奇对称**版本：两侧系数先对称化。
///
/// ⚠️ 什么时候该用它：需要"推它一下、它不会持续往一个方向漂"的场合
///    （零力拖动、导纳模式）。代价是丢掉辨识出的不对称性
///    （那份不对称里含重力/装配偏置，未必都是摩擦）。
///
/// `mode`：0 = 取平均（保守，推荐）/ 1 = 取绝对值较大者（更"顶得住"）
inline double frictionTorqueSymmetric(double c1_pos, double c1_neg, double c2,
                                      double v, double eps = 0.01,
                                      int mode = 0) noexcept
{
  const double c1 = (mode == 0)
                    ? 0.5 * (c1_pos + c1_neg)
                    : ((std::fabs(c1_pos) >= std::fabs(c1_neg)) ? c1_pos : c1_neg);
  return frictionTorque(c1, c1, c2, v, eps);
}

/// 逐关节钳位（对 |τ| 夹到 limit）。
inline void clampTorque(std::vector<double> & tau, const std::vector<double> & limit) noexcept
{
  const std::size_t n = std::min(tau.size(), limit.size());
  for (std::size_t i = 0; i < n; ++i) {
    if (!std::isfinite(tau[i])) {tau[i] = 0.0; continue;}
    const double l = std::abs(limit[i]);
    if (tau[i] > l) {tau[i] = l;}
    else if (tau[i] < -l) {tau[i] = -l;}
  }
}

/// 用简单字符串哈希做 URDF 一致性提示（**不追求 sha256**：见下方说明）。
///
/// ⚠️ 为什么不用 sha256：本层要保持"零第三方依赖的纯逻辑核心"，
///    而 sha256 需要一个实现（~100 行）或 OpenSSL。这里用一个
///    **足够长的 FNV-1a 64 位**：它的用途只是"**发现**模型被换过"，
///    不是防篡改。发现不一致就拒绝启动，这个强度足够。
inline std::uint64_t fnv1a64(const std::string & s) noexcept
{
  std::uint64_t h = 1469598103934665603ULL;
  for (unsigned char c : s) {
    h ^= static_cast<std::uint64_t>(c);
    h *= 1099511628211ULL;
  }
  return h;
}

}  // namespace lite_move_controller
