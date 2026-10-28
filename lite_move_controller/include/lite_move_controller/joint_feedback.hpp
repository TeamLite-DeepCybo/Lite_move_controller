#pragma once
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace lite_move_controller
{
inline bool validJointFeedback(const std::vector<std::string> & names,
  const std::vector<double> & q, const std::vector<double> & v, const std::vector<double> & effort)
{
  if (names.empty() || q.size() != names.size() ||
      (!v.empty() && v.size() != names.size()) ||
      (!effort.empty() && effort.size() != names.size())) {return false;}
  std::set<std::string> unique;
  for (const auto & name : names) {
    if (name.empty() || !unique.insert(name).second) {return false;}
  }
  for (const auto * values : {&q, &v, &effort}) {
    for (double value : *values) {if (!std::isfinite(value)) {return false;}}
  }
  return true;
}
}  // namespace lite_move_controller
