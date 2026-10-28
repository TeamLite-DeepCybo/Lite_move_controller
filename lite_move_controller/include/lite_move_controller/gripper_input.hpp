#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace lite_move_controller
{
// Validate before changing ANY target. Used by both normalized and metre inputs.
inline bool validateGripperConfiguration(
  const std::vector<std::string> & names,
  const std::vector<double> & lower, const std::vector<double> & upper,
  std::string & error)
{
  error.clear();
  if (names.empty() || lower.size() != names.size() || upper.size() != names.size()) {
    error = "gripper names/limits must be nonempty and have equal lengths";
    return false;
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i].empty() ||
        std::find(names.begin(), names.begin() + i, names[i]) != names.begin() + i ||
        !std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] >= upper[i]) {
      error = "gripper names must be unique; limits must be finite and ordered";
      return false;
    }
  }
  return true;
}

inline bool validateGripperBatch(
  const std::vector<std::string> & names, const std::vector<double> & positions,
  const std::vector<std::string> & allowed,
  const std::vector<double> & lower, const std::vector<double> & upper,
  std::vector<std::size_t> & indices, std::string & error)
{
  indices.clear();
  if (!validateGripperConfiguration(allowed, lower, upper, error)) {return false;}
  if (names.empty() || names.size() != positions.size()) {
    error = "command names/positions must be nonempty and have equal lengths";
    return false;
  }
  std::vector<std::size_t> checked;
  for (std::size_t i = 0; i < names.size(); ++i) {
    const auto found = std::find(allowed.begin(), allowed.end(), names[i]);
    if (found == allowed.end()) {
      error = "unknown gripper: " + names[i];
      return false;
    }
    const auto index = static_cast<std::size_t>(found - allowed.begin());
    if (std::find(checked.begin(), checked.end(), index) != checked.end()) {
      error = "duplicate gripper: " + names[i];
      return false;
    }
    if (!std::isfinite(positions[i]) || positions[i] < lower[index] ||
        positions[i] > upper[index]) {
      error = "nonfinite or out-of-range gripper position: " + names[i];
      return false;
    }
    checked.push_back(index);
  }
  indices = std::move(checked);
  return true;
}
}  // namespace lite_move_controller
