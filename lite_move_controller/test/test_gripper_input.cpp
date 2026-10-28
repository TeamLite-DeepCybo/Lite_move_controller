#include "lite_move_controller/gripper_input.hpp"

#include <cstdlib>
#include <iostream>
#include <limits>

using namespace lite_move_controller;

int main()
{
  const std::vector<std::string> allowed{"left_gripper", "right_gripper"};
  std::vector<std::size_t> indices;
  std::string error;
  int checks = 0;
  auto check = [&](bool value) {
      ++checks;
      if (!value) {std::cerr << "check " << checks << " failed\n"; std::exit(1);}
    };
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  check(validateGripperConfiguration(allowed, {0, 0}, {0.047, 0.047}, error));
  check(!validateGripperConfiguration(allowed, {0}, {1, 1}, error));
  check(!validateGripperConfiguration({"left", "left"}, {0, 0}, {1, 1}, error));
  check(!validateGripperConfiguration({""}, {0}, {1}, error));
  check(!validateGripperConfiguration({"left"}, {1}, {0}, error));
  check(!validateGripperConfiguration({"left"}, {0}, {inf}, error));
  check(validateGripperBatch({"right_gripper", "left_gripper"}, {0, 1}, allowed,
                            {0, 0}, {1, 1}, indices, error));
  check(indices == std::vector<std::size_t>({1, 0}));
  check(validateGripperBatch({"right_gripper"}, {0.023}, allowed,
                            {0, 0}, {0.047, 0.047}, indices, error));
  check(indices == std::vector<std::size_t>({1}));
  check(validateGripperBatch({"right_gripper"}, {0.02}, {"right_gripper"},
                            {0}, {0.047}, indices, error));
  check(indices == std::vector<std::size_t>({0}));
  for (const auto & positions : std::vector<std::vector<double>>{
      {0.2, nan}, {0.2, inf}, {0.2, -0.1}, {0.2, 1.1}, {0.2}, {0.2, 0.3, 0.4}}) {
    check(!validateGripperBatch(allowed, positions, allowed, {0, 0}, {1, 1}, indices, error));
    check(indices.empty() && !error.empty());
  }
  for (const auto & names : std::vector<std::vector<std::string>>{
      {}, {"unknown"}, {"left_gripper", "unknown"}, {"left_gripper", "left_gripper"}}) {
    check(!validateGripperBatch(names, std::vector<double>(names.size(), 0.2), allowed,
                               {0, 0}, {1, 1}, indices, error));
    check(indices.empty() && !error.empty());
  }
  check(!validateGripperBatch(allowed, {0.02, 0.048}, allowed,
                             {0, 0}, {0.047, 0.047}, indices, error));
  check(indices.empty());
  std::cout << checks << " checks, 0 failures\n";
}
