#pragma once
#include "five_dof_arm/kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <stdexcept>
#include <vector>

namespace five_dof_arm
{
struct Config
{
  std::array<uint8_t, 5> ids;
  Positions lower, upper, initial;
  explicit Config(rclcpp::Node & node)
  {
    const auto id = node.declare_parameter<std::vector<int64_t>>("motor_ids", {1, 2, 3, 4, 5});
    const auto lo = node.declare_parameter<std::vector<int64_t>>("position_min", {0, 0, 0, 0, 0});
    const auto hi = node.declare_parameter<std::vector<int64_t>>("position_max", {4095, 4095, 4095, 4095, 4095});
    const auto init = node.declare_parameter<std::vector<int64_t>>(
      "initial_position", {2048, 2160, 1910, 1360, 2048});
    if (id.size() != 5 || lo.size() != 5 || hi.size() != 5 || init.size() != 5) {
      throw std::invalid_argument("motor_ids and position arrays must contain exactly five values");
    }
    std::set<int64_t> unique;
    for (size_t i = 0; i < 5; ++i) {
      if (id[i] < 0 || id[i] > 252 || !unique.insert(id[i]).second ||
        lo[i] < 0 || hi[i] > 4095 || lo[i] > hi[i] || init[i] < lo[i] || init[i] > hi[i])
      {
        throw std::invalid_argument("Invalid/duplicate motor IDs or single-turn position limits");
      }
      ids[i] = static_cast<uint8_t>(id[i]);
      lower[i] = static_cast<int32_t>(lo[i]); upper[i] = static_cast<int32_t>(hi[i]);
      initial[i] = static_cast<int32_t>(init[i]);
    }
  }
  bool valid(const Positions & p) const
  {
    for (size_t i = 0; i < 5; ++i) {
      if (p[i] < lower[i] || p[i] > upper[i]) {return false;}
    }
    return true;
  }
};
}  // namespace five_dof_arm
