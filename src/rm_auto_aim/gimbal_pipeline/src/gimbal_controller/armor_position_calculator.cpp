// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gimbal_controller/armor_position_calculator.hpp"

#include <rclcpp/rclcpp.hpp>

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

namespace gimbal_controller
{

std::vector<Eigen::Vector3d> ArmorPositionCalculator::calculate(
  const rm_interfaces::msg::TrackedRobot & robot) const
{
  const auto normalized_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(robot);

  if (!normalized_robot.armors_offset.empty()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ArmorPositionCalculator"),
      "Using armors_offset from TrackedRobot (size=%zu, num_armors=%d)",
      normalized_robot.armors_offset.size(), normalized_robot.num_armors);
  } else {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ArmorPositionCalculator"),
      "armors_offset empty, using module fallback (num_armors=%d)",
      normalized_robot.num_armors);
  }

  return fyt::auto_aim::robot_description::TrackedRobotUsage::calculateArmorWorldPositionsEigen(
    normalized_robot,
    0.0,
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY,
    [](const rm_interfaces::msg::TrackedRobot & fallback_robot) {
      return ArmorPositionCalculator::generateDefaultOffsets(
        fallback_robot.robot_type,
        fallback_robot.num_armors,
        fallback_robot.radius,
        fallback_robot.radius_2,
        fallback_robot.d_za,
        fallback_robot.d_zc);
    });
}

std::vector<Eigen::Vector3d> ArmorPositionCalculator::calculatePredicted(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt) const
{
  const auto normalized_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(robot);
  return fyt::auto_aim::robot_description::TrackedRobotUsage::calculateArmorWorldPositionsEigen(
    normalized_robot,
    dt,
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY,
    [](const rm_interfaces::msg::TrackedRobot & fallback_robot) {
      return ArmorPositionCalculator::generateDefaultOffsets(
        fallback_robot.robot_type,
        fallback_robot.num_armors,
        fallback_robot.radius,
        fallback_robot.radius_2,
        fallback_robot.d_za,
        fallback_robot.d_zc);
    });
}

std::vector<Eigen::Vector3d> ArmorPositionCalculator::generateDefaultOffsets(
  uint8_t /* robot_type */,
  int num_armors,
  double radius,
  double radius_2,
  double d_za,
  double d_zc)
{
  std::vector<Eigen::Vector3d> offsets;
  offsets.reserve(static_cast<size_t>(num_armors));

  for (int i = 0; i < num_armors; i++) {

    double panel_angle = i * (2.0 * M_PI / num_armors);

    double r = (i % 2 == 0) ? radius : radius_2;

    double dz = (i % 2 == 0) ? -d_za : d_za;

    Eigen::Vector3d offset(
        r * std::cos(panel_angle),
        r * std::sin(panel_angle),
        dz);

    offsets.push_back(offset);
  }

  return offsets;
}

}  // namespace gimbal_controller
