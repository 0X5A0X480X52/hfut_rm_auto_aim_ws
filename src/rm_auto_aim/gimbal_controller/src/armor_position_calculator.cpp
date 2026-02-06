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

namespace gimbal_controller
{

std::vector<Eigen::Vector3d> ArmorPositionCalculator::calculate(
  const rm_interfaces::msg::TrackedRobot & robot) const
{
  Eigen::Vector3d center(
    robot.center_position.x,
    robot.center_position.y,
    robot.center_position.z);

  std::vector<Eigen::Vector3d> positions;

  // 如果有自定义的装甲板偏移，使用它们
  if (!robot.armors_offset.empty()) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("ArmorPositionCalculator"),
      "Using armors_offset from TrackedRobot (size=%zu, num_armors=%d)",
      robot.armors_offset.size(), robot.num_armors);
    
    positions.reserve(robot.armors_offset.size());
    for (const auto & pose : robot.armors_offset) {
      Eigen::Vector3d offset(pose.position.x, pose.position.y, pose.position.z);
      positions.push_back(transformToWorld(center, robot.yaw, offset));
    }
  } else {
    // 否则使用默认几何生成
    RCLCPP_DEBUG(
      rclcpp::get_logger("ArmorPositionCalculator"),
      "armors_offset empty, using generateDefaultOffsets (num_armors=%d)",
      robot.num_armors);
    
    auto offsets = generateDefaultOffsets(
      robot.robot_type,
      robot.num_armors,
      robot.radius,
      robot.radius_2,
      robot.d_za,
      robot.d_zc);

    positions.reserve(offsets.size());
    for (const auto & offset : offsets) {
      positions.push_back(transformToWorld(center, robot.yaw, offset));
    }
  }

  return positions;
}

std::vector<Eigen::Vector3d> ArmorPositionCalculator::calculatePredicted(
  const rm_interfaces::msg::TrackedRobot & robot,
  double dt) const
{
  // 预测机器人中心位置
  Eigen::Vector3d predicted_center(
    robot.center_position.x + dt * robot.center_velocity.x,
    robot.center_position.y + dt * robot.center_velocity.y,
    robot.center_position.z + dt * robot.center_velocity.z);

  // 预测机器人yaw角
  double predicted_yaw = robot.yaw + dt * robot.yaw_velocity;

  std::vector<Eigen::Vector3d> positions;

  if (!robot.armors_offset.empty()) {
    positions.reserve(robot.armors_offset.size());
    for (const auto & pose : robot.armors_offset) {
      Eigen::Vector3d offset(pose.position.x, pose.position.y, pose.position.z);
      positions.push_back(transformToWorld(predicted_center, predicted_yaw, offset));
    }
  } else {
    auto offsets = generateDefaultOffsets(
      robot.robot_type,
      robot.num_armors,
      robot.radius,
      robot.radius_2,
      robot.d_za,
      robot.d_zc);

    positions.reserve(offsets.size());
    for (const auto & offset : offsets) {
      positions.push_back(transformToWorld(predicted_center, predicted_yaw, offset));
    }
  }

  return positions;
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

  bool is_current_pair = true;
  double r = 0., target_dz = 0.;

  for (int i = 0; i < num_armors; i++) {
    // 装甲板在机器人坐标系中的角度 (相对于机器人正前方)
    double angle = i * (2 * M_PI / num_armors);

    if (num_armors == 4) {
      r = is_current_pair ? radius : radius_2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = radius;
      target_dz = d_zc;
    }

    // 装甲板在机器人坐标系中的偏移
    // 注意: 这里假设机器人正前方是 x 轴正方向
    Eigen::Vector3d offset(-r * std::cos(angle), -r * std::sin(angle), target_dz);
    offsets.push_back(offset);
  }

  return offsets;
}

Eigen::Vector3d ArmorPositionCalculator::transformToWorld(
  const Eigen::Vector3d & center,
  double yaw,
  const Eigen::Vector3d & offset) const
{
  // 2D 旋转矩阵 (绕 z 轴)
  Eigen::Matrix3d rotation;
  rotation << std::cos(yaw), -std::sin(yaw), 0,
              std::sin(yaw),  std::cos(yaw), 0,
              0,              0,             1;

  return center + rotation * offset;
}

}  // namespace gimbal_controller
