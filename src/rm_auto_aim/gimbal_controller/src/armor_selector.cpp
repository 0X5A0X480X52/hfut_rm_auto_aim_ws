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

#include "gimbal_controller/armor_selector.hpp"
#include <angles/angles.h>

namespace gimbal_controller
{

void ArmorSelector::setParameters(double side_angle, double min_switching_v_yaw)
{
  side_angle_ = side_angle;
  min_switching_v_yaw_ = min_switching_v_yaw;
}

ArmorSelectionResult ArmorSelector::selectByMinMovement(
  const std::vector<Eigen::Vector3d> & armor_positions,
  double current_yaw,
  double current_pitch) const
{
  ArmorSelectionResult result;
  result.selected_index = -1;
  result.gimbal_movement = std::numeric_limits<double>::max();
  result.distance = std::numeric_limits<double>::max();

  if (armor_positions.empty()) {
    return result;
  }

  // 先过滤掉距离最远的装甲板
  auto valid_indices = filterByDistance(armor_positions);

  for (int idx : valid_indices) {
    const auto & pos = armor_positions[idx];

    double yaw, pitch;
    calculateYawPitch(pos, current_yaw, yaw, pitch);

    double yaw_diff = angles::normalize_angle(yaw - current_yaw);
    double pitch_diff = pitch - current_pitch;

    double movement = yaw_diff * yaw_diff + pitch_diff * pitch_diff;
    double distance = pos.norm();

    // 选择云台移动最小的目标
    if (movement < result.gimbal_movement ||
        (std::abs(movement - result.gimbal_movement) < 0.01 && distance < result.distance))
    {
      result.selected_index = idx;
      result.position = pos;
      result.gimbal_movement = movement;
      result.distance = distance;
    }
  }

  return result;
}

int ArmorSelector::selectByDecisionAngle(
  const std::vector<Eigen::Vector3d> & armor_positions,
  const Eigen::Vector3d & target_center,
  double target_yaw,
  double target_v_yaw) const
{
  if (armor_positions.empty()) {
    return -1;
  }

  std::size_t armors_num = armor_positions.size();

  // 车中心与X轴的夹角
  double alpha = std::atan2(target_center.y(), target_center.x());
  // 观测到的装甲板正面与X轴的夹角
  double beta = target_yaw;

  Eigen::Matrix2d R_odom2center;
  Eigen::Matrix2d R_odom2armor;
  R_odom2center << std::cos(alpha), std::sin(alpha),
                   -std::sin(alpha), std::cos(alpha);
  R_odom2armor << std::cos(beta), std::sin(beta),
                  -std::sin(beta), std::cos(beta);

  Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

  // 决策角度
  double decision_angle = -std::asin(R_center2armor(0, 1));

  // 跳板角度阈值
  double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

  // 避免频繁切换
  if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
    theta = 0;
  }

  double temp_angle = decision_angle + M_PI / armors_num - theta;

  if (temp_angle < 0) {
    temp_angle += 2 * M_PI;
  }

  int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
  return selected_id;
}

std::vector<int> ArmorSelector::filterByDistance(
  const std::vector<Eigen::Vector3d> & armor_positions) const
{
  std::vector<int> indices;

  if (armor_positions.size() <= 1) {
    for (size_t i = 0; i < armor_positions.size(); ++i) {
      indices.push_back(static_cast<int>(i));
    }
    return indices;
  }

  // 找到距离最远的装甲板
  double max_dist = 0;
  int max_idx = -1;
  for (size_t i = 0; i < armor_positions.size(); ++i) {
    double dist = armor_positions[i].head<2>().norm();  // 只考虑水平距离
    if (dist > max_dist) {
      max_dist = dist;
      max_idx = static_cast<int>(i);
    }
  }

  // 排除最远的装甲板
  for (size_t i = 0; i < armor_positions.size(); ++i) {
    if (static_cast<int>(i) != max_idx) {
      indices.push_back(static_cast<int>(i));
    }
  }

  return indices;
}

void ArmorSelector::calculateYawPitch(
  const Eigen::Vector3d & target_position,
  double /* current_yaw */,
  double & yaw,
  double & pitch)
{
  double distance_xy = std::sqrt(
    target_position.x() * target_position.x() +
    target_position.y() * target_position.y());

  yaw = std::atan2(target_position.y(), target_position.x());
  pitch = std::atan2(target_position.z(), distance_xy);
}

}  // namespace gimbal_controller
