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

#include "gimbal_controller/fire_advisor.hpp"
#include <algorithm>

namespace gimbal_controller
{

void FireAdvisor::setParameters(
  double shooting_range_w,
  double shooting_range_h,
  double min_shooting_angle)
{
  shooting_range_w_ = shooting_range_w;
  shooting_range_h_ = shooting_range_h;
  min_shooting_angle_ = min_shooting_angle;
}

bool FireAdvisor::shouldFire(
  double current_yaw,
  double current_pitch,
  double target_yaw,
  double target_pitch,
  double distance) const
{
  // 计算射击范围对应的角度
  double shooting_range_yaw = std::abs(std::atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(std::atan2(shooting_range_h_ / 2, distance));

  // 限制最小射击角度
  double min_angle_rad = min_shooting_angle_ * M_PI / 180.0;
  shooting_range_yaw = std::max(shooting_range_yaw, min_angle_rad);
  shooting_range_pitch = std::max(shooting_range_pitch, min_angle_rad);

  // 判断是否在射击范围内
  if (std::abs(current_yaw - target_yaw) < shooting_range_yaw &&
      std::abs(current_pitch - target_pitch) < shooting_range_pitch)
  {
    return true;
  }

  return false;
}

double FireAdvisor::getFireConfidence(
  double yaw_diff,
  double pitch_diff,
  double distance) const
{
  // 计算射击范围对应的角度
  double shooting_range_yaw = std::abs(std::atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(std::atan2(shooting_range_h_ / 2, distance));

  // 限制最小射击角度
  double min_angle_rad = min_shooting_angle_ * M_PI / 180.0;
  shooting_range_yaw = std::max(shooting_range_yaw, min_angle_rad);
  shooting_range_pitch = std::max(shooting_range_pitch, min_angle_rad);

  // 计算归一化偏差
  double normalized_yaw = std::abs(yaw_diff) / shooting_range_yaw;
  double normalized_pitch = std::abs(pitch_diff) / shooting_range_pitch;

  // 使用高斯函数计算置信度
  double combined = std::sqrt(normalized_yaw * normalized_yaw + normalized_pitch * normalized_pitch);
  double confidence = std::exp(-combined * combined);

  return std::clamp(confidence, 0.0, 1.0);
}

}  // namespace gimbal_controller
