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

#include <angles/angles.h>

#include <algorithm>

namespace gimbal_controller
{

bool AxisThresholdFireDecisionPolicy::decide(
  double yaw_diff,
  double pitch_diff,
  double yaw_threshold,
  double pitch_threshold) const
{
  return std::abs(yaw_diff) < yaw_threshold && std::abs(pitch_diff) < pitch_threshold;
}

bool EllipseFireDecisionPolicy::decide(
  double yaw_diff,
  double pitch_diff,
  double yaw_threshold,
  double pitch_threshold) const
{
  if (yaw_threshold <= 1e-9 || pitch_threshold <= 1e-9) {
    return false;
  }

  const double normalized_yaw = yaw_diff / yaw_threshold;
  const double normalized_pitch = pitch_diff / pitch_threshold;
  return normalized_yaw * normalized_yaw + normalized_pitch * normalized_pitch < 1.0;
}

FireAdvisor::FireAdvisor()
: decision_policy_(std::make_shared<AxisThresholdFireDecisionPolicy>())
{}

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
  return shouldFireWithDelay(
    current_yaw,
    current_pitch,
    target_yaw,
    target_pitch,
    distance,
    0.0);
}

bool FireAdvisor::shouldFireWithDelay(
  double current_yaw,
  double current_pitch,
  double target_yaw,
  double target_pitch,
  double distance,
  double muzzle_delay_s,
  double yaw_rate,
  double pitch_rate,
  double yaw_accel,
  double pitch_accel) const
{
  FireAdviceInput input;
  input.current_yaw = current_yaw;
  input.current_pitch = current_pitch;
  input.target_yaw = target_yaw;
  input.target_pitch = target_pitch;
  input.distance = distance;
  input.muzzle_delay_s = muzzle_delay_s;
  input.yaw_rate = yaw_rate;
  input.pitch_rate = pitch_rate;
  input.yaw_accel = yaw_accel;
  input.pitch_accel = pitch_accel;

  return evaluate(input).fire;
}

FireAdviceResult FireAdvisor::evaluate(const FireAdviceInput & input) const
{
  FireAdviceResult result;

  const double distance = std::max(input.distance, 1e-3);
  const double delay_s = std::max(input.muzzle_delay_s, 0.0);

  result.predicted_yaw_at_muzzle = angles::normalize_angle(
    input.current_yaw +
    input.yaw_rate * delay_s +
    0.5 * input.yaw_accel * delay_s * delay_s);
  result.predicted_pitch_at_muzzle =
    input.current_pitch +
    input.pitch_rate * delay_s +
    0.5 * input.pitch_accel * delay_s * delay_s;

  result.yaw_diff = angles::normalize_angle(input.target_yaw - result.predicted_yaw_at_muzzle);
  result.pitch_diff = input.target_pitch - result.predicted_pitch_at_muzzle;

  result.yaw_threshold = std::abs(std::atan2(shooting_range_w_ / 2.0, distance));
  result.pitch_threshold = std::abs(std::atan2(shooting_range_h_ / 2.0, distance));

  const double min_angle_rad = min_shooting_angle_ * M_PI / 180.0;
  result.yaw_threshold = std::max(result.yaw_threshold, min_angle_rad);
  result.pitch_threshold = std::max(result.pitch_threshold, min_angle_rad);

  auto policy = decision_policy_;
  if (!policy) {
    policy = std::make_shared<AxisThresholdFireDecisionPolicy>();
  }

  result.fire = policy->decide(
    result.yaw_diff,
    result.pitch_diff,
    result.yaw_threshold,
    result.pitch_threshold);

  result.confidence = getFireConfidence(result.yaw_diff, result.pitch_diff, distance);
  return result;
}

void FireAdvisor::setDecisionPolicy(std::shared_ptr<FireDecisionPolicy> decision_policy)
{
  if (decision_policy) {
    decision_policy_ = decision_policy;
  }
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
