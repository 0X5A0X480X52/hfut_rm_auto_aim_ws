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

#include "gimbal_controller/gimbal_control_strategy.hpp"
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/ballistic_solver_client.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"
#include "gimbal_controller/fire_advisor.hpp"

namespace gimbal_controller
{

void GimbalControlStrategy::setComponents(
  std::shared_ptr<ArmorPositionCalculator> position_calculator,
  std::shared_ptr<ArmorSelector> armor_selector,
  std::shared_ptr<BallisticSolverClient> ballistic_client,
  std::shared_ptr<LocalTrajectoryCompensator> local_compensator,
  std::shared_ptr<FireAdvisor> fire_advisor)
{
  position_calculator_ = position_calculator;
  armor_selector_ = armor_selector;
  ballistic_client_ = ballistic_client;
  local_compensator_ = local_compensator;
  fire_advisor_ = fire_advisor;
}

rm_interfaces::msg::GimbalCmd GimbalControlStrategy::createIdleCmd() const
{
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.yaw = 0;
  cmd.pitch = 0;
  cmd.yaw_diff = 0;
  cmd.pitch_diff = 0;
  cmd.distance = -1;
  cmd.yaw_v = 0;
  cmd.pitch_v = 0;
  cmd.fire_advice = false;
  return cmd;
}

bool GimbalControlStrategy::computeBallistic(
  const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_velocity,
  double bullet_speed,
  double & pitch,
  double & yaw,
  double & flight_time) const
{
  // 首先尝试使用 ballistic_solver 服务
  if (ballistic_client_ && ballistic_client_->isServiceAvailable()) {
    auto result = ballistic_client_->solve(target_position, target_velocity, bullet_speed);
    if (result.success) {
      pitch = result.pitch;
      yaw = result.yaw;
      flight_time = result.flight_time;
      return true;
    }
  }

  // Fallback: 使用本地弹道补偿器
  if (local_compensator_) {
    local_compensator_->setBulletSpeed(bullet_speed);
    auto result = local_compensator_->compensate(target_position);
    if (result.success) {
      pitch = result.pitch;
      yaw = result.yaw;
      flight_time = result.flight_time;
      return true;
    }
  }

  // 最后的 fallback: 直接计算角度
  double distance_xy = std::sqrt(
    target_position.x() * target_position.x() +
    target_position.y() * target_position.y());

  yaw = std::atan2(target_position.y(), target_position.x());
  pitch = std::atan2(target_position.z(), distance_xy);
  flight_time = distance_xy / bullet_speed;

  return true;
}

}  // namespace gimbal_controller
