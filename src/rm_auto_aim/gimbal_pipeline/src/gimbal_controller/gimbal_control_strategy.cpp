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
#include "gimbal_controller/fire_advice_engine.hpp"

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

void GimbalControlStrategy::setFireAdviceEngine(
  std::shared_ptr<FireAdviceEngine> fire_advice_engine)
{
  fire_advice_engine_ = fire_advice_engine;
}

void GimbalControlStrategy::setBallisticMode(const std::string & mode)
{
  prefer_local_ballistic_ = (mode == "local");
}

rm_interfaces::msg::GimbalCmd GimbalControlStrategy::createIdleCmd() const
{
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.yaw = 0;
  cmd.pitch = 0;
  cmd.yaw_diff = 0;
  cmd.pitch_diff = 0;
  cmd.distance = 0;
  cmd.yaw_v = 0;
  cmd.pitch_v = 0;
  cmd.fire_advice = false;
  cmd.mode = rm_interfaces::msg::GimbalCmd::MODE_NO_VALID_MEASUREMENT;
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
  // service 模式: 优先使用 ballistic_solver 服务
  // local 模式: 完全跳过 service，避免 timeout 告警。
  if (!prefer_local_ballistic_) {
    if (ballistic_client_ && ballistic_client_->isServiceAvailable()) {
      auto result = ballistic_client_->solve(target_position, target_velocity, bullet_speed);
      if (result.success) {
        pitch = result.pitch;
        yaw = result.yaw;
        flight_time = result.flight_time;
        return true;
      }
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

void GimbalControlStrategy::markDelayAuditInvalid(
  const std::string & strategy_name,
  bool tracking)
{
  last_delay_audit_ = DelayAuditSnapshot{};
  last_delay_audit_.strategy_name = strategy_name;
  last_delay_audit_.tracking = tracking;
  last_delay_audit_.valid = false;
}

void GimbalControlStrategy::markDelayAuditValid(const DelayAuditSnapshot & snapshot)
{
  last_delay_audit_ = snapshot;
  last_delay_audit_.valid = true;
}

}  // namespace gimbal_controller
