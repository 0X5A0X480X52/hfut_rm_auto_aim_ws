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

#include "gimbal_controller/strategies/current_position_strategy.hpp"
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include <angles/angles.h>

namespace gimbal_controller
{

rm_interfaces::msg::GimbalCmd CurrentPositionStrategy::solve(
  const GimbalControlContext & context)
{
  // 检查是否在跟踪状态
  if (!context.is_tracking) {
    if (armor_selector_) {
      armor_selector_->resetState();
    }
    return createIdleCmd();
  }

  // 检查组件是否已设置
  if (!position_calculator_ || !armor_selector_ || !fire_advisor_) {
    return createIdleCmd();
  }

  // 计算所有装甲板的当前位置
  auto armor_positions = position_calculator_->calculate(context.target_robot);

  if (armor_positions.empty()) {
    return createIdleCmd();
  }

  // 构建目标中心位置
  Eigen::Vector3d target_center(
    context.target_robot.center_position.x,
    context.target_robot.center_position.y,
    context.target_robot.center_position.z);

  // 选择最佳装甲板 (带 Facing 过滤 + Hysteresis)
  auto selection = armor_selector_->selectByMinMovementWithFacing(
    armor_positions,
    target_center,
    context.target_robot.yaw,
    context.target_robot.num_armors,
    context.current_yaw,
    context.current_pitch);

  // center fallback 时 selected_index == -1 但仍有有效 position
  Eigen::Vector3d target_position = selection.position;
  Eigen::Vector3d target_velocity(
    context.target_robot.center_velocity.x,
    context.target_robot.center_velocity.y,
    context.target_robot.center_velocity.z);

  // 计算弹道补偿
  double pitch, yaw, flight_time;
  if (!computeBallistic(target_position, target_velocity, context.bullet_speed,
                        pitch, yaw, flight_time))
  {
    return createIdleCmd();
  }

  // 应用手动补偿
  double pitch_offset_rad = pitch_offset_ * M_PI / 180.0;
  double yaw_offset_rad = yaw_offset_ * M_PI / 180.0;

  double cmd_pitch = pitch + pitch_offset_rad;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset_rad);

  // 计算偏差
  double yaw_diff = angles::normalize_angle(cmd_yaw - context.current_yaw);
  double pitch_diff = cmd_pitch - context.current_pitch;

  // 判断是否应该开火
  bool fire_advice = fire_advisor_->shouldFire(
    context.current_yaw,
    context.current_pitch,
    cmd_yaw,
    cmd_pitch,
    selection.distance);

  // 构建控制命令
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header = context.target_robot.header;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.distance = selection.distance;
  cmd.fire_advice = fire_advice;

  return cmd;
}

void CurrentPositionStrategy::setManualOffset(double pitch_offset, double yaw_offset)
{
  pitch_offset_ = pitch_offset;
  yaw_offset_ = yaw_offset;
}

}  // namespace gimbal_controller
