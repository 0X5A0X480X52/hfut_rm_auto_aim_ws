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
    // 跟丢目标时重置自适应 delay 状态
    if (adaptive_delay_enabled_) {
      adaptive_ctrl_.reset();
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

  // 选择最佳装甲板（开火判断用，路由到配置的选板策略）
  auto fire_selection = armor_selector_->selectBest(
    armor_positions,
    target_center,
    context.target_robot.yaw,
    context.target_robot.num_armors,
    context.target_robot.yaw_velocity,
    context.current_yaw,
    context.current_pitch);

  // controller_delay 前馈：当 controller_delay_ > 0 时，
  // 用预测位置作为云台控制目标，开火判断仍使用当前位置
  // 若启用自适应模式，则使用 adaptive_ctrl_ 的当前 delay 替代静态值
  Eigen::Vector3d control_position = fire_selection.position;
  double control_distance = fire_selection.distance;

  double effective_ctrl_delay =
    adaptive_delay_enabled_ ? adaptive_ctrl_.getDelay() : controller_delay_;

  if (effective_ctrl_delay > 0.0) {
    auto predicted_positions = position_calculator_->calculatePredicted(
      context.target_robot, effective_ctrl_delay);
    if (!predicted_positions.empty()) {
      Eigen::Vector3d predicted_center(
        context.target_robot.center_position.x + effective_ctrl_delay * context.target_robot.center_velocity.x,
        context.target_robot.center_position.y + effective_ctrl_delay * context.target_robot.center_velocity.y,
        context.target_robot.center_position.z + effective_ctrl_delay * context.target_robot.center_velocity.z);
      double predicted_yaw = context.target_robot.yaw
                             + effective_ctrl_delay * context.target_robot.yaw_velocity;
      auto ctrl_selection = armor_selector_->selectBest(
        predicted_positions,
        predicted_center,
        predicted_yaw,
        context.target_robot.num_armors,
        context.target_robot.yaw_velocity,
        context.current_yaw,
        context.current_pitch);
      control_position = ctrl_selection.position;
      control_distance = ctrl_selection.distance;
    }
  }
  Eigen::Vector3d target_velocity(
    context.target_robot.center_velocity.x,
    context.target_robot.center_velocity.y,
    context.target_robot.center_velocity.z);

  // 计算弹道补偿（使用云台控制位置）
  double pitch, yaw, flight_time;
  if (!computeBallistic(control_position, target_velocity, context.bullet_speed,
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

  // 判断是否应该开火（始终使用当前位置）
  bool fire_advice = fire_advisor_->shouldFire(
    context.current_yaw,
    context.current_pitch,
    cmd_yaw,
    cmd_pitch,
    fire_selection.distance);

  // 自适应 delay 更新（根据本帧 fire_advice 和目标速度）
  if (adaptive_delay_enabled_) {
    Eigen::Vector3d vel(
      context.target_robot.center_velocity.x,
      context.target_robot.center_velocity.y,
      context.target_robot.center_velocity.z);
    double v_linear  = vel.norm();
    double v_angular = std::abs(context.target_robot.yaw_velocity);
    adaptive_ctrl_.update(fire_advice, v_linear, v_angular);
  }

  // 构建控制命令
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header = context.target_robot.header;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.distance = control_distance;
  cmd.fire_advice = fire_advice;

  return cmd;
}

void CurrentPositionStrategy::setControllerDelay(double controller_delay)
{
  controller_delay_ = controller_delay;
}

void CurrentPositionStrategy::setManualOffset(double pitch_offset, double yaw_offset)
{
  pitch_offset_ = pitch_offset;
  yaw_offset_ = yaw_offset;
}

void CurrentPositionStrategy::setAdaptiveDelayParams(
  bool enable,
  double initial_delay,
  double min_delay,
  double max_delay,
  double add_step,
  double mul_factor,
  int    fire_wait_threshold,
  double max_linear_speed,
  double max_angular_speed)
{
  adaptive_delay_enabled_ = enable;
  if (enable) {
    adaptive_ctrl_.init(
      initial_delay, min_delay, max_delay,
      add_step, mul_factor, fire_wait_threshold,
      max_linear_speed, max_angular_speed);
  }
}

}  // namespace gimbal_controller
