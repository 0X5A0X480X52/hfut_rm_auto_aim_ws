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

#include "gimbal_controller/strategies/predicted_position_strategy.hpp"
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"
#include <angles/angles.h>

#include <iostream>

namespace gimbal_controller
{

rm_interfaces::msg::GimbalCmd PredictedPositionStrategy::solve(
  const GimbalControlContext & context)
{
  // 检查是否在跟踪状态
  if (!context.is_tracking) {
    state_ = TRACKING_ARMOR;
    overflow_count_ = 0;
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

  const auto & robot = context.target_robot;

  // 估计飞行时间 (使用当前位置)
  Eigen::Vector3d current_center(
    robot.center_position.x,
    robot.center_position.y,
    robot.center_position.z);

  double flight_time = 0;
  if (local_compensator_) {
    local_compensator_->setBulletSpeed(context.bullet_speed);
    flight_time = local_compensator_->getFlyingTime(current_center);
  } else {
    flight_time = current_center.norm() / context.bullet_speed;
  }

  // 计算总预测时间 = 处理延迟 + 飞行时间 + 额外预测延迟
  double processing_delay = (context.current_time - context.target_stamp).seconds();
  double total_prediction_time = processing_delay + flight_time + prediction_delay_;
  total_prediction_time = std::min(total_prediction_time, max_prediction_time_);

  std::cout << "Processing delay: " << processing_delay
            << " s, Flight time: " << flight_time
            << " s, Total prediction time: " << total_prediction_time << " s" << std::endl;

  // 计算预测位置的装甲板坐标
  auto predicted_armor_positions = position_calculator_->calculatePredicted(
    robot, total_prediction_time);

  // 计算当前位置的装甲板坐标 (用于开火判断)
  auto current_armor_positions = position_calculator_->calculate(robot);

  if (predicted_armor_positions.empty() || current_armor_positions.empty()) {
    return createIdleCmd();
  }

  // 预测中心位置
  Eigen::Vector3d predicted_center(
    robot.center_position.x + total_prediction_time * robot.center_velocity.x,
    robot.center_position.y + total_prediction_time * robot.center_velocity.y,
    robot.center_position.z + total_prediction_time * robot.center_velocity.z);

  // 预测 yaw
  double predicted_yaw = robot.yaw + total_prediction_time * robot.yaw_velocity;

  // 选择最佳装甲板 (基于预测位置，路由到配置的选板策略)
  auto predicted_selection = armor_selector_->selectBest(
    predicted_armor_positions,
    predicted_center,
    predicted_yaw,
    robot.num_armors,
    robot.yaw_velocity,
    context.current_yaw,
    context.current_pitch);

  // 选择最佳装甲板 (基于当前位置，用于开火判断，始终使用最小运动量算法)
  auto current_selection = armor_selector_->selectByMinMovement(
    current_armor_positions,
    context.current_yaw,
    context.current_pitch);

  if (current_selection.selected_index < 0) {
    return createIdleCmd();
  }

  // 高转速状态机处理
  double abs_v_yaw = std::abs(robot.yaw_velocity);

  switch (state_) {
    case TRACKING_ARMOR:
      if (abs_v_yaw > max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }
      if (overflow_count_ > transfer_thresh_) {
        state_ = TRACKING_CENTER;
      }
      break;

    case TRACKING_CENTER:
      if (abs_v_yaw < max_tracking_v_yaw_) {
        overflow_count_++;
      } else {
        overflow_count_ = 0;
      }
      if (overflow_count_ > transfer_thresh_) {
        state_ = TRACKING_ARMOR;
        overflow_count_ = 0;
      }
      break;
  }

  // 根据状态选择目标位置
  Eigen::Vector3d control_target_position;
  Eigen::Vector3d fire_target_position = current_selection.position;

  if (state_ == TRACKING_CENTER) {
    // 高转速时跟踪机器人中心 (预测位置)
    control_target_position = predicted_center;
  } else {
    // 正常跟踪预测装甲板位置 (含 facing 过滤后的结果或 center fallback)
    control_target_position = predicted_selection.position;

    // controller_delay 前馈：在 TRACKING_ARMOR 状态且配置了 controller_delay 时，
    // 在 total_prediction_time 基础上再额外预测 controller_delay_ 秒，
    // 强制云台超前运动（与 armor_solver 原版 controller_delay 机制一致）
    // 若启用自适应模式，则使用 adaptive_ctrl_ 的当前 delay 替代静态 controller_delay_
    double effective_ctrl_delay =
      adaptive_delay_enabled_ ? adaptive_ctrl_.getDelay() : controller_delay_;
    if (effective_ctrl_delay > 0.0) {
      double extra_dt = total_prediction_time + effective_ctrl_delay;
      extra_dt = std::min(extra_dt, max_prediction_time_);
      auto extra_positions = position_calculator_->calculatePredicted(robot, extra_dt);
      if (!extra_positions.empty()) {
        Eigen::Vector3d extra_center(
          robot.center_position.x + extra_dt * robot.center_velocity.x,
          robot.center_position.y + extra_dt * robot.center_velocity.y,
          robot.center_position.z + extra_dt * robot.center_velocity.z);
        double extra_yaw = robot.yaw + extra_dt * robot.yaw_velocity;
        auto extra_selection = armor_selector_->selectBest(
          extra_positions, extra_center, extra_yaw,
          robot.num_armors, robot.yaw_velocity,
          context.current_yaw, context.current_pitch);
        control_target_position = extra_selection.position;
      }
    }
  }

  Eigen::Vector3d target_velocity(
    robot.center_velocity.x,
    robot.center_velocity.y,
    robot.center_velocity.z);

  // 计算云台控制角度 (使用预测位置)
  double control_pitch, control_yaw, control_flight_time;
  if (!computeBallistic(control_target_position, target_velocity, context.bullet_speed,
                        control_pitch, control_yaw, control_flight_time))
  {
    return createIdleCmd();
  }

  // 计算开火判断角度 (使用当前位置)
  double fire_pitch, fire_yaw, fire_flight_time;
  if (!computeBallistic(fire_target_position, target_velocity, context.bullet_speed,
                        fire_pitch, fire_yaw, fire_flight_time))
  {
    return createIdleCmd();
  }

  // 应用手动补偿
  double pitch_offset_rad = pitch_offset_ * M_PI / 180.0;
  double yaw_offset_rad = yaw_offset_ * M_PI / 180.0;

  double cmd_pitch = control_pitch + pitch_offset_rad;
  double cmd_yaw = angles::normalize_angle(control_yaw + yaw_offset_rad);

  // 计算偏差
  double yaw_diff = angles::normalize_angle(cmd_yaw - context.current_yaw);
  double pitch_diff = cmd_pitch - context.current_pitch;

  // 判断是否应该开火 (使用当前位置)
  bool fire_advice = fire_advisor_->shouldFire(
    context.current_yaw,
    context.current_pitch,
    fire_yaw + yaw_offset_rad,
    fire_pitch + pitch_offset_rad,
    current_selection.distance);

  // 高转速时始终建议开火
  if (state_ == TRACKING_CENTER) {
    fire_advice = true;
  }

  // 自适应 delay 更新（根据本帧 fire_advice 和目标速度）
  if (adaptive_delay_enabled_) {
    Eigen::Vector3d vel(
      robot.center_velocity.x,
      robot.center_velocity.y,
      robot.center_velocity.z);
    double v_linear  = vel.norm();
    double v_angular = std::abs(robot.yaw_velocity);
    adaptive_ctrl_.update(fire_advice, v_linear, v_angular);
  }

  // 构建控制命令
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header = robot.header;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.distance = current_selection.distance;
  cmd.fire_advice = fire_advice;

  return cmd;
}

void PredictedPositionStrategy::setPredictionParameters(
  double prediction_delay,
  double max_prediction_time)
{
  prediction_delay_ = prediction_delay;
  max_prediction_time_ = max_prediction_time;
}

void PredictedPositionStrategy::setManualOffset(double pitch_offset, double yaw_offset)
{
  pitch_offset_ = pitch_offset;
  yaw_offset_ = yaw_offset;
}

void PredictedPositionStrategy::setControllerDelay(double controller_delay)
{
  controller_delay_ = controller_delay;
}

void PredictedPositionStrategy::setAdaptiveDelayParams(
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

void PredictedPositionStrategy::setTrackingCenterParams(double max_tracking_v_yaw, int transfer_thresh)
{
  max_tracking_v_yaw_ = max_tracking_v_yaw;
  transfer_thresh_ = transfer_thresh;
}

}  // namespace gimbal_controller
