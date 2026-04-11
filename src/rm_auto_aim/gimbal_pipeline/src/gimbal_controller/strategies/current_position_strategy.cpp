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
#include "gimbal_controller/fire_advice_engine.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"
#include <angles/angles.h>
#include <algorithm>

namespace gimbal_controller
{

rm_interfaces::msg::GimbalCmd CurrentPositionStrategy::solve(
  const GimbalControlContext & context)
{
  // 检查是否在跟踪状态
  if (!context.is_tracking) {
    markDelayAuditInvalid(getName(), false);
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
  if (!position_calculator_ || !armor_selector_ || (!fire_advice_engine_ && !fire_advisor_)) {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  const auto target_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);
  const auto center_position =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerPosition(target_robot);
  const auto linear_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::linearVelocity(target_robot);
  const double target_yaw =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yaw(target_robot);
  const double target_yaw_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yawVelocity(target_robot);

  // 计算所有装甲板的当前位置
  auto armor_positions = position_calculator_->calculate(target_robot);

  if (armor_positions.empty()) {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  const Eigen::Vector3d target_center = center_position;

  // 选择最佳装甲板（开火判断用，路由到配置的选板策略）
  auto fire_selection = armor_selector_->selectBest(
    armor_positions,
    target_center,
    target_yaw,
    target_robot.num_armors,
    target_yaw_velocity,
    context.current_yaw,
    context.current_pitch);

  // controller_delay 前馈：当 controller_delay_ > 0 时，
  // 使用 DelaySemanticManager 计算 processing_delay（有上限），
  // 并在 processing_delay 基础上额外预测 controller_delay 作为云台控制目标，
  // 开火判断仍使用当前位置。若启用自适应模式，则使用 adaptive_ctrl_ 的当前 delay 替代静态值
  Eigen::Vector3d control_position = fire_selection.position;
  double control_distance = fire_selection.distance;

  double effective_ctrl_delay = adaptive_delay_enabled_ ? adaptive_ctrl_.getDelay() : controller_delay_;
  delay_management::DelayRawInputs delay_raw;
  delay_raw.current_time = context.current_time;
  delay_raw.observation_stamp = context.target_stamp;
  delay_raw.prediction_extra_s = 0.0;  // 当前策略没有额外预测延迟
  delay_raw.max_processing_delay_s = max_processing_delay_s_;

  const double processing_delay = delay_manager_.computeProcessingDelay(delay_raw);
  double applied_prediction_time_s = processing_delay;

  if (effective_ctrl_delay > 0.0) {
    double extra_dt = std::max(processing_delay + effective_ctrl_delay, 0.0);
    applied_prediction_time_s = extra_dt;

    auto predicted_positions = position_calculator_->calculatePredicted(target_robot, extra_dt);
    if (!predicted_positions.empty()) {
      Eigen::Vector3d predicted_center =
        fyt::auto_aim::robot_description::TrackedRobotUsage::predictCenter(
          target_robot,
          extra_dt,
          fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
      double predicted_yaw =
        fyt::auto_aim::robot_description::TrackedRobotUsage::predictYaw(
          target_robot,
          extra_dt,
          fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
      auto ctrl_selection = armor_selector_->selectBest(
        predicted_positions,
        predicted_center,
        predicted_yaw,
        target_robot.num_armors,
        target_yaw_velocity,
        context.current_yaw,
        context.current_pitch);
      control_position = ctrl_selection.position;
      control_distance = ctrl_selection.distance;
    }
  }
  const Eigen::Vector3d target_velocity = linear_velocity;

  // 计算弹道补偿（使用云台控制位置）
  double pitch, yaw, flight_time;
  if (!computeBallistic(control_position, target_velocity, context.bullet_speed,
                        pitch, yaw, flight_time))
  {
    markDelayAuditInvalid(getName(), true);
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

  bool fire_advice = false;
  double fire_compensation_s = trigger_to_muzzle_s_;
  if (fire_advice_engine_) {
    FireAdviceEngineRequest fire_request;
    fire_request.target_robot = target_robot;
    fire_request.current_time = context.current_time;
    fire_request.observation_stamp = context.target_stamp;
    fire_request.current_yaw = context.current_yaw;
    fire_request.current_pitch = context.current_pitch;
    fire_request.bullet_speed = context.bullet_speed;
    fire_request.yaw_offset_rad = yaw_offset_rad;
    fire_request.pitch_offset_rad = pitch_offset_rad;
    fire_request.timing.prediction_delay_s = 0.0;
    fire_request.timing.control_latency_s = std::max(effective_ctrl_delay, 0.0);
    fire_request.timing.trigger_to_muzzle_s = trigger_to_muzzle_s_;
    fire_request.timing.max_processing_delay_s = max_processing_delay_s_;
    fire_request.timing.include_processing_delay = true;
    fire_request.timing.include_control_latency_in_target_prediction = false;

    const auto fire_result = fire_advice_engine_->evaluate(fire_request);
    if (fire_result.valid) {
      fire_advice = fire_result.fire_advice;
      fire_compensation_s = fire_result.timeline.muzzle_delay_s;
    }
  } else if (fire_advisor_) {
    const double fire_muzzle_delay_s = std::max(processing_delay, 0.0) + trigger_to_muzzle_s_;
    fire_advice = fire_advisor_->shouldFireWithDelay(
      context.current_yaw,
      context.current_pitch,
      cmd_yaw,
      cmd_pitch,
      fire_selection.distance,
      fire_muzzle_delay_s);
    fire_compensation_s = trigger_to_muzzle_s_;
  }

  // 自适应 delay 更新（根据本帧 fire_advice 和目标速度）
  if (adaptive_delay_enabled_) {
    double v_linear  = linear_velocity.norm();
    double v_angular = std::abs(target_yaw_velocity);
    adaptive_ctrl_.update(fire_advice, v_linear, v_angular);
  }

  // 构建控制命令
  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header = target_robot.header;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.distance = control_distance;
  cmd.fire_advice = fire_advice;

  DelayAuditSnapshot audit;
  audit.strategy_name = getName();
  audit.tracking = true;
  audit.processing_delay_s = processing_delay;
  audit.prediction_extra_s = 0.0;
  audit.flight_time_s = flight_time;
  audit.total_prediction_time_s = applied_prediction_time_s;
  audit.control_latency_s = std::max(effective_ctrl_delay, 0.0);
  audit.fire_control_compensation_s = fire_compensation_s;
  audit.control_delay_steps = 0;
  audit.uses_delayed_b = false;
  audit.double_compensation_risk = false;
  markDelayAuditValid(audit);

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

void CurrentPositionStrategy::setMaxProcessingDelay(double max_processing_delay)
{
  max_processing_delay_s_ = max_processing_delay > 0.0 ? max_processing_delay : 0.0;
}

void CurrentPositionStrategy::setTriggerToMuzzleDelay(double trigger_to_muzzle_s)
{
  trigger_to_muzzle_s_ = trigger_to_muzzle_s > 0.0 ? trigger_to_muzzle_s : 0.0;
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
