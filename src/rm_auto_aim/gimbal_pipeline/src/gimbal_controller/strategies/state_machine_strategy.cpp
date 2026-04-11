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

#include "gimbal_controller/strategies/state_machine_strategy.hpp"
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/fire_advice_engine.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"
#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"
#include <angles/angles.h>
#include <algorithm>
#include <limits>

namespace gimbal_controller
{

// =====================================================================
//  solve — 主入口
// =====================================================================

rm_interfaces::msg::GimbalCmd StateMachineStrategy::solve(
  const GimbalControlContext & context)
{
  GimbalControlContext normalized_context = context;
  normalized_context.target_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(context.target_robot);
  const auto & target_robot = normalized_context.target_robot;
  const auto center_position =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerPosition(target_robot);
  const double target_yaw =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yaw(target_robot);
  const double target_yaw_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yawVelocity(target_robot);

  // 无跟踪 → LOST
  if (!normalized_context.is_tracking) {
    markDelayAuditInvalid(getName(), false);
    resetStateMachine();
    return createIdleCmd();
  }

  if (!position_calculator_ || !armor_selector_ || (!fire_advice_engine_ && !fire_advisor_)) {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  // 保存上次目标位置 (用于 LOST → 保持瞄准)
  const Eigen::Vector3d current_center = center_position;
  last_target_position_ = current_center;

  // ---- 自旋检测计数 ----
  double abs_v_yaw = std::abs(target_yaw_velocity);

  if (abs_v_yaw > spin_v_yaw_thresh_) {
    spin_count_++;
    calm_count_ = 0;
  } else if (abs_v_yaw < calm_v_yaw_thresh_) {
    calm_count_++;
    spin_count_ = 0;
  } else {
    // 在两个阈值之间, 不改变计数
  }

  // ---- 状态转移检查 (先检查全局转移) ----
  switch (state_) {
    case State::LOST:
      // 有目标 → CENTER
      state_ = State::CENTER;
      spin_count_ = 0;
      calm_count_ = 0;
      break;

    case State::CENTER:
      // 检查是否进入 SPIN
      if (spin_count_ >= spin_enter_count_) {
        state_ = State::SPIN;
        spin_decision_index_ = -1;
        break;
      }
      // 检查是否有正面装甲板 → SINGLE
      {
        auto armor_positions = position_calculator_->calculate(target_robot);
        auto facing_angles = ArmorSelector::computeFacingAngles(
          armor_positions, target_yaw, target_robot.num_armors);
        int best = selectBestFacingArmor(
          armor_positions, facing_angles, facing_enter_angle_,
          normalized_context.current_yaw, normalized_context.current_pitch);
        if (best >= 0) {
          state_ = State::SINGLE;
          locked_armor_index_ = best;
        }
      }
      break;

    case State::SINGLE:
      // 检查是否进入 SPIN
      if (spin_count_ >= spin_enter_count_) {
        state_ = State::SPIN;
        spin_decision_index_ = locked_armor_index_;
        locked_armor_index_ = -1;
        break;
      }
      // 检查锁定板是否仍有效
      {
        auto armor_positions = position_calculator_->calculate(target_robot);
        if (locked_armor_index_ < 0 ||
            locked_armor_index_ >= static_cast<int>(armor_positions.size()))
        {
          // 板索引越界 → CENTER
          state_ = State::CENTER;
          locked_armor_index_ = -1;
          break;
        }
        auto facing_angles = ArmorSelector::computeFacingAngles(
          armor_positions, target_yaw, target_robot.num_armors);
        double fa = facing_angles[locked_armor_index_];
        double exit_rad = facing_exit_angle_ * M_PI / 180.0;
        if (fa > exit_rad) {
          // 锁定板转走 → CENTER
          state_ = State::CENTER;
          locked_armor_index_ = -1;
        }
      }
      break;

    case State::SPIN:
      // 检查是否退出 SPIN
      if (calm_count_ >= spin_exit_count_) {
        // 尝试找正面装甲板 → SINGLE, 否则 → CENTER
        auto armor_positions = position_calculator_->calculate(target_robot);
        auto facing_angles = ArmorSelector::computeFacingAngles(
          armor_positions, target_yaw, target_robot.num_armors);
        int best = selectBestFacingArmor(
          armor_positions, facing_angles, facing_enter_angle_,
          normalized_context.current_yaw, normalized_context.current_pitch);
        if (best >= 0) {
          state_ = State::SINGLE;
          locked_armor_index_ = best;
        } else {
          state_ = State::CENTER;
          locked_armor_index_ = -1;
        }
        spin_decision_index_ = -1;
      }
      break;
  }

  // ---- 执行当前状态行为 ----
  switch (state_) {
    case State::LOST:
      return handleLost(normalized_context);
    case State::CENTER:
      return handleCenter(normalized_context);
    case State::SINGLE:
      return handleSingle(normalized_context);
    case State::SPIN:
      return handleSpin(normalized_context);
  }

  return createIdleCmd();  // 不应到达
}

// =====================================================================
//  各状态处理函数
// =====================================================================

rm_interfaces::msg::GimbalCmd StateMachineStrategy::handleLost(
  const GimbalControlContext & context)
{
  markDelayAuditInvalid(getName(), context.is_tracking);
  return createIdleCmd();
}

rm_interfaces::msg::GimbalCmd StateMachineStrategy::handleCenter(
  const GimbalControlContext & context)
{
  const auto & robot = context.target_robot;
  const auto center_position =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerPosition(robot);
  const Eigen::Vector3d current_center = center_position;

  double dt = computePredictionTime(context, current_center);

  // 预测中心位置
  Eigen::Vector3d predicted_center =
    fyt::auto_aim::robot_description::TrackedRobotUsage::predictCenter(
    robot,
    dt,
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);

  // 当前位置用于开火判断 — 使用最近的装甲板
  auto current_positions = position_calculator_->calculate(robot);
  auto current_sel = armor_selector_->selectByMinMovement(
    current_positions, context.current_yaw, context.current_pitch);

  double fire_dist = current_sel.selected_index >= 0
    ? current_sel.distance : current_center.norm();
  Eigen::Vector3d fire_target = current_sel.selected_index >= 0
    ? current_sel.position : current_center;

  // CENTER 状态始终建议开火 (打中心)
  return buildCommand(context, predicted_center, fire_target, fire_dist, true);
}

rm_interfaces::msg::GimbalCmd StateMachineStrategy::handleSingle(
  const GimbalControlContext & context)
{
  const auto & robot = context.target_robot;
  const auto center_position =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerPosition(robot);
  const Eigen::Vector3d current_center = center_position;

  double dt = computePredictionTime(context, current_center);

  // 计算预测装甲板位置
  auto predicted_positions = position_calculator_->calculatePredicted(robot, dt);

  if (locked_armor_index_ < 0 ||
      locked_armor_index_ >= static_cast<int>(predicted_positions.size()))
  {
    // Fallback
    state_ = State::CENTER;
    locked_armor_index_ = -1;
    return handleCenter(context);
  }

  Eigen::Vector3d control_target = predicted_positions[locked_armor_index_];

  // 当前位置用于开火判断
  auto current_positions = position_calculator_->calculate(robot);
  Eigen::Vector3d fire_target = (locked_armor_index_ < static_cast<int>(current_positions.size()))
    ? current_positions[locked_armor_index_]
    : control_target;
  double fire_dist = fire_target.norm();

  return buildCommand(context, control_target, fire_target, fire_dist, false);
}

rm_interfaces::msg::GimbalCmd StateMachineStrategy::handleSpin(
  const GimbalControlContext & context)
{
  const auto & robot = context.target_robot;
  const auto center_position =
    fyt::auto_aim::robot_description::TrackedRobotUsage::centerPosition(robot);
  const double yaw_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yawVelocity(robot);
  const Eigen::Vector3d current_center = center_position;

  double dt = computePredictionTime(context, current_center);

  // 计算预测装甲板位置
  auto predicted_positions = position_calculator_->calculatePredicted(robot, dt);
  auto current_positions = position_calculator_->calculate(robot);

  if (predicted_positions.empty() || current_positions.empty()) {
    return createIdleCmd();
  }

  // 使用 selectByDecisionAngle 选择装甲板
  double predicted_yaw =
    fyt::auto_aim::robot_description::TrackedRobotUsage::predictYaw(
    robot,
    dt,
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
  Eigen::Vector3d predicted_center =
    fyt::auto_aim::robot_description::TrackedRobotUsage::predictCenter(
    robot,
    dt,
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);

  int decision_id = armor_selector_->selectByDecisionAngle(
    predicted_positions, predicted_center, predicted_yaw, yaw_velocity);

  if (decision_id < 0 || decision_id >= static_cast<int>(predicted_positions.size())) {
    // Fallback: 跟踪中心
    auto current_sel = armor_selector_->selectByMinMovement(
      current_positions, context.current_yaw, context.current_pitch);
    double fire_dist = current_sel.selected_index >= 0
      ? current_sel.distance : current_center.norm();
    Eigen::Vector3d fire_target = current_sel.selected_index >= 0
      ? current_sel.position : current_center;
    return buildCommand(context, predicted_center, fire_target, fire_dist, true);
  }

  // 顺序保护: 只允许切换到相邻装甲板
  int num_armors = static_cast<int>(predicted_positions.size());
  if (spin_decision_index_ >= 0 && spin_decision_index_ < num_armors) {
    if (!isAdjacentArmor(decision_id, spin_decision_index_, num_armors)) {
      // 不相邻 → 保持当前板
      decision_id = spin_decision_index_;
    }
  }

  spin_decision_index_ = decision_id;

  Eigen::Vector3d control_target = predicted_positions[decision_id];
  Eigen::Vector3d fire_target = (decision_id < static_cast<int>(current_positions.size()))
    ? current_positions[decision_id]
    : control_target;
  double fire_dist = fire_target.norm();

  // SPIN 模式始终建议开火
  return buildCommand(context, control_target, fire_target, fire_dist, true);
}

// =====================================================================
//  辅助方法
// =====================================================================

double StateMachineStrategy::computePredictionTime(
  const GimbalControlContext & context,
  const Eigen::Vector3d & current_center)
{
  double flight_time = 0;
  if (local_compensator_) {
    local_compensator_->setBulletSpeed(context.bullet_speed);
    flight_time = local_compensator_->getFlyingTime(current_center);
  } else {
    flight_time = current_center.norm() / context.bullet_speed;
  }

  delay_management::DelayRawInputs delay_raw;
  delay_raw.current_time = context.current_time;
  delay_raw.observation_stamp = context.target_stamp;
  delay_raw.prediction_extra_s = prediction_delay_;
  delay_raw.max_processing_delay_s = max_processing_delay_s_;

  const double processing_delay = delay_manager_.computeProcessingDelay(delay_raw);
  const double total_prediction_time =
    delay_manager_.computePredictionTime(delay_raw, flight_time, max_prediction_time_);

  last_processing_delay_s_ = processing_delay;
  last_flight_time_s_ = flight_time;
  last_total_prediction_time_s_ = total_prediction_time;

  return total_prediction_time;
}

int StateMachineStrategy::selectBestFacingArmor(
  const std::vector<Eigen::Vector3d> & armor_positions,
  const std::vector<double> & facing_angles,
  double threshold_deg,
  double current_yaw,
  double current_pitch) const
{
  double threshold_rad = threshold_deg * M_PI / 180.0;
  int best_idx = -1;
  double min_movement = std::numeric_limits<double>::max();

  for (size_t i = 0; i < armor_positions.size(); ++i) {
    if (facing_angles[i] > threshold_rad) {
      continue;
    }

    const auto & pos = armor_positions[i];
    double yaw, pitch;
    ArmorSelector::calculateYawPitch(pos, current_yaw, yaw, pitch);
    double yaw_diff = angles::normalize_angle(yaw - current_yaw);
    double pitch_diff = pitch - current_pitch;
    double movement = yaw_diff * yaw_diff + pitch_diff * pitch_diff;

    if (movement < min_movement) {
      min_movement = movement;
      best_idx = static_cast<int>(i);
    }
  }

  return best_idx;
}

rm_interfaces::msg::GimbalCmd StateMachineStrategy::buildCommand(
  const GimbalControlContext & context,
  const Eigen::Vector3d & control_target,
  const Eigen::Vector3d & fire_target,
  double fire_distance,
  bool force_fire)
{
  const Eigen::Vector3d target_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::linearVelocity(context.target_robot);

  // 弹道补偿 (控制目标)
  double control_pitch, control_yaw, control_flight;
  if (!computeBallistic(control_target, target_velocity, context.bullet_speed,
                        control_pitch, control_yaw, control_flight))
  {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  // 弹道补偿 (开火判断目标)
  double fire_pitch, fire_yaw, fire_flight;
  if (!computeBallistic(fire_target, target_velocity, context.bullet_speed,
                        fire_pitch, fire_yaw, fire_flight))
  {
    markDelayAuditInvalid(getName(), true);
    return createIdleCmd();
  }

  // 应用手动补偿
  double pitch_offset_rad = pitch_offset_ * M_PI / 180.0;
  double yaw_offset_rad = yaw_offset_ * M_PI / 180.0;

  double cmd_pitch = control_pitch + pitch_offset_rad;
  double cmd_yaw = angles::normalize_angle(control_yaw + yaw_offset_rad);

  double yaw_diff = angles::normalize_angle(cmd_yaw - context.current_yaw);
  double pitch_diff = cmd_pitch - context.current_pitch;

  bool fire_advice = force_fire;
  double fire_compensation_s = trigger_to_muzzle_s_;
  if (!force_fire && (fire_advice_engine_ || fire_advisor_)) {
    if (fire_advice_engine_) {
      FireAdviceEngineRequest fire_request;
      fire_request.target_robot = context.target_robot;
      fire_request.current_time = context.current_time;
      fire_request.observation_stamp = context.target_stamp;
      fire_request.current_yaw = context.current_yaw;
      fire_request.current_pitch = context.current_pitch;
      fire_request.bullet_speed = context.bullet_speed;
      fire_request.yaw_offset_rad = yaw_offset_rad;
      fire_request.pitch_offset_rad = pitch_offset_rad;
      fire_request.timing.prediction_delay_s = std::max(prediction_delay_, 0.0);
      fire_request.timing.control_latency_s = 0.0;
      fire_request.timing.trigger_to_muzzle_s = trigger_to_muzzle_s_;
      fire_request.timing.max_processing_delay_s = max_processing_delay_s_;
      fire_request.timing.include_processing_delay = true;
      fire_request.timing.include_control_latency_in_target_prediction = false;

      const auto fire_result = fire_advice_engine_->evaluate(fire_request);
      if (fire_result.valid) {
        fire_advice = fire_result.fire_advice;
        fire_compensation_s = fire_result.timeline.muzzle_delay_s;
      }
    } else {
      const double fire_muzzle_delay_s =
        std::max(last_processing_delay_s_, 0.0) + std::max(prediction_delay_, 0.0) +
        trigger_to_muzzle_s_;
      fire_advice = fire_advisor_->shouldFireWithDelay(
        context.current_yaw, context.current_pitch,
        fire_yaw + yaw_offset_rad, fire_pitch + pitch_offset_rad,
        fire_distance,
        fire_muzzle_delay_s);
      fire_compensation_s = trigger_to_muzzle_s_;
    }
  }

  rm_interfaces::msg::GimbalCmd cmd;
  cmd.header = context.target_robot.header;
  cmd.yaw = cmd_yaw * 180.0 / M_PI;
  cmd.pitch = cmd_pitch * 180.0 / M_PI;
  cmd.yaw_diff = yaw_diff * 180.0 / M_PI;
  cmd.pitch_diff = pitch_diff * 180.0 / M_PI;
  cmd.distance = fire_distance;
  cmd.fire_advice = fire_advice;

  DelayAuditSnapshot audit;
  audit.strategy_name = getName();
  audit.tracking = true;
  audit.processing_delay_s = last_processing_delay_s_;
  audit.prediction_extra_s = std::max(prediction_delay_, 0.0);
  audit.flight_time_s = last_flight_time_s_;
  audit.total_prediction_time_s = last_total_prediction_time_s_;
  audit.control_latency_s = 0.0;
  audit.fire_control_compensation_s = fire_compensation_s;
  audit.control_delay_steps = 0;
  audit.uses_delayed_b = false;
  audit.double_compensation_risk = false;
  markDelayAuditValid(audit);

  return cmd;
}

bool StateMachineStrategy::isAdjacentArmor(int idx_a, int idx_b, int num_armors)
{
  if (num_armors <= 1) return true;
  int diff = std::abs(idx_a - idx_b);
  return diff == 1 || diff == (num_armors - 1);
}

void StateMachineStrategy::resetStateMachine()
{
  state_ = State::LOST;
  locked_armor_index_ = -1;
  spin_decision_index_ = -1;
  spin_count_ = 0;
  calm_count_ = 0;
  last_processing_delay_s_ = 0.0;
  last_flight_time_s_ = 0.0;
  last_total_prediction_time_s_ = 0.0;
}

// =====================================================================
//  参数设置
// =====================================================================

void StateMachineStrategy::setFacingParameters(double enter_angle, double exit_angle)
{
  facing_enter_angle_ = enter_angle;
  facing_exit_angle_ = exit_angle;
}

void StateMachineStrategy::setSpinParameters(
  double spin_thresh, double calm_thresh,
  int enter_count, int exit_count, double side_angle)
{
  spin_v_yaw_thresh_ = spin_thresh;
  calm_v_yaw_thresh_ = calm_thresh;
  spin_enter_count_ = enter_count;
  spin_exit_count_ = exit_count;
  side_angle_ = side_angle;
}

void StateMachineStrategy::setPredictionParameters(
  double prediction_delay, double max_prediction_time)
{
  prediction_delay_ = prediction_delay;
  max_prediction_time_ = max_prediction_time;
}

void StateMachineStrategy::setMaxProcessingDelay(double max_processing_delay)
{
  max_processing_delay_s_ = max_processing_delay > 0.0 ? max_processing_delay : 0.0;
}

void StateMachineStrategy::setTriggerToMuzzleDelay(double trigger_to_muzzle_s)
{
  trigger_to_muzzle_s_ = trigger_to_muzzle_s > 0.0 ? trigger_to_muzzle_s : 0.0;
}

void StateMachineStrategy::setManualOffset(double pitch_offset, double yaw_offset)
{
  pitch_offset_ = pitch_offset;
  yaw_offset_ = yaw_offset;
}

std::string StateMachineStrategy::stateToString(State s)
{
  switch (s) {
    case State::LOST:   return "LOST";
    case State::CENTER: return "CENTER";
    case State::SINGLE: return "SINGLE";
    case State::SPIN:   return "SPIN";
    default:            return "UNKNOWN";
  }
}

}  // namespace gimbal_controller
