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

#include "gimbal_controller/fire_advice_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/ballistic_solver_client.hpp"
#include "gimbal_controller/fire_advisor.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"
#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

namespace gimbal_controller
{

namespace
{

constexpr double kMinDistance = 1e-3;
constexpr double kMinBulletSpeed = 1e-3;
constexpr double kDeg2Rad = M_PI / 180.0;

void assignVector3(geometry_msgs::msg::Vector3 & dst, const Eigen::Vector3d & src)
{
  dst.x = src.x();
  dst.y = src.y();
  dst.z = src.z();
}

Eigen::Vector3d fallbackArmorNormal(
  const Eigen::Vector3d & armor_position,
  const Eigen::Vector3d & center_position)
{
  Eigen::Vector3d normal = armor_position - center_position;
  if (normal.norm() <= 1e-6) {
    return Eigen::Vector3d::UnitX();
  }
  return normal.normalized();
}

Eigen::Vector3d computeArmorNormalFromPose(
  const rm_interfaces::msg::TrackedRobot & normalized_robot,
  int candidate_index,
  double hit_dt_s,
  const Eigen::Vector3d & armor_position,
  const Eigen::Vector3d & center_position)
{
  if (candidate_index < 0 || normalized_robot.armors_offset.empty()) {
    return fallbackArmorNormal(armor_position, center_position);
  }
  return fyt::auto_aim::robot_description::TrackedRobotUsage::calculateArmorWorldNormal(
    normalized_robot,
    candidate_index,
    hit_dt_s,
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY,
    fyt::auto_aim::robot_description::TrackedRobotUsage::ProjectionMode::AUTO);
}

}  // namespace

void CandidateImpactSolver::setFacingFilterOpeningAngleDeg(double opening_angle_deg)
{
  const double clamped_opening_angle_deg = std::clamp(opening_angle_deg, 0.0, 180.0);
  facing_filter_enabled_ = clamped_opening_angle_deg < 180.0 - 1e-9;
  facing_filter_cos_threshold_ = std::cos(0.5 * clamped_opening_angle_deg * kDeg2Rad);
}

delay_management::FireTimelineResult FireTimingResolver::resolve(
  const FireAdviceEngineRequest & request) const
{
  delay_management::DelayRawInputs raw;
  raw.current_time = request.current_time;
  raw.observation_stamp = request.observation_stamp;
  raw.prediction_extra_s = request.timing.prediction_delay_s;
  raw.control_latency_s = request.timing.control_latency_s;
  raw.trigger_to_muzzle_s = request.timing.trigger_to_muzzle_s;
  raw.max_processing_delay_s = request.timing.max_processing_delay_s;

  return delay_manager_.computeFireTimeline(
    raw,
    request.timing.include_processing_delay,
    request.timing.include_control_latency_in_target_prediction);
}

std::pair<double, double> GimbalPosePredictor::predictMuzzlePose(
  const FireAdviceEngineRequest & request,
  const delay_management::FireTimelineResult & timeline,
  bool use_gimbal_kinematics) const
{
  const double delay_s = std::max(timeline.muzzle_delay_s, 0.0);

  if (!use_gimbal_kinematics) {
    return {
      angles::normalize_angle(request.current_yaw),
      request.current_pitch};
  }

  const double predicted_yaw = angles::normalize_angle(
    request.current_yaw +
    request.current_yaw_rate * delay_s +
    0.5 * request.current_yaw_accel * delay_s * delay_s);
  const double predicted_pitch =
    request.current_pitch +
    request.current_pitch_rate * delay_s +
    0.5 * request.current_pitch_accel * delay_s * delay_s;

  return {predicted_yaw, predicted_pitch};
}

void CandidateImpactSolver::setComponents(
  std::shared_ptr<ArmorPositionCalculator> position_calculator,
  std::shared_ptr<BallisticSolverClient> ballistic_client,
  std::shared_ptr<LocalTrajectoryCompensator> local_compensator)
{
  position_calculator_ = position_calculator;
  ballistic_client_ = ballistic_client;
  local_compensator_ = local_compensator;
}

bool CandidateImpactSolver::solveBallistic(
  const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_velocity,
  double bullet_speed,
  double & pitch,
  double & yaw,
  double & flight_time) const
{
  const double bounded_bullet_speed = std::max(bullet_speed, kMinBulletSpeed);

  // service 模式: 优先使用 service；local 模式: 完全跳过 service。
  if (!prefer_local_ballistic_) {
    if (ballistic_client_ && ballistic_client_->isServiceAvailable()) {
      auto result = ballistic_client_->solve(target_position, target_velocity, bounded_bullet_speed);
      if (result.success) {
        pitch = result.pitch;
        yaw = result.yaw;
        flight_time = result.flight_time;
        return true;
      }
    }
  }

  if (local_compensator_) {
    local_compensator_->setBulletSpeed(bounded_bullet_speed);
    auto result = local_compensator_->compensate(target_position);
    if (result.success) {
      pitch = result.pitch;
      yaw = result.yaw;
      flight_time = result.flight_time;
      return true;
    }
  }

  const double distance_xy = std::sqrt(
    target_position.x() * target_position.x() +
    target_position.y() * target_position.y());

  yaw = std::atan2(target_position.y(), target_position.x());
  pitch = std::atan2(target_position.z(), std::max(distance_xy, kMinDistance));
  flight_time = distance_xy / bounded_bullet_speed;

  return true;
}

CandidateImpactSolution CandidateImpactSolver::solveSingleCandidate(
  const rm_interfaces::msg::TrackedRobot & robot,
  int candidate_index,
  const FireAdviceEngineRequest & request,
  const delay_management::FireTimelineResult & timeline,
  int flight_time_iters) const
{
  CandidateImpactSolution solution;

  if (!position_calculator_) {
    return solution;
  }

  const double base_dt = std::max(timeline.target_prediction_base_s, 0.0);
  Eigen::Vector3d target_position = Eigen::Vector3d::Zero();

  if (candidate_index >= 0) {
    auto armor_positions = position_calculator_->calculatePredicted(robot, base_dt);
    if (candidate_index >= static_cast<int>(armor_positions.size())) {
      return solution;
    }
    target_position = armor_positions[candidate_index];
  } else {
    target_position = fyt::auto_aim::robot_description::TrackedRobotUsage::predictCenter(
      robot,
      base_dt,
      fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
  }

  const Eigen::Vector3d target_velocity =
    fyt::auto_aim::robot_description::TrackedRobotUsage::linearVelocity(robot);
  const double target_yaw_rate =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yawVelocity(robot);

  double flight_time = 0.0;
  double target_yaw = 0.0;
  double target_pitch = 0.0;
  const int iterations = std::max(1, flight_time_iters);

  for (int i = 0; i < iterations; ++i) {
    const double hit_dt = base_dt + std::max(flight_time, 0.0);

    if (candidate_index >= 0) {
      auto hit_positions = position_calculator_->calculatePredicted(robot, hit_dt);
      if (candidate_index >= static_cast<int>(hit_positions.size())) {
        break;
      }
      target_position = hit_positions[candidate_index];
    } else {
      target_position = fyt::auto_aim::robot_description::TrackedRobotUsage::predictCenter(
        robot,
        hit_dt,
        fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
    }

    if (!solveBallistic(
      target_position,
      target_velocity,
      request.bullet_speed,
      target_pitch,
      target_yaw,
      flight_time))
    {
      return solution;
    }
  }

  solution.valid = true;
  solution.candidate_index = candidate_index;
  solution.distance = target_position.norm();
  solution.flight_time_s = std::max(flight_time, 0.0);
  solution.target_yaw = angles::normalize_angle(target_yaw + request.yaw_offset_rad);
  solution.target_pitch = target_pitch + request.pitch_offset_rad;
  solution.armor_position = target_position;
  solution.center_position = fyt::auto_aim::robot_description::TrackedRobotUsage::predictCenter(
    robot,
    base_dt + std::max(flight_time, 0.0),
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
  solution.center_velocity = target_velocity;
  solution.armor_yaw_rate = target_yaw_rate;

  return solution;
}

std::vector<CandidateImpactSolution> CandidateImpactSolver::solve(
  const FireAdviceEngineRequest & request,
  const delay_management::FireTimelineResult & timeline,
  int flight_time_iters) const
{
  std::vector<CandidateImpactSolution> results;

  if (!position_calculator_) {
    return results;
  }

  const auto robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(request.target_robot);

  const double base_dt = std::max(timeline.target_prediction_base_s, 0.0);
  const auto center_position = fyt::auto_aim::robot_description::TrackedRobotUsage::predictCenter(
    robot,
    base_dt,
    fyt::auto_aim::robot_description::TrackedRobotUsage::MotionModel::CONSTANT_VELOCITY);
  auto base_positions = position_calculator_->calculatePredicted(robot, base_dt);

  if (base_positions.empty()) {
    auto center_solution = solveSingleCandidate(robot, -1, request, timeline, flight_time_iters);
    if (center_solution.valid) {
      results.push_back(center_solution);
    }
    return results;
  }

  results.reserve(base_positions.size());
  for (int i = 0; i < static_cast<int>(base_positions.size()); ++i) {
    const double facing_cos =
      fyt::auto_aim::robot_description::TrackedRobotUsage::computeFacingCos(
      center_position, base_positions[i]);
    const bool facing_ok = !facing_filter_enabled_ || (facing_cos >= facing_filter_cos_threshold_);

    auto solution = solveSingleCandidate(robot, i, request, timeline, flight_time_iters);
    if (solution.valid) {
      solution.facing_cos = facing_cos;
      solution.facing_ok = facing_ok;
      results.push_back(solution);
    }
  }

  return results;
}

void FireAdviceEngine::setComponents(
  std::shared_ptr<ArmorPositionCalculator> position_calculator,
  std::shared_ptr<BallisticSolverClient> ballistic_client,
  std::shared_ptr<LocalTrajectoryCompensator> local_compensator,
  std::shared_ptr<FireAdvisor> fire_advisor)
{
  candidate_solver_.setComponents(position_calculator, ballistic_client, local_compensator);
  fire_advisor_ = fire_advisor;
}

void FireAdviceEngine::setVelocityLowPassConfig(
  const FireAdviceVelocityLowPassConfig & config)
{
  velocity_filter_cfg_ = config;
  velocity_filter_cfg_.alpha = std::clamp(velocity_filter_cfg_.alpha, 0.0, 1.0);
  velocity_filter_cfg_.reset_timeout_s = std::max(velocity_filter_cfg_.reset_timeout_s, 0.0);
  resetVelocityLowPass();
}

void FireAdviceEngine::resetVelocityLowPass() const
{
  velocity_filter_initialized_ = false;
  velocity_filter_robot_id_.clear();
  velocity_filter_last_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  velocity_filter_linear_.setZero();
  velocity_filter_yaw_rate_ = 0.0;
}

FireAdviceEngineRequest FireAdviceEngine::applyVelocityLowPass(
  const FireAdviceEngineRequest & request) const
{
  if (!velocity_filter_cfg_.enable || velocity_filter_cfg_.alpha >= 1.0) {
    resetVelocityLowPass();
    return request;
  }

  FireAdviceEngineRequest filtered_request = request;
  auto filtered_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(request.target_robot);

  const Eigen::Vector3d raw_linear =
    fyt::auto_aim::robot_description::TrackedRobotUsage::linearVelocity(filtered_robot);
  const double raw_yaw_rate =
    fyt::auto_aim::robot_description::TrackedRobotUsage::yawVelocity(filtered_robot);

  bool should_reset = !velocity_filter_initialized_ ||
    filtered_robot.robot_id != velocity_filter_robot_id_;

  const bool has_last_stamp = velocity_filter_last_stamp_.nanoseconds() > 0;
  const bool has_current_stamp = request.observation_stamp.nanoseconds() > 0;
  if (!should_reset && has_last_stamp && has_current_stamp) {
    const double dt = (request.observation_stamp - velocity_filter_last_stamp_).seconds();
    should_reset = dt < 0.0 || dt > velocity_filter_cfg_.reset_timeout_s;
  }

  const double alpha = std::clamp(velocity_filter_cfg_.alpha, 0.0, 1.0);
  if (should_reset) {
    velocity_filter_linear_ = raw_linear;
    velocity_filter_yaw_rate_ = raw_yaw_rate;
  } else {
    velocity_filter_linear_ =
      alpha * raw_linear + (1.0 - alpha) * velocity_filter_linear_;
    velocity_filter_yaw_rate_ =
      alpha * raw_yaw_rate + (1.0 - alpha) * velocity_filter_yaw_rate_;
  }

  velocity_filter_initialized_ = true;
  velocity_filter_robot_id_ = filtered_robot.robot_id;
  velocity_filter_last_stamp_ = request.observation_stamp;

  assignVector3(filtered_robot.center_velocity, velocity_filter_linear_);
  assignVector3(filtered_robot.center_twist.linear, velocity_filter_linear_);
  filtered_robot.yaw_velocity = velocity_filter_yaw_rate_;
  filtered_robot.center_twist.angular.z = velocity_filter_yaw_rate_;
  filtered_robot.full_state_valid = true;

  filtered_request.target_robot = filtered_robot;
  return filtered_request;
}

FireAdviceEngineResult FireAdviceEngine::evaluate(const FireAdviceEngineRequest & request) const
{
  const auto filtered_request = applyVelocityLowPass(request);

  FireAdviceEngineResult result;
  result.timeline = timing_resolver_.resolve(filtered_request);

  auto impacts = candidate_solver_.solve(filtered_request, result.timeline, flight_time_iters_);
  if (impacts.empty()) {
    return result;
  }

  FireAdvisor default_advisor;
  FireAdvisor * advisor = fire_advisor_ ? fire_advisor_.get() : &default_advisor;

  const auto [muzzle_yaw, muzzle_pitch] =
    gimbal_pose_predictor_.predictMuzzlePose(
      filtered_request,
      result.timeline,
      use_gimbal_kinematics_);
  const auto normalized_robot =
    fyt::auto_aim::robot_description::TrackedRobotUsage::normalizeState(
      filtered_request.target_robot);

  bool has_best = false;
  for (const auto & impact : impacts) {
    FireAdviceInput input;
    input.current_yaw = muzzle_yaw;
    input.current_pitch = muzzle_pitch;
    input.target_yaw = impact.target_yaw;
    input.target_pitch = impact.target_pitch;
    input.distance = std::max(impact.distance, kMinDistance);
    input.muzzle_delay_s = 0.0;

    const auto eval = advisor->evaluate(input);

    FireAdviceCandidateResult candidate;
    candidate.candidate_index = impact.candidate_index;
    candidate.distance = impact.distance;
    candidate.flight_time_s = impact.flight_time_s;
    candidate.target_yaw = impact.target_yaw;
    candidate.target_pitch = impact.target_pitch;
    candidate.yaw_error = eval.yaw_diff;
    candidate.pitch_error = eval.pitch_diff;
    candidate.confidence = eval.confidence;
    candidate.facing_cos = impact.facing_cos;
    candidate.facing_ok = impact.facing_ok;
    candidate.armor_position = impact.armor_position;
    const double hit_dt_s =
      std::max(result.timeline.target_prediction_base_s, 0.0) + std::max(impact.flight_time_s, 0.0);
    candidate.armor_normal = computeArmorNormalFromPose(
      normalized_robot,
      impact.candidate_index,
      hit_dt_s,
      impact.armor_position,
      impact.center_position);
    candidate.fire = eval.fire && impact.facing_ok;
    candidate.center_velocity = impact.center_velocity;
    candidate.armor_yaw_rate = impact.armor_yaw_rate;
    result.candidates.push_back(candidate);

    ++result.candidate_count_total;
    if (impact.facing_ok) {
      ++result.candidate_count_facing_eligible;
    } else {
      ++result.candidate_count_facing_rejected;
    }

    if (!has_best) {
      has_best = true;
      result.fire_advice = candidate.fire;
      result.best_candidate_index = candidate.candidate_index;
      result.yaw_error = candidate.yaw_error;
      result.pitch_error = candidate.pitch_error;
      result.distance = candidate.distance;
      result.confidence = candidate.confidence;
      continue;
    }

    const bool better_fire_state = candidate.fire && !result.fire_advice;
    const bool better_same_state =
      candidate.fire == result.fire_advice && candidate.confidence > result.confidence;
    if (better_fire_state || better_same_state) {
      result.fire_advice = candidate.fire;
      result.best_candidate_index = candidate.candidate_index;
      result.yaw_error = candidate.yaw_error;
      result.pitch_error = candidate.pitch_error;
      result.distance = candidate.distance;
      result.confidence = candidate.confidence;
    }
  }

  result.valid = has_best;

  if (has_best && probability_cfg_.enable) {
    result.probability_enabled = true;
    const auto best_it = std::find_if(
      result.candidates.begin(),
      result.candidates.end(),
      [&](const FireAdviceCandidateResult & c) {return c.candidate_index == result.best_candidate_index;});
    if (best_it != result.candidates.end()) {
      const auto prob = probability_engine_.evaluate(
        filtered_request.target_robot,
        std::max(best_it->distance, kMinDistance),
        best_it->yaw_error,
        best_it->pitch_error,
        std::max(best_it->flight_time_s, 0.0),
        std::max(filtered_request.bullet_speed, kMinBulletSpeed),
        muzzle_yaw,
        muzzle_pitch,
        filtered_request.current_yaw_rate,
        filtered_request.current_pitch_rate,
        filtered_request.current_yaw_accel,
        filtered_request.current_pitch_accel,
        best_it->armor_position,
        best_it->armor_normal,
        best_it->center_velocity,
        best_it->armor_yaw_rate,
        1.0 / 120.0,
        filtered_request.target_robot.robot_id);
      if (prob.valid) {
        result.p_hit_window = prob.p_window;
        result.fire_score = prob.fire_score;
        result.burst_probability = prob.burst_probability;
        result.log_evidence = prob.log_evidence;
        result.evidence_sum = prob.evidence_sum;
        result.evidence_strength = prob.evidence_strength;
        result.gate_strategy = prob.gate_strategy;
        result.gate_state = prob.gate_state;
        result.best_tau_s = prob.best_tau_s;
        result.e_u = prob.best_e_u;
        result.e_v = prob.best_e_v;
        result.sigma_u = prob.best_sigma_u;
        result.sigma_v = prob.best_sigma_v;
        result.armor_width_m = prob.armor_width_m;
        result.armor_height_m = prob.armor_height_m;
        result.tau_samples = prob.tau_samples;
        result.armor_center = prob.armor_center;
        result.armor_right = prob.armor_right;
        result.armor_up = prob.armor_up;
        result.fire_advice = prob.fire_state;
      }
    }
  }
  return result;
}

}  // namespace gimbal_controller
