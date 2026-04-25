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
    if (facing_filter_enabled_) {
      const Eigen::Vector3d armor_normal = base_positions[i] - center_position;
      const Eigen::Vector3d armor_to_muzzle = -base_positions[i];
      const double normal_norm = armor_normal.norm();
      const double to_muzzle_norm = armor_to_muzzle.norm();
      if (normal_norm > kMinDistance && to_muzzle_norm > kMinDistance) {
        const double facing_cos = armor_normal.dot(armor_to_muzzle) / (normal_norm * to_muzzle_norm);
        if (facing_cos < facing_filter_cos_threshold_) {
          continue;
        }
      }
    }

    auto solution = solveSingleCandidate(robot, i, request, timeline, flight_time_iters);
    if (solution.valid) {
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

FireAdviceEngineResult FireAdviceEngine::evaluate(const FireAdviceEngineRequest & request) const
{
  FireAdviceEngineResult result;
  result.timeline = timing_resolver_.resolve(request);

  auto impacts = candidate_solver_.solve(request, result.timeline, flight_time_iters_);
  if (impacts.empty()) {
    return result;
  }

  FireAdvisor default_advisor;
  FireAdvisor * advisor = fire_advisor_ ? fire_advisor_.get() : &default_advisor;

  const auto [muzzle_yaw, muzzle_pitch] =
    gimbal_pose_predictor_.predictMuzzlePose(request, result.timeline, use_gimbal_kinematics_);

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
    candidate.fire = eval.fire;
    result.candidates.push_back(candidate);

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
  return result;
}

}  // namespace gimbal_controller
