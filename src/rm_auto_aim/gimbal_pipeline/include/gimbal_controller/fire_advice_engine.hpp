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

#ifndef GIMBAL_CONTROLLER__FIRE_ADVICE_ENGINE_HPP_
#define GIMBAL_CONTROLLER__FIRE_ADVICE_ENGINE_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_controller/fire_advice/probability_engine.hpp"
#include "gimbal_controller/delay_management/delay_semantic_manager.hpp"
#include "rm_interfaces/msg/tracked_robot.hpp"

namespace gimbal_controller
{

class ArmorPositionCalculator;
class BallisticSolverClient;
class LocalTrajectoryCompensator;
class FireAdvisor;

struct FireAdviceEngineTimingRequest
{
  double prediction_delay_s{0.0};
  double control_latency_s{0.0};
  double trigger_to_muzzle_s{0.0};
  double max_processing_delay_s{0.5};
  bool include_processing_delay{true};
  bool include_control_latency_in_target_prediction{false};
};

struct FireAdviceVelocityLowPassConfig
{
  bool enable{true};
  double alpha{0.35};
  double reset_timeout_s{0.25};
};

struct FireAdviceEngineRequest
{
  rm_interfaces::msg::TrackedRobot target_robot;
  rclcpp::Time current_time;
  rclcpp::Time observation_stamp;

  double current_yaw{0.0};
  double current_pitch{0.0};
  double current_yaw_rate{0.0};
  double current_pitch_rate{0.0};
  double current_yaw_accel{0.0};
  double current_pitch_accel{0.0};

  double bullet_speed{20.0};
  double yaw_offset_rad{0.0};
  double pitch_offset_rad{0.0};

  FireAdviceEngineTimingRequest timing;
};

struct FireAdviceCandidateResult
{
  int candidate_index{-1};
  double distance{0.0};
  double flight_time_s{0.0};
  double target_yaw{0.0};
  double target_pitch{0.0};
  double yaw_error{0.0};
  double pitch_error{0.0};
  double confidence{0.0};
  double facing_cos{1.0};
  bool facing_ok{true};
  bool fire{false};
  Eigen::Vector3d armor_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d armor_normal = Eigen::Vector3d::UnitX();
  Eigen::Vector3d center_velocity = Eigen::Vector3d::Zero();
  double armor_yaw_rate{0.0};
};

struct FireAdviceEngineResult
{
  bool valid{false};
  bool fire_advice{false};
  int best_candidate_index{-1};
  double yaw_error{0.0};
  double pitch_error{0.0};
  double distance{0.0};
  double confidence{0.0};
  int candidate_count_total{0};
  int candidate_count_facing_eligible{0};
  int candidate_count_facing_rejected{0};
  bool probability_enabled{false};
  double p_hit_window{0.0};
  double fire_score{0.0};
  double burst_probability{0.0};
  double log_evidence{0.0};
  double evidence_sum{0.0};
  double evidence_strength{0.0};
  int gate_strategy{0};
  int gate_state{0};
  double best_tau_s{0.0};
  double e_u{0.0};
  double e_v{0.0};
  double sigma_u{0.0};
  double sigma_v{0.0};
  double armor_width_m{0.135};
  double armor_height_m{0.125};
  std::vector<fire_advice::TauDebugSample> tau_samples;
  Eigen::Vector3d armor_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d armor_right = Eigen::Vector3d::UnitY();
  Eigen::Vector3d armor_up = Eigen::Vector3d::UnitZ();
  delay_management::FireTimelineResult timeline;
  std::vector<FireAdviceCandidateResult> candidates;
};

class FireTimingResolver
{
public:
  FireTimingResolver() = default;
  ~FireTimingResolver() = default;

  delay_management::FireTimelineResult resolve(const FireAdviceEngineRequest & request) const;

private:
  delay_management::DelaySemanticManager delay_manager_;
};

class GimbalPosePredictor
{
public:
  GimbalPosePredictor() = default;
  ~GimbalPosePredictor() = default;

  std::pair<double, double> predictMuzzlePose(
    const FireAdviceEngineRequest & request,
    const delay_management::FireTimelineResult & timeline,
    bool use_gimbal_kinematics) const;
};

struct CandidateImpactSolution
{
  bool valid{false};
  int candidate_index{-1};
  double distance{0.0};
  double flight_time_s{0.0};
  double target_yaw{0.0};
  double target_pitch{0.0};
  double facing_cos{1.0};
  bool facing_ok{true};
  Eigen::Vector3d armor_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_position = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_velocity = Eigen::Vector3d::Zero();
  double armor_yaw_rate{0.0};
};

class CandidateImpactSolver
{
public:
  CandidateImpactSolver() = default;
  ~CandidateImpactSolver() = default;

  void setBallisticMode(const std::string & mode)
  {
    prefer_local_ballistic_ = (mode == "local");
  }

  void setFacingFilterOpeningAngleDeg(double opening_angle_deg);

  void setComponents(
    std::shared_ptr<ArmorPositionCalculator> position_calculator,
    std::shared_ptr<BallisticSolverClient> ballistic_client,
    std::shared_ptr<LocalTrajectoryCompensator> local_compensator);

  std::vector<CandidateImpactSolution> solve(
    const FireAdviceEngineRequest & request,
    const delay_management::FireTimelineResult & timeline,
    int flight_time_iters) const;

private:
  bool solveBallistic(
    const Eigen::Vector3d & target_position,
    const Eigen::Vector3d & target_velocity,
    double bullet_speed,
    double & pitch,
    double & yaw,
    double & flight_time) const;

  CandidateImpactSolution solveSingleCandidate(
    const rm_interfaces::msg::TrackedRobot & robot,
    int candidate_index,
    const FireAdviceEngineRequest & request,
    const delay_management::FireTimelineResult & timeline,
    int flight_time_iters) const;

  std::shared_ptr<ArmorPositionCalculator> position_calculator_;
  std::shared_ptr<BallisticSolverClient> ballistic_client_;
  std::shared_ptr<LocalTrajectoryCompensator> local_compensator_;
  bool prefer_local_ballistic_{false};
  bool facing_filter_enabled_{false};
  double facing_filter_cos_threshold_{-1.0};
};

class FireAdviceEngine
{
public:
  FireAdviceEngine() = default;
  ~FireAdviceEngine() = default;

  void setComponents(
    std::shared_ptr<ArmorPositionCalculator> position_calculator,
    std::shared_ptr<BallisticSolverClient> ballistic_client,
    std::shared_ptr<LocalTrajectoryCompensator> local_compensator,
    std::shared_ptr<FireAdvisor> fire_advisor);

  void setFlightTimeIterations(int flight_time_iters)
  {
    flight_time_iters_ = std::max(1, flight_time_iters);
  }

  void setUseGimbalKinematics(bool use_gimbal_kinematics)
  {
    use_gimbal_kinematics_ = use_gimbal_kinematics;
  }

  void setBallisticMode(const std::string & mode)
  {
    candidate_solver_.setBallisticMode(mode);
  }

  void setFacingFilterOpeningAngleDeg(double opening_angle_deg)
  {
    candidate_solver_.setFacingFilterOpeningAngleDeg(opening_angle_deg);
  }

  void setVelocityLowPassConfig(const FireAdviceVelocityLowPassConfig & config);

  void setProbabilityConfig(
    const fire_advice::ProbabilityConfig & probability_cfg,
    const fire_advice::SigmaPointConfig & sigma_cfg,
    const fire_advice::FireGateConfig & gate_cfg)
  {
    probability_cfg_ = probability_cfg;
    sigma_cfg_ = sigma_cfg;
    gate_cfg_ = gate_cfg;
    probability_engine_.setConfig(probability_cfg_, sigma_cfg_, gate_cfg_);
  }

  FireAdviceEngineResult evaluate(const FireAdviceEngineRequest & request) const;

private:
  FireAdviceEngineRequest applyVelocityLowPass(const FireAdviceEngineRequest & request) const;
  void resetVelocityLowPass() const;

  std::shared_ptr<FireAdvisor> fire_advisor_;
  FireTimingResolver timing_resolver_;
  CandidateImpactSolver candidate_solver_;
  GimbalPosePredictor gimbal_pose_predictor_;
  mutable fire_advice::ProbabilityEngine probability_engine_;
  fire_advice::ProbabilityConfig probability_cfg_;
  fire_advice::SigmaPointConfig sigma_cfg_;
  fire_advice::FireGateConfig gate_cfg_;

  int flight_time_iters_{2};
  bool use_gimbal_kinematics_{false};
  FireAdviceVelocityLowPassConfig velocity_filter_cfg_;
  mutable bool velocity_filter_initialized_{false};
  mutable std::string velocity_filter_robot_id_;
  mutable rclcpp::Time velocity_filter_last_stamp_{0, 0, RCL_ROS_TIME};
  mutable Eigen::Vector3d velocity_filter_linear_{Eigen::Vector3d::Zero()};
  mutable double velocity_filter_yaw_rate_{0.0};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__FIRE_ADVICE_ENGINE_HPP_
