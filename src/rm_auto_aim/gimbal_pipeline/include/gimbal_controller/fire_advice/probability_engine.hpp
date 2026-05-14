#ifndef GIMBAL_CONTROLLER__FIRE_ADVICE__PROBABILITY_ENGINE_HPP_
#define GIMBAL_CONTROLLER__FIRE_ADVICE__PROBABILITY_ENGINE_HPP_

#include <deque>
#include <string>

#include "gimbal_controller/fire_advice/types.hpp"
#include "rm_interfaces/msg/tracked_robot.hpp"

namespace gimbal_controller::fire_advice
{

class ProbabilityEngine
{
public:
  void setConfig(const ProbabilityConfig & cfg, const SigmaPointConfig & sigma_cfg, const FireGateConfig & gate_cfg);

  ProbabilityDebugResult evaluate(
    const rm_interfaces::msg::TrackedRobot & robot,
    double distance,
    double yaw_error,
    double pitch_error,
    double flight_time_s,
    double bullet_speed,
    double muzzle_yaw,
    double muzzle_pitch,
    double muzzle_yaw_rate,
    double muzzle_pitch_rate,
    double muzzle_yaw_accel,
    double muzzle_pitch_accel,
    const Eigen::Vector3d & armor_center,
    const Eigen::Vector3d & armor_normal,
    const Eigen::Vector3d & target_center_velocity,
    double armor_yaw_rate,
    double dt_s,
    const std::string & target_id);

private:
  static double clamp(double v, double lo, double hi);
  static double normalCdf(double x);
  static double probabilityInside1d(double mean, double sigma, double half_size);
  static double hitProbabilityIndependent(double e_u, double e_v, double sigma_u, double sigma_v, double width, double height);
  static std::vector<Eigen::Vector2d> buildSigmaPoints(
    const SigmaPointConfig & cfg,
    std::vector<double> & wm,
    std::vector<double> & wc);

  void updateGate(double p_window, const std::vector<double> & p_hits, double dt_s);
  void updateGateLegacy(double p_window, double dt_s);
  void updateGateBurstEvidence(const std::vector<double> & p_hits, double dt_s);
  void resetGateState();
  static double burstProbability(
    const std::vector<double> & p_hits,
    int burst_count,
    int min_hit_count);

  enum class BurstGateState
  {
    kIdle = 0,
    kFireCommit = 1
  };
  bool extractTrackerCovariance(
    const rm_interfaces::msg::TrackedRobot & robot,
    Eigen::Matrix3d & cov_xyz,
    std::string & source) const;

  ProbabilityConfig cfg_;
  SigmaPointConfig sigma_cfg_;
  FireGateConfig gate_cfg_;

  double score_{0.0};
  bool fire_state_{false};
  double burst_probability_{0.0};
  double log_evidence_{0.0};
  double evidence_sum_{0.0};
  double evidence_strength_{0.0};
  std::deque<double> evidence_window_;
  BurstGateState burst_state_{BurstGateState::kIdle};
  double burst_state_time_s_{0.0};
  std::string active_target_id_;
};

}  // namespace gimbal_controller::fire_advice

#endif  // GIMBAL_CONTROLLER__FIRE_ADVICE__PROBABILITY_ENGINE_HPP_
