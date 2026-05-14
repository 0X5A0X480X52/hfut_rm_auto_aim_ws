#ifndef GIMBAL_CONTROLLER__FIRE_ADVICE__TYPES_HPP_
#define GIMBAL_CONTROLLER__FIRE_ADVICE__TYPES_HPP_

#include <vector>

#include <Eigen/Dense>

namespace gimbal_controller::fire_advice
{

struct SigmaPointConfig
{
  bool enable{true};
  bool use_unscented{true};
  double sigma_v0{0.3};
  double sigma_delay{0.005};
  double rho{0.0};
  double alpha{0.7};
  double beta{2.0};
  double kappa{0.0};
};

struct FireGateConfig
{
  enum class Strategy
  {
    kLegacy = 0,
    kBurstEvidence = 1
  };

  Strategy strategy{Strategy::kLegacy};
  bool integrator_mode{false};
  double alpha{0.85};
  double fire_on_th{0.65};
  double fire_off_th{0.35};
  double integrator_base_probability{0.45};
  double integrator_rise{8.0};
  double integrator_fall{6.0};

  int burst_bullet_count{5};
  int min_hit_count{1};
  double reference_probability_p0{0.60};
  double evidence_window_ms{50.0};
  double log_evidence_clip{2.0};
  double evidence_epsilon{1e-3};
  bool neutralize_unshootable_samples{true};
  double negative_evidence_scale{0.35};
  double negative_clip_scale{0.35};
  double evidence_deadband{0.10};

  double temperature{0.5};
  double theta_on_cold{0.90};
  double theta_on_hot{0.75};
  double theta_hold_cold{0.70};
  double theta_hold_hot{0.55};
  double theta_reset_cold{0.45};
  double theta_reset_hot{0.35};

  double min_fire_ms{20.0};
  double cooldown_ms{80.0};
};

struct ProbabilityConfig
{
  bool enable{false};
  double future_window_ms{50.0};
  double future_step_ms{10.0};
  bool softmax_fusion{false};
  double softmax_beta{20.0};

  bool use_tracker_covariance{true};
  bool strict_covariance{false};
  double fallback_sigma_x{0.02};
  double fallback_sigma_y{0.02};
  double fallback_sigma_z{0.03};

  bool enable_normal_velocity_weight{false};
  double normal_v_ref{28.0};
  double normal_w_min{0.5};
  bool enable_normal_velocity_gate{true};
  bool require_front_face{true};
  double normal_v_activate_min{8.0};
  double front_face_epsilon{1e-4};
  double max_complement_angle_deg{90.0};

  double sigma_x0{0.010};
  double sigma_y0{0.015};
  double sigma_z0{0.015};
  double growth_x{0.03};
  double growth_y{0.06};
  double growth_z{0.08};

  // Armor hit rectangle size in meters (SI).
  double armor_width_m{0.135};
  double armor_height_m{0.125};
};

struct TauDebugSample
{
  double tau_s{0.0};
  double p_hit{0.0};
  double e_u{0.0};
  double e_v{0.0};
  double sigma_u{0.0};
  double sigma_v{0.0};
  bool front_ok{true};
  bool normal_gate_pass{true};
  double normal_velocity{0.0};
  double normal_weight{1.0};
  double impact_x{0.0};
  double impact_y{0.0};
  double impact_z{0.0};
};

struct ProbabilityDebugResult
{
  bool valid{false};
  double p_window{0.0};
  double fire_score{0.0};
  bool fire_state{false};
  double burst_probability{0.0};
  double log_evidence{0.0};
  double evidence_sum{0.0};
  double evidence_strength{0.0};
  int gate_strategy{0};
  int gate_state{0};
  double best_tau_s{0.0};
  double best_p_hit{0.0};
  double best_e_u{0.0};
  double best_e_v{0.0};
  double best_sigma_u{0.0};
  double best_sigma_v{0.0};
  double armor_width_m{0.135};
  double armor_height_m{0.125};
  Eigen::Vector3d armor_center = Eigen::Vector3d::Zero();
  Eigen::Vector3d armor_right = Eigen::Vector3d::UnitY();
  Eigen::Vector3d armor_up = Eigen::Vector3d::UnitZ();
  std::vector<TauDebugSample> tau_samples;
};

}  // namespace gimbal_controller::fire_advice

#endif  // GIMBAL_CONTROLLER__FIRE_ADVICE__TYPES_HPP_
