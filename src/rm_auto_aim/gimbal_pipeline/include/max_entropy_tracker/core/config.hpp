// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_CORE_CONFIG_HPP_
#define MAX_ENTROPY_TRACKER_CORE_CONFIG_HPP_

#include <cmath>
#include <string>

namespace fyt::auto_aim {

// ======================== Enums ========================

enum class TranslationModel { CV, CA, SINGER };

enum class RotationModel { CV, CA };

enum class FilterType { STANDARD, DECOMPOSED };

// ======================== Parameter Structs ========================

struct UKFParameters {
  // Sigma point sampling
  double alpha = 0.001;
  double beta = 2.0;
  double kappa = 0.0;

  // Single-observation noise
  double obs_noise_pos = 0.05;
  double obs_noise_yaw = 0.05;

  // Dual-observation noise
  double dual_obs_noise_pos = 0.01;
  double dual_obs_noise_yaw = 0.03;
  double dual_obs_geometry_noise_scale = 0.2;

  // Single-observation position update weight
  double single_obs_update_weight_pos = 0.05;

  // Innovation gating
  bool enable_innovation_gating = false;
  double innovation_gate_chi2_threshold = 9.49;
};

struct MotionModelParameters {
  TranslationModel translation_model = TranslationModel::CA;

  // CV
  double cv_process_noise_vel = 0.5;
  // CA
  double ca_process_noise_acc = 1.0;
  // Singer
  double singer_alpha = 0.5;
  double singer_sigma = 2.0;

  // Structural
  double process_noise_r = 0.02;
  double process_noise_dz = 0.005;
};

struct SpinModelParameters {
  double spin_process_noise_yaw_rate = 0.3;
  double spin_process_noise_yaw_acc = 1.0;
  double spin_process_noise_delta_rate = 0.3;
  double spin_process_noise_delta_acc = 3.0;
};

struct MaxEntropyParameters {
  double temperature = 2.0;
  bool use_adaptive = true;
  double k_prior_weight = 0.7;
};

struct TrackerParameters {
  int tracking_thres = 2;
  int lost_thres = 8;
  int temp_lost_thres = 3;

  double max_match_distance = 2.0;
  double max_match_yaw_diff = 1.0;

  int n_panels = 4;
  double panel_angle_step = M_PI / 2.0;

  // Optional periodic dz prior for standard 4-panel association.
  // Template: -dz, +dz, -dz, +dz (sign flips with spin direction).
  bool periodic_binding_enable = false;
  double periodic_binding_weight = 0.35;
  double periodic_binding_spin_rate_gate = 0.8;
};

struct ConstraintParameters {
  double min_radius = 0.12;
  double max_radius = 0.5;
  double min_dz = -1.0;
  double max_dz = 1.0;
};

struct PanelMismatchParameters {
  bool   enable        = true;
  int    window_size   = 8;       ///< rolling buffer length W (frames)
  double threshold_t1  = 0.0009;  ///< dz² mean threshold below which z is OK (3cm²)
  int    confirm_count = 3;       ///< consecutive suspect frames to trigger PATCH
  int    reinit_count  = 5;       ///< consecutive suspect frames to trigger REINIT
};

struct OutpostParameters {
  // Motion model selection (same style as 4-panel tracker)
  TranslationModel translation_model = TranslationModel::CV;
  RotationModel rotation_model = RotationModel::CV;

  // Optional outpost-specific Singer params (fallback to motion.* when <= 0)
  double singer_alpha = 0.0;
  double singer_sigma = 0.0;

  // Optional outpost-specific spin process noise (fallback to spin.* when <= 0)
  double spin_process_noise_theta_rate = 0.0;
  double spin_process_noise_theta_acc = 0.0;

  // Known geometric profile (relative to outpost center)
  // Semantic contract:
  //   panel 0 = highest, panel 1 = middle, panel 2 = lowest.
  // Top-down clockwise order: 0 deg(panel 0) -> panel 2 -> panel 1.
  double radius = 0.26;
  double z_offset_0 = 0.06;
  double z_offset_1 = 0.00;
  double z_offset_2 = -0.06;
  double panel_angle_step = 2.0 * M_PI / 3.0;

  // Max-entropy panel posterior
  double softmax_temperature = 1.5;
  double weight_yaw = 1.0;
  double weight_z_state = 6.0;
  double weight_z_history = 2.0;
  double weight_xy_residual = 2.5;
  double weight_switch_penalty = 0.05;

  // Hysteresis gating for mode switch (3-armors <-> single-armor)
  double entropy_enter = 0.75;
  double entropy_exit = 0.55;
  double max_prob_enter = 0.60;
  double max_prob_exit = 0.75;
  int stable_frames = 4;
  int z_history_window = 15;

  // Single-armor output confidence scaling
  double single_mode_confidence_scale = 0.70;

  // Binding engine controls (periodic evidence + transition confirmation)
  bool binding_enable_multi_obs = true;
  int binding_transition_confirm_frames = 3;
  double binding_same_panel_yaw_gate = 0.35;
  double binding_same_panel_z_gate = 0.08;
  double binding_same_panel_xy_gate = 0.18;
  double binding_min_candidate_prob = 0.40;
  double binding_min_candidate_margin = 0.12;
  double binding_switch_strong_score = 0.60;
  int binding_period_window = 12;
  double binding_period_weight = 0.60;
  double binding_period_min_spin_rate = 0.8;
  double binding_period_update_min_confidence = 0.55;
  double binding_period_update_min_jump = 0.015;
  double binding_dz_ema_alpha = 0.20;
  double binding_confidence_floor = 0.15;

  // Kinematic smoothing gains
  double alpha_pos = 0.65;
  double beta_vel = 0.30;
  double alpha_yaw = 0.60;
  double beta_yaw_rate = 0.25;

  // Physical constraints / damping
  bool assume_static_center = true;
  double linear_velocity_damping = 0.90;
  double yaw_rate_damping = 0.98;
  double max_center_speed = 1.00;
  double max_yaw_rate = 12.0;
};

struct ManeuverDetectionParameters {
  bool   enable                      = true;
  double nis_threshold_single        = 238.807;
  double nis_threshold_dual          = 4132.110;
  double innov_norm_threshold_single = 0.1279;
  double innov_norm_threshold_dual   = 0.0613;

  // Optional MAD-based outlier filter applied before threshold comparison.
  // When enabled, nis and innov_norm are each filtered through a rolling
  // window of mad_window samples per update_type before the decision rule.
  bool   mad_filter_enable = false;
  int    mad_window        = 10;    ///< rolling window size (samples)
  double mad_k             = 3.0;   ///< outlier threshold = mad_k * MAD
};

// ======================== Unified Config ========================

struct UnifiedConfig {
  FilterType filter_type = FilterType::DECOMPOSED;
  double dt = 0.05;

  UKFParameters ukf;
  MotionModelParameters motion;
  SpinModelParameters spin;
  MaxEntropyParameters entropy;
  TrackerParameters tracker;
  ConstraintParameters constraints;
  ManeuverDetectionParameters maneuver;
  PanelMismatchParameters panel_mismatch;
  OutpostParameters outpost;

  static UnifiedConfig create_default() { return UnifiedConfig{}; }

  static UnifiedConfig create_optimized() {
    UnifiedConfig config;
    config.motion.ca_process_noise_acc = 1.5;
    config.spin.spin_process_noise_delta_rate = 1.2;
    config.spin.spin_process_noise_delta_acc = 12.0;
    config.ukf.obs_noise_pos = 0.008;
    config.ukf.obs_noise_yaw = 0.015;
    config.motion.process_noise_r = 0.008;
    config.motion.process_noise_dz = 0.003;
    config.entropy.temperature = 1.5;
    config.entropy.k_prior_weight = 0.6;
    return config;
  }
};

/// Parse TranslationModel from string
inline TranslationModel translation_model_from_string(const std::string &s) {
  if (s == "CV" || s == "cv") return TranslationModel::CV;
  if (s == "CA" || s == "ca") return TranslationModel::CA;
  if (s == "Singer" || s == "singer" || s == "SINGER")
    return TranslationModel::SINGER;
  return TranslationModel::CA;  // default
}

/// Parse RotationModel from string
inline RotationModel rotation_model_from_string(const std::string &s) {
  if (s == "CV" || s == "cv") return RotationModel::CV;
  if (s == "CA" || s == "ca") return RotationModel::CA;
  return RotationModel::CV;  // default
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_CORE_CONFIG_HPP_
