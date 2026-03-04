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
};

struct ConstraintParameters {
  double min_radius = 0.12;
  double max_radius = 0.5;
  double min_dz = -1.0;
  double max_dz = 1.0;
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

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_CORE_CONFIG_HPP_
