#pragma once

namespace auto_buff {

// Inlined from confs/Basic.hpp
struct Vec2d {
  double x;
  double y;
};
struct Vec3d {
  double x;
  double y;
  double z;
};

struct BuffBladeMatchConfig {
  double max_match_distance_m;
  double max_match_roll_diff_degree;
};

struct BuffBladeNoiseConfig {
  Vec2d pixel_error;
  Vec3d position_noise_m;
  double roll_noise_degree;
};

struct BuffCenterNoiseConfig {
  Vec3d position_consistency_noise_m;
  double roll_noise_degree;
  double vroll_noise_rad;
  Vec3d position_prior_noise_m;
  double roll_prior_noise_degree;
  double vroll_prior_noise_rad;
};

struct BuffFitterConfig {
  int queue_upper_limit;
  int queue_lower_limit;
  double param_lower_bound_scale{0.5};
  double param_upper_bound_scale{1.5};
  int curve_fitting_interval_time_ms;
};

struct SmallBuffConfig {
  BuffBladeMatchConfig match_conf;
  BuffBladeNoiseConfig blade_conf;
  BuffCenterNoiseConfig center_conf;
  double lost_threshold_sec;
};

struct BigBuffConfig {
  BuffFitterConfig fitter_conf;
  BuffBladeMatchConfig match_conf;
  BuffBladeNoiseConfig blade_conf;
  BuffCenterNoiseConfig center_conf;
  double lost_threshold_sec;
};

} // namespace auto_buff
