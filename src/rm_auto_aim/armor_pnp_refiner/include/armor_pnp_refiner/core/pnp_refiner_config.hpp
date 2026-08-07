#pragma once

#include <string>

namespace armor_pnp_refiner {

struct PnpRefinerConfig {
  // Supported modes: "none" and "single_xyz_yaw".
  std::string mode{"none"};

  int max_iterations{5};
  bool use_robust_kernel{true};

  double pixel_sigma{1.5};
  double huber_delta_px{3.0};

  double prior_sigma_xy{0.08};
  double prior_sigma_z{0.15};
  double prior_sigma_yaw_rad{0.10};

  double max_reproj_error_px{3.0};
  double max_pose_delta_m{0.20};
  double max_yaw_delta_rad{0.349065850398866};
  double max_chi2_per_dof{5.0};
  double max_condition_number{1e7};

  double min_var_x{0.01 * 0.01};
  double min_var_y{0.01 * 0.01};
  double min_var_z{0.02 * 0.02};
  double min_var_yaw{0.01 * 0.01};

  double good_confidence{0.75};
  double reject_confidence{0.40};

  bool require_positive_depth{true};
  bool require_finite{true};
};

}  // namespace armor_pnp_refiner
