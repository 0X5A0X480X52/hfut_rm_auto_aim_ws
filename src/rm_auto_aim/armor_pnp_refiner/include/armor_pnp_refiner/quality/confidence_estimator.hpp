#pragma once

#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"

namespace armor_pnp_refiner {

// Compute composite confidence score from optimization diagnostics.
// confidence = c_reproj^0.40 * c_condition^0.25 * c_inlier^0.20 * c_improvement^0.15
class ConfidenceEstimator {
public:
  ConfidenceEstimator() = default;

  double compute(const PnpRefineOutput &output);

  // Individual components (exposed for logging).
  static double cReproj(double reproj_rms, double sigma_ref = 3.0);
  static double cCondition(double condition_number,
                           double good_kappa = 1e3,
                           double bad_kappa = 1e7);
  static double cInlier(int num_inliers, int num_points);
  static double cImprovement(double cost_before, double cost_after);
};

}  // namespace armor_pnp_refiner
