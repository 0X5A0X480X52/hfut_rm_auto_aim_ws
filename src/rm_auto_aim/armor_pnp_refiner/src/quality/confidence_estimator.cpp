#include "armor_pnp_refiner/quality/confidence_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace armor_pnp_refiner {

double ConfidenceEstimator::compute(const PnpRefineOutput &output) {
  double c_r = cReproj(output.reproj_error_refined_px);
  double c_c = cCondition(output.condition_number > 0 ? output.condition_number : 1e3);
  double c_i = cInlier(output.num_inliers, output.num_points);
  double c_m = cImprovement(output.cost_before, output.cost_after);

  // Weighted product.
  double conf =
    std::pow(c_r, 0.40) * std::pow(c_c, 0.25) * std::pow(c_i, 0.20) * std::pow(c_m, 0.15);

  return std::clamp(conf, 0.0, 1.0);
}

double ConfidenceEstimator::cReproj(double reproj_rms, double sigma_ref) {
  if (reproj_rms <= 0) return 1.0;
  return std::exp(-0.5 * (reproj_rms * reproj_rms) / (sigma_ref * sigma_ref));
}

double ConfidenceEstimator::cCondition(double condition_number,
                                       double good_kappa,
                                       double bad_kappa) {
  if (condition_number <= good_kappa) return 1.0;
  if (condition_number >= bad_kappa) return 0.0;
  double c = std::log(bad_kappa / condition_number) / std::log(bad_kappa / good_kappa);
  return std::clamp(c, 0.0, 1.0);
}

double ConfidenceEstimator::cInlier(int num_inliers, int num_points) {
  if (num_points <= 0) return 0.0;
  return static_cast<double>(num_inliers) / static_cast<double>(num_points);
}

double ConfidenceEstimator::cImprovement(double cost_before, double cost_after) {
  if (cost_before <= 0) return 1.0;  // no change, neutral
  double ratio = (cost_before - cost_after) / cost_before;
  // Improvement of ~10% is good; negative improvement is bad.
  double expected = 0.10;
  return std::clamp(ratio / expected, 0.0, 1.0);
}

}  // namespace armor_pnp_refiner
