#include "armor_pnp_refiner/covariance/covariance_estimator.hpp"

#include <g2o/core/sparse_optimizer.h>
#include "armor_pnp_refiner/covariance/covariance_clamp.hpp"

namespace armor_pnp_refiner {

CovarianceEstimator::CovarianceEstimator(const PnpRefinerConfig& config)
  : config_(config) {}

Eigen::Matrix4d CovarianceEstimator::computeSingleFrame(
    const g2o::SparseOptimizer& optimizer,
    int vertex_id,
    double chi2,
    int dof,
    const PnpCovarianceDiagnostics& diagnostics)
{
  // For single-frame xyz-yaw, attempt to extract marginal covariance.
  // Fall back to conservative diagonal if Hessian is not accessible.
  Eigen::Matrix4d R_ba_raw = Eigen::Matrix4d::Identity();

  // Try to get marginal covariance from g2o.
  // g2o::SparseOptimizer::computeMarginals provides this.
  // For simplicity in Phase1, use conservative diagonal.
  double sigma_px = config_.pixel_sigma;
  R_ba_raw(0, 0) = sigma_px * sigma_px * config_.prior_sigma_xy * config_.prior_sigma_xy;
  R_ba_raw(1, 1) = sigma_px * sigma_px * config_.prior_sigma_xy * config_.prior_sigma_xy;
  R_ba_raw(2, 2) = sigma_px * sigma_px * config_.prior_sigma_z  * config_.prior_sigma_z;
  R_ba_raw(3, 3) = sigma_px * sigma_px * config_.prior_sigma_yaw_rad * config_.prior_sigma_yaw_rad;

  double residual_scale = std::max(1.0, (dof > 0 ? chi2 / dof : chi2));
  return buildFinalCovariance(R_ba_raw, residual_scale,
                               diagnostics.condition_number, 0.5);
}

Eigen::Matrix4d CovarianceEstimator::computeWindowMarginal(
    const g2o::SparseOptimizer& optimizer,
    int current_vertex_id,
    int num_vertices,
    double chi2,
    int dof)
{
  // For sliding window, marginalize out old frames.
  // Phase2 initial: conservative approximation.
  Eigen::Matrix4d R_ba_raw = Eigen::Matrix4d::Identity();
  double sigma_px = config_.pixel_sigma;
  R_ba_raw(0, 0) = sigma_px * sigma_px * config_.prior_sigma_xy * config_.prior_sigma_xy;
  R_ba_raw(1, 1) = sigma_px * sigma_px * config_.prior_sigma_xy * config_.prior_sigma_xy;
  R_ba_raw(2, 2) = sigma_px * sigma_px * config_.prior_sigma_z  * config_.prior_sigma_z;
  R_ba_raw(3, 3) = sigma_px * sigma_px * config_.prior_sigma_yaw_rad * config_.prior_sigma_yaw_rad;

  // Window prior makes covariance optimistic - inflate.
  double inflate = std::sqrt(static_cast<double>(num_vertices));
  R_ba_raw *= inflate;

  double residual_scale = std::max(1.0, (dof > 0 ? chi2 / dof : chi2));
  return buildFinalCovariance(R_ba_raw, residual_scale, 1e4, 0.5);
}

Eigen::Matrix4d CovarianceEstimator::buildFinalCovariance(
    const Eigen::Matrix4d& R_ba_raw,
    double residual_scale,
    double condition_number,
    double confidence)
{
  // R_final = R_floor + scale * R_ba_raw
  Eigen::Matrix4d R_final = residual_scale * R_ba_raw;

  // Add floor.
  R_final(0, 0) += config_.min_var_x;
  R_final(1, 1) += config_.min_var_y;
  R_final(2, 2) += config_.min_var_z;
  R_final(3, 3) += config_.min_var_yaw;

  // Inflate if condition number is poor.
  if (condition_number > 1e6) {
    R_final *= 2.0;
  }

  // Inflate if confidence is low.
  if (confidence < config_.good_confidence) {
    double inflate = 1.0 / std::max(confidence, 0.2);
    R_final *= inflate;
  }

  // Ensure validity.
  auto check = checkCovariance(R_final, config_.max_condition_number);
  if (!check.passed) {
    R_final = clampEigenvalues(enforceSymmetry(R_final));
  }

  return R_final;
}

}  // namespace armor_pnp_refiner
