#pragma once

#include <Eigen/Dense>
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"

namespace g2o { class SparseOptimizer; }

namespace armor_pnp_refiner {

// Computes marginal covariance for the current-frame xyz-yaw state
// from the Hessian at the final linearization point.
class CovarianceEstimator {
public:
  explicit CovarianceEstimator(const PnpRefinerConfig& config);

  // Compute R_ba = sigma_px^2 * marginal_cov for single-frame optimization.
  // chi2/dof scale correction is applied.
  Eigen::Matrix4d computeSingleFrame(
      const g2o::SparseOptimizer& optimizer,
      int vertex_id,
      double chi2,
      int dof,
      const PnpCovarianceDiagnostics& diagnostics);

  // Compute current frame marginal covariance for sliding window.
  // Marginalizes out old frame states to get current-frame covariance.
  Eigen::Matrix4d computeWindowMarginal(
      const g2o::SparseOptimizer& optimizer,
      int current_vertex_id,
      int num_vertices,
      double chi2,
      int dof);

  // Apply R_floor and residual scale to raw BA covariance.
  Eigen::Matrix4d buildFinalCovariance(
      const Eigen::Matrix4d& R_ba_raw,
      double residual_scale,
      double condition_number,
      double confidence);

private:
  PnpRefinerConfig config_;
};

}  // namespace armor_pnp_refiner
