#pragma once

#include <Eigen/Dense>

namespace armor_pnp_refiner {

// Diagnostics after covariance checks.
struct CovarianceCheckResult {
  bool finite{false};
  bool symmetric{false};
  bool positive_definite{false};
  double condition_number{0.0};
  double min_eigenvalue{0.0};
  double max_eigenvalue{0.0};
  bool passed{false};
  std::string reason;
};

// Run all safety checks on a 4x4 covariance matrix.
CovarianceCheckResult checkCovariance(const Eigen::Matrix4d& cov,
                                       double max_condition_number = 1e7);

// Clamp eigenvalues to enforce positive definiteness and bounds.
Eigen::Matrix4d clampEigenvalues(const Eigen::Matrix4d& cov,
                                  double min_eigenvalue = 1e-12,
                                  double max_eigenvalue = 1e6);

// Ensure symmetry: (M + M^T) / 2.
Eigen::Matrix4d enforceSymmetry(const Eigen::Matrix4d& cov);

}  // namespace armor_pnp_refiner
