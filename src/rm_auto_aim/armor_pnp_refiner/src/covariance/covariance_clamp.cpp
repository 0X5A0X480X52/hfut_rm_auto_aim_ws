#include "armor_pnp_refiner/covariance/covariance_clamp.hpp"

#include <Eigen/Eigenvalues>
#include <cmath>
#include <algorithm>

namespace armor_pnp_refiner {

CovarianceCheckResult checkCovariance(const Eigen::Matrix4d& cov,
                                       double max_condition_number)
{
  CovarianceCheckResult result;

  // Finite check.
  result.finite = cov.allFinite();
  if (!result.finite) {
    result.reason = "non-finite values in covariance";
    return result;
  }

  // Symmetry check.
  result.symmetric = cov.isApprox(cov.transpose(), 1e-10);
  if (!result.symmetric) {
    result.reason = "covariance not symmetric";
  }

  // Eigenvalue decomposition.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(
      enforceSymmetry(cov));
  if (solver.info() != Eigen::Success) {
    result.reason = "eigenvalue decomposition failed";
    return result;
  }

  auto eigenvalues = solver.eigenvalues();
  result.min_eigenvalue = eigenvalues.minCoeff();
  result.max_eigenvalue = eigenvalues.maxCoeff();

  result.positive_definite = result.min_eigenvalue > 1e-12;

  if (result.min_eigenvalue > 1e-12 && result.max_eigenvalue > 0) {
    result.condition_number = result.max_eigenvalue / result.min_eigenvalue;
  } else {
    result.condition_number = 1e9;
  }

  result.passed = result.finite && result.symmetric &&
                  result.positive_definite &&
                  result.condition_number < max_condition_number;

  if (!result.passed && result.reason.empty()) {
    if (!result.positive_definite)
      result.reason = "covariance not positive definite";
    else if (result.condition_number >= max_condition_number)
      result.reason = "condition number too large";
  }

  return result;
}

Eigen::Matrix4d clampEigenvalues(const Eigen::Matrix4d& cov,
                                  double min_eigenvalue,
                                  double max_eigenvalue)
{
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(
      enforceSymmetry(cov));
  if (solver.info() != Eigen::Success) {
    return cov;
  }

  auto eigenvalues = solver.eigenvalues();
  auto eigenvectors = solver.eigenvectors();

  for (int i = 0; i < 4; ++i) {
    eigenvalues[i] = std::clamp(eigenvalues[i], min_eigenvalue, max_eigenvalue);
  }

  return eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
}

Eigen::Matrix4d enforceSymmetry(const Eigen::Matrix4d& cov)
{
  return (cov + cov.transpose()) / 2.0;
}

}  // namespace armor_pnp_refiner
