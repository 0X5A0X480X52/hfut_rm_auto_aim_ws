// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_UTILS_CONSTRAINTS_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_CONSTRAINTS_HPP_

#include <Eigen/Dense>
#include <algorithm>

namespace fyt::auto_aim {

inline double apply_radius_constraint(double r, double min_r = 0.12,
                                      double max_r = 0.5) {
  return std::clamp(r, min_r, max_r);
}

inline double apply_dz_constraint(double dz, double min_dz = 0.0,
                                  double max_dz = 1.0) {
  return std::clamp(dz, min_dz, max_dz);
}

/// Apply state constraints (r1, r2, dza) by indices
inline Eigen::VectorXd apply_state_constraints(
    const Eigen::VectorXd &x, int r1_idx, int r2_idx, int dza_idx,
    double min_radius = 0.12, double max_radius = 0.5, double min_dz = 0.0,
    double max_dz = 1.0) {
  Eigen::VectorXd xc = x;
  xc(r1_idx) = apply_radius_constraint(xc(r1_idx), min_radius, max_radius);
  xc(r2_idx) = apply_radius_constraint(xc(r2_idx), min_radius, max_radius);
  xc(dza_idx) = apply_dz_constraint(xc(dza_idx), min_dz, max_dz);
  return xc;
}

/// Ensure a covariance matrix is symmetric positive-definite
inline Eigen::MatrixXd ensure_positive_definite(const Eigen::MatrixXd &P,
                                                double eps = 1e-6) {
  Eigen::MatrixXd Ps = 0.5 * (P + P.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(Ps);
  if (solver.info() != Eigen::Success) {
    // Fallback: add diagonal perturbation
    return Ps + Eigen::MatrixXd::Identity(Ps.rows(), Ps.cols()) * eps;
  }
  Eigen::VectorXd evals = solver.eigenvalues();
  evals = evals.cwiseMax(eps);  // clip negative eigenvalues
  Eigen::MatrixXd Ppd =
      solver.eigenvectors() * evals.asDiagonal() *
      solver.eigenvectors().transpose();
  Ppd = 0.5 * (Ppd + Ppd.transpose());
  Ppd += Eigen::MatrixXd::Identity(Ppd.rows(), Ppd.cols()) * 1e-12;
  return Ppd;
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_CONSTRAINTS_HPP_
