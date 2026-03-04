// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_UTILS_SIGMA_POINTS_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_SIGMA_POINTS_HPP_

#include <Eigen/Dense>
#include <stdexcept>
#include <vector>

namespace fyt::auto_aim {

/// Scaled Unscented Transform sigma-point generator
class SigmaPointGenerator {
 public:
  SigmaPointGenerator() = default;

  SigmaPointGenerator(int n, double alpha = 0.001, double beta = 2.0,
                      double kappa = 0.0)
      : n_(n), alpha_(alpha), beta_(beta), kappa_(kappa) {
    lambda_ = alpha * alpha * (n + kappa) - n;
    compute_weights();
  }

  /// Generate 2n+1 sigma points from (x, P)
  /// Returns matrix of shape (2n+1, n)
  Eigen::MatrixXd generate(const Eigen::VectorXd &x,
                           const Eigen::MatrixXd &P) const {
    const int n = n_;
    Eigen::MatrixXd sigma(2 * n + 1, n);
    sigma.row(0) = x.transpose();

    Eigen::MatrixXd scaled_P = (n + lambda_) * P;

    // Try Cholesky decomposition with progressive regularization
    Eigen::LLT<Eigen::MatrixXd> llt(scaled_P);
    if (llt.info() != Eigen::Success) {
      static const double reg_vals[] = {1e-6, 1e-5, 1e-4, 1e-3, 1e-2};
      bool success = false;
      for (double reg : reg_vals) {
        Eigen::MatrixXd P_reg =
            scaled_P + Eigen::MatrixXd::Identity(n, n) * reg;
        llt.compute(P_reg);
        if (llt.info() == Eigen::Success) {
          success = true;
          break;
        }
      }
      if (!success) {
        throw std::runtime_error(
            "Cholesky failed even after progressive regularization");
      }
    }
    Eigen::MatrixXd L = llt.matrixL();

    for (int i = 0; i < n; ++i) {
      sigma.row(i + 1) = x.transpose() + L.col(i).transpose();
      sigma.row(n + i + 1) = x.transpose() - L.col(i).transpose();
    }
    return sigma;
  }

  const Eigen::VectorXd &Wm() const { return Wm_; }
  const Eigen::VectorXd &Wc() const { return Wc_; }
  int n() const { return n_; }

 private:
  void compute_weights() {
    const int total = 2 * n_ + 1;
    Wm_.resize(total);
    Wc_.resize(total);

    Wm_(0) = lambda_ / (n_ + lambda_);
    Wc_(0) = Wm_(0) + (1.0 - alpha_ * alpha_ + beta_);
    for (int i = 1; i < total; ++i) {
      Wm_(i) = 0.5 / (n_ + lambda_);
      Wc_(i) = Wm_(i);
    }
  }

  int n_ = 0;
  double alpha_ = 0.001;
  double beta_ = 2.0;
  double kappa_ = 0.0;
  double lambda_ = 0.0;
  Eigen::VectorXd Wm_;
  Eigen::VectorXd Wc_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_SIGMA_POINTS_HPP_
