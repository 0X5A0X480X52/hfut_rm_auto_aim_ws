// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_BASE_UKF_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_BASE_UKF_HPP_

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/utils/constraints.hpp"
#include "max_entropy_tracker/utils/sigma_points.hpp"

namespace fyt::auto_aim {

/// Abstract UKF base class.
class BaseUKF {
 public:
  explicit BaseUKF(const UnifiedConfig &config, double dt = 0.05)
      : config_(config), dt_(dt) {}

  virtual ~BaseUKF() = default;

  /* ---------- pure virtual ---------- */
  virtual int state_dim() const = 0;
  virtual int obs_dim() const = 0;

  virtual void initialize(const std::vector<ObservationData> &observations,
                          double r1 = 0.15, double r2 = 0.20,
                          double dza = 0.0,
                          int panel_id = -1) = 0;

  virtual void predict(std::optional<double> dt = std::nullopt) = 0;

  virtual bool update(const std::vector<ObservationData> &observations,
                      const std::vector<std::string> &r_types = {},
                      const std::vector<std::string> &armor_layers = {},
                      double height_confidence = 1.0,
                      double position_confidence = 1.0,
                      double panel_angle = 0.0) = 0;

  /* ---------- common accessors ---------- */
  const Eigen::VectorXd &x() const { return x_; }
  Eigen::VectorXd &x() { return x_; }
  const Eigen::MatrixXd &P() const { return P_; }
  Eigen::MatrixXd &P() { return P_; }
  bool initialized() const { return initialized_; }
  double dt() const { return dt_; }
  const UnifiedConfig &config() const { return config_; }

 protected:
  /* ---------- sigma helpers ---------- */
  void init_sigma_generator() {
    sigma_gen_ = std::make_unique<SigmaPointGenerator>(
        state_dim(), config_.ukf.alpha, config_.ukf.beta, config_.ukf.kappa);
  }

  Eigen::MatrixXd generate_sigma_points(const Eigen::VectorXd &x,
                                        const Eigen::MatrixXd &P) {
    if (!sigma_gen_) init_sigma_generator();
    Eigen::MatrixXd P_safe = ensure_positive_definite(P, 1e-6);
    return sigma_gen_->generate(x, P_safe);
  }

  std::pair<Eigen::VectorXd, Eigen::VectorXd> get_sigma_weights() {
    if (!sigma_gen_) init_sigma_generator();
    return {sigma_gen_->Wm(), sigma_gen_->Wc()};
  }

  void ensure_covariance_valid() { P_ = ensure_positive_definite(P_); }

  /* ---------- Kalman gain ---------- */
  std::optional<Eigen::MatrixXd> compute_kalman_gain(
      const Eigen::MatrixXd &Pxz, const Eigen::MatrixXd &Pzz) {
    Eigen::MatrixXd Pzz_inv = Pzz.inverse();
    if (Pzz_inv.hasNaN()) return std::nullopt;
    return Pxz * Pzz_inv;
  }

  /// Apply standard Kalman update: x += K * innov, P -= K * Pzz * K^T
  void apply_kalman_update(const Eigen::MatrixXd &K,
                           const Eigen::VectorXd &innovation,
                           const Eigen::MatrixXd &Pzz) {
    Eigen::VectorXd delta = K * innovation;
    x_ += delta;
    P_ -= K * Pzz * K.transpose();
    ensure_covariance_valid();
  }

  /* ---------- state ---------- */
  UnifiedConfig config_;
  double dt_;
  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  bool initialized_ = false;
  std::unique_ptr<SigmaPointGenerator> sigma_gen_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_BASE_UKF_HPP_
