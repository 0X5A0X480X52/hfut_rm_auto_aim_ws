// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_STRUCTURE_PROVIDER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_STRUCTURE_PROVIDER_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/trackers/norm4_v3/models/norm4_motion_model_bundle.hpp"

namespace fyt::auto_aim::norm4_v3 {

class IStructureProvider {
 public:
  virtual ~IStructureProvider() = default;

  virtual Eigen::Vector3d get_structure() const = 0;

  /// Update structure estimate from posterior.
  /// @param is_dual  true for dual-armor update (more trust), false for single
  virtual void update(const Eigen::VectorXd &x_post,
                       const Eigen::MatrixXd &P_post, int best_k,
                       double dt, bool is_dual) = 0;

  virtual bool converged() const = 0;
};

/// Reads structural parameters directly from the UKF/InEKF posterior.
/// This mirrors V1 behavior: R1, R2, DZA are part of the main state and
/// updated at the same rate as fast states.
class UkfSnapshotStructureProvider : public IStructureProvider {
 public:
  explicit UkfSnapshotStructureProvider(
      const IMotionModelBundle &motion_bundle)
      : state_idx_(motion_bundle.state_idx()) {}

  Eigen::Vector3d get_structure() const override { return structure_; }

  void update(const Eigen::VectorXd &x_post, const Eigen::MatrixXd &,
              int, double, bool) override {
    structure_(0) = x_post(state_idx_.R1());
    structure_(1) = x_post(state_idx_.R2());
    structure_(2) = x_post(state_idx_.DZA());
  }

  bool converged() const override { return true; }

 private:
  Eigen::Vector3d structure_{0.15, 0.20, 0.0};
  DynamicStateIndex state_idx_;
};

/// Full slow structure error updater.
///
/// Maintains a separate Kalman filter on [R1, R2, DZA] with:
///   - Tiny process noise Q_θ (slow time scale)
///   - Structural prior N(θ₀, Σ_θ) as pseudo-measurement
///   - Λ_θ gain scaling (single: near-zero, dual: conservative)
///   - Physical range clamp
///
/// Gating conditions (checked by caller):
///   - Top1 NIS passes gate
///   - confidence_top1 high enough
///   - Top1/Top2 margin sufficient
///   - Not ambiguous mode
///   - Not early initialization frames
class SlowStructureErrorUpdaterProvider : public IStructureProvider {
 public:
  struct Config {
    // ── Slow process noise Q_θ ──
    double q_theta_r1 = 1.0e-6;
    double q_theta_r2 = 1.0e-6;
    double q_theta_dza = 5.0e-7;

    // ── Structural prior N(θ₀, Σ_θ) ──
    double prior_r1 = 0.15;
    double prior_r2 = 0.20;
    double prior_dza = 0.0;
    double prior_sigma_r = 0.06;
    double prior_sigma_dza = 0.06;

    // ── Λ_θ: per-parameter update gain ──
    double alpha_r1_single = 0.00;
    double alpha_r2_single = 0.00;
    double alpha_dza_single = 0.00;
    double alpha_r1_dual = 0.05;
    double alpha_r2_dual = 0.05;
    double alpha_dza_dual = 0.02;

    // ── Prior pull strength (Kalman gain toward θ₀ each update) ──
    double prior_pull_gain = 0.002;

    // ── Physical bounds ──
    double min_r = 0.05;
    double max_r = 0.50;
    double min_dza = 0.0;
    double max_dza = 0.12;
  };

  explicit SlowStructureErrorUpdaterProvider(
      const Config &cfg, const IMotionModelBundle &motion_bundle)
      : cfg_(cfg), state_idx_(motion_bundle.state_idx()) {
    theta_ << cfg_.prior_r1, cfg_.prior_r2, cfg_.prior_dza;
    P_theta_ = Eigen::Vector3d(cfg_.prior_sigma_r * cfg_.prior_sigma_r,
                                cfg_.prior_sigma_r * cfg_.prior_sigma_r,
                                cfg_.prior_sigma_dza * cfg_.prior_sigma_dza)
                   .asDiagonal();
  }

  Eigen::Vector3d get_structure() const override { return theta_; }

  void update(const Eigen::VectorXd &x_post, const Eigen::MatrixXd &P_post,
              int, double dt, bool is_dual) override {
    ++frames_;

    auto idx = state_idx_;

    // ── Predict: random walk with tiny Q_θ ──
    Eigen::Vector3d q_diag(cfg_.q_theta_r1 * dt, cfg_.q_theta_r2 * dt,
                            cfg_.q_theta_dza * dt);
    P_theta_ += q_diag.asDiagonal();

    // ── Measurement: structure from posterior ──
    Eigen::Vector3d z_theta(x_post(idx.R1()), x_post(idx.R2()),
                             x_post(idx.DZA()));

    // Measurement covariance from posterior sub-block
    Eigen::Matrix3d R_meas;
    R_meas << P_post(idx.R1(), idx.R1()), P_post(idx.R1(), idx.R2()),
        P_post(idx.R1(), idx.DZA()), P_post(idx.R2(), idx.R1()),
        P_post(idx.R2(), idx.R2()), P_post(idx.R2(), idx.DZA()),
        P_post(idx.DZA(), idx.R1()), P_post(idx.DZA(), idx.R2()),
        P_post(idx.DZA(), idx.DZA());

    // ── Λ_θ diagonal gain ──
    Eigen::Vector3d alpha;
    if (is_dual) {
      alpha << cfg_.alpha_r1_dual, cfg_.alpha_r2_dual, cfg_.alpha_dza_dual;
    } else {
      alpha << cfg_.alpha_r1_single, cfg_.alpha_r2_single,
          cfg_.alpha_dza_single;
    }

    // Innovation: structure posterior − prior estimate
    Eigen::Vector3d innov = z_theta - theta_;

    // Slow Kalman update: K = P_θ (P_θ + R_meas)⁻¹
    Eigen::Matrix3d S_theta = P_theta_ + R_meas;
    // Use scalar division for diagonal-dominant cases, full solve for general
    Eigen::Matrix3d K_theta;
    if (S_theta.determinant() > 1e-12) {
      K_theta = P_theta_ * S_theta.inverse();
    } else {
      // Degenerate: use diagonal approximation
      K_theta = Eigen::Matrix3d::Zero();
      for (int i = 0; i < 3; ++i) {
        if (S_theta(i, i) > 1e-10)
          K_theta(i, i) = P_theta_(i, i) / S_theta(i, i);
      }
    }

    // Apply Λ_θ scaling to the correction
    Eigen::Vector3d correction = K_theta * innov;
    correction(0) *= alpha(0);
    correction(1) *= alpha(1);
    correction(2) *= alpha(2);

    theta_ += correction;

    // Joseph-form covariance update for structure
    Eigen::Matrix3d I_KH = Eigen::Matrix3d::Identity() - K_theta;
    P_theta_ = I_KH * P_theta_ * I_KH.transpose() +
               K_theta * R_meas * K_theta.transpose();

    // ── Prior pseudo-measurement pull ──
    Eigen::Vector3d innov_prior = theta_prior() - theta_;
    theta_ += cfg_.prior_pull_gain * innov_prior;

    // ── Physical clamp ──
    theta_(0) = std::clamp(theta_(0), cfg_.min_r, cfg_.max_r);
    theta_(1) = std::clamp(theta_(1), cfg_.min_r, cfg_.max_r);
    theta_(2) = std::clamp(theta_(2), cfg_.min_dza, cfg_.max_dza);

    // ── Write back to posterior if converged (for external readers) ──
    converged_ = frames_ > 20;
  }

  bool converged() const override { return converged_; }

 private:
  Eigen::Vector3d theta_prior() const {
    return {cfg_.prior_r1, cfg_.prior_r2, cfg_.prior_dza};
  }

  Config cfg_;
  DynamicStateIndex state_idx_;
  Eigen::Vector3d theta_;
  Eigen::Matrix3d P_theta_;
  int frames_ = 0;
  bool converged_ = false;
};

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_STRUCTURE_PROVIDER_HPP_
