// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
//
// Robbins-Monro Stochastic Approximation Estimator
//
// Used for structural parameters (r1, r2, dza) that should converge
// to stable values over time.
//
// Core iteration:
//   θ_{n+1} = θ_n + a_n * (z_n - θ_n)
//
// Step-size schedule (Polyak-Ruppert):
//   a_n = c / (n + n0)^γ
//
// Properties:
//   - Σ a_n = ∞  (can reach any target)  when γ ≤ 1
//   - Σ a_n² < ∞ (noise dies out)        when γ > 0.5
//   - Choosing γ ∈ (0.5, 1] ensures both conditions
//
// Dual-observation boost:
//   When the UKF gets a dual-armor observation, r1/r2/dza are updated
//   via geometric constraints and are much more reliable.  We boost the
//   effective step size by `dual_obs_boost` in that case.

#ifndef MAX_ENTROPY_TRACKER_UTILS_ROBBINS_MONRO_ESTIMATOR_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_ROBBINS_MONRO_ESTIMATOR_HPP_

#include <algorithm>
#include <cmath>
#include <tuple>

namespace fyt::auto_aim {

// ──────────────────────────────────────────────────────────────────
//  RobbinsMonroEstimator  —  scalar stochastic approximation
// ──────────────────────────────────────────────────────────────────
class RobbinsMonroEstimator {
 public:
  struct Config {
    double initial_step = 0.5;      // c — initial step scale
    double gamma = 0.75;            // γ — decay exponent ∈ (0.5, 1]
    int n0 = 5;                     // offset to prevent huge early steps
    double dual_obs_boost = 3.0;    // multiplier for dual-observation steps
    double min_value = -1e9;        // lower clamp
    double max_value = 1e9;         // upper clamp
    double convergence_eps = 1e-4;  // step size below this → converged
  };

  RobbinsMonroEstimator() = default;
  explicit RobbinsMonroEstimator(const Config &cfg) : cfg_(cfg) {}

  /// Reset to a new initial value.  Step counter → 0.
  void reset(double initial_value) {
    theta_ = std::clamp(initial_value, cfg_.min_value, cfg_.max_value);
    n_ = 0;
  }

  /// Single Robbins-Monro update.
  /// @param z_obs  Observation (from UKF state after update)
  /// @param is_dual_obs  Whether this came from a dual-armor observation
  /// @return Updated estimate
  double update(double z_obs, bool is_dual_obs = false) {
    double a_n = step_size(is_dual_obs);
    double innovation = z_obs - theta_;
    theta_ += a_n * innovation;
    theta_ = std::clamp(theta_, cfg_.min_value, cfg_.max_value);
    ++n_;
    return theta_;
  }

  double estimate() const { return theta_; }
  int step_count() const { return n_; }

  /// Current step size (for diagnostics / convergence check)
  double current_step_size(bool is_dual = false) const {
    return step_size(is_dual);
  }

  bool is_converged() const {
    return step_size(false) < cfg_.convergence_eps;
  }

  // Accessors
  const Config &config() const { return cfg_; }
  Config &config() { return cfg_; }

 private:
  double step_size(bool is_dual) const {
    double base = cfg_.initial_step /
                  std::pow(static_cast<double>(n_ + cfg_.n0), cfg_.gamma);
    return is_dual ? base * cfg_.dual_obs_boost : base;
  }

  Config cfg_;
  double theta_{0.0};
  int n_{0};
};

// ──────────────────────────────────────────────────────────────────
//  StructuralRMEstimator  —  aggregates r1, r2, dza estimators
// ──────────────────────────────────────────────────────────────────
class StructuralRMEstimator {
 public:
  struct Config {
    double initial_step = 0.5;
    double gamma = 0.75;
    int n0 = 5;
    double dual_obs_boost = 3.0;
    double min_radius = 0.12;
    double max_radius = 0.5;
    double min_dz = -1.0;
    double max_dz = 1.0;
    double convergence_eps = 1e-4;
  };

  StructuralRMEstimator() = default;
  explicit StructuralRMEstimator(const Config &cfg) { configure(cfg); }

  void configure(const Config &cfg) {
    cfg_ = cfg;

    RobbinsMonroEstimator::Config r_cfg;
    r_cfg.initial_step = cfg.initial_step;
    r_cfg.gamma = cfg.gamma;
    r_cfg.n0 = cfg.n0;
    r_cfg.dual_obs_boost = cfg.dual_obs_boost;
    r_cfg.min_value = cfg.min_radius;
    r_cfg.max_value = cfg.max_radius;
    r_cfg.convergence_eps = cfg.convergence_eps;
    r1_est_ = RobbinsMonroEstimator(r_cfg);
    r2_est_ = RobbinsMonroEstimator(r_cfg);

    RobbinsMonroEstimator::Config dz_cfg = r_cfg;
    dz_cfg.min_value = cfg.min_dz;
    dz_cfg.max_value = cfg.max_dz;
    dza_est_ = RobbinsMonroEstimator(dz_cfg);
  }

  /// Initialize with first estimates (from UKF init or default params)
  void initialize(double r1, double r2, double dza) {
    r1_est_.reset(r1);
    r2_est_.reset(r2);
    dza_est_.reset(dza);
    initialized_ = true;
  }

  /// Update all three parameters with new observations
  void update(double r1_obs, double r2_obs, double dza_obs,
              bool is_dual_obs) {
    if (!initialized_) return;
    r1_est_.update(r1_obs, is_dual_obs);
    r2_est_.update(r2_obs, is_dual_obs);
    dza_est_.update(dza_obs, is_dual_obs);
  }

  /// Get current smoothed parameters
  std::tuple<double, double, double> get_smoothed_params() const {
    return {r1_est_.estimate(), r2_est_.estimate(), dza_est_.estimate()};
  }

  bool is_converged() const {
    return r1_est_.is_converged() && r2_est_.is_converged() &&
           dza_est_.is_converged();
  }

  bool is_initialized() const { return initialized_; }

  void reset() {
    r1_est_.reset(0.0);
    r2_est_.reset(0.0);
    dza_est_.reset(0.0);
    initialized_ = false;
  }

  // Diagnostics
  int r1_step_count() const { return r1_est_.step_count(); }
  int r2_step_count() const { return r2_est_.step_count(); }
  int dza_step_count() const { return dza_est_.step_count(); }

  const Config &config() const { return cfg_; }

 private:
  Config cfg_;
  RobbinsMonroEstimator r1_est_;
  RobbinsMonroEstimator r2_est_;
  RobbinsMonroEstimator dza_est_;
  bool initialized_{false};
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_ROBBINS_MONRO_ESTIMATOR_HPP_
