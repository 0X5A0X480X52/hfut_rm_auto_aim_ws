// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
//
// 1€ (One Euro) Filter — Adaptive low-pass filter for noisy input
//
// Paper: "1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input
//         in Interactive Systems" (Casiez et al., 2012)
//
// Core idea:
//   - Slow / stationary → low cutoff frequency → strong smoothing
//   - Fast motion       → high cutoff frequency → weak smoothing (preserve responsiveness)
//
// Cutoff adaptation:  f_c = min_cutoff + beta * |dx/dt|
// Smoothing factor:   alpha = 1 / (1 + tau / Te),  tau = 1/(2*pi*f_c),  Te = 1/freq

#ifndef MAX_ENTROPY_TRACKER_UTILS_ONE_EURO_FILTER_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_ONE_EURO_FILTER_HPP_

#include <cmath>
#include <optional>

#include <Eigen/Dense>

namespace fyt::auto_aim {

// ──────────────────────────────────────────────────────────────────
//  LowPassFilter  —  first-order exponential moving average
// ──────────────────────────────────────────────────────────────────
class LowPassFilter {
 public:
  LowPassFilter() = default;

  double filter(double x, double alpha) {
    if (!initialized_) {
      y_ = x;
      initialized_ = true;
    } else {
      y_ = alpha * x + (1.0 - alpha) * y_;
    }
    return y_;
  }

  double value() const { return y_; }
  bool initialized() const { return initialized_; }

  void reset() {
    y_ = 0.0;
    initialized_ = false;
  }

 private:
  double y_{0.0};
  bool initialized_{false};
};

// ──────────────────────────────────────────────────────────────────
//  OneEuroFilter  —  scalar 1€ filter
// ──────────────────────────────────────────────────────────────────
class OneEuroFilter {
 public:
  OneEuroFilter(double freq = 30.0, double min_cutoff = 1.0,
                double beta = 0.007, double d_cutoff = 1.0)
      : freq_(freq),
        min_cutoff_(min_cutoff),
        beta_(beta),
        d_cutoff_(d_cutoff) {}

  /// Filter a new sample.  If @p timestamp is provided, the sampling
  /// frequency is automatically updated from consecutive timestamps.
  double filter(double x, std::optional<double> timestamp = std::nullopt) {
    // Update frequency from timestamps
    if (timestamp.has_value() && last_time_.has_value()) {
      double dt = timestamp.value() - last_time_.value();
      if (dt > 1e-6) freq_ = 1.0 / dt;
    }
    if (timestamp.has_value()) last_time_ = timestamp;

    // Derivative estimation
    double dx = x_filter_.initialized()
                    ? (x - x_filter_.value()) * freq_
                    : 0.0;
    double alpha_d = compute_alpha(d_cutoff_);
    double dx_hat = dx_filter_.filter(dx, alpha_d);

    // Adaptive cutoff
    double cutoff = min_cutoff_ + beta_ * std::abs(dx_hat);
    double alpha = compute_alpha(cutoff);
    return x_filter_.filter(x, alpha);
  }

  double value() const { return x_filter_.value(); }

  void reset() {
    x_filter_.reset();
    dx_filter_.reset();
    last_time_.reset();
  }

  // Parameter accessors
  void set_min_cutoff(double v) { min_cutoff_ = v; }
  void set_beta(double v) { beta_ = v; }
  void set_d_cutoff(double v) { d_cutoff_ = v; }
  void set_freq(double v) { freq_ = v; }

 private:
  double compute_alpha(double cutoff) const {
    double tau = 1.0 / (2.0 * M_PI * cutoff);
    double te = 1.0 / freq_;
    return 1.0 / (1.0 + tau / te);
  }

  double freq_;
  double min_cutoff_;
  double beta_;
  double d_cutoff_;

  LowPassFilter x_filter_;
  LowPassFilter dx_filter_;
  std::optional<double> last_time_;
};

// ──────────────────────────────────────────────────────────────────
//  OneEuroFilter3D  —  independent 1€ filter per axis (x, y, z)
// ──────────────────────────────────────────────────────────────────
class OneEuroFilter3D {
 public:
  OneEuroFilter3D(double freq = 30.0, double min_cutoff = 1.0,
                  double beta = 0.007, double d_cutoff = 1.0)
      : filters_{OneEuroFilter(freq, min_cutoff, beta, d_cutoff),
                 OneEuroFilter(freq, min_cutoff, beta, d_cutoff),
                 OneEuroFilter(freq, min_cutoff, beta, d_cutoff)} {}

  Eigen::Vector3d filter(const Eigen::Vector3d &x,
                         std::optional<double> timestamp = std::nullopt) {
    return {filters_[0].filter(x(0), timestamp),
            filters_[1].filter(x(1), timestamp),
            filters_[2].filter(x(2), timestamp)};
  }

  Eigen::Vector3d value() const {
    return {filters_[0].value(), filters_[1].value(), filters_[2].value()};
  }

  void reset() {
    for (auto &f : filters_) f.reset();
  }

  void set_params(double min_cutoff, double beta, double d_cutoff) {
    for (auto &f : filters_) {
      f.set_min_cutoff(min_cutoff);
      f.set_beta(beta);
      f.set_d_cutoff(d_cutoff);
    }
  }

 private:
  OneEuroFilter filters_[3];
};

// ──────────────────────────────────────────────────────────────────
//  OneEuroFilterAngle  —  1€ filter for angles with wrap-around
// ──────────────────────────────────────────────────────────────────
class OneEuroFilterAngle {
 public:
  OneEuroFilterAngle(double freq = 30.0, double min_cutoff = 1.0,
                     double beta = 0.001, double d_cutoff = 1.0)
      : freq_(freq),
        min_cutoff_(min_cutoff),
        beta_(beta),
        d_cutoff_(d_cutoff) {}

  double filter(double x, std::optional<double> timestamp = std::nullopt) {
    // Update frequency from timestamps
    if (timestamp.has_value() && last_time_.has_value()) {
      double dt = timestamp.value() - last_time_.value();
      if (dt > 1e-6) freq_ = 1.0 / dt;
    }
    if (timestamp.has_value()) last_time_ = timestamp;

    x = normalize(x);

    if (!initialized_) {
      value_ = x;
      dx_filter_.filter(0.0, 1.0);
      initialized_ = true;
      return value_;
    }

    // Shortest angular difference
    double diff = shortest_angle_diff(x, value_);
    double dx = diff * freq_;

    double alpha_d = compute_alpha(d_cutoff_);
    double dx_hat = dx_filter_.filter(dx, alpha_d);

    double cutoff = min_cutoff_ + beta_ * std::abs(dx_hat);
    double alpha = compute_alpha(cutoff);

    value_ = normalize(value_ + alpha * diff);
    return value_;
  }

  double value() const { return initialized_ ? value_ : 0.0; }

  void reset() {
    value_ = 0.0;
    initialized_ = false;
    dx_filter_.reset();
    last_time_.reset();
  }

  void set_min_cutoff(double v) { min_cutoff_ = v; }
  void set_beta(double v) { beta_ = v; }
  void set_d_cutoff(double v) { d_cutoff_ = v; }
  void set_freq(double v) { freq_ = v; }

 private:
  double compute_alpha(double cutoff) const {
    double tau = 1.0 / (2.0 * M_PI * cutoff);
    double te = 1.0 / freq_;
    return 1.0 / (1.0 + tau / te);
  }

  static double normalize(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static double shortest_angle_diff(double target, double source) {
    return std::atan2(std::sin(target - source),
                      std::cos(target - source));
  }

  double freq_;
  double min_cutoff_;
  double beta_;
  double d_cutoff_;

  double value_{0.0};
  bool initialized_{false};
  LowPassFilter dx_filter_;
  std::optional<double> last_time_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_ONE_EURO_FILTER_HPP_
