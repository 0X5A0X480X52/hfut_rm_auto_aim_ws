// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// ObservationOutlierFilter — pre-smoother outlier detection layer
//
// Sits between the UKF tracker output and OutputSmoother::smooth().
// When an outlier is detected the caller should HOLD the last valid
// SmoothedOutput instead of calling smooth(), so One-Euro filter
// internal state is never corrupted by spurious jumps.
//
// Three detector algorithms, selected by OutlierFilterConfig::method:
//
//   "mad"  (default)
//       Rolling median + MAD (median absolute deviation) on each of
//       x, y, z, yaw independently.  Outlier if
//           |val − median| > mad_k × MAD
//       for ANY axis.  Same algorithm as ManeuverDetector::push_and_filter()
//       in maneuver_detector.hpp.
//
//   "iqr"
//       Rolling interquartile-range test on each axis.  Outlier if
//           |val − median| > iqr_k × IQR
//       for ANY axis.
//
//   "mahalanobis"
//       Squared Mahalanobis distance on 3-D position against the rolling
//       window's sample mean and covariance.  Outlier if
//           d² = (pos − μ)ᵀ Σ⁻¹ (pos − μ) > mahal_threshold
//       Falls back to MAD (position axes only) when fewer than
//       min_samples are available or the covariance is singular.
//
// All incoming values are pushed into the window regardless of outlier
// status (robust estimators tolerate minority contamination).
// Yaw deviations are computed via the shortest angular difference
// (atan2(sin,cos)) to handle the ±π wrap-around boundary correctly.
//
// Independent switch: enable_outlier_filter in SmootherConfig is
// a separate flag from smoother.enable — both can be toggled freely.

#ifndef MAX_ENTROPY_TRACKER_UTILS_OBSERVATION_OUTLIER_FILTER_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_OBSERVATION_OUTLIER_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace fyt::auto_aim {

// ──────────────────────────────────────────────────────────────────
//  OutlierFilterConfig
// ──────────────────────────────────────────────────────────────────
struct OutlierFilterConfig {
  bool        enable{false};
  std::string method{"mad"};          // "mad" | "iqr" | "mahalanobis"
  int         window_size{10};        // rolling window length
  int         min_samples{5};         // start filtering only after N samples
  double      mad_k{3.5};             // |val − median| > mad_k × MAD  → outlier
  double      iqr_k{1.5};             // |val − median| > iqr_k × IQR  → outlier
  double      mahal_threshold{9.21};  // chi²(3,0.99)≈11.34; 0.95≈7.81; 0.90≈6.25
};

// ──────────────────────────────────────────────────────────────────
//  ObservationOutlierFilter  (header-only, no .cpp required)
// ──────────────────────────────────────────────────────────────────
class ObservationOutlierFilter {
 public:
  ObservationOutlierFilter() = default;
  explicit ObservationOutlierFilter(const OutlierFilterConfig &cfg)
      : cfg_(cfg) {}

  void configure(const OutlierFilterConfig &cfg) {
    cfg_ = cfg;
    reset();
  }

  /// Feed a new frame.  Returns true if (pos, yaw) is considered an outlier.
  /// Values are always pushed into the internal window.
  bool update(const Eigen::Vector3d &pos, double yaw) {
    // Push scalar windows
    push(wx_, pos.x());
    push(wy_, pos.y());
    push(wz_, pos.z());

    const double ny = normalize_angle(yaw);
    wyaw_.push_back(ny);
    if (static_cast<int>(wyaw_.size()) > cfg_.window_size) wyaw_.pop_front();

    // Push 3-D window (Mahalanobis)
    wpos_.push_back(pos);
    if (static_cast<int>(wpos_.size()) > cfg_.window_size) wpos_.pop_front();

    if (static_cast<int>(wx_.size()) < cfg_.min_samples) return false;

    if (cfg_.method == "iqr")         return detect_iqr(pos, ny);
    if (cfg_.method == "mahalanobis") return detect_mahalanobis(pos);
    return detect_mad(pos, ny);  // default: "mad"
  }

  void reset() {
    wx_.clear();
    wy_.clear();
    wz_.clear();
    wyaw_.clear();
    wpos_.clear();
  }

 private:
  OutlierFilterConfig          cfg_;
  std::deque<double>           wx_, wy_, wz_, wyaw_;
  std::deque<Eigen::Vector3d>  wpos_;

  // ── window helpers ──────────────────────────────────────────────
  void push(std::deque<double> &w, double v) {
    w.push_back(v);
    if (static_cast<int>(w.size()) > cfg_.window_size) w.pop_front();
  }

  static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  /// Shortest signed angular difference a − b, result ∈ (−π, π].
  static double ang_diff(double a, double b) {
    return std::atan2(std::sin(a - b), std::cos(a - b));
  }

  // ── generic statistics ──────────────────────────────────────────
  /// Median of a vector that is ALREADY SORTED.
  static double sorted_median(const std::vector<double> &s) {
    const std::size_t n = s.size();
    return (n % 2 == 1) ? s[n / 2]
                        : 0.5 * (s[n / 2 - 1] + s[n / 2]);
  }

  // ── MAD (same algorithm as maneuver_detector.hpp) ───────────────
  /// Returns true if val is an outlier in buf by the MAD criterion.
  static bool mad_outlier(const std::deque<double> &buf,
                          double val, double k) {
    std::vector<double> s(buf.begin(), buf.end());
    std::sort(s.begin(), s.end());
    const double med = sorted_median(s);
    const double dev = std::abs(val - med);

    std::vector<double> ad;
    ad.reserve(s.size());
    for (double v : s) ad.push_back(std::abs(v - med));
    std::sort(ad.begin(), ad.end());
    const double mad = sorted_median(ad);

    return mad > 1e-12 && dev > k * mad;
  }

  /// MAD variant that uses angular distance for deviation (for yaw).
  static bool mad_outlier_angle(const std::deque<double> &buf,
                                double val, double k) {
    std::vector<double> s(buf.begin(), buf.end());
    std::sort(s.begin(), s.end());
    const double med = sorted_median(s);
    const double dev = std::abs(ang_diff(val, med));

    std::vector<double> ad;
    ad.reserve(buf.size());
    for (double v : buf) ad.push_back(std::abs(ang_diff(v, med)));
    std::sort(ad.begin(), ad.end());
    const double mad = sorted_median(ad);

    return mad > 1e-12 && dev > k * mad;
  }

  bool detect_mad(const Eigen::Vector3d &pos, double ny) const {
    return mad_outlier(wx_, pos.x(), cfg_.mad_k) ||
           mad_outlier(wy_, pos.y(), cfg_.mad_k) ||
           mad_outlier(wz_, pos.z(), cfg_.mad_k) ||
           mad_outlier_angle(wyaw_, ny, cfg_.mad_k);
  }

  // ── IQR ─────────────────────────────────────────────────────────
  static bool iqr_outlier(const std::deque<double> &buf,
                           double val, double k) {
    std::vector<double> s(buf.begin(), buf.end());
    std::sort(s.begin(), s.end());
    const std::size_t n = s.size();
    const double q1  = s[n / 4];
    const double q3  = s[(3 * n) / 4];
    const double iqr = q3 - q1;
    const double med = sorted_median(s);
    return iqr > 1e-12 && std::abs(val - med) > k * iqr;
  }

  static bool iqr_outlier_angle(const std::deque<double> &buf,
                                 double val, double k) {
    std::vector<double> s(buf.begin(), buf.end());
    std::sort(s.begin(), s.end());
    const std::size_t n = s.size();
    const double q1  = s[n / 4];
    const double q3  = s[(3 * n) / 4];
    const double iqr = q3 - q1;
    const double med = sorted_median(s);
    return iqr > 1e-12 && std::abs(ang_diff(val, med)) > k * iqr;
  }

  bool detect_iqr(const Eigen::Vector3d &pos, double ny) const {
    return iqr_outlier(wx_, pos.x(), cfg_.iqr_k) ||
           iqr_outlier(wy_, pos.y(), cfg_.iqr_k) ||
           iqr_outlier(wz_, pos.z(), cfg_.iqr_k) ||
           iqr_outlier_angle(wyaw_, ny, cfg_.iqr_k);
  }

  // ── Mahalanobis (3-D position only) ─────────────────────────────
  bool detect_mahalanobis(const Eigen::Vector3d &pos) const {
    // Fallback to MAD when sample count is insufficient
    if (static_cast<int>(wpos_.size()) < cfg_.min_samples) {
      return mad_outlier(wx_, pos.x(), cfg_.mad_k) ||
             mad_outlier(wy_, pos.y(), cfg_.mad_k) ||
             mad_outlier(wz_, pos.z(), cfg_.mad_k);
    }

    const int m = static_cast<int>(wpos_.size());

    // Sample mean
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto &p : wpos_) mean += p;
    mean /= static_cast<double>(m);

    // Sample covariance  (unbiased, m − 1)
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto &p : wpos_) {
      const Eigen::Vector3d d = p - mean;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(m - 1);

    // Tikhonov regularisation — guard against near-singular matrices
    cov += Eigen::Matrix3d::Identity() * 1e-6;

    Eigen::Matrix3d cov_inv;
    bool invertible = false;
    cov.computeInverseWithCheck(cov_inv, invertible);
    if (!invertible) {
      // Fallback to MAD when covariance is singular
      return mad_outlier(wx_, pos.x(), cfg_.mad_k) ||
             mad_outlier(wy_, pos.y(), cfg_.mad_k) ||
             mad_outlier(wz_, pos.z(), cfg_.mad_k);
    }

    const Eigen::Vector3d d = pos - mean;
    const double d2 = d.transpose() * cov_inv * d;
    return d2 > cfg_.mahal_threshold;
  }
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_OBSERVATION_OUTLIER_FILTER_HPP_
