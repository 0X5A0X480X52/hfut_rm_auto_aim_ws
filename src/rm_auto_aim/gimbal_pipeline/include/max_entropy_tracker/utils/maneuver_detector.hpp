// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_UTILS_MANEUVER_DETECTOR_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_MANEUVER_DETECTOR_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"

namespace fyt::auto_aim {

/// Result of a single maneuver detection query.
struct ManeuverResult {
  bool   is_maneuvering = false;
  double nis            = -1.0;   ///< NIS value from last UKF update (-1 if no update)
  double innov_norm     = 0.0;    ///< Innovation xyz norm from last UKF update
  int    update_type    = 0;      ///< 0=none, 1=single-obs, 2=dual-obs
};

/**
 * Maneuver detector with optional rolling MAD outlier filter.
 *
 * Decision rule (stratified by update_type):
 *   update_type == 1: maneuvering if nis > nis_threshold_single
 *                     AND innov_norm > innov_norm_threshold_single
 *   update_type == 2: maneuvering if nis > nis_threshold_dual
 *                     AND innov_norm > innov_norm_threshold_dual
 *
 * When ManeuverDetectionParameters::mad_filter_enable is true, each incoming
 * nis (tracked per update_type separately) and innov_norm value is passed
 * through a rolling-window median + MAD outlier replacement before the
 * threshold comparison:
 *
 *   if |x - median(window)| > mad_k * MAD(window)  →  use median
 *   else                                            →  use x
 *
 * Thresholds are empirically derived at FPR ≤ 10 % from offline log analysis
 * (see scripts/analyze_maneuver_metrics.py).
 */
class ManeuverDetector {
 public:
  explicit ManeuverDetector(const ManeuverDetectionParameters &params)
      : params_(params) {}

  ManeuverResult detect(double nis, double innov_norm, int update_type) const {
    ManeuverResult r;
    r.nis         = nis;
    r.innov_norm  = innov_norm;
    r.update_type = update_type;

    if (!params_.enable || update_type == 0) return r;

    double filtered_nis        = nis;
    double filtered_innov_norm = innov_norm;

    if (params_.mad_filter_enable) {
      auto &nis_buf = (update_type == 1) ? nis_buf_single_ : nis_buf_dual_;
      filtered_nis        = push_and_filter(nis_buf, nis);
      filtered_innov_norm = push_and_filter(innov_buf_, innov_norm);
    }

    if (update_type == 1) {
      r.is_maneuvering = (filtered_nis > params_.nis_threshold_single &&
                          filtered_innov_norm > params_.innov_norm_threshold_single);
    } else if (update_type == 2) {
      r.is_maneuvering = (filtered_nis > params_.nis_threshold_dual &&
                          filtered_innov_norm > params_.innov_norm_threshold_dual);
    }
    return r;
  }

  void set_params(const ManeuverDetectionParameters &p) {
    params_ = p;
    // clear history so the new window/k take effect cleanly
    nis_buf_single_.clear();
    nis_buf_dual_.clear();
    innov_buf_.clear();
  }
  const ManeuverDetectionParameters &params() const { return params_; }

 private:
  ManeuverDetectionParameters params_;

  mutable std::deque<double> nis_buf_single_;  ///< rolling NIS window for update_type == 1
  mutable std::deque<double> nis_buf_dual_;    ///< rolling NIS window for update_type == 2
  mutable std::deque<double> innov_buf_;       ///< rolling innov_norm window (all update_types)

  /// Push a new sample into the buffer (capped at mad_window) and return the
  /// MAD-filtered value (original or median replacement).
  double push_and_filter(std::deque<double> &buf, double value) const {
    int win = std::max(1, params_.mad_window);
    buf.push_back(value);
    while (static_cast<int>(buf.size()) > win) buf.pop_front();

    // copy to a sorted vector to compute median and MAD
    std::vector<double> sorted(buf.begin(), buf.end());
    std::sort(sorted.begin(), sorted.end());
    const std::size_t n = sorted.size();
    double med = (n % 2 == 1)
                   ? sorted[n / 2]
                   : 0.5 * (sorted[n / 2 - 1] + sorted[n / 2]);

    std::vector<double> abs_dev(n);
    for (std::size_t i = 0; i < n; ++i) abs_dev[i] = std::abs(sorted[i] - med);
    std::sort(abs_dev.begin(), abs_dev.end());
    double mad = (n % 2 == 1)
                   ? abs_dev[n / 2]
                   : 0.5 * (abs_dev[n / 2 - 1] + abs_dev[n / 2]);

    if (mad < 1e-12) return value;
    return (std::abs(value - med) > params_.mad_k * mad) ? med : value;
  }
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_MANEUVER_DETECTOR_HPP_
