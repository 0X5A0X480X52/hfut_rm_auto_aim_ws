// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_UTILS_MANEUVER_DETECTOR_HPP_
#define MAX_ENTROPY_TRACKER_UTILS_MANEUVER_DETECTOR_HPP_

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
 * Stateless maneuver detector.
 *
 * Decision rule (stratified by update_type):
 *   update_type == 1: maneuvering if nis > nis_threshold_single
 *                     AND innov_norm > innov_norm_threshold_single
 *   update_type == 2: maneuvering if nis > nis_threshold_dual
 *                     AND innov_norm > innov_norm_threshold_dual
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

    if (update_type == 1) {
      r.is_maneuvering = (nis > params_.nis_threshold_single &&
                          innov_norm > params_.innov_norm_threshold_single);
    } else if (update_type == 2) {
      r.is_maneuvering = (nis > params_.nis_threshold_dual &&
                          innov_norm > params_.innov_norm_threshold_dual);
    }
    return r;
  }

  void set_params(const ManeuverDetectionParameters &p) { params_ = p; }
  const ManeuverDetectionParameters &params() const { return params_; }

 private:
  ManeuverDetectionParameters params_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_UTILS_MANEUVER_DETECTOR_HPP_
