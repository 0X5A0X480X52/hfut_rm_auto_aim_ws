// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/decoder/outpost_z_audit.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

OutpostZAudit::OutpostZAudit(const std::array<double, 3> & z_offsets)
    : z_offsets_(z_offsets) {}

void OutpostZAudit::reset() {
  initialized_ = false;
  center_est_ = std::numeric_limits<double>::quiet_NaN();
  prev_obs_z_ = std::numeric_limits<double>::quiet_NaN();
  prev_panel_id_ = -1;
  confidence_ = 0.0;
}

OutpostZAuditResult OutpostZAudit::update(const ObservationData & obs) {
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  OutpostZAuditResult result;
  confidence_ = 0.0;

  if (!initialized_ || !std::isfinite(center_est_)) {
    center_est_ = obs.z - z_offsets_[1];
    prev_obs_z_ = obs.z;
    prev_panel_id_ = -1;
    initialized_ = true;
    return result;
  }

  const bool has_prev = std::isfinite(prev_obs_z_);
  const double z_jump = has_prev ? (obs.z - prev_obs_z_) : 0.0;
  result.z_jump = has_prev ? z_jump : kNaN;

  constexpr double kWeightLevel = 1.0;
  constexpr double kWeightJump = 2.5;
  constexpr double kWeightCenter = 1.0;
  constexpr double kSwitchPenalty = 0.02;

  int best_panel = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    const double center_i = obs.z - z_offsets_[i];
    const double level_err =
        std::abs(obs.z - (center_est_ + z_offsets_[i]));

    double jump_err = 0.0;
    if (has_prev) {
      if (prev_panel_id_ >= 0) {
        const double expected_jump = z_offsets_[i] - z_offsets_[prev_panel_id_];
        jump_err = std::abs(z_jump - expected_jump);
      } else {
        double min_jump_err = std::numeric_limits<double>::infinity();
        for (int j = 0; j < 3; ++j) {
          const double expected_jump = z_offsets_[i] - z_offsets_[j];
          const double err = std::abs(z_jump - expected_jump);
          if (err < min_jump_err) {
            min_jump_err = err;
          }
        }
        jump_err = min_jump_err;
      }
    }

    const double center_err = std::abs(center_i - center_est_);
    const double switch_penalty =
        (prev_panel_id_ >= 0 && i != prev_panel_id_) ? kSwitchPenalty : 0.0;
    const double cost = kWeightLevel * level_err + kWeightJump * jump_err +
                        kWeightCenter * center_err + switch_penalty;
    result.costs[i] = cost;
    if (cost < best_cost) {
      best_cost = cost;
      best_panel = i;
    }
  }

  if (best_panel >= 0) {
    const double center_best = obs.z - z_offsets_[best_panel];
    constexpr double kAlphaCenter = 0.20;
    center_est_ = (1.0 - kAlphaCenter) * center_est_ +
                  kAlphaCenter * center_best;

    prev_obs_z_ = obs.z;
    prev_panel_id_ = best_panel;

    result.panel_id = best_panel;

    double best_audit = std::numeric_limits<double>::infinity();
    double second_audit = std::numeric_limits<double>::infinity();
    for (double cost : result.costs) {
      if (cost < best_audit) {
        second_audit = best_audit;
        best_audit = cost;
      } else if (cost < second_audit) {
        second_audit = cost;
      }
    }
    if (std::isfinite(best_audit) && std::isfinite(second_audit)) {
      confidence_ =
          std::clamp(std::max(0.0, second_audit - best_audit) / 0.10,
                     0.0, 1.0);
    }
  } else {
    result.panel_id = -1;
  }

  return result;
}

}  // namespace fyt::auto_aim::binder
