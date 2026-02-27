// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_ASSOCIATION_PANEL_ASSOCIATOR_HPP_
#define MAX_ENTROPY_TRACKER_ASSOCIATION_PANEL_ASSOCIATOR_HPP_

#include <cmath>
#include <optional>
#include <string>
#include <tuple>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

/**
 * Associates observed armor yaw to a panel id (0–3).
 *
 * Layout (4-panel fixed):
 *   Panel 0: offset=0°, r1, lower
 *   Panel 1: offset=90°, r2, upper
 *   Panel 2: offset=180°, r1, lower
 *   Panel 3: offset=270°, r2, upper
 */
class PanelAssociator {
 public:
  static constexpr int N_PANELS = 4;
  static constexpr double PANEL_ANGLE_STEP = M_PI / 2.0;

  PanelAssociator() = default;

  /// Returns (panel_id, center_yaw, matching_error).
  std::tuple<int, double, double> associate_panel(
      double armor_yaw, std::optional<double> center_yaw_pred,
      std::optional<double> z_obs = std::nullopt,
      std::optional<double> center_z = std::nullopt) const {
    if (!center_yaw_pred.has_value()) {
      // First frame
      double ay = std::atan2(std::sin(armor_yaw), std::cos(armor_yaw));
      double ay_pos = std::fmod(ay + 2.0 * M_PI, 2.0 * M_PI);
      int panel_id = static_cast<int>(std::round(ay_pos / PANEL_ANGLE_STEP)) % 4;
      double cw = normalize_angle(ay - panel_id * PANEL_ANGLE_STEP);
      return {panel_id, cw, 0.0};
    }

    double cyp = center_yaw_pred.value();
    int best_id = 0;
    double best_err = 1e9, second_err = 1e9;

    for (int pid = 0; pid < 4; ++pid) {
      double expected = cyp + pid * PANEL_ANGLE_STEP;
      double diff = normalize_angle(armor_yaw - expected);
      double err = std::abs(diff);
      if (err < best_err) {
        second_err = best_err;
        best_id = pid;
        best_err = err;
      } else if (err < second_err) {
        second_err = err;
      }
    }

    int panel_id = best_id;

    // Z-assisted association when yaw error large
    if (z_obs.has_value() && center_z.has_value() &&
        best_err > 30.0 * M_PI / 180.0) {
      panel_id = z_assisted_association(armor_yaw, cyp, z_obs.value(),
                                        center_z.value(), best_id, best_err);
    } else {
      bool ambiguous =
          (best_err > 20.0 * M_PI / 180.0) &&
          (std::abs(best_err - second_err) < 10.0 * M_PI / 180.0);
      if (ambiguous && last_confident_panel_.has_value())
        panel_id = last_confident_panel_.value();
      if (best_err < 15.0 * M_PI / 180.0)
        last_confident_panel_ = panel_id;
    }

    double cw = normalize_angle(armor_yaw - panel_id * PANEL_ANGLE_STEP);
    return {panel_id, cw, best_err};
  }

  static std::string get_r_type(int panel_id) {
    return (panel_id % 2 == 0) ? "r1" : "r2";
  }

  static std::string get_default_layer(int panel_id) {
    return (panel_id % 2 == 0) ? "lower" : "upper";
  }

 private:
  int z_assisted_association(double armor_yaw, double center_yaw_pred,
                             double z_obs, double center_z, int yaw_best,
                             double yaw_best_err) const {
    bool is_upper = (z_obs > center_z);
    int preferred_parity = is_upper ? 0 : 1;

    int best_p = preferred_parity;
    double best_pe = 1e9;
    for (int p = preferred_parity; p < 4; p += 2) {
      double expected = center_yaw_pred + p * PANEL_ANGLE_STEP;
      double err = std::abs(normalize_angle(armor_yaw - expected));
      if (err < best_pe) {
        best_pe = err;
        best_p = p;
      }
    }

    if ((yaw_best % 2 != preferred_parity) && (best_pe < yaw_best_err * 0.8))
      return best_p;
    return yaw_best;
  }

  mutable std::optional<int> last_confident_panel_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_ASSOCIATION_PANEL_ASSOCIATOR_HPP_
