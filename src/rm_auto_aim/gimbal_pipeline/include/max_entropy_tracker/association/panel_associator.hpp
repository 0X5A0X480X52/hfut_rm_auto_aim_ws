// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_ASSOCIATION_PANEL_ASSOCIATOR_HPP_
#define MAX_ENTROPY_TRACKER_ASSOCIATION_PANEL_ASSOCIATOR_HPP_

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <tuple>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

/**
 * Associates observed armor yaw to a panel id (0–3).
 *
 * Uses a combined cost function: cost = yaw_error + w_pos * position_error
 * When position information is available, this prevents mismatches where
 * yaw angles are similar but positions differ significantly.
 *
 * Layout (4-panel fixed):
 *   Panel 0: offset=0°, r1, lower
 *   Panel 1: offset=90°, r2, upper
 *   Panel 2: offset=180°, r1, lower
 *   Panel 3: offset=270°, r2, upper
 */
class PanelAssociator {
 public:
  struct AssociationDiagnostics {
    int selected_id = -1;
    int best_id = -1;
    int second_id = -1;
    double selected_yaw_err = std::numeric_limits<double>::quiet_NaN();
    double best_yaw_err = std::numeric_limits<double>::quiet_NaN();
    double second_yaw_err = std::numeric_limits<double>::quiet_NaN();
    double best_cost = std::numeric_limits<double>::quiet_NaN();
    double second_cost = std::numeric_limits<double>::quiet_NaN();
    double cost_margin = std::numeric_limits<double>::quiet_NaN();
    bool is_ambiguous = false;
    bool used_z_assist = false;
  };

  static constexpr int N_PANELS = 4;
  static constexpr double PANEL_ANGLE_STEP = M_PI / 2.0;
  /// Position weight in combined cost (rad per meter).
  /// 2.0 means 1m position error ≈ 2rad yaw error in cost.
  static constexpr double DEFAULT_POS_WEIGHT = 2.0;

  PanelAssociator() = default;

  /// Optional periodic dz prior for 4-panel robots.
  /// When enabled, association cost is augmented by a lightweight prior
  /// consistent with alternating layer jumps (-dz, +dz, -dz, +dz), where
  /// sign is chosen by spin direction.
  void configure_periodic_binding(bool enable, double weight,
                                  double spin_rate_gate) {
    periodic_binding_enable_ = enable;
    periodic_binding_weight_ = std::max(0.0, weight);
    periodic_spin_rate_gate_ = std::max(0.0, spin_rate_gate);
  }

  /// Clear internal history used for ambiguity resolution and periodic prior.
  void reset_history() {
    last_confident_panel_.reset();
    prev_z_obs_.reset();
  }

  /**
   * Associate observed armor to a panel id.
   *
   * @param armor_yaw      Observed armor yaw (radial: center→armor).
   * @param center_yaw_pred  Predicted center yaw (nullopt for first frame).
   * @param z_obs          Optional observed z for height-assisted fallback.
   * @param center_z       Optional predicted center z.
   * @param obs_x          Optional observed armor x position.
   * @param obs_y          Optional observed armor y position.
   * @param center_x       Optional predicted center x position.
   * @param center_y       Optional predicted center y position.
   * @param r1             Optional predicted radius for even panels.
   * @param r2             Optional predicted radius for odd panels.
   * @return (panel_id, center_yaw, matching_error)
   */
  std::tuple<int, double, double> associate_panel(
      double armor_yaw, std::optional<double> center_yaw_pred,
      std::optional<double> z_obs = std::nullopt,
      std::optional<double> center_z = std::nullopt,
      std::optional<double> obs_x = std::nullopt,
      std::optional<double> obs_y = std::nullopt,
      std::optional<double> center_x = std::nullopt,
      std::optional<double> center_y = std::nullopt,
      std::optional<double> r1 = std::nullopt,
      std::optional<double> r2 = std::nullopt,
      std::optional<double> yaw_rate_hint = std::nullopt,
      std::optional<double> dz_unit_hint = std::nullopt,
      AssociationDiagnostics *diagnostics = nullptr) const {
    if (diagnostics != nullptr) {
      *diagnostics = AssociationDiagnostics{};
    }

    const int spin_direction =
        (yaw_rate_hint.has_value() &&
         std::abs(yaw_rate_hint.value()) >= periodic_spin_rate_gate_)
            ? ((yaw_rate_hint.value() >= 0.0) ? 1 : -1)
            : 0;

    const bool has_z_jump = z_obs.has_value() && prev_z_obs_.has_value();
    const double z_jump = has_z_jump
                              ? (z_obs.value() - prev_z_obs_.value())
                              : std::numeric_limits<double>::quiet_NaN();

    if (!center_yaw_pred.has_value()) {
      // First frame — no prediction available
      double ay = std::atan2(std::sin(armor_yaw), std::cos(armor_yaw));
      double ay_pos = std::fmod(ay + 2.0 * M_PI, 2.0 * M_PI);
      int panel_id = static_cast<int>(std::round(ay_pos / PANEL_ANGLE_STEP)) % 4;
      double cw = normalize_angle(ay - panel_id * PANEL_ANGLE_STEP);
      if (diagnostics != nullptr) {
        diagnostics->selected_id = panel_id;
        diagnostics->best_id = panel_id;
        diagnostics->second_id = panel_id;
        diagnostics->selected_yaw_err = 0.0;
        diagnostics->best_yaw_err = 0.0;
        diagnostics->second_yaw_err = 0.0;
        diagnostics->best_cost = 0.0;
        diagnostics->second_cost = 0.0;
        diagnostics->cost_margin = 0.0;
      }
      if (z_obs.has_value()) prev_z_obs_ = z_obs.value();
      return {panel_id, cw, 0.0};
    }

    double cyp = center_yaw_pred.value();
    bool has_pos = obs_x.has_value() && obs_y.has_value() &&
                   center_x.has_value() && center_y.has_value() &&
                   r1.has_value() && r2.has_value();

    int best_id = 0, second_id = 0;
    double best_cost = 1e9, second_cost = 1e9;
    double best_yaw_err = 1e9;
    double second_yaw_err = 1e9;
    bool ambiguous = false;

    for (int pid = 0; pid < 4; ++pid) {
      double expected = cyp + pid * PANEL_ANGLE_STEP;
      double yaw_err = std::abs(normalize_angle(armor_yaw - expected));

      double cost = yaw_err;  // default: yaw-only cost
      if (has_pos) {
        double pos_err = predict_position_error(
            cyp, pid, center_x.value(), center_y.value(),
            r1.value(), r2.value(), obs_x.value(), obs_y.value());
        cost = yaw_err + DEFAULT_POS_WEIGHT * pos_err;
      }

      if (periodic_binding_enable_ && has_z_jump &&
          dz_unit_hint.has_value() &&
          std::abs(dz_unit_hint.value()) > 1e-4 &&
          last_confident_panel_.has_value() && spin_direction != 0) {
        const int prev_pid = last_confident_panel_.value();
        const auto layer_value = [](int panel) {
          return (panel % 2 == 0) ? -0.5 : 0.5;
        };

        const double dz_unit = std::max(1e-4, std::abs(dz_unit_hint.value()));
        const double expected_jump =
            (layer_value(pid) - layer_value(prev_pid)) * dz_unit;

        const int expected_next =
            (spin_direction > 0) ? ((prev_pid + 1) % 4) : ((prev_pid + 3) % 4);
        const double direction_penalty =
            (pid != prev_pid && pid != expected_next) ? 0.35 : 0.0;

        const double periodic_err = std::abs(z_jump - expected_jump) / dz_unit;
        cost += periodic_binding_weight_ * (periodic_err + direction_penalty);
      }

      if (cost < best_cost) {
        second_cost = best_cost;
        second_id = best_id;
        second_yaw_err = best_yaw_err;
        best_cost = cost;
        best_id = pid;
        best_yaw_err = yaw_err;
      } else if (cost < second_cost) {
        second_cost = cost;
        second_id = pid;
        second_yaw_err = yaw_err;
      }
    }

    int panel_id = best_id;

    // Z-assisted association when yaw error large and position not available
    if (z_obs.has_value() && center_z.has_value()) {
      panel_id = z_assisted_association(armor_yaw, cyp, z_obs.value(),
                                        center_z.value(), best_id, best_yaw_err);
      if (diagnostics != nullptr) {
        diagnostics->used_z_assist = true;
      }
    } else {
      // Ambiguity detection
      ambiguous =
          (best_yaw_err > 20.0 * M_PI / 180.0) &&
          (std::abs(best_cost - second_cost) < 0.1);

      if (ambiguous) {
        if (has_pos) {
          // Use position distance as tiebreaker
          double pos_best = predict_position_error(
              cyp, best_id, center_x.value(), center_y.value(),
              r1.value(), r2.value(), obs_x.value(), obs_y.value());
          double pos_second = predict_position_error(
              cyp, second_id, center_x.value(), center_y.value(),
              r1.value(), r2.value(), obs_x.value(), obs_y.value());
          if (pos_second < pos_best)
            panel_id = second_id;
        } else if (last_confident_panel_.has_value()) {
          panel_id = last_confident_panel_.value();
        }
      }

      if (best_yaw_err < 15.0 * M_PI / 180.0)
        last_confident_panel_ = panel_id;
    }

    if (z_obs.has_value()) prev_z_obs_ = z_obs.value();

    const double selected_expected = cyp + panel_id * PANEL_ANGLE_STEP;
    const double selected_yaw_err =
        std::abs(normalize_angle(armor_yaw - selected_expected));
    const double cost_margin = std::max(0.0, second_cost - best_cost);

    if (diagnostics != nullptr) {
      diagnostics->selected_id = panel_id;
      diagnostics->best_id = best_id;
      diagnostics->second_id = second_id;
      diagnostics->selected_yaw_err = selected_yaw_err;
      diagnostics->best_yaw_err = best_yaw_err;
      diagnostics->second_yaw_err = second_yaw_err;
      diagnostics->best_cost = best_cost;
      diagnostics->second_cost = second_cost;
      diagnostics->cost_margin = cost_margin;
      diagnostics->is_ambiguous = ambiguous;
    }

    double cw = normalize_angle(armor_yaw - panel_id * PANEL_ANGLE_STEP);
    return {panel_id, cw, selected_yaw_err};
  }

  static std::string get_r_type(int panel_id) {
    return (panel_id % 2 == 0) ? "r1" : "r2";
  }

  static std::string get_default_layer(int panel_id) {
    return (panel_id % 2 == 0) ? "lower" : "upper";
  }

 private:
  /// Compute predicted armor position for a given panel and return
  /// Euclidean distance to the observed position.
  static double predict_position_error(
      double center_yaw_pred, int panel_id,
      double cx, double cy, double r1_val, double r2_val,
      double obs_x_val, double obs_y_val) {
    double radius = (panel_id % 2 == 0) ? r1_val : r2_val;
    double armor_yaw = normalize_angle(
        center_yaw_pred + panel_id * PANEL_ANGLE_STEP);
    double pred_x = cx + radius * std::cos(armor_yaw);
    double pred_y = cy + radius * std::sin(armor_yaw);
    return std::hypot(obs_x_val - pred_x, obs_y_val - pred_y);
  }

  int z_assisted_association(double armor_yaw, double center_yaw_pred,
                             double z_obs, double center_z, int yaw_best,
                             double yaw_best_err) const {
    bool is_upper = (z_obs > center_z);
    int preferred_parity = is_upper ? 1 : 0;

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
  mutable std::optional<double> prev_z_obs_;

  bool periodic_binding_enable_ = false;
  double periodic_binding_weight_ = 0.0;
  double periodic_spin_rate_gate_ = 0.8;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_ASSOCIATION_PANEL_ASSOCIATOR_HPP_
