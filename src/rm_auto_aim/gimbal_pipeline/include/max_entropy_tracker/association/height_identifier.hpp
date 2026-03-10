// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_ASSOCIATION_HEIGHT_IDENTIFIER_HPP_
#define MAX_ENTROPY_TRACKER_ASSOCIATION_HEIGHT_IDENTIFIER_HPP_

#include <cmath>
#include <optional>
#include <string>
#include <tuple>

namespace fyt::auto_aim {

enum class HeightLabel { UNKNOWN = 0, UPPER = 1, LOWER = 2 };

/// Max-entropy height classifier for armor panels.
class HeightIdentifier {
 public:
  HeightIdentifier() = default;

  /* ---------- single-observation ---------- */
  std::pair<HeightLabel, double> identify_single(
      double z_obs, int panel_id, std::optional<double> z_mean = std::nullopt,
      std::optional<double> dza = std::nullopt,
      bool dza_converged = false) {
    // Mechanism 1: state-estimate based (most reliable)
    if (dza_converged && z_mean.has_value() && dza.has_value()) {
      double z_upper = z_mean.value() + dza.value();
      double z_lower = z_mean.value() - dza.value();
      double d_up = std::abs(z_obs - z_upper);
      double d_lo = std::abs(z_obs - z_lower);
      HeightLabel label = (d_up < d_lo) ? HeightLabel::UPPER : HeightLabel::LOWER;
      double conf = 1.0 - std::min(d_up, d_lo) / (d_up + d_lo + 0.001);
      update_history(z_obs, panel_id, label);
      return {label, conf};
    }

    // Mechanism 2: panel-jump detection
    if (panel_last_.has_value() && label_last_.has_value()) {
      int diff = std::abs(panel_id - panel_last_.value());
      if (diff == 1 || diff == 3) {
        HeightLabel label = (label_last_.value() == HeightLabel::UPPER)
                                ? HeightLabel::LOWER
                                : HeightLabel::UPPER;
        update_history(z_obs, panel_id, label);
        return {label, 0.7};
      }
    }

    // Mechanism 3: history consistency
    if (z_last_.has_value() && label_last_.has_value()) {
      double z_diff = std::abs(z_obs - z_last_.value());
      if (z_diff < 0.05) {
        double conf = std::max(0.3, 0.6 - z_diff * 2.0);
        update_history(z_obs, panel_id, label_last_.value());
        return {label_last_.value(), conf};
      }
    }

    // Cannot determine → max-entropy UNKNOWN
    panel_last_ = panel_id;
    return {HeightLabel::UNKNOWN, 0.0};
  }

  /* ---------- dual-observation ---------- */
  std::tuple<std::string, std::string, double> identify_dual(
      double z1, double z2, double z_diff_threshold = 0.015) {
    double z_diff = std::abs(z1 - z2);
    if (z_diff > z_diff_threshold) {
      std::string l1 = (z1 > z2) ? "upper" : "lower";
      std::string l2 = (z1 > z2) ? "lower" : "upper";
      double conf = (z_diff > 0.05) ? 1.0 : (z_diff > 0.03 ? 0.8 : 0.6);
      return {l1, l2, conf};
    }

    // Try history
    if (label_last_.has_value() && z_last_.has_value()) {
      double d1 = std::abs(z1 - z_last_.value());
      double d2 = std::abs(z2 - z_last_.value());
      bool last_upper = (label_last_.value() == HeightLabel::UPPER);
      if (d1 < d2 && d1 < 0.03) {
        return {last_upper ? "upper" : "lower", last_upper ? "lower" : "upper",
                0.4};
      } else if (d2 < 0.03) {
        return {last_upper ? "lower" : "upper", last_upper ? "upper" : "lower",
                0.4};
      }
    }
    return {"", "", 0.0};
  }

  void reset() {
    z_last_.reset();
    label_last_.reset();
    panel_last_.reset();
  }

  /**
   * Reset history and seed it with a known-correct label.
   * Called after panel_id correction so the next frames benefit from
   * history-consistency (Mechanism 3) with the correct label.
   *
   * @param hint  Expected layer label for the corrected panel_id.
   */
  void reset_with_hint(HeightLabel hint) {
    z_last_.reset();
    label_last_ = hint;
    panel_last_.reset();
  }

 private:
  void update_history(double z, int panel_id, HeightLabel label) {
    z_last_ = z;
    label_last_ = label;
    panel_last_ = panel_id;
  }

  std::optional<double> z_last_;
  std::optional<HeightLabel> label_last_;
  std::optional<int> panel_last_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_ASSOCIATION_HEIGHT_IDENTIFIER_HPP_
