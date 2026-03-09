// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_ADAPTIVE_ARMOR_TRACKER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_ADAPTIVE_ARMOR_TRACKER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "max_entropy_tracker/association/height_identifier.hpp"
#include "max_entropy_tracker/association/oscillation_detector.hpp"
#include "max_entropy_tracker/association/panel_associator.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/dual_radius_spin_ukf.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"

namespace fyt::auto_aim {

/**
 * Adaptive armor tracker that combines UKF + panel association + height ID.
 */
class AdaptiveArmorTracker : public BaseTracker {
 public:
  explicit AdaptiveArmorTracker(const UnifiedConfig &config, double dt = 0.05,
                                bool enable_oscillation = false);

  /* ---------- BaseTracker interface ---------- */
  void initialize(const std::vector<ObservationData> &obs, double r1 = 0.15,
                  double r2 = 0.20, double dza = 0.0) override;
  void predict(std::optional<double> target_time = std::nullopt) override;
  bool update(const std::vector<ObservationData> &obs) override;

  Eigen::Vector3d get_center_position() const override;
  double get_yaw() const override;
  std::pair<double, double> get_radii() const override;

  /* ---------- Extra queries ---------- */
  double get_dza() const;
  int get_k() const;
  double get_delta() const;
  int get_panel_id() const { return current_panel_id_; }
  HeightLabel get_height_label() const { return height_label_; }
  double get_height_confidence() const { return height_confidence_; }

  DualRadiusSpinUKF &ukf() { return ukf_; }
  const DualRadiusSpinUKF &ukf() const { return ukf_; }

  /// Assess whether the tracked robot is currently maneuvering.
  ManeuverResult assess_maneuver() const;

 private:
  bool update_single(const ObservationData &obs,
                     double override_pos_confidence = -1.0);
  bool update_dual(const ObservationData &obs1, const ObservationData &obs2);
  double compute_position_confidence(const std::string &armor_layer,
                                     double height_confidence,
                                     const std::string &r_type) const;
  void reset_parameters();

  UnifiedConfig config_;
  DualRadiusSpinUKF ukf_;
  PanelAssociator panel_associator_;
  HeightIdentifier height_identifier_;
  OscillationDetector osc_detector_;

  int current_panel_id_ = 0;
  std::optional<double> reference_center_yaw_;
  HeightLabel height_label_ = HeightLabel::UNKNOWN;
  double height_confidence_ = 0.0;

  ManeuverDetector maneuver_detector_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_ADAPTIVE_ARMOR_TRACKER_HPP_
