// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_ADAPTIVE_ARMOR_TRACKER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_ADAPTIVE_ARMOR_TRACKER_HPP_

#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "max_entropy_tracker/association/height_identifier.hpp"
#include "max_entropy_tracker/association/oscillation_detector.hpp"
#include "max_entropy_tracker/association/panel_associator.hpp"
#include "max_entropy_tracker/association/panel_mismatch_detector.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/dual_radius_spin_ukf.hpp"
#include "max_entropy_tracker/filters/spin_filter_interface.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"

namespace fyt::auto_aim {

/**
 * Adaptive armor tracker that combines UKF + panel association + height ID.
 */
class AdaptiveArmorTracker : public BaseTracker {
 public:
  struct DebugSnapshot {
    bool valid = false;
    int current_panel_id = -1;
    int bound_panel_id = -1;
    int current_height_label = -1;
    int bound_height_label = -1;
    int binding_transition_state = 0;
    int transition_candidate_panel = -1;
    int transition_confirm_count = 0;
    int switch_cooldown_frames = 0;
    double bound_confidence = std::numeric_limits<double>::quiet_NaN();
    double height_confidence = std::numeric_limits<double>::quiet_NaN();
    bool degraded_single_obs_mode = false;
    int single_obs_streak = 0;
    double dz_jump_est = std::numeric_limits<double>::quiet_NaN();
  };

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
  DebugSnapshot debug_snapshot() const;

  SpinFilterInterface &spin_filter() override { return ukf_; }
  const SpinFilterInterface &spin_filter() const override { return ukf_; }

  /// Assess whether the tracked robot is currently maneuvering.
  ManeuverResult assess_maneuver() const override;

 private:
  enum class BindingTransitionState {
    LOCKED = 0,
    TRANSITION_CANDIDATE = 1,
  };

  bool update_single(const ObservationData &obs,
                     double override_pos_confidence = -1.0);
  bool update_dual(const ObservationData &obs1, const ObservationData &obs2);
  double compute_position_confidence(const std::string &armor_layer,
                                     double height_confidence,
                                     const std::string &r_type) const;
  void reset_parameters();

  static HeightLabel default_label_from_panel(int panel_id);
  static std::string label_to_layer(HeightLabel label);
  void reset_jump_binding(int panel_id, HeightLabel label,
                          std::optional<double> obs_z,
                          std::optional<double> obs_time);
  void update_degraded_single_obs_mode(bool is_single_obs);
  bool update_jump_binding(
      const ObservationData &obs, int candidate_panel,
      const PanelAssociator::AssociationDiagnostics &diag,
      HeightLabel candidate_label, double candidate_height_conf,
      int *selected_panel, HeightLabel *selected_label,
      double *selected_height_conf);
  HeightLabel resolve_layer_from_jump(HeightLabel fallback_label,
                                      double z_jump,
                                      bool has_z_jump) const;
  void update_jump_statistics(double z_jump, bool switch_confirmed);
  double compute_jump_binding_confidence(
      const PanelAssociator::AssociationDiagnostics &diag,
      bool jump_gate_passed) const;

  /**
   * Apply in-place panel_id correction (PATCH level).
   * Swaps R1/R2 in UKF state, recomputes center_yaw, inflates covariances,
   * resets HeightIdentifier and PanelAssociator history.
   *
   * @param new_panel_id   Corrected panel id (= old_panel_id ^ 1)
   * @param armor_yaw      Observed armor yaw used to recompute center_yaw
   */
  void correct_panel_id(int new_panel_id, double armor_yaw);

  /**
   * Full re-initialization using the most recent observation.
   * Loses velocity/acceleration estimates but completely resets panel binding.
   *
   * @param obs  The latest observation to seed the new tracker state
   */
  void reinitialize_tracker(const ObservationData &obs);

  UnifiedConfig config_;
  DualRadiusSpinUKF ukf_;
  PanelAssociator panel_associator_;
  HeightIdentifier height_identifier_;
  OscillationDetector osc_detector_;
  PanelMismatchDetector mismatch_detector_;

  int current_panel_id_ = 0;
  std::optional<double> reference_center_yaw_;
  HeightLabel height_label_ = HeightLabel::UNKNOWN;
  double height_confidence_ = 0.0;

  // Jump binder state
  int bound_panel_id_ = -1;
  HeightLabel bound_height_label_ = HeightLabel::UNKNOWN;
  double bound_confidence_ = 0.0;
  BindingTransitionState binding_transition_state_ =
      BindingTransitionState::LOCKED;
  int transition_candidate_panel_ = -1;
  int transition_confirm_count_ = 0;
  int switch_cooldown_frames_ = 0;
  int last_panel_id_ = -1;
  std::optional<double> last_obs_z_;
  std::optional<double> last_obs_time_;
  std::deque<double> z_jump_history_;
  double dz_jump_est_ = std::numeric_limits<double>::quiet_NaN();

  // Long single-observation degraded mode state
  int single_obs_streak_ = 0;
  bool degraded_single_obs_mode_ = false;

  // Cached r1/r2 defaults for re-initialization
  double default_r1_ = 0.15;
  double default_r2_ = 0.20;
  double default_dza_ = 0.0;

  ManeuverDetector maneuver_detector_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_ADAPTIVE_ARMOR_TRACKER_HPP_
