// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM_4ARMOR_TRACKER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM_4ARMOR_TRACKER_HPP_

#include <limits>
#include <optional>
#include <vector>

#include "max_entropy_tracker/association/height_identifier.hpp"
#include "max_entropy_tracker/association/panel_mismatch_detector.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/evidence/evidence_builder.hpp"
#include "max_entropy_tracker/mode/evidence_fuser.hpp"
#include "max_entropy_tracker/mode/mode_fsm.hpp"
#include "max_entropy_tracker/pipeline/serial_tracker_pipeline.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_ambiguous_backend.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_binder_bridge.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_observation_frontend.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_output_adapter.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_phase_sequence_memory.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_runtime_context.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_structured_backend.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"

namespace fyt::auto_aim {

class Norm4ArmorTracker : public BaseTracker {
 public:
  struct DebugSnapshot {
    bool valid = false;
    int track_mode = 1;  // 0=STRUCTURED, 1=AMBIGUOUS
    int current_panel_id = -1;
    int bound_panel_id = -1;
    int bound_height_label = -1;
    int candidate_panel_id = -1;
    double candidate_prob = std::numeric_limits<double>::quiet_NaN();
    double candidate_margin = std::numeric_limits<double>::quiet_NaN();
    double selected_yaw_err = std::numeric_limits<double>::quiet_NaN();
    double selected_xy_residual = std::numeric_limits<double>::quiet_NaN();
    double entropy_norm = 1.0;
    double max_prob = 0.0;

    int binding_fsm_state = 0;
    int switch_event = 0;
    int switch_reason = 0;
    double binding_confidence = std::numeric_limits<double>::quiet_NaN();
    double height_confidence = std::numeric_limits<double>::quiet_NaN();

    bool degraded_single_obs_mode = false;
    int single_obs_streak = 0;
    bool dza_converged = false;

    bool has_observation = false;
    double obs_x = std::numeric_limits<double>::quiet_NaN();
    double obs_y = std::numeric_limits<double>::quiet_NaN();
    double obs_z = std::numeric_limits<double>::quiet_NaN();
    double obs_yaw = std::numeric_limits<double>::quiet_NaN();
    double obs_z_jump = std::numeric_limits<double>::quiet_NaN();

    // Phase 4: ping-pong suppression debug.
    double ping_pong_risk = std::numeric_limits<double>::quiet_NaN();
    bool ping_pong_hold = false;
    int ping_pong_reason = 0;
    int ping_pong_hold_ctr = 0;
  };

  explicit Norm4ArmorTracker(const UnifiedConfig &config, double dt = 0.05,
                             bool enable_oscillation = false);

  void initialize(const std::vector<ObservationData> &obs, double r1 = 0.15,
                  double r2 = 0.20, double dza = 0.0) override;
  void predict(std::optional<double> target_time = std::nullopt) override;
  bool update(const std::vector<ObservationData> &obs) override;

  Eigen::Vector3d get_center_position() const override;
  double get_yaw() const override;
  std::pair<double, double> get_radii() const override;
  SpinFilterInterface &spin_filter() override;
  const SpinFilterInterface &spin_filter() const override;
  ManeuverResult assess_maneuver() const override;

  Eigen::Vector3d get_publish_velocity() const override;
  bool is_ambiguous_single_mode() const override;
  bool supports_ambiguous_single_semantics() const override { return true; }
  int effective_num_armors() const override;
  double confidence_scale() const override;
  std::vector<geometry_msgs::msg::Pose> build_armors_offset_for_message() const override;

  const DebugSnapshot &debug_snapshot() const { return debug_snapshot_; }
  const evidence::ArmorEvidenceFrame &last_evidence_frame() const { return ctx_.evidence_frame; }

 private:
  static int clamp_panel(int panel_id);
  static binder::HeightLabel default_height_label_for_panel(int panel_id);
  static std::string label_to_layer(binder::HeightLabel label, int panel_id);

  void sync_runtime_from_backend(const norm4_v2::BackendStateSnapshot &snap);
  void update_degraded_single_obs_mode(bool is_single_obs);
  int apply_panel_mismatch_if_needed(const ObservationData &obs,
                                     int selected_panel,
                                     binder::HeightLabel selected_label);
  void refresh_debug(const ObservationData *obs,
                     const norm4_v2::BindingCandidate &candidate,
                     const binder::BinderOutput &binder_out,
                     const binder::BinderDebugSnapshot &binder_dbg,
                     const mode::ModeDecision &mode_decision,
                     double height_confidence);

  UnifiedConfig config_;
  norm4_v2::ObservationFrontend obs_frontend_;
  norm4_v2::Norm4BinderBridge binder_bridge_;
  mode::EvidenceFuser evidence_fuser_;
  mode::ModeFSM mode_fsm_;
  norm4_v2::Norm4AmbiguousBackend ambiguous_backend_;
  norm4_v2::Norm4StructuredBackend structured_backend_;
  norm4_v2::Norm4OutputAdapter output_adapter_;
  HeightIdentifier height_identifier_;
  PanelMismatchDetector mismatch_detector_;
  ManeuverDetector maneuver_detector_;

  norm4_v2::PhaseSequenceMemory phase_memory_;

  evidence::EvidenceBuilder evidence_builder_;

  std::unique_ptr<pipeline::SerialTrackerPipeline> serial_pipeline_;

  norm4_v2::Norm4RuntimeContext ctx_;
  double default_r1_ = 0.15;
  double default_r2_ = 0.20;
  double default_dza_ = 0.0;

  int single_obs_streak_ = 0;
  bool degraded_single_obs_mode_ = false;

  DebugSnapshot debug_snapshot_{};
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM_4ARMOR_TRACKER_HPP_
