// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_TRACKER_V2_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_TRACKER_V2_HPP_

#include <array>
#include <limits>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "max_entropy_tracker/mode/evidence_fuser.hpp"
#include "max_entropy_tracker/mode/mode_fsm.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_ambiguous_backend.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_binder_bridge.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_observation_frontend.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_output_adapter.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_runtime_context.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_structured_backend.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"

namespace fyt::auto_aim {

class OutpostTrackerV2 : public BaseTracker {
 public:
  struct DebugSnapshot {
    bool valid = false;
    int track_mode = 1;  // 0=STRUCTURED_3_ARMORS, 1=AMBIGUOUS_SINGLE_ARMOR
    int estimated_id = -1;
    int runtime_panel_id = -1;
    int bound_height_label = -1;
    int obs_inferred_id = -1;
    int obs_inferred_id_z = -1;
    int candidate_panel_id = -1;
    double candidate_prob = std::numeric_limits<double>::quiet_NaN();
    double candidate_margin = std::numeric_limits<double>::quiet_NaN();
    double selected_xy_residual = std::numeric_limits<double>::quiet_NaN();
    double entropy_norm = 1.0;
    double max_prob = 0.0;
    std::array<double, 3> hyp_costs{0.0, 0.0, 0.0};
    std::array<double, 3> hyp_probs{0.0, 0.0, 0.0};
    double center_yaw_est = 0.0;
    bool has_observation = false;
    double obs_x = std::numeric_limits<double>::quiet_NaN();
    double obs_y = std::numeric_limits<double>::quiet_NaN();
    double obs_z = std::numeric_limits<double>::quiet_NaN();
    double obs_yaw = std::numeric_limits<double>::quiet_NaN();
    double obs_z_jump = std::numeric_limits<double>::quiet_NaN();
    double obs_dz_from_audit_center = std::numeric_limits<double>::quiet_NaN();
    std::array<double, 3> obs_z_audit_costs{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};

    double binding_confidence = std::numeric_limits<double>::quiet_NaN();
    int switch_event = 0;
    int switch_reason = 0;
    int transition_state = 0;
    int z_audit_conflict_count = 0;
    double z_audit_confidence = std::numeric_limits<double>::quiet_NaN();
    double publish_x = std::numeric_limits<double>::quiet_NaN();
    double publish_y = std::numeric_limits<double>::quiet_NaN();
    double publish_z = std::numeric_limits<double>::quiet_NaN();
    double period_confidence = std::numeric_limits<double>::quiet_NaN();
    int period_update_applied = 0;
    int period_phase_index = -1;
    int spin_direction = 0;
    double dz_small_est = std::numeric_limits<double>::quiet_NaN();
    double dz_large_est = std::numeric_limits<double>::quiet_NaN();
  };

  explicit OutpostTrackerV2(const UnifiedConfig &config, double dt = 0.05,
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

  int selected_panel_id() const { return ctx_.selected_panel_id; }
  double normalized_entropy() const { return ctx_.entropy_norm; }
  double max_panel_probability() const { return ctx_.max_prob; }
  const DebugSnapshot &debug_snapshot() const { return debug_snapshot_; }

 private:
  struct WarmupGroup {
    int sample_count = 0;
    double mean_z = 0.0;
    Eigen::Vector3d last_pos = Eigen::Vector3d::Zero();
    double last_yaw = 0.0;
  };

  struct WarmupCommit {
    bool ready = false;
    int current_panel = -1;
    double dz_small = std::numeric_limits<double>::quiet_NaN();
    double dz_large = std::numeric_limits<double>::quiet_NaN();
  };

  int infer_init_panel(const ObservationData &obs) const;
  int semantic_from_panel(int panel_id) const;
  void sync_runtime_from_backend(const outpost_v2::BackendStateSnapshot &snap);
  void reset_warmup();
  void update_warmup_evidence(const ObservationData &obs);
  WarmupCommit try_commit_warmup() const;
  bool run_warmup_update(const ObservationData &obs);
  void refresh_debug(const ObservationData *obs,
                     const outpost_v2::BindingCandidate &candidate,
                     const binder::BinderOutput &binder_out,
                     const binder::BinderDebugSnapshot &binder_dbg,
                     const mode::ModeDecision &mode_decision);

  UnifiedConfig config_;
  double radius_ = 0.26;
  std::array<double, 3> z_offsets_{0.06, 0.0, -0.06};

  outpost_v2::ObservationFrontend obs_frontend_;
  outpost_v2::OutpostBinderBridge binder_bridge_;
  mode::EvidenceFuser evidence_fuser_;
  mode::ModeFSM mode_fsm_;
  outpost_v2::OutpostAmbiguousBackend ambiguous_backend_;
  outpost_v2::OutpostStructuredBackend structured_backend_;
  outpost_v2::OutpostOutputAdapter output_adapter_;
  ManeuverDetector maneuver_detector_;

  outpost_v2::OutpostRuntimeContext ctx_;
  DebugSnapshot debug_snapshot_{};

  bool warmup_active_ = false;
  int warmup_frames_ = 0;
  int warmup_current_group_ = -1;
  std::vector<WarmupGroup> warmup_groups_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_TRACKER_V2_HPP_
