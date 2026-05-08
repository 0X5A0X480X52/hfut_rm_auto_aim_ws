// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDER_TYPES_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDER_TYPES_HPP_

#include <cstdint>
#include <limits>
#include <vector>

#include "max_entropy_tracker/binder/model/binder_enums.hpp"
#include "max_entropy_tracker/binder/model/robot_binding_profile.hpp"

namespace fyt::auto_aim::binder {

struct BinderFrameInput {
  double timestamp = 0.0;
  const RobotBindingProfile * profile = nullptr;
  int obs_count = 1;

  int candidate_id = -1;
  double candidate_prob = 0.0;
  double candidate_margin = 0.0;

  std::vector<double> obs_z_values;
  std::vector<double> obs_yaw_values;
  std::vector<int> obs_panel_ids;
  std::vector<HeightLabel> obs_height_labels;
  double z_jump = 0.0;
  bool has_z_jump = false;

  double yaw_rate_est = 0.0;
  int spin_direction_hint = 0;

  double selected_yaw_err = 0.0;
  double cost_margin = 0.0;

  double same_panel_residual = 0.0;
  double nis = 0.0;
  bool has_history = false;

  TrackEventType event_type = TrackEventType::AMBIGUOUS;
  bool is_reacquired = false;
  double gap_dt = 0.0;
  int lost_frames = 0;

  // Phase 6 soft fusion: 2D/proxy/phase evidence for binding scoring.
  double phase_confidence = 0.0;
  double ping_pong_risk = 0.0;
  double track_continuity_score = 1.0;
  double topology_consistency_score = 1.0;
  double kinematic_consistency = 1.0;
  double velocity_dir_cos = 1.0;
  double acc_norm = 0.0;
  bool has_soft_fusion = false;
};

struct JumpDecision {
  bool detected = false;
  JumpKind jump_kind = JumpKind::NONE;
  int from_id = -1;
  int to_id = -1;
  double confidence = 0.0;
  int evidence_mask = 0;
  int reason_code = 0;
  double signature_score = 0.0;
};

struct TargetDecision {
  int target_id = -1;
  double confidence = 0.0;
  HeightLabel height_label = HeightLabel::UNKNOWN;
};

struct BinderOutput {
  // Panel used by the downstream filter for this frame. During a pending
  // transition this can be the pending candidate before bound_id is committed.
  int selected_id = -1;
  int bound_id = -1;
  int pending_id = -1;
  HeightLabel height_label = HeightLabel::UNKNOWN;
  BindingFSMState fsm_state = BindingFSMState::LOCKED;
  BindingAction action = BindingAction::HOLD;
  bool switch_occurred = false;
  int switch_reason = 0;
  double binding_confidence = 0.0;
  bool binding_conflict_for_update = false;
  double same_panel_score = 0.0;
  double switch_score = 0.0;
};

struct BindingHealth {
  double score = 1.0;
  bool anomaly_detected = false;
  int consecutive_bad_frames = 0;
  bool force_rebind_recommend = false;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDER_TYPES_HPP_
