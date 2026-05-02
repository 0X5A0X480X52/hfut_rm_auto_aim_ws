// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_DEBUG_BINDER_DEBUG_SNAPSHOT_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_DEBUG_BINDER_DEBUG_SNAPSHOT_HPP_

#include <limits>
#include <string>

#include "max_entropy_tracker/binder/model/binder_enums.hpp"

namespace fyt::auto_aim::binder {

struct BinderDebugSnapshot {
  bool valid = false;

  std::string decoder_name;
  std::string id_binder_name;
  std::string scorer_name;

  bool jump_detected = false;
  JumpKind jump_kind = JumpKind::NONE;
  double jump_confidence = 0.0;
  int from_id = -1;
  int to_id = -1;

  int target_id = -1;
  double target_confidence = 0.0;

  BindingFSMState fsm_state = BindingFSMState::LOCKED;
  BindingAction action = BindingAction::HOLD;
  int switch_reason = 0;

  double health_score = 1.0;
  int consecutive_bad_frames = 0;
  bool force_rebind_flag = false;

  int obs_count = 0;
  double z_jump = 0.0;
  int spin_direction = 0;
  double candidate_prob = 0.0;
  double candidate_margin = 0.0;

  double dz_jump_est = std::numeric_limits<double>::quiet_NaN();
  double dz_small_est = std::numeric_limits<double>::quiet_NaN();
  double dz_large_est = std::numeric_limits<double>::quiet_NaN();
  double period_confidence = 0.0;
  int period_phase = -1;
  double signature_score = 0.0;
  TrackEventType event_type = TrackEventType::AMBIGUOUS;
  bool is_reacquired = false;
  double gap_dt = 0.0;
  int lost_frames = 0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_DEBUG_BINDER_DEBUG_SNAPSHOT_HPP_
