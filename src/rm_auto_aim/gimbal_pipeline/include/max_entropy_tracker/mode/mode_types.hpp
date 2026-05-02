// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_MODE_MODE_TYPES_HPP_
#define MAX_ENTROPY_TRACKER_MODE_MODE_TYPES_HPP_

#include "max_entropy_tracker/mode/mode_enums.hpp"

namespace fyt::auto_aim::mode {

struct ModeEvidence {
  double timestamp = 0.0;
  int obs_count = 0;
  bool has_dual_obs = false;
  int candidate_id = -1;

  bool jump_detected = false;
  bool jump_event_detected = false;
  bool has_2dz_signature = false;
  double jump_confidence = 0.0;
  double candidate_margin = 0.0;

  double binder_health_score = 1.0;
  int binder_bad_frames = 0;
  bool binder_force_rebind = false;

  double entropy_norm = 1.0;
  double max_prob = 0.0;

  double enter_score = 0.0;
  double exit_score = 0.0;
  double continuous_enter_score = 0.0;
  double event_enter_score = 0.0;
};

struct ModeDecision {
  TrackMode mode = TrackMode::AMBIGUOUS;
  bool switched = false;
  TransitionReason reason = TransitionReason::NONE;
  double confidence = 0.0;
};

struct ModeDebugSnapshot {
  bool valid = false;
  TrackMode mode = TrackMode::AMBIGUOUS;
  double enter_score = 0.0;
  double exit_score = 0.0;
  int enter_counter = 0;
  int exit_counter = 0;
  int stable_counter = 0;
  int last_candidate_id = -1;
  int dwell_counter = 0;
  TransitionReason last_reason = TransitionReason::NONE;
};

}  // namespace fyt::auto_aim::mode

#endif  // MAX_ENTROPY_TRACKER_MODE_MODE_TYPES_HPP_
