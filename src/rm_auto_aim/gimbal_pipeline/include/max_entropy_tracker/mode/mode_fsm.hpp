// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_MODE_MODE_FSM_HPP_
#define MAX_ENTROPY_TRACKER_MODE_MODE_FSM_HPP_

#include "max_entropy_tracker/mode/mode_types.hpp"

namespace fyt::auto_aim::mode {

struct ModeFSMConfig {
  int enter_confirm_frames = 3;
  int exit_confirm_frames = 4;
  int min_dwell_frames = 6;
  double enter_threshold = 0.72;
  double exit_threshold = 0.45;
  double entropy_enter = 0.75;
  double entropy_exit = 0.55;
  double max_prob_enter = 0.60;
  double max_prob_exit = 0.75;
  int stable_frames = 4;
};

class ModeFSM {
 public:
  explicit ModeFSM(const ModeFSMConfig & cfg);

  void reset(TrackMode init_mode);
  ModeDecision step(const ModeEvidence & ev);

  TrackMode mode() const { return mode_; }
  const ModeDebugSnapshot & debug_snapshot() const { return debug_; }

 private:
  ModeFSMConfig cfg_;
  TrackMode mode_ = TrackMode::AMBIGUOUS;
  int enter_counter_ = 0;
  int exit_counter_ = 0;
  int stable_counter_ = 0;
  int last_candidate_id_ = -1;
  int dwell_counter_ = 0;
  ModeDebugSnapshot debug_;
};

}  // namespace fyt::auto_aim::mode

#endif  // MAX_ENTROPY_TRACKER_MODE_MODE_FSM_HPP_
