// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_CORE_BINDING_FSM_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_CORE_BINDING_FSM_HPP_

#include "max_entropy_tracker/binder/core/binding_window_counter.hpp"
#include "max_entropy_tracker/binder/model/binder_enums.hpp"
#include "max_entropy_tracker/binder/model/binder_types.hpp"

namespace fyt::auto_aim::binder {

struct BindingFSMConfig {
  int confirm_frames = 3;
  int lock_new_hold_frames = 2;
  int force_rebind_bad_frames = 10;
  int pending_window_frames = 0;  // 0 -> auto derive from confirm_frames
  double post_jump_min_confidence = 0.45;
};

class BindingFSM {
 public:
  explicit BindingFSM(const BindingFSMConfig & config);

  void reset(int panel_id, HeightLabel label);

  BindingAction step(int target_id, HeightLabel target_label,
                     double target_confidence,
                     const JumpDecision & jump,
                     const BindingHealth & health);

  int selected_id() const { return bound_id_; }
  HeightLabel selected_label() const { return bound_label_; }
  int pending_id() const { return pending_target_; }
  BindingFSMState state() const { return state_; }
  bool switch_occurred() const { return switch_occurred_; }
  int switch_reason() const { return switch_reason_; }
  double binding_confidence() const { return confidence_; }
  void set_confidence(double conf) { confidence_ = conf; }

 private:
  BindingFSMConfig config_;
  int bound_id_ = -1;
  HeightLabel bound_label_ = HeightLabel::UNKNOWN;
  BindingFSMState state_ = BindingFSMState::LOCKED;
  int pending_target_ = -1;
  BindingWindowCounter confirm_counter_;
  BindingWindowCounter bad_health_counter_;
  int hold_remaining_ = 0;
  int pending_window_remaining_ = 0;
  bool switch_occurred_ = false;
  int switch_reason_ = 0;
  double confidence_ = 0.0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_CORE_BINDING_FSM_HPP_
