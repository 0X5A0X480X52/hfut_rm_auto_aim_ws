// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDER_TYPES_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDER_TYPES_HPP_

#include "max_entropy_tracker/binder/model/binder_enums.hpp"

namespace fyt::auto_aim::binder {

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

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDER_TYPES_HPP_
