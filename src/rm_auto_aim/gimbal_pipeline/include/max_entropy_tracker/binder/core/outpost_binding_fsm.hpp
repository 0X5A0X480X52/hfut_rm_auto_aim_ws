// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_CORE_OUTPOST_BINDING_FSM_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_CORE_OUTPOST_BINDING_FSM_HPP_

#include "max_entropy_tracker/binder/model/outpost_binding_types.hpp"
#include "max_entropy_tracker/core/config.hpp"

namespace fyt::auto_aim::binder {

class OutpostBindingFSM {
 public:
  explicit OutpostBindingFSM(const UnifiedConfig & config);

  void reset(int init_panel_id);
  OutpostBindingFSMOutput step(const OutpostBindingFSMInput & input);
  OutpostBindingFSMOutput force_rebind(int panel_id, int reason);
  OutpostBindingFSMOutput output() const;

  int selected_id() const { return selected_panel_id_; }
  int bound_id() const { return bound_panel_id_; }
  int pending_id() const { return transition_candidate_panel_; }
  BindingFSMState state() const { return state_; }

 private:
  UnifiedConfig config_;
  int selected_panel_id_ = 0;
  int bound_panel_id_ = -1;
  BindingFSMState state_ = BindingFSMState::LOCKED;
  int transition_candidate_panel_ = -1;
  int transition_confirm_count_ = 0;
  bool switch_event_ = false;
  int switch_reason_ = 0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_CORE_OUTPOST_BINDING_FSM_HPP_
