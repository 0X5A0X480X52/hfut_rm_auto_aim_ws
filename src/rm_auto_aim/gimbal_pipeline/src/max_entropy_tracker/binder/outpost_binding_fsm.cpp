// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/core/outpost_binding_fsm.hpp"

#include <algorithm>

namespace fyt::auto_aim::binder {

OutpostBindingFSM::OutpostBindingFSM(const UnifiedConfig & config)
    : config_(config) {}

void OutpostBindingFSM::reset(int init_panel_id) {
  selected_panel_id_ = std::clamp(init_panel_id, 0, 2);
  bound_panel_id_ = -1;
  state_ = BindingFSMState::LOCKED;
  transition_candidate_panel_ = -1;
  transition_confirm_count_ = 0;
  switch_event_ = false;
  switch_reason_ = 0;
}

OutpostBindingFSMOutput OutpostBindingFSM::step(
    const OutpostBindingFSMInput & input) {
  switch_event_ = false;
  switch_reason_ = 0;

  const double min_candidate_prob =
      std::clamp(config_.outpost.binding_min_candidate_prob, 0.0, 1.0);
  const double min_candidate_margin =
      std::clamp(config_.outpost.binding_min_candidate_margin, 0.0, 1.0);
  const double switch_strong_score =
      std::clamp(config_.outpost.binding_switch_strong_score, 0.0, 1.0);

  if (input.candidate_prob < min_candidate_prob) {
    switch_reason_ = 2;
  } else if (input.candidate_margin < min_candidate_margin) {
    switch_reason_ = 3;
  }

  if (bound_panel_id_ < 0) {
    if (input.candidate_id < 0 || input.candidate_prob < min_candidate_prob ||
        input.candidate_margin < min_candidate_margin) {
      state_ = BindingFSMState::LOCKED;
      transition_candidate_panel_ = -1;
      transition_confirm_count_ = 0;
      selected_panel_id_ = input.candidate_id;
      return output();
    }
    bound_panel_id_ = input.candidate_id;
    state_ = BindingFSMState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    selected_panel_id_ = bound_panel_id_;
    return output();
  }

  const int confirm_required =
      std::max(1, config_.outpost.binding_transition_confirm_frames);
  const bool candidate_valid = input.candidate_prob >= min_candidate_prob &&
                               input.candidate_margin >= min_candidate_margin;
  if (!candidate_valid) {
    state_ = BindingFSMState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    selected_panel_id_ = bound_panel_id_;
    return output();
  }

  if (state_ == BindingFSMState::LOCKED) {
    const bool trigger_transition =
        (input.candidate_id != bound_panel_id_) &&
        (input.switch_score > (0.50 + 0.20 * input.same_panel_score));
    if (trigger_transition) {
      if (confirm_required <= 1) {
        bound_panel_id_ = input.candidate_id;
        state_ = BindingFSMState::LOCKED;
        transition_candidate_panel_ = -1;
        transition_confirm_count_ = 0;
        selected_panel_id_ = bound_panel_id_;
        switch_event_ = true;
        switch_reason_ = 1;
        return output();
      }
      state_ = BindingFSMState::PENDING_SWITCH;
      transition_candidate_panel_ = input.candidate_id;
      transition_confirm_count_ = 1;
      selected_panel_id_ = transition_candidate_panel_;
    } else {
      transition_candidate_panel_ = -1;
      transition_confirm_count_ = 0;
      selected_panel_id_ = bound_panel_id_;
    }
    return output();
  }

  if (input.candidate_id == transition_candidate_panel_ &&
      input.switch_score > switch_strong_score) {
    ++transition_confirm_count_;
  } else if (input.candidate_id != bound_panel_id_ &&
             input.switch_score > std::max(0.60, switch_strong_score)) {
    transition_candidate_panel_ = input.candidate_id;
    transition_confirm_count_ = 1;
  } else {
    state_ = BindingFSMState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    switch_reason_ = 4;
    selected_panel_id_ = bound_panel_id_;
    return output();
  }

  if (transition_confirm_count_ >= confirm_required) {
    bound_panel_id_ = transition_candidate_panel_;
    state_ = BindingFSMState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    switch_event_ = true;
    switch_reason_ = 1;
    selected_panel_id_ = bound_panel_id_;
  } else {
    selected_panel_id_ = transition_candidate_panel_;
  }

  return output();
}

OutpostBindingFSMOutput OutpostBindingFSM::force_rebind(
    int panel_id, int reason) {
  bound_panel_id_ = panel_id;
  selected_panel_id_ = panel_id;
  state_ = BindingFSMState::LOCKED;
  transition_candidate_panel_ = -1;
  transition_confirm_count_ = 0;
  switch_event_ = true;
  switch_reason_ = reason;
  return output();
}

OutpostBindingFSMOutput OutpostBindingFSM::output() const {
  OutpostBindingFSMOutput out;
  out.selected_id = selected_panel_id_;
  out.bound_id = bound_panel_id_;
  out.pending_id = transition_candidate_panel_;
  out.height_label = outpostHeightLabelFromPanel(selected_panel_id_);
  out.state = state_;
  out.action = switch_event_
                   ? BindingAction::SWITCH
                   : (state_ == BindingFSMState::PENDING_SWITCH
                          ? BindingAction::PENDING
                          : BindingAction::HOLD);
  out.switch_occurred = switch_event_;
  out.switch_reason = switch_reason_;
  out.transition_state = (state_ == BindingFSMState::PENDING_SWITCH) ? 1 : 0;
  return out;
}

}  // namespace fyt::auto_aim::binder
