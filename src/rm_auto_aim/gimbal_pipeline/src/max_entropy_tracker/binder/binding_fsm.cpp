// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/core/binding_fsm.hpp"

namespace fyt::auto_aim::binder {

BindingFSM::BindingFSM(const BindingFSMConfig & config)
    : config_(config),
      confirm_counter_(std::max(1, config.confirm_frames)),
      bad_health_counter_(std::max(1, config.force_rebind_bad_frames)) {}

void BindingFSM::reset(int panel_id, HeightLabel label) {
  bound_id_ = panel_id;
  bound_label_ = (label == HeightLabel::UNKNOWN) ? HeightLabel::LOWER : label;
  state_ = BindingFSMState::LOCKED;
  pending_target_ = -1;
  confirm_counter_.reset();
  bad_health_counter_.reset();
  hold_remaining_ = 0;
  switch_occurred_ = false;
  switch_reason_ = 0;
  confidence_ = 0.5;
}

BindingAction BindingFSM::step(int target_id, double /*target_confidence*/,
                               const JumpDecision & jump,
                               const BindingHealth & health) {
  switch_occurred_ = false;
  switch_reason_ = 0;

  // ── UNLOCKED: wait for health recovery ──
  if (state_ == BindingFSMState::UNLOCKED) {
    if (!health.force_rebind_recommend && target_id >= 0) {
      if (confirm_counter_.tick(true)) {
        bound_id_ = target_id;
        state_ = BindingFSMState::LOCKED;
        confirm_counter_.reset();
        switch_reason_ = 5;
        return BindingAction::RELOCK;
      }
    } else {
      confirm_counter_.reset();
    }
    return BindingAction::HOLD;
  }

  // ── Health-triggered force rebind ──
  if (health.force_rebind_recommend) {
    state_ = BindingFSMState::UNLOCKED;
    confirm_counter_.reset();
    switch_occurred_ = true;
    switch_reason_ = 6;
    return BindingAction::FORCE_REBIND;
  }

  // ── LOCKED_NEW: hold period after switch ──
  if (state_ == BindingFSMState::LOCKED_NEW) {
    if (hold_remaining_ > 0) --hold_remaining_;
    if (hold_remaining_ == 0) {
      state_ = BindingFSMState::LOCKED;
      return BindingAction::RELOCK;
    }
    return BindingAction::HOLD;
  }

  // ── Same target: stay locked ──
  if (target_id < 0 || target_id == bound_id_) {
    if (state_ == BindingFSMState::PENDING_SWITCH) {
      state_ = BindingFSMState::LOCKED;
      pending_target_ = -1;
      confirm_counter_.reset();
    }
    return BindingAction::HOLD;
  }

  // ── No jump detected: retain current binding ──
  if (!jump.detected) {
    if (state_ == BindingFSMState::PENDING_SWITCH) {
      state_ = BindingFSMState::LOCKED;
      pending_target_ = -1;
      confirm_counter_.reset();
    }
    return BindingAction::HOLD;
  }

  // ── Jump detected, target differs from bound ──
  if (state_ == BindingFSMState::LOCKED) {
    if (config_.confirm_frames <= 1) {
      // Immediate switch
      bound_id_ = target_id;
      state_ = BindingFSMState::LOCKED_NEW;
      hold_remaining_ = config_.lock_new_hold_frames;
      switch_occurred_ = true;
      switch_reason_ = 1;
      return BindingAction::SWITCH;
    }
    state_ = BindingFSMState::PENDING_SWITCH;
    pending_target_ = target_id;
    confirm_counter_.reset();
    confirm_counter_.tick(true);
    return BindingAction::PENDING;
  }

  // ── PENDING_SWITCH: accumulate confirmation ──
  if (target_id == pending_target_) {
    if (confirm_counter_.tick(true)) {
      bound_id_ = target_id;
      state_ = BindingFSMState::LOCKED_NEW;
      hold_remaining_ = config_.lock_new_hold_frames;
      pending_target_ = -1;
      confirm_counter_.reset();
      switch_occurred_ = true;
      switch_reason_ = 1;
      return BindingAction::SWITCH;
    }
    return BindingAction::PENDING;
  }

  // Different candidate during pending: restart transition
  pending_target_ = target_id;
  confirm_counter_.reset();
  confirm_counter_.tick(true);
  return BindingAction::PENDING;
}

}  // namespace fyt::auto_aim::binder
