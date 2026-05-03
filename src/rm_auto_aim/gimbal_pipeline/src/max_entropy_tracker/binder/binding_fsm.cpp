// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/core/binding_fsm.hpp"

#include <algorithm>
#include <iostream>

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
  pending_window_remaining_ = 0;
  switch_occurred_ = false;
  switch_reason_ = 0;
  confidence_ = 0.5;
}

BindingAction BindingFSM::step(int target_id, HeightLabel target_label,
                               double target_confidence,
                               const JumpDecision & jump,
                               const BindingHealth & health) {
  switch_occurred_ = false;
  switch_reason_ = 0;
  const double target_conf =
      std::clamp(target_confidence, 0.0, 1.0);
  auto resolved_target_label = [&]() -> HeightLabel {
    if (target_label != HeightLabel::UNKNOWN) {
      return target_label;
    }
    if (bound_label_ != HeightLabel::UNKNOWN) {
      return bound_label_;
    }
    return HeightLabel::LOWER;
  };
  const int confirm_required = std::max(1, config_.confirm_frames);
  const int pending_window = std::max(
      confirm_required,
      (config_.pending_window_frames > 0 ? config_.pending_window_frames
                                         : confirm_required + 1));
  const double post_jump_min_conf =
      std::clamp(config_.post_jump_min_confidence, 0.0, 1.0);

  std::cout << "[BindingFSM] BindingFSM::step: target_id=" << target_id
            << ", target_confidence=" << target_confidence 
            << ", jump_detected=" << jump.detected
            << ", jump_kind=" << static_cast<int>(jump.jump_kind)
            << ", health_force_rebind=" << health.force_rebind_recommend
            << ", state=" << static_cast<int>(state_)
            << std::endl;

  // ── UNLOCKED: wait for health recovery ──
  if (state_ == BindingFSMState::UNLOCKED) {
    if (!health.force_rebind_recommend && target_id >= 0) {
      if (confirm_counter_.tick(true)) {
        bound_id_ = target_id;
        bound_label_ = resolved_target_label();
        confidence_ = target_conf;
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
    confidence_ = std::max(0.10, 0.50 * confidence_);
    confirm_counter_.reset();
    switch_occurred_ = true;
    switch_reason_ = 6;
    return BindingAction::FORCE_REBIND;
  }

  // ── LOCKED_NEW: hold period after switch ──
  if (state_ == BindingFSMState::LOCKED_NEW) {
    if (target_id == bound_id_) {
      confidence_ = std::clamp(0.85 * confidence_ + 0.15 * target_conf, 0.0, 1.0);
    }
    if (hold_remaining_ > 0) --hold_remaining_;
    if (hold_remaining_ == 0) {
      state_ = BindingFSMState::LOCKED;
      return BindingAction::RELOCK;
    }
    return BindingAction::HOLD;
  }

  // ── LOCKED: normal steady phase ──
  if (state_ == BindingFSMState::LOCKED) {
    if (target_id < 0 || target_id == bound_id_ || !jump.detected) {
      if (target_id == bound_id_ && target_conf > 0.0) {
        confidence_ = std::clamp(0.85 * confidence_ + 0.15 * target_conf, 0.0, 1.0);
      }
      return BindingAction::HOLD;
    }

    if (config_.confirm_frames <= 1) {
      // Immediate switch
      bound_id_ = target_id;
      bound_label_ = resolved_target_label();
      confidence_ = target_conf;
      state_ = BindingFSMState::LOCKED_NEW;
      hold_remaining_ = config_.lock_new_hold_frames;
      switch_occurred_ = true;
      switch_reason_ = 1;
      return BindingAction::SWITCH;
    }

    state_ = BindingFSMState::PENDING_SWITCH;
    pending_target_ = target_id;
    pending_window_remaining_ = pending_window;
    confirm_counter_.reset();
    confirm_counter_.tick(true);  // Jump pulse is the trigger anchor.
    confidence_ = std::clamp(std::max(confidence_, 0.60 * target_conf), 0.0, 1.0);
    --pending_window_remaining_;
    return BindingAction::PENDING;
  }

  // ── PENDING_SWITCH: post-jump confirmation window ──
  if (state_ == BindingFSMState::PENDING_SWITCH) {
    if (pending_target_ < 0 || pending_target_ == bound_id_) {
      state_ = BindingFSMState::LOCKED;
      pending_target_ = -1;
      pending_window_remaining_ = 0;
      confirm_counter_.reset();
      return BindingAction::HOLD;
    }

    if (jump.detected && target_id >= 0 && target_id != pending_target_ &&
        target_id != bound_id_) {
      // New jump pulse points to a different panel: restart pending candidate.
      pending_target_ = target_id;
      pending_window_remaining_ = pending_window;
      confirm_counter_.reset();
      confirm_counter_.tick(true);
      --pending_window_remaining_;
      return BindingAction::PENDING;
    }

    const bool same_pending_target = (target_id == pending_target_);
    const bool support =
        same_pending_target &&
        (jump.detected || target_confidence >= post_jump_min_conf);

    if (confirm_counter_.tick(support)) {
      bound_id_ = target_id;
      bound_label_ = resolved_target_label();
      confidence_ = target_conf;
      state_ = BindingFSMState::LOCKED_NEW;
      hold_remaining_ = config_.lock_new_hold_frames;
      pending_target_ = -1;
      pending_window_remaining_ = 0;
      confirm_counter_.reset();
      switch_occurred_ = true;
      switch_reason_ = 1;
      return BindingAction::SWITCH;
    }

    --pending_window_remaining_;
    if (support) {
      confidence_ = std::clamp(std::max(confidence_, 0.70 * target_conf), 0.0, 1.0);
    }
    if (pending_window_remaining_ > 0) {
      return BindingAction::PENDING;
    }

    // Window expired without enough support: cancel pending switch.
    state_ = BindingFSMState::LOCKED;
    pending_target_ = -1;
    pending_window_remaining_ = 0;
    confirm_counter_.reset();
    switch_reason_ = 7;
    return BindingAction::HOLD;
  }

  return BindingAction::HOLD;
}

}  // namespace fyt::auto_aim::binder
