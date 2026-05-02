// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/mode/mode_fsm.hpp"

#include <algorithm>
#include <iostream>

namespace fyt::auto_aim::mode {

namespace {

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

}  // namespace

ModeFSM::ModeFSM(const ModeFSMConfig & cfg) : cfg_(cfg) {}

void ModeFSM::reset(TrackMode init_mode) {
  mode_ = init_mode;
  enter_counter_ = 0;
  exit_counter_ = 0;
  dwell_counter_ = 0;
  debug_ = ModeDebugSnapshot{};
  debug_.valid = true;
  debug_.mode = mode_;
}

ModeDecision ModeFSM::step(const ModeEvidence & ev) {
  ModeDecision out;
  out.mode = mode_;
  out.confidence = (mode_ == TrackMode::STRUCTURED)
                       ? clamp01(ev.enter_score)
                       : clamp01(1.0 - ev.exit_score);

  ++dwell_counter_;
  debug_.valid = true;
  debug_.mode = mode_;
  debug_.enter_score = ev.enter_score;
  debug_.exit_score = ev.exit_score;
  debug_.enter_counter = enter_counter_;
  debug_.exit_counter = exit_counter_;
  debug_.dwell_counter = dwell_counter_;
  debug_.last_reason = TransitionReason::NONE;

  std::cout << "[ModeFSM Step] ModeFSM Step: mode=" << (mode_ == TrackMode::STRUCTURED ? "STRUCTURED" : "AMBIGUOUS")
            << ", enter_score=" << ev.enter_score
            << "enter_threshold=" << cfg_.enter_threshold
            << ", exit_score=" << ev.exit_score
            << ", exit_threshold=" << cfg_.exit_threshold
            << ", enter_counter=" << enter_counter_
            << ", exit_counter=" << exit_counter_
            << ", dwell_counter=" << dwell_counter_
            << std::endl;

  if (mode_ == TrackMode::AMBIGUOUS) {
    const bool strong_enter = (ev.enter_score >= cfg_.enter_threshold);
    if (strong_enter) {
      // jump_event_detected is a one-frame pulse; do not require multi-frame
      // confirmation for this case.
      if (ev.jump_event_detected) {
        mode_ = TrackMode::STRUCTURED;
        out.mode = mode_;
        out.switched = true;
        out.reason = TransitionReason::STRONG_EVIDENCE_ENTER;
        out.confidence = clamp01(ev.enter_score);
        enter_counter_ = 0;
        exit_counter_ = 0;
        dwell_counter_ = 0;
      } else {
        ++enter_counter_;
        if (enter_counter_ >= std::max(1, cfg_.enter_confirm_frames)) {
          mode_ = TrackMode::STRUCTURED;
          out.mode = mode_;
          out.switched = true;
          out.reason = TransitionReason::STRONG_EVIDENCE_ENTER;
          out.confidence = clamp01(ev.enter_score);
          enter_counter_ = 0;
          exit_counter_ = 0;
          dwell_counter_ = 0;
        }
      }
    } else {
      enter_counter_ = 0;
    }
  } else {
    if (ev.binder_force_rebind) {
      mode_ = TrackMode::AMBIGUOUS;
      out.mode = mode_;
      out.switched = true;
      out.reason = TransitionReason::FORCED_REBIND_EXIT;
      out.confidence = clamp01(1.0 - ev.exit_score);
      enter_counter_ = 0;
      exit_counter_ = 0;
      dwell_counter_ = 0;
    } else if (dwell_counter_ >= std::max(1, cfg_.min_dwell_frames)) {
      if (ev.exit_score >= cfg_.exit_threshold) {
        ++exit_counter_;
        if (exit_counter_ >= std::max(1, cfg_.exit_confirm_frames)) {
          mode_ = TrackMode::AMBIGUOUS;
          out.mode = mode_;
          out.switched = true;
          out.reason = TransitionReason::WEAK_EVIDENCE_EXIT;
          out.confidence = clamp01(1.0 - ev.exit_score);
          enter_counter_ = 0;
          exit_counter_ = 0;
          dwell_counter_ = 0;
        }
      } else {
        exit_counter_ = 0;
      }
    }
  }

  debug_.mode = mode_;
  debug_.enter_counter = enter_counter_;
  debug_.exit_counter = exit_counter_;
  debug_.dwell_counter = dwell_counter_;
  debug_.last_reason = out.reason;
  return out;
}

}  // namespace fyt::auto_aim::mode
