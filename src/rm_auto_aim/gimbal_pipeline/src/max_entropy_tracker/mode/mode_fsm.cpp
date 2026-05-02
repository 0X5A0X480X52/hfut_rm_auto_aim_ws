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
  stable_counter_ = 0;
  last_candidate_id_ = -1;
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
  debug_.stable_counter = stable_counter_;
  debug_.last_candidate_id = last_candidate_id_;
  debug_.dwell_counter = dwell_counter_;
  debug_.last_reason = TransitionReason::NONE;

  if (ev.candidate_id >= 0) {
    if (ev.candidate_id == last_candidate_id_) {
      ++stable_counter_;
    } else {
      last_candidate_id_ = ev.candidate_id;
      stable_counter_ = 1;
    }
  } else {
    stable_counter_ = 0;
    last_candidate_id_ = -1;
  }

  const bool posterior_enter =
      (ev.entropy_norm < std::clamp(cfg_.entropy_exit, 0.0, 1.0)) &&
      (ev.max_prob > std::clamp(cfg_.max_prob_exit, 0.0, 1.0));
  const bool posterior_exit =
      (ev.entropy_norm > std::clamp(cfg_.entropy_enter, 0.0, 1.0)) ||
      (ev.max_prob < std::clamp(cfg_.max_prob_enter, 0.0, 1.0));

  std::cout << "[ModeFSM Step] ModeFSM Step: mode=" << (mode_ == TrackMode::STRUCTURED ? "STRUCTURED" : "AMBIGUOUS")
            << ", enter_score=" << ev.enter_score
            << ", continuous_enter_score=" << ev.continuous_enter_score
            << ", event_enter_score=" << ev.event_enter_score
            << ", enter_threshold=" << cfg_.enter_threshold
            << ", exit_score=" << ev.exit_score
            << ", exit_threshold=" << cfg_.exit_threshold
            << ", entropy_norm=" << ev.entropy_norm
            << ", max_prob=" << ev.max_prob
            << ", candidate_id=" << ev.candidate_id
            << ", stable_counter=" << stable_counter_
            << ", posterior_enter=" << posterior_enter
            << ", posterior_exit=" << posterior_exit
            << ", enter_counter=" << enter_counter_
            << ", exit_counter=" << exit_counter_
            << ", dwell_counter=" << dwell_counter_
            << std::endl;

  if (mode_ == TrackMode::AMBIGUOUS) {
    const bool strong_enter = (ev.enter_score >= cfg_.enter_threshold);
    const bool stable_posterior_enter =
        posterior_enter && stable_counter_ >= std::max(1, cfg_.stable_frames);
    const bool event_enter =
        (ev.jump_event_detected || ev.has_2dz_signature) && strong_enter;

    if (event_enter || stable_posterior_enter) {
      mode_ = TrackMode::STRUCTURED;
      out.mode = mode_;
      out.switched = true;
      out.reason = TransitionReason::STRONG_EVIDENCE_ENTER;
      out.confidence = clamp01(std::max(ev.enter_score, ev.max_prob));
      enter_counter_ = 0;
      exit_counter_ = 0;
      dwell_counter_ = 0;
    } else if (strong_enter) {
      ++enter_counter_;
      const int confirm_required = std::max(1, cfg_.enter_confirm_frames);
      const int stable_required =
          std::max(confirm_required, std::max(1, cfg_.stable_frames));
      if (enter_counter_ >= confirm_required &&
          stable_counter_ >= stable_required) {
        mode_ = TrackMode::STRUCTURED;
        out.mode = mode_;
        out.switched = true;
        out.reason = TransitionReason::STRONG_EVIDENCE_ENTER;
        out.confidence = clamp01(std::max(ev.enter_score, ev.max_prob));
        enter_counter_ = 0;
        exit_counter_ = 0;
        dwell_counter_ = 0;
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
      const bool weak_exit =
          posterior_exit || (ev.exit_score >= cfg_.exit_threshold);
      if (weak_exit) {
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
  debug_.stable_counter = stable_counter_;
  debug_.last_candidate_id = last_candidate_id_;
  debug_.dwell_counter = dwell_counter_;
  debug_.last_reason = out.reason;
  return out;
}

}  // namespace fyt::auto_aim::mode
