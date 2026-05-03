// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/pipeline/binder_pipeline.hpp"

#include <algorithm>

namespace fyt::auto_aim::binder {

BinderPipeline::BinderPipeline(
    std::unique_ptr<JumpEventDecoder> decoder,
    std::unique_ptr<IDBinder> id_binder,
    std::unique_ptr<HypothesisScorer> scorer,
    const BinderPipelineConfig & config)
    : decoder_(std::move(decoder)),
      id_binder_(std::move(id_binder)),
      scorer_(std::move(scorer)),
      config_(config),
      fsm_(config.fsm) {}

void BinderPipeline::reset(int init_panel_id, HeightLabel init_label,
                           std::optional<double> obs_z) {
  fsm_.reset(init_panel_id, init_label);

  decoder_ctx_ = DecoderContext{};
  decoder_ctx_.last_obs_z = obs_z;
  decoder_ctx_.last_panel_id = init_panel_id;

  binder_ctx_ = BinderContext{};
  binder_ctx_.current_bound_id = init_panel_id;
  binder_ctx_.current_bound_label = init_label;

  scorer_ctx_ = ScorerContext{};

  debug_ = BinderDebugSnapshot{};
  debug_.valid = true;
  if (decoder_) debug_.decoder_name = decoder_->name();
  if (id_binder_) debug_.id_binder_name = id_binder_->name();
  if (scorer_) debug_.scorer_name = scorer_->name();
}

BinderOutput BinderPipeline::step(const BinderFrameInput & input) {
  // 1. Jump detection
  JumpDecision jump;
  if (decoder_) {
    jump = decoder_->decode(input, decoder_ctx_);
  }

  // 2. ID proposal
  TargetDecision target;
  if (id_binder_) {
    target = id_binder_->propose(input, jump, binder_ctx_);
  } else {
    target.target_id = input.candidate_id;
    target.confidence = input.candidate_prob;
  }

  // 3. Build provisional output for scorer
  BinderOutput provisional;
  provisional.selected_id = fsm_.selected_id();
  provisional.height_label = fsm_.selected_label();
  provisional.fsm_state = fsm_.state();

  // 4. Health scoring
  BindingHealth health;
  if (scorer_) {
    health = scorer_->evaluate(input, provisional, scorer_ctx_);
  }

  // 5. FSM step
  BindingAction action = fsm_.step(target.target_id, target.height_label,
                                   target.confidence,
                                   jump, health);

  // 6. Build output
  BinderOutput output;
  output.pending_id = fsm_.pending_id();
  output.selected_id = fsm_.selected_id();
  output.bound_id = fsm_.selected_id();
  output.height_label = fsm_.selected_label();
  output.fsm_state = fsm_.state();
  output.action = action;
  output.switch_occurred = fsm_.switch_occurred();
  output.switch_reason = fsm_.switch_reason();
  output.binding_confidence = fsm_.binding_confidence();

  // 7. Update contexts for next frame
  if (!input.obs_z_values.empty()) {
    decoder_ctx_.last_obs_z = input.obs_z_values[0];
  }
  decoder_ctx_.last_panel_id = output.bound_id;
  binder_ctx_.current_bound_id = output.bound_id;
  binder_ctx_.current_bound_label = output.height_label;

  // Maintain history windows
  if (!input.obs_z_values.empty()) {
    binder_ctx_.z_history.push_back(input.obs_z_values[0]);
    const int max_hist = 20;
    while (static_cast<int>(binder_ctx_.z_history.size()) > max_hist) {
      binder_ctx_.z_history.pop_front();
    }
  }
  if (input.has_z_jump) {
    binder_ctx_.z_jump_history.push_back(input.z_jump);
    const int max_jump = 20;
    while (static_cast<int>(binder_ctx_.z_jump_history.size()) > max_jump) {
      binder_ctx_.z_jump_history.pop_front();
    }
  }
  binder_ctx_.panel_id_history.push_back(output.bound_id);
  const int max_pid = 20;
  while (static_cast<int>(binder_ctx_.panel_id_history.size()) > max_pid) {
    binder_ctx_.panel_id_history.pop_front();
  }

  // 8. Populate debug
  debug_.valid = true;
  debug_.jump_detected = jump.detected;
  debug_.jump_kind = jump.jump_kind;
  debug_.jump_confidence = jump.confidence;
  debug_.from_id = jump.from_id;
  debug_.to_id = jump.to_id;
  debug_.target_id = target.target_id;
  debug_.target_confidence = target.confidence;
  debug_.fsm_state = output.fsm_state;
  debug_.action = output.action;
  debug_.switch_reason = output.switch_reason;
  debug_.health_score = health.score;
  debug_.consecutive_bad_frames = health.consecutive_bad_frames;
  debug_.force_rebind_flag = health.force_rebind_recommend;
  debug_.obs_count = input.obs_count;
  debug_.z_jump = input.z_jump;
  debug_.spin_direction = decoder_ctx_.spin_direction;
  debug_.candidate_prob = input.candidate_prob;
  debug_.candidate_margin = input.candidate_margin;
  debug_.dz_jump_est = decoder_ctx_.dz_jump_est;
  debug_.dz_small_est = decoder_ctx_.dz_small_est;
  debug_.dz_large_est = decoder_ctx_.dz_large_est;
  debug_.period_confidence = decoder_ctx_.period_confidence;
  debug_.period_phase = decoder_ctx_.period_phase;
  debug_.signature_score = jump.signature_score;
  debug_.event_type = input.event_type;
  debug_.is_reacquired = input.is_reacquired;
  debug_.gap_dt = input.gap_dt;
  debug_.lost_frames = input.lost_frames;

  return output;
}

}  // namespace fyt::auto_aim::binder
