// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/pipeline/serial_tracker_pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::pipeline {

namespace {

int clamp_panel(int pid) {
  int v = pid % 4;
  if (v < 0) v += 4;
  return v;
}

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

binder::HeightLabel to_height_label(int panel_id) {
  return (clamp_panel(panel_id) % 2 == 0) ? binder::HeightLabel::LOWER
                                           : binder::HeightLabel::UPPER;
}

}  // namespace

SerialTrackerPipeline::SerialTrackerPipeline(const UnifiedConfig &config,
                                             double dt)
    : config_(config),
      dt_(dt),
      obs_frontend_(config),
      binder_bridge_(config),
      ambiguous_backend_(config),
      structured_backend_(config, dt),
      output_adapter_(config),
      phase_memory_(config.norm4_v2.phase_memory),
      evidence_builder_([&config]() {
        evidence::EvidenceBuilderConfig ecfg;
        ecfg.enable_2d_tracker = config.norm4_v2.enable_2d_tracker;
        ecfg.enable_proxy_manager = config.norm4_v2.enable_proxy_manager;
        return ecfg;
      }(), config),
      backend_planner_(config),
      evidence_fuser_(mode::EvidenceFuserConfig{
          /*w_dual=*/0.35,
          /*w_margin=*/0.25,
          /*w_health=*/0.20,
          /*w_entropy=*/0.20,
          /*jump_event_weight=*/0.30}),
      mode_fsm_(mode::ModeFSMConfig{
          config.outpost.mode_enter_confirm_frames,
          config.outpost.mode_exit_confirm_frames,
          config.outpost.mode_min_dwell_frames,
          config.outpost.mode_enter_threshold,
          config.outpost.mode_exit_threshold,
          config.outpost.entropy_enter,
          config.outpost.entropy_exit,
          config.outpost.max_prob_enter,
          config.outpost.max_prob_exit,
          config.outpost.stable_frames}),
      mismatch_detector_(config.panel_mismatch.window_size,
                         config.panel_mismatch.threshold_t1,
                         config.panel_mismatch.confirm_count,
                         config.panel_mismatch.reinit_count,
                         config.panel_mismatch.enable) {}

void SerialTrackerPipeline::initialize(
    const std::vector<ObservationData> &obs,
    norm4_v2::Norm4RuntimeContext *ctx) {
  if (!ctx || obs.empty()) return;

  obs_frontend_.reset_history();
  height_identifier_.reset();
  mismatch_detector_.reset();
  phase_memory_.reset();

  const ObservationData *selected =
      obs_frontend_.select_primary_observation(obs, *ctx);
  if (!selected) return;

  const int init_panel = selected->panel_id.has_value()
                             ? clamp_panel(selected->panel_id.value())
                             : obs_frontend_.infer_panel_for_observation(
                                   *selected, *ctx, nullptr);
  const auto init_label = to_height_label(init_panel);

  ambiguous_backend_.reset(*selected, init_panel, default_r1_, default_r2_,
                           default_dza_);
  structured_backend_.reset(*selected, init_panel, default_r1_, default_r2_,
                            default_dza_);
  binder_bridge_.reset(init_panel, init_label, selected->z);
  mode_fsm_.reset(mode::TrackMode::AMBIGUOUS);

  ctx->mode = mode::TrackMode::AMBIGUOUS;
  ctx->selected_panel_id = init_panel;
  ctx->bound_panel_id = init_panel;
  ctx->bound_height_label = init_label;
  ctx->binding_confidence = 0.5;
  ctx->entropy_norm = 1.0;
  ctx->max_prob = 0.25;
  ctx->r1 = default_r1_;
  ctx->r2 = default_r2_;
  ctx->dza = default_dza_;
  ctx->last_obs_z = selected->z;
  if (selected->timestamp.has_value()) {
    ctx->last_timestamp = selected->timestamp.value();
  }

  const auto snap = ambiguous_backend_.snapshot();
  ctx->center_pos = snap.center_pos;
  ctx->center_vel = snap.center_vel;
  ctx->center_yaw = snap.center_yaw;
  ctx->yaw_rate = snap.yaw_rate;
  ctx->r1 = snap.r1;
  ctx->r2 = snap.r2;
  ctx->dza = snap.dza;

  norm4_v2::PublishStateInput input;
  input.mode = mode::TrackMode::AMBIGUOUS;
  input.backend_snap = &snap;
  input.armor_snap = &ambiguous_backend_.ambiguous_snapshot();
  output_adapter_.update_publish_state(ctx, input);
}

void SerialTrackerPipeline::predict(double dt,
                                    norm4_v2::Norm4RuntimeContext *ctx) {
  if (!ctx) return;
  ambiguous_backend_.predict(dt);
  structured_backend_.predict(dt);

  const auto snap = (ctx->mode == mode::TrackMode::STRUCTURED)
                        ? structured_backend_.snapshot()
                        : ambiguous_backend_.snapshot();
  ctx->center_pos = snap.center_pos;
  ctx->center_vel = snap.center_vel;
  ctx->center_yaw = snap.center_yaw;
  ctx->yaw_rate = snap.yaw_rate;
  ctx->r1 = snap.r1;
  ctx->r2 = snap.r2;
  ctx->dza = snap.dza;

  norm4_v2::PublishStateInput input;
  input.mode = ctx->mode;
  input.backend_snap = &snap;
  if (ctx->mode == mode::TrackMode::AMBIGUOUS) {
    input.armor_snap = &ambiguous_backend_.ambiguous_snapshot();
  }
  output_adapter_.update_publish_state(ctx, input);
}

void SerialTrackerPipeline::build_backend_intent(
    const ObservationData &selected,
    const norm4_v2::BindingCandidate &candidate,
    const binder::BinderOutput &binder_out,
    const norm4_v2::Norm4RuntimeContext &ctx,
    int /*obs_count*/, BackendIntent *intent) {
  if (!intent) return;

  int sp = (binder_out.selected_id >= 0) ? binder_out.selected_id
                                          : candidate.candidate_panel_id;
  if (binder_out.fsm_state == binder::BindingFSMState::PENDING_SWITCH &&
      binder_out.pending_id >= 0) {
    sp = binder_out.pending_id;
  }
  if (sp < 0) sp = ctx.selected_panel_id;
  sp = clamp_panel(sp);

  binder::HeightLabel sl = binder_out.height_label;
  if (sl == binder::HeightLabel::UNKNOWN) sl = candidate.candidate_height_label;
  if (sl == binder::HeightLabel::UNKNOWN) sl = to_height_label(sp);

  intent->target_panel_id = sp;
  intent->height_label = sl;
  intent->mode = ctx.mode;
  intent->r1 = ctx.r1;
  intent->r2 = ctx.r2;
  intent->dza = ctx.dza;
  intent->height_confidence =
      std::clamp(std::max(candidate.height_confidence,
                          binder_out.binding_confidence),
                 0.0, 1.0);
  intent->position_confidence =
      std::max(0.05, binder_out.binding_confidence);
  if (binder_out.binding_conflict_for_update) {
    intent->position_confidence =
        std::clamp(intent->position_confidence * 0.35, 0.05, 1.0);
  }
  intent->enforce_panel_constraint = true;
  intent->obs = &selected;
  intent->has_dual = false;
  intent->obs1 = &selected;
  intent->obs2 = nullptr;
}

SerialPipelineOutput SerialTrackerPipeline::step(
    const std::vector<ObservationData> &obs,
    norm4_v2::Norm4RuntimeContext *ctx) {
  SerialPipelineOutput output;
  last_trace_.reset();
  last_trace_.timestamp =
      obs.empty() ? 0.0
                  : obs[0].timestamp.value_or(ctx ? ctx->last_timestamp.value_or(0.0) : 0.0);
  last_trace_.valid = true;

  if (!ctx || obs.empty()) {
    output.ok = false;
    return output;
  }

  // ── Stage 1: EvidenceBuilder ──
  auto &stg1 = last_trace_.add_stage("EvidenceBuilder");
  if (config_.norm4_v2.enable_common_pipeline) {
    ctx->evidence_frame = evidence_builder_.build(
        obs, obs[0].timestamp.value_or(ctx->last_timestamp.value_or(0.0)));
    stg1.active = true;
    stg1.add("completeness", ctx->evidence_frame.completeness.fraction());
  }

  // ── Stage 2: Observation frontend ──
  auto &stg2 = last_trace_.add_stage("ObservationFrontend");
  const ObservationData *selected =
      obs_frontend_.select_primary_observation(obs, *ctx);
  if (!selected) {
    stg2.ok = false;
    stg2.error = "no_primary_obs";
    output.ok = false;
    return output;
  }
  stg2.active = true;

  // ── Stage 3: Build binding candidate ──
  auto &stg3 = last_trace_.add_stage("BindCandidate");
  auto association_ctx = *ctx;
  if (ctx->mode == mode::TrackMode::AMBIGUOUS) {
    const auto amb_snap = ambiguous_backend_.snapshot();
    association_ctx.center_pos = amb_snap.center_pos;
    association_ctx.center_vel = amb_snap.center_vel;
    association_ctx.center_yaw = amb_snap.center_yaw;
    association_ctx.yaw_rate = amb_snap.yaw_rate;
    association_ctx.reference_center_yaw = amb_snap.center_yaw;
  }

  norm4_v2::DualObservationAssignment dual_assignment;
  int selected_obs_index = 0;
  for (size_t i = 0; i < obs.size(); ++i) {
    if (&obs[i] == selected) {
      selected_obs_index = static_cast<int>(i);
      break;
    }
  }

  if (obs.size() >= 2) {
    dual_assignment =
        obs_frontend_.assign_dual_observations(obs[0], obs[1], association_ctx);
    if (dual_assignment.valid) {
      if (ctx->bound_panel_id == dual_assignment.panel_id_1) {
        selected_obs_index = 0;
      } else if (ctx->bound_panel_id == dual_assignment.panel_id_2) {
        selected_obs_index = 1;
      } else if (selected == &obs[1]) {
        selected_obs_index = 1;
      } else if (selected_obs_index > 1) {
        selected_obs_index = 0;
      }
      selected = &obs[selected_obs_index];
    }
  }

  auto candidate =
      obs_frontend_.build_binding_candidate(*selected, association_ctx);
  if (dual_assignment.valid) {
    candidate.obs_panel_ids = {dual_assignment.panel_id_1,
                               dual_assignment.panel_id_2};
    candidate.obs_height_labels = {dual_assignment.label_1,
                                   dual_assignment.label_2};

    const int forced_panel = selected_obs_index == 0
                                 ? dual_assignment.panel_id_1
                                 : dual_assignment.panel_id_2;
    const auto forced_label = selected_obs_index == 0
                                  ? dual_assignment.label_1
                                  : dual_assignment.label_2;
    obs_frontend_.apply_forced_assignment(&candidate, *selected, association_ctx,
                                          forced_panel, forced_label);
    candidate.height_confidence =
        std::max(candidate.height_confidence,
                 dual_assignment.height_confidence);
  }
  stg3.active = true;
  stg3.add("candidate_panel", candidate.candidate_panel_id);
  stg3.add("candidate_prob", candidate.candidate_prob);
  stg3.add("candidate_margin", candidate.candidate_margin);

  // ── Stage 4: Binder bridge ──
  auto &stg4 = last_trace_.add_stage("BinderBridge");
  auto binder_out =
      binder_bridge_.step(*selected, obs, static_cast<int>(obs.size()),
                          candidate, *ctx);
  stg4.active = true;
  stg4.add("bound_id", binder_out.bound_id);
  stg4.add("binding_confidence", binder_out.binding_confidence);

  // ── Stage 5: Build backend intent (resolve panel / height) ──
  auto &stg5 = last_trace_.add_stage("BuildIntent");
  BackendIntent intent;
  build_backend_intent(*selected, candidate, binder_out, *ctx,
                       static_cast<int>(obs.size()), &intent);
  if (dual_assignment.valid) {
    intent.has_dual = true;
    intent.obs1 = &obs[0];
    intent.obs2 = &obs[1];
    intent.dual_panel_id_1 = dual_assignment.panel_id_1;
    intent.dual_panel_id_2 = dual_assignment.panel_id_2;
    intent.dual_layer_1 = dual_assignment.layer_1;
    intent.dual_layer_2 = dual_assignment.layer_2;
    intent.dual_height_confidence = dual_assignment.height_confidence;
  }
  stg5.active = true;
  stg5.add("target_panel", intent.target_panel_id);

  // ── Stage 6: Ping-pong assessment ──
  auto &stg6 = last_trace_.add_stage("PingPongCheck");
  if (config_.norm4_v2.enable_phase_memory) {
    const KinematicSummary *kin_summary = nullptr;
    std::optional<int> selected_track2d_id;
    if (config_.norm4_v2.enable_common_pipeline &&
        selected_obs_index >= 0 &&
        selected_obs_index < static_cast<int>(ctx->evidence_frame.observations.size())) {
      selected_track2d_id =
          ctx->evidence_frame.observations[selected_obs_index].track2d_id;
    }
    if (selected_track2d_id.has_value()) {
      for (const auto &pe : ctx->evidence_frame.proxy_evidence) {
        if (pe.valid && pe.track2d_id == selected_track2d_id.value()) {
          kin_summary = &pe.kin_summary;
          break;
        }
      }
    }
    if (kin_summary == nullptr) {
      for (const auto &pe : ctx->evidence_frame.proxy_evidence) {
        if (pe.valid) {
          kin_summary = &pe.kin_summary;
          break;
        }
      }
    }
    auto risk = phase_memory_.assess(
        intent.target_panel_id, intent.height_confidence,
        selected->timestamp.value_or(ctx->last_timestamp.value_or(0.0)),
        kin_summary);
    if (risk.should_hold && ctx->bound_panel_id >= 0) {
      intent.target_panel_id = ctx->bound_panel_id;
      if (ctx->bound_height_label != binder::HeightLabel::UNKNOWN) {
        intent.height_label = ctx->bound_height_label;
      }
    }
    ctx->ping_pong_risk_score = risk.risk_score;
    ctx->ping_pong_pending = risk.pending;
    ctx->ping_pong_should_hold = risk.should_hold;
    ctx->ping_pong_reason = static_cast<int>(risk.reason);
    ctx->ping_pong_hold_counter = phase_memory_.hold_counter();
    ctx->ping_pong_consistent_counter = phase_memory_.consistent_counter();
    stg6.active = true;
    stg6.add("risk", risk.risk_score);
    stg6.add("hold", risk.should_hold ? 1.0 : 0.0);
  }

  // ── Stage 7: Mode FSM ──
  auto &stg7 = last_trace_.add_stage("ModeFSM");
  const auto &binder_dbg = binder_bridge_.debug_snapshot();
  const bool has_2dz =
      binder_dbg.jump_detected &&
      binder_dbg.jump_kind == binder::JumpKind::DZ &&
      binder_dbg.jump_confidence >= 0.60;
  mode::ModeEvidence evidence = evidence_fuser_.fuse(
      selected->timestamp.value_or(ctx->last_timestamp.value_or(0.0)),
      static_cast<int>(obs.size()), candidate.candidate_panel_id,
      has_2dz, candidate.entropy_norm, candidate.max_prob,
      candidate.candidate_margin, binder_dbg);
  auto mode_decision = mode_fsm_.step(evidence);
  stg7.active = true;
  stg7.add("mode", mode_decision.mode == mode::TrackMode::STRUCTURED ? 0.0 : 1.0);
  stg7.add("switched", mode_decision.switched ? 1.0 : 0.0);

  if (mode_decision.switched) {
    ctx->mode = mode_decision.mode;
    intent.mode = mode_decision.mode;
    if (ctx->mode == mode::TrackMode::STRUCTURED) {
      structured_backend_.reset(*selected, intent.target_panel_id,
                                ctx->r1, ctx->r2, ctx->dza);
    } else {
      ambiguous_backend_.reset(*selected, intent.target_panel_id,
                               ctx->r1, ctx->r2, ctx->dza);
    }
    intent.force_reinit = false;
  }

  // ── Stage 8: Backend Planner ──
  auto &stg8 = last_trace_.add_stage("BackendPlanner");
  auto plan = backend_planner_.plan(intent, *ctx);
  output.backend_plan = plan;
  stg8.active = true;
  stg8.add("num_steps", plan.size());

  // ── Stage 9: Backend Executor ──
  auto &stg9 = last_trace_.add_stage("BackendExecutor");
  BackendExecutor executor(config_, intent.obs, intent.obs2);
  auto exec_result = executor.execute(plan, ctx, &ambiguous_backend_,
                                      &structured_backend_,
                                      &output_adapter_, &mismatch_detector_);
  stg9.active = true;
  stg9.ok = exec_result.ok;
  stg9.add("ambiguous_updated", exec_result.ambiguous_updated ? 1.0 : 0.0);
  stg9.add("structured_updated", exec_result.structured_updated ? 1.0 : 0.0);

  // ── Stage 10: Finalise runtime context ──
  ctx->bound_panel_id = (binder_out.bound_id >= 0)
                            ? clamp_panel(binder_out.bound_id)
                            : ctx->selected_panel_id;
  ctx->bound_height_label =
      (binder_out.height_label != binder::HeightLabel::UNKNOWN)
          ? binder_out.height_label
          : intent.height_label;
  ctx->binding_confidence = binder_out.binding_confidence;
  ctx->entropy_norm = candidate.entropy_norm;
  ctx->max_prob = candidate.max_prob;
  ctx->last_obs_z = selected->z;
  if (selected->timestamp.has_value()) {
    ctx->last_timestamp = selected->timestamp.value();
  }

  output.intent = intent;
  output.ok = exec_result.ok;
  output.trace = last_trace_;
  return output;
}

}  // namespace fyt::auto_aim::pipeline
