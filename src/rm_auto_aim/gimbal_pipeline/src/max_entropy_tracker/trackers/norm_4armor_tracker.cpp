// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm_4armor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <iostream>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

namespace {

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

}  // namespace

Norm4ArmorTracker::Norm4ArmorTracker(const UnifiedConfig &config, double dt,
                                     bool enable_oscillation)
    : BaseTracker(dt),
      config_(config),
      obs_frontend_(config),
      binder_bridge_(config),
      evidence_fuser_([]() {
        mode::EvidenceFuserConfig cfg;
        cfg.w_dual = 0.35;
        cfg.w_margin = 0.25;
        cfg.w_health = 0.20;
        cfg.w_entropy = 0.20;
        cfg.jump_event_weight = 0.30;
        return cfg;
      }()),
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
      ambiguous_backend_(config),
      structured_backend_(config, dt),
      output_adapter_(config),
      mismatch_detector_(config.panel_mismatch.window_size,
                         config.panel_mismatch.threshold_t1,
                         config.panel_mismatch.confirm_count,
                         config.panel_mismatch.reinit_count,
                         config.panel_mismatch.enable),
      maneuver_detector_(config.maneuver),
      phase_memory_(config.norm4_v2.phase_memory),
      evidence_builder_([&config]() {
        evidence::EvidenceBuilderConfig ecfg;
        ecfg.enable_2d_tracker = config.norm4_v2.enable_2d_tracker;
        ecfg.enable_proxy_manager = config.norm4_v2.enable_proxy_manager;
        ecfg.iou_2d = IoU2DTrackerConfig{};
        ecfg.proxy = SingleProxyManagerConfig{};
        return ecfg;
      }(), config) {
  (void)enable_oscillation;

  // Phase 7: construct serial pipeline when common pipeline is requested.
  if (config.norm4_v2.enable_common_pipeline) {
    serial_pipeline_ =
        std::make_unique<pipeline::SerialTrackerPipeline>(config, dt);
  }
}

int Norm4ArmorTracker::clamp_panel(int panel_id) {
  int v = panel_id % 4;
  if (v < 0) v += 4;
  return v;
}

binder::HeightLabel Norm4ArmorTracker::default_height_label_for_panel(int panel_id) {
  const int p = clamp_panel(panel_id);
  return (p % 2 == 0) ? binder::HeightLabel::LOWER : binder::HeightLabel::UPPER;
}

std::string Norm4ArmorTracker::label_to_layer(binder::HeightLabel label,
                                              int panel_id) {
  if (label == binder::HeightLabel::UPPER) return "upper";
  if (label == binder::HeightLabel::LOWER) return "lower";
  return (clamp_panel(panel_id) % 2 == 0) ? "lower" : "upper";
}

void Norm4ArmorTracker::sync_runtime_from_backend(
    const norm4_v2::BackendStateSnapshot &snap) {
  ctx_.center_pos = snap.center_pos;
  ctx_.center_vel = snap.center_vel;
  ctx_.center_yaw = snap.center_yaw;
  ctx_.yaw_rate = snap.yaw_rate;
  ctx_.r1 = snap.r1;
  ctx_.r2 = snap.r2;
  ctx_.dza = snap.dza;
  ctx_.dza_converged = snap.dza_converged;
  ctx_.reference_center_yaw = snap.center_yaw;
}

void Norm4ArmorTracker::initialize(const std::vector<ObservationData> &obs, double r1,
                                   double r2, double dza) {
  if (obs.empty()) {
    throw std::invalid_argument("Norm4ArmorTracker requires one observation");
  }

  default_r1_ = std::max(0.05, r1);
  default_r2_ = std::max(0.05, r2);
  default_dza_ = std::max(0.0, dza);

  // Phase 7: delegate to serial pipeline.
  if (serial_pipeline_) {
    ctx_ = norm4_v2::Norm4RuntimeContext{};
    serial_pipeline_->initialize(obs, &ctx_);

    if (obs[0].timestamp.has_value()) {
      current_time_ = obs[0].timestamp.value();
      last_update_time_ = obs[0].timestamp.value();
    }
    mark_initialized();
    transition_to(TrackerState::INITIALIZING);
    increment_frame();

    norm4_v2::BindingCandidate candidate;
    binder::BinderOutput binder_out;
    mode::ModeDecision mode_decision;
    refresh_debug(&obs[0], candidate, binder_out,
                  binder_bridge_.debug_snapshot(),
                  mode_decision, 0.0);
    return;
  }

  obs_frontend_.reset_history();
  height_identifier_.reset();
  mismatch_detector_.reset();
  phase_memory_.reset();
  single_obs_streak_ = 0;
  degraded_single_obs_mode_ = false;

  default_r1_ = std::max(0.05, r1);
  default_r2_ = std::max(0.05, r2);
  default_dza_ = std::max(0.0, dza);

  ctx_ = norm4_v2::Norm4RuntimeContext{};
  const ObservationData *selected =
      obs_frontend_.select_primary_observation(obs, ctx_);
  if (selected == nullptr) {
    throw std::invalid_argument("Norm4ArmorTracker cannot select observation");
  }

  PanelAssociator::AssociationDiagnostics init_diag;
  const int init_panel = selected->panel_id.has_value()
                             ? clamp_panel(selected->panel_id.value())
                             : obs_frontend_.infer_panel_for_observation(
                                   *selected, ctx_, &init_diag);
  const auto init_label = default_height_label_for_panel(init_panel);

  ambiguous_backend_.reset(*selected, init_panel, default_r1_, default_r2_, default_dza_);
  structured_backend_.reset(*selected, init_panel, default_r1_, default_r2_, default_dza_);
  binder_bridge_.reset(init_panel, init_label, selected->z);
  mode_fsm_.reset(mode::TrackMode::AMBIGUOUS);

  ctx_.mode = mode::TrackMode::AMBIGUOUS;
  ctx_.selected_panel_id = init_panel;
  ctx_.bound_panel_id = init_panel;
  ctx_.bound_height_label = init_label;
  ctx_.binding_confidence = 0.5;
  ctx_.entropy_norm = 1.0;
  ctx_.max_prob = 0.25;
  ctx_.r1 = default_r1_;
  ctx_.r2 = default_r2_;
  ctx_.dza = default_dza_;
  ctx_.last_obs_z = selected->z;
  if (selected->timestamp.has_value()) {
    ctx_.last_timestamp = selected->timestamp.value();
    current_time_ = selected->timestamp.value();
    last_update_time_ = selected->timestamp.value();
  }

  sync_runtime_from_backend(ambiguous_backend_.snapshot());
  {
    norm4_v2::PublishStateInput input;
    const auto snap = ambiguous_backend_.snapshot();
    input.mode = mode::TrackMode::AMBIGUOUS;
    input.backend_snap = &snap;
    input.armor_snap = &ambiguous_backend_.ambiguous_snapshot();
    output_adapter_.update_publish_state(&ctx_, input);
  }

  mark_initialized();
  transition_to(TrackerState::INITIALIZING);
  increment_frame();

  norm4_v2::BindingCandidate candidate;
  binder::BinderOutput binder_out;
  mode::ModeDecision mode_decision;
  refresh_debug(selected, candidate, binder_out, binder_bridge_.debug_snapshot(),
                mode_decision, 0.0);
}

void Norm4ArmorTracker::predict(std::optional<double> target_time) {
  if (!is_initialized()) return;

  const double dt = compute_dt(target_time);

  // Phase 7: delegate to serial pipeline.
  if (serial_pipeline_) {
    serial_pipeline_->predict(dt, &ctx_);
    if (target_time.has_value()) {
      current_time_ = target_time.value();
      ctx_.last_timestamp = target_time.value();
    } else if (current_time_.has_value()) {
      current_time_ = current_time_.value() + dt;
      ctx_.last_timestamp = current_time_.value();
    }
    return;
  }
  ambiguous_backend_.predict(dt);
  structured_backend_.predict(dt);

  if (target_time.has_value()) {
    current_time_ = target_time.value();
    ctx_.last_timestamp = target_time.value();
  } else if (current_time_.has_value()) {
    current_time_ = current_time_.value() + dt;
    ctx_.last_timestamp = current_time_.value();
  }

  const auto snap = (ctx_.mode == mode::TrackMode::STRUCTURED)
                        ? structured_backend_.snapshot()
                        : ambiguous_backend_.snapshot();
  sync_runtime_from_backend(snap);

  norm4_v2::PublishStateInput input;
  input.mode = ctx_.mode;
  input.backend_snap = &snap;
  if (ctx_.mode == mode::TrackMode::AMBIGUOUS) {
    input.armor_snap = &ambiguous_backend_.ambiguous_snapshot();
  }
  output_adapter_.update_publish_state(&ctx_, input);
}

void Norm4ArmorTracker::update_degraded_single_obs_mode(bool is_single_obs) {
  if (!config_.tracker.degraded_single_obs_enable) {
    single_obs_streak_ = 0;
    degraded_single_obs_mode_ = false;
    structured_backend_.ukf().set_structural_noise_scales(1.0, 1.0);
    return;
  }

  if (is_single_obs) {
    ++single_obs_streak_;
  } else {
    single_obs_streak_ = 0;
  }

  const bool dza_not_converged = !structured_backend_.ukf().is_dza_converged();
  const int streak_thres = std::max(1, config_.tracker.degraded_single_obs_streak);
  degraded_single_obs_mode_ =
      is_single_obs && dza_not_converged && single_obs_streak_ >= streak_thres;

  if (degraded_single_obs_mode_) {
    structured_backend_.ukf().set_structural_noise_scales(
        config_.tracker.degraded_q_scale_r, config_.tracker.degraded_q_scale_dza);
  } else {
    structured_backend_.ukf().set_structural_noise_scales(1.0, 1.0);
  }
}

int Norm4ArmorTracker::apply_panel_mismatch_if_needed(
    const ObservationData &obs, int selected_panel, binder::HeightLabel selected_label) {
  if (!structured_backend_.initialized()) return selected_panel;

  auto &ukf = structured_backend_.ukf();
  const auto idx = ukf.state_idx();
  const auto result = mismatch_detector_.update(
      selected_panel, obs.z, ukf.x()(idx.Z()), ukf.x()(idx.DZA()),
      label_to_layer(selected_label, selected_panel), ukf.is_dza_converged(),
      ukf.last_z_innovation());

  if (result.action == PanelMismatchDetector::Action::NONE ||
      !config_.panel_mismatch.apply_correction) {
    return selected_panel;
  }

  const int corrected_panel = clamp_panel(result.new_panel_id);
  if (result.action == PanelMismatchDetector::Action::REINIT) {
    structured_backend_.reset(obs, corrected_panel, default_r1_, default_r2_,
                              default_dza_);
    height_identifier_.reset();
    mismatch_detector_.reset();
    return corrected_panel;
  }

  structured_backend_.apply_panel_correction(corrected_panel, obs.yaw);
  const HeightLabel hint_label = (corrected_panel % 2 == 0) ? HeightLabel::LOWER
                                                             : HeightLabel::UPPER;
  height_identifier_.reset_with_hint(hint_label);
  mismatch_detector_.reset();
  return corrected_panel;
}

bool Norm4ArmorTracker::update(const std::vector<ObservationData> &obs) {
  norm4_v2::BindingCandidate candidate;
  binder::BinderOutput binder_out;
  mode::ModeDecision mode_decision;

  if (!is_initialized() || obs.empty()) {
    handle_observation_loss(config_.tracker.tracking_thres, config_.tracker.lost_thres);
    refresh_debug(nullptr, candidate, binder_out, binder_bridge_.debug_snapshot(),
                  mode_decision, 0.0);
    return false;
  }

  // Phase 7: common pipeline fast path.
  if (serial_pipeline_) {
    handle_observation_received(config_.tracker.tracking_thres);

    // Predict to current time.
    double obs_ts = obs[0].timestamp.value_or(current_time_.value_or(0.0));
    if (current_time_.has_value() && obs_ts > current_time_.value()) {
      serial_pipeline_->predict(obs_ts - current_time_.value(), &ctx_);
    }

    auto result = serial_pipeline_->step(obs, &ctx_);

    // Update tracker state.
    if (result.ok) {
      if (obs[0].timestamp.has_value()) {
        update_time(obs[0].timestamp.value());
      }
      increment_frame();
    } else {
      handle_observation_loss(config_.tracker.tracking_thres, config_.tracker.lost_thres);
    }

    // Sync the degraded single obs mode.
    update_degraded_single_obs_mode(obs.size() == 1);

    // Reuse the existing debug refresh for compatibility.
    const ObservationData *sel =
        result.intent.obs ? result.intent.obs : &obs[0];
    const auto &serial_binder_dbg =
        serial_pipeline_->binder_bridge().debug_snapshot();
    refresh_debug(sel, candidate, binder_out,
                  serial_binder_dbg,
                  mode_decision, result.intent.height_confidence);
    return result.ok;
  }

  const ObservationData *selected =
      obs_frontend_.select_primary_observation(obs, ctx_);
  if (selected == nullptr) {
    handle_observation_loss(config_.tracker.tracking_thres, config_.tracker.lost_thres);
    refresh_debug(nullptr, candidate, binder_out, binder_bridge_.debug_snapshot(),
                  mode_decision, 0.0);
    return false;
  }

  if (selected->timestamp.has_value() && current_time_.has_value()) {
    const double d = selected->timestamp.value() - current_time_.value();
    if (d > min_dt_) {
      predict(selected->timestamp.value());
    }
  }

  update_degraded_single_obs_mode(obs.size() == 1);

  ctx_.lost_frames = lost_count();
  handle_observation_received(config_.tracker.tracking_thres);

  // Phase 5: build unified evidence frame (idempotent; stages run only when enabled).
  if (config_.norm4_v2.enable_common_pipeline) {
    ctx_.evidence_frame = evidence_builder_.build(
        obs, selected->timestamp.value_or(ctx_.last_timestamp.value_or(0.0)));

    // Debug: print evidence frame.
    std::cout << "[enable_common_pipeline] Evidence Frame at t=" << ctx_.evidence_frame.timestamp
              << " with " << ctx_.evidence_frame.observations.size()
              << " observations and " << ctx_.evidence_frame.proxy_evidence.size()
              << " proxy evidence entries." << std::endl;

  }

  auto association_ctx = ctx_;
  if (ctx_.mode == mode::TrackMode::AMBIGUOUS) {
    const auto amb_center_snap = ambiguous_backend_.snapshot();
    association_ctx.center_pos = amb_center_snap.center_pos;
    association_ctx.center_vel = amb_center_snap.center_vel;
    association_ctx.center_yaw = amb_center_snap.center_yaw;
    association_ctx.yaw_rate = amb_center_snap.yaw_rate;
    association_ctx.reference_center_yaw = amb_center_snap.center_yaw;
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
      if (ctx_.bound_panel_id == dual_assignment.panel_id_1) {
        selected_obs_index = 0;
      } else if (ctx_.bound_panel_id == dual_assignment.panel_id_2) {
        selected_obs_index = 1;
      } else if (selected == &obs[1]) {
        selected_obs_index = 1;
      } else if (selected_obs_index > 1) {
        selected_obs_index = 0;
      }
      selected = &obs[selected_obs_index];
    }
  }

  candidate = obs_frontend_.build_binding_candidate(*selected, association_ctx);
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

  binder_out = binder_bridge_.step(*selected, obs, static_cast<int>(obs.size()),
                                   candidate, ctx_);
  const auto &binder_dbg = binder_bridge_.debug_snapshot();

  const bool has_2dz_signature =
      binder_dbg.jump_detected &&
      binder_dbg.jump_kind == binder::JumpKind::DZ &&
      binder_dbg.jump_confidence >= 0.60;
  mode::ModeEvidence evidence = evidence_fuser_.fuse(
      selected->timestamp.value_or(ctx_.last_timestamp.value_or(0.0)),
      static_cast<int>(obs.size()), candidate.candidate_panel_id, has_2dz_signature,
      candidate.entropy_norm, candidate.max_prob, candidate.candidate_margin,
      binder_dbg);
  mode_decision = mode_fsm_.step(evidence);

  int selected_panel = (binder_out.selected_id >= 0)
                           ? binder_out.selected_id
                           : candidate.candidate_panel_id;
  if (binder_out.fsm_state == binder::BindingFSMState::PENDING_SWITCH &&
      binder_out.pending_id >= 0) {
    selected_panel = binder_out.pending_id;
  }
  if (selected_panel < 0) {
    selected_panel = ctx_.selected_panel_id;
  }
  selected_panel = clamp_panel(selected_panel);

  binder::HeightLabel selected_label = binder_out.height_label;
  if (selected_label == binder::HeightLabel::UNKNOWN) {
    selected_label = candidate.candidate_height_label;
  }
  if (selected_label == binder::HeightLabel::UNKNOWN) {
    selected_label = default_height_label_for_panel(selected_panel);
  }
  double height_confidence =
      std::clamp(std::max(candidate.height_confidence, binder_out.binding_confidence),
                 0.0, 1.0);

  // Phase 4: ping-pong suppression.
  norm4_v2::PingPongRisk ping_pong_risk;
  if (config_.norm4_v2.enable_phase_memory) {
    // Resolve kin_summary from proxy evidence when available (Phase 5 wire).
    const KinematicSummary *kin_summary = nullptr;
    if (config_.norm4_v2.enable_common_pipeline) {
      std::optional<int> selected_track2d_id;
      if (selected_obs_index >= 0 &&
          selected_obs_index <
              static_cast<int>(ctx_.evidence_frame.observations.size())) {
        selected_track2d_id =
            ctx_.evidence_frame.observations[selected_obs_index].track2d_id;
      }
      if (selected_track2d_id.has_value()) {
        for (const auto &pe : ctx_.evidence_frame.proxy_evidence) {
          if (pe.valid && pe.track2d_id == selected_track2d_id.value()) {
            kin_summary = &pe.kin_summary;
            break;
          }
        }
      }
      if (kin_summary == nullptr) {
        for (const auto &pe : ctx_.evidence_frame.proxy_evidence) {
          if (pe.valid && pe.track2d_id >= 0) {
            kin_summary = &pe.kin_summary;
            break;
          }
        }
      }
    }
    ping_pong_risk = phase_memory_.assess(
        selected_panel, height_confidence,
        selected->timestamp.value_or(ctx_.last_timestamp.value_or(0.0)),
        kin_summary);
    if (ping_pong_risk.should_hold && ctx_.bound_panel_id >= 0) {
      selected_panel = ctx_.bound_panel_id;
      if (ctx_.bound_height_label != binder::HeightLabel::UNKNOWN) {
        selected_label = ctx_.bound_height_label;
      }
    }
    ctx_.ping_pong_risk_score = ping_pong_risk.risk_score;
    ctx_.ping_pong_pending = ping_pong_risk.pending;
    ctx_.ping_pong_should_hold = ping_pong_risk.should_hold;
    ctx_.ping_pong_reason = static_cast<int>(ping_pong_risk.reason);
    ctx_.ping_pong_hold_counter = phase_memory_.hold_counter();
    ctx_.ping_pong_consistent_counter = phase_memory_.consistent_counter();
  }

  if (mode_decision.switched) {
    ctx_.mode = mode_decision.mode;
    if (ctx_.mode == mode::TrackMode::STRUCTURED) {
      structured_backend_.reset(*selected, selected_panel, ctx_.r1, ctx_.r2, ctx_.dza);
    } else {
      ambiguous_backend_.reset(*selected, selected_panel, ctx_.r1, ctx_.r2, ctx_.dza);
    }
  }

  norm4_v2::BackendUpdateHint hint;
  hint.panel_id = selected_panel;
  hint.height_label = selected_label;
  hint.height_confidence = height_confidence;
  hint.position_confidence = std::max(0.05, binder_out.binding_confidence);
  if (binder_out.binding_conflict_for_update) {
    hint.position_confidence =
        std::clamp(hint.position_confidence * 0.35, 0.05, 1.0);
  }
  hint.r1_hint = ctx_.r1;
  hint.r2_hint = ctx_.r2;
  hint.dza_hint = ctx_.dza;
  hint.enforce_panel_constraint = true;

  bool ok = false;
  if (ctx_.mode == mode::TrackMode::STRUCTURED) {
    ok = structured_backend_.update(*selected, hint);
    if (ok && dual_assignment.valid) {
      structured_backend_.update_dual(
          obs[0], obs[1],
          dual_assignment.panel_id_1, dual_assignment.panel_id_2,
          dual_assignment.layer_1, dual_assignment.layer_2,
          dual_assignment.height_confidence);
      height_confidence =
          std::max(height_confidence, dual_assignment.height_confidence);
    }

    norm4_v2::BackendUpdateHint shadow = hint;
    shadow.position_confidence =
        std::clamp(0.25 + 0.25 * hint.position_confidence, 0.05, 0.60);
    ambiguous_backend_.update(*selected, shadow);
  } else {
    ok = ambiguous_backend_.update(*selected, hint);

    norm4_v2::BackendUpdateHint shadow = hint;
    shadow.position_confidence =
        std::clamp(0.25 + 0.25 * hint.position_confidence, 0.05, 0.60);
    structured_backend_.update(*selected, shadow);
    if (dual_assignment.valid) {
      structured_backend_.update_dual(
          obs[0], obs[1],
          dual_assignment.panel_id_1, dual_assignment.panel_id_2,
          dual_assignment.layer_1, dual_assignment.layer_2,
          dual_assignment.height_confidence);
    }
  }

  if (!ok) {
    refresh_debug(selected, candidate, binder_out, binder_dbg, mode_decision,
                  height_confidence);
    return false;
  }

  if (ctx_.mode == mode::TrackMode::STRUCTURED && obs.size() == 1) {
    selected_panel = apply_panel_mismatch_if_needed(*selected, selected_panel, selected_label);
  }

  const auto active_snap = (ctx_.mode == mode::TrackMode::STRUCTURED)
                               ? structured_backend_.snapshot()
                               : ambiguous_backend_.snapshot();
  sync_runtime_from_backend(active_snap);

  ctx_.selected_panel_id = (active_snap.panel_id >= 0) ? active_snap.panel_id : selected_panel;
  if (binder_out.bound_id >= 0) {
    ctx_.bound_panel_id = clamp_panel(binder_out.bound_id);
  } else {
    ctx_.bound_panel_id = ctx_.selected_panel_id;
  }
  ctx_.bound_height_label =
      (binder_out.height_label != binder::HeightLabel::UNKNOWN)
          ? binder_out.height_label
          : selected_label;
  ctx_.binding_confidence = binder_out.binding_confidence;
  ctx_.entropy_norm = candidate.entropy_norm;
  ctx_.max_prob = candidate.max_prob;
  ctx_.spin_direction = binder_dbg.spin_direction;
  ctx_.last_obs_z = selected->z;
  if (selected->timestamp.has_value()) {
    ctx_.last_timestamp = selected->timestamp.value();
    update_time(selected->timestamp.value());
  }

  {
    norm4_v2::PublishStateInput input;
    input.mode = ctx_.mode;
    input.backend_snap = &active_snap;
    if (ctx_.mode == mode::TrackMode::AMBIGUOUS) {
      input.armor_snap = &ambiguous_backend_.ambiguous_snapshot();
    }
    output_adapter_.update_publish_state(&ctx_, input);
  }

  refresh_debug(selected, candidate, binder_out, binder_dbg, mode_decision,
                height_confidence);
  increment_frame();
  return true;
}

Eigen::Vector3d Norm4ArmorTracker::get_center_position() const {
  return ctx_.center_pos;
}

double Norm4ArmorTracker::get_yaw() const { return normalize_angle(ctx_.center_yaw); }

std::pair<double, double> Norm4ArmorTracker::get_radii() const {
  return {ctx_.r1, ctx_.r2};
}

SpinFilterInterface &Norm4ArmorTracker::spin_filter() {
  return structured_backend_.ukf();
}

const SpinFilterInterface &Norm4ArmorTracker::spin_filter() const {
  return structured_backend_.ukf();
}

ManeuverResult Norm4ArmorTracker::assess_maneuver() const {
  const auto &ukf = structured_backend_.ukf();
  const double innov_norm = ukf.last_innov_xyz().size() >= 3
                                ? ukf.last_innov_xyz().norm()
                                : 0.0;
  return maneuver_detector_.detect(
      ukf.last_nis(), innov_norm, ukf.last_update_type());
}

Eigen::Vector3d Norm4ArmorTracker::get_publish_velocity() const {
  return ctx_.publish_vel;
}

bool Norm4ArmorTracker::is_ambiguous_single_mode() const {
  return ctx_.mode == mode::TrackMode::AMBIGUOUS;
}

int Norm4ArmorTracker::effective_num_armors() const {
  return is_ambiguous_single_mode() ? 1 : 4;
}

double Norm4ArmorTracker::confidence_scale() const {
  if (!is_ambiguous_single_mode()) return 1.0;
  return clamp01(config_.outpost.single_mode_confidence_scale);
}

std::vector<geometry_msgs::msg::Pose>
Norm4ArmorTracker::build_armors_offset_for_message() const {
  return output_adapter_.build_armors_offset_for_message(ctx_);
}

void Norm4ArmorTracker::refresh_debug(
    const ObservationData *obs, const norm4_v2::BindingCandidate &candidate,
    const binder::BinderOutput &binder_out,
    const binder::BinderDebugSnapshot & /*binder_dbg*/,
    const mode::ModeDecision &mode_decision, double height_confidence) {
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  debug_snapshot_ = DebugSnapshot{};
  debug_snapshot_.valid = true;
  debug_snapshot_.track_mode =
      (ctx_.mode == mode::TrackMode::STRUCTURED) ? 0 : 1;
  debug_snapshot_.current_panel_id = ctx_.selected_panel_id;
  debug_snapshot_.bound_panel_id = ctx_.bound_panel_id;
  debug_snapshot_.bound_height_label =
      static_cast<int>(ctx_.bound_height_label);
  debug_snapshot_.candidate_panel_id = candidate.candidate_panel_id;
  debug_snapshot_.candidate_prob = candidate.candidate_prob;
  debug_snapshot_.candidate_margin = candidate.candidate_margin;
  debug_snapshot_.selected_yaw_err = candidate.selected_yaw_err;
  debug_snapshot_.selected_xy_residual = candidate.selected_xy_residual;
  debug_snapshot_.entropy_norm = ctx_.entropy_norm;
  debug_snapshot_.max_prob = ctx_.max_prob;
  debug_snapshot_.binding_fsm_state = static_cast<int>(binder_out.fsm_state);
  debug_snapshot_.switch_event = binder_out.switch_occurred ? 1 : 0;
  debug_snapshot_.switch_reason =
      mode_decision.switched ? static_cast<int>(mode_decision.reason)
                             : binder_out.switch_reason;
  debug_snapshot_.binding_confidence = ctx_.binding_confidence;
  debug_snapshot_.height_confidence = height_confidence;
  debug_snapshot_.degraded_single_obs_mode = degraded_single_obs_mode_;
  debug_snapshot_.single_obs_streak = single_obs_streak_;
  debug_snapshot_.dza_converged = ctx_.dza_converged;
  debug_snapshot_.has_observation = (obs != nullptr);
  debug_snapshot_.obs_x = (obs != nullptr) ? obs->x : kNaN;
  debug_snapshot_.obs_y = (obs != nullptr) ? obs->y : kNaN;
  debug_snapshot_.obs_z = (obs != nullptr) ? obs->z : kNaN;
  debug_snapshot_.obs_yaw = (obs != nullptr) ? obs->yaw : kNaN;
  debug_snapshot_.obs_z_jump = candidate.z_jump;
  debug_snapshot_.ping_pong_risk = ctx_.ping_pong_risk_score;
  debug_snapshot_.ping_pong_hold = ctx_.ping_pong_should_hold;
  debug_snapshot_.ping_pong_reason = ctx_.ping_pong_reason;
  debug_snapshot_.ping_pong_hold_ctr = ctx_.ping_pong_hold_counter;
}

}  // namespace fyt::auto_aim
