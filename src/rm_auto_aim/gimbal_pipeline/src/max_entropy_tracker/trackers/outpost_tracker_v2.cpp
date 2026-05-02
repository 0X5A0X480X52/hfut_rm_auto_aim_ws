// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_tracker_v2.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

namespace {

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

int clamp_panel(int panel) {
  int v = panel % 3;
  if (v < 0) v += 3;
  return v;
}

}  // namespace

OutpostTrackerV2::OutpostTrackerV2(const UnifiedConfig &config, double dt,
                                   bool enable_oscillation)
    : BaseTracker(dt),
      config_(config),
      radius_(std::max(0.05, config.outpost.radius)),
      z_offsets_{config.outpost.z_offset_0, config.outpost.z_offset_1,
                 config.outpost.z_offset_2},
      obs_frontend_(config),
      binder_bridge_(config),
      evidence_fuser_([&config]() {
        mode::EvidenceFuserConfig cfg;
        cfg.w_dual = config.outpost.mode_weight_dual;
        cfg.w_margin = config.outpost.mode_weight_margin;
        cfg.w_health = config.outpost.mode_weight_health;
        cfg.w_entropy = config.outpost.mode_weight_entropy;
        cfg.jump_event_weight = config.outpost.mode_weight_jump;
        return cfg;
      }()),
      mode_fsm_(mode::ModeFSMConfig{
          config.outpost.mode_enter_confirm_frames,
          config.outpost.mode_exit_confirm_frames,
          config.outpost.mode_min_dwell_frames,
          config.outpost.mode_enter_threshold,
          config.outpost.mode_exit_threshold}),
      ambiguous_backend_(config),
      structured_backend_(config, dt),
      output_adapter_(config),
      maneuver_detector_(config.maneuver) {
  (void)enable_oscillation;
}

int OutpostTrackerV2::infer_init_panel(const ObservationData &obs) const {
  int init_panel = 0;
  double min_abs_cz = std::abs(obs.z - z_offsets_[0]);
  for (int i = 1; i < 3; ++i) {
    const double abs_cz_i = std::abs(obs.z - z_offsets_[i]);
    if (abs_cz_i < min_abs_cz) {
      min_abs_cz = abs_cz_i;
      init_panel = i;
    }
  }
  return init_panel;
}

int OutpostTrackerV2::semantic_from_panel(int panel_id) const {
  if (panel_id == 0) return 0;
  if (panel_id == 1) return 1;
  if (panel_id == 2) return 2;
  return -1;
}

void OutpostTrackerV2::sync_runtime_from_backend(
    const outpost_v2::BackendStateSnapshot &snap) {
  ctx_.center_pos = snap.center_pos;
  ctx_.center_vel = snap.center_vel;
  ctx_.center_yaw = snap.center_yaw;
  ctx_.yaw_rate = snap.yaw_rate;
}

void OutpostTrackerV2::initialize(const std::vector<ObservationData> &obs,
                                  double /*r1*/, double /*r2*/,
                                  double /*dza*/) {
  if (obs.empty()) {
    throw std::invalid_argument("OutpostTrackerV2 requires one observation");
  }
  const ObservationData *selected =
      obs_frontend_.select_primary_observation(obs, ctx_);
  if (selected == nullptr) {
    throw std::invalid_argument("OutpostTrackerV2 cannot select observation");
  }

  const int init_panel = infer_init_panel(*selected);
  ambiguous_backend_.reset(*selected, init_panel);
  structured_backend_.reset(*selected, init_panel);
  binder_bridge_.reset(init_panel, selected->z);
  mode_fsm_.reset(mode::TrackMode::AMBIGUOUS);

  ctx_ = outpost_v2::OutpostRuntimeContext{};
  ctx_.mode = mode::TrackMode::AMBIGUOUS;
  ctx_.selected_panel_id = init_panel;
  ctx_.bound_panel_id = init_panel;
  ctx_.binding_confidence = 1.0 / 3.0;
  ctx_.entropy_norm = 1.0;
  ctx_.max_prob = 1.0 / 3.0;
  ctx_.last_obs_z = selected->z;
  if (selected->timestamp.has_value()) {
    ctx_.last_timestamp = selected->timestamp.value();
    current_time_ = selected->timestamp.value();
    last_update_time_ = selected->timestamp.value();
  }

  sync_runtime_from_backend(ambiguous_backend_.snapshot());
  {
    const auto init_snap = ambiguous_backend_.snapshot();
    outpost_v2::PublishStateInput pub_input;
    pub_input.mode = mode::TrackMode::AMBIGUOUS;
    pub_input.backend_snap = &init_snap;
    pub_input.armor_snap = &ambiguous_backend_.ambiguous_snapshot();
    output_adapter_.update_publish_state(&ctx_, pub_input);
  }

  mark_initialized();
  transition_to(TrackerState::INITIALIZING);
  increment_frame();

  outpost_v2::BindingCandidate candidate;
  binder::BinderOutput binder_out;
  mode::ModeDecision mode_decision;
  refresh_debug(selected, candidate, binder_out, binder_bridge_.debug_snapshot(),
                mode_decision);
}

void OutpostTrackerV2::predict(std::optional<double> target_time) {
  if (!is_initialized()) return;
  const double dt = compute_dt(target_time);
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
  {
    outpost_v2::PublishStateInput pub_input;
    pub_input.mode = ctx_.mode;
    pub_input.backend_snap = &snap;
    if (ctx_.mode == mode::TrackMode::AMBIGUOUS) {
      pub_input.armor_snap = &ambiguous_backend_.ambiguous_snapshot();
    }
    output_adapter_.update_publish_state(&ctx_, pub_input);
  }
}

bool OutpostTrackerV2::update(const std::vector<ObservationData> &obs) {
  outpost_v2::BindingCandidate candidate;
  binder::BinderOutput binder_out;
  mode::ModeDecision mode_decision;

  if (!is_initialized() || obs.empty()) {
    handle_observation_loss(config_.outpost.tracking_thres,
                            config_.outpost.lost_thres);
    refresh_debug(nullptr, candidate, binder_out, binder_bridge_.debug_snapshot(),
                  mode_decision);
    return false;
  }

  const ObservationData *selected =
      obs_frontend_.select_primary_observation(obs, ctx_);
  if (selected == nullptr) {
    handle_observation_loss(config_.outpost.tracking_thres,
                            config_.outpost.lost_thres);
    refresh_debug(nullptr, candidate, binder_out, binder_bridge_.debug_snapshot(),
                  mode_decision);
    return false;
  }

  if (selected->timestamp.has_value() && current_time_.has_value()) {
    const double d = selected->timestamp.value() - current_time_.value();
    if (d > min_dt_) predict(selected->timestamp.value());
  }

  handle_observation_received(config_.outpost.tracking_thres);
  candidate = obs_frontend_.build_binding_candidate(*selected, ctx_);

  binder_out = binder_bridge_.step(*selected, obs, static_cast<int>(obs.size()),
                                   candidate, ctx_);
  const auto &binder_dbg = binder_bridge_.debug_snapshot();

  const bool has_2dz_signature =
      binder_dbg.jump_detected &&
      binder_dbg.jump_kind == binder::JumpKind::DOUBLE_DZ;
  mode::ModeEvidence evidence = evidence_fuser_.fuse(
      selected->timestamp.value_or(ctx_.last_timestamp.value_or(0.0)),
      static_cast<int>(obs.size()), has_2dz_signature, candidate.entropy_norm,
      candidate.max_prob, candidate.candidate_margin, binder_dbg);
  mode_decision = mode_fsm_.step(evidence);

  int selected_panel = (binder_out.selected_id >= 0)
                           ? binder_out.selected_id
                           : candidate.candidate_panel_id;
  if (selected_panel < 0) selected_panel = ctx_.selected_panel_id;
  selected_panel = clamp_panel(selected_panel);

  if (mode_decision.switched) {
    ctx_.mode = mode_decision.mode;
    if (ctx_.mode == mode::TrackMode::STRUCTURED) {
      structured_backend_.reset(*selected, selected_panel);
    } else {
      ambiguous_backend_.reset(*selected, selected_panel);
    }
  }

  outpost_v2::BackendUpdateHint hint;
  hint.panel_id = selected_panel;
  hint.position_confidence = std::max(0.05, binder_out.binding_confidence);
  hint.enforce_panel_constraint = true;

  bool ok = false;
  if (ctx_.mode == mode::TrackMode::STRUCTURED) {
    ok = structured_backend_.update(*selected, hint);
    outpost_v2::BackendUpdateHint shadow = hint;
    shadow.position_confidence =
        std::clamp(0.25 + 0.25 * hint.position_confidence, 0.05, 0.60);
    ambiguous_backend_.update(*selected, shadow);
  } else {
    ok = ambiguous_backend_.update(*selected, hint);
    outpost_v2::BackendUpdateHint shadow = hint;
    shadow.position_confidence =
        std::clamp(0.25 + 0.25 * hint.position_confidence, 0.05, 0.60);
    structured_backend_.update(*selected, shadow);
  }
  if (!ok) {
    refresh_debug(selected, candidate, binder_out, binder_dbg, mode_decision);
    return false;
  }

  const auto active_snap = (ctx_.mode == mode::TrackMode::STRUCTURED)
                               ? structured_backend_.snapshot()
                               : ambiguous_backend_.snapshot();
  sync_runtime_from_backend(active_snap);

  ctx_.selected_panel_id = selected_panel;
  if (binder_out.selected_id >= 0) {
    ctx_.bound_panel_id = binder_out.selected_id;
  } else {
    ctx_.bound_panel_id = selected_panel;
  }
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
    outpost_v2::PublishStateInput pub_input;
    pub_input.mode = ctx_.mode;
    pub_input.backend_snap = &active_snap;
    if (ctx_.mode == mode::TrackMode::AMBIGUOUS) {
      pub_input.armor_snap = &ambiguous_backend_.ambiguous_snapshot();
    }
    output_adapter_.update_publish_state(&ctx_, pub_input);
  }
  refresh_debug(selected, candidate, binder_out, binder_dbg, mode_decision);
  increment_frame();
  return true;
}

Eigen::Vector3d OutpostTrackerV2::get_center_position() const {
  return ctx_.center_pos;
}

double OutpostTrackerV2::get_yaw() const {
  if (is_ambiguous_single_mode() &&
      config_.outpost.ambiguous_publish_single_armor_semantics) {
    return normalize_angle(ctx_.center_yaw);
  }
  return normalize_angle(ctx_.center_yaw + M_PI);
}

std::pair<double, double> OutpostTrackerV2::get_radii() const {
  return {radius_, radius_};
}

SpinFilterInterface &OutpostTrackerV2::spin_filter() {
  return structured_backend_.ukf();
}

const SpinFilterInterface &OutpostTrackerV2::spin_filter() const {
  return structured_backend_.ukf();
}

ManeuverResult OutpostTrackerV2::assess_maneuver() const {
  const auto &ukf = structured_backend_.ukf();
  const double innov_norm = ukf.last_innov_xyz().size() >= 3
                                ? ukf.last_innov_xyz().norm()
                                : 0.0;
  return maneuver_detector_.detect(
      ukf.last_nis(), innov_norm, ukf.last_update_type());
}

Eigen::Vector3d OutpostTrackerV2::get_publish_velocity() const {
  return ctx_.publish_vel;
}

bool OutpostTrackerV2::is_ambiguous_single_mode() const {
  return ctx_.mode == mode::TrackMode::AMBIGUOUS;
}

int OutpostTrackerV2::effective_num_armors() const {
  return is_ambiguous_single_mode() ? 1 : 3;
}

double OutpostTrackerV2::confidence_scale() const {
  if (!is_ambiguous_single_mode()) return 1.0;
  return clamp01(config_.outpost.single_mode_confidence_scale);
}

std::vector<geometry_msgs::msg::Pose>
OutpostTrackerV2::build_armors_offset_for_message() const {
  return output_adapter_.build_armors_offset_for_message(ctx_);
}

void OutpostTrackerV2::refresh_debug(
    const ObservationData *obs, const outpost_v2::BindingCandidate &candidate,
    const binder::BinderOutput &binder_out,
    const binder::BinderDebugSnapshot &binder_dbg,
    const mode::ModeDecision &mode_decision) {
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  debug_snapshot_ = DebugSnapshot{};
  debug_snapshot_.valid = true;
  debug_snapshot_.track_mode =
      (ctx_.mode == mode::TrackMode::STRUCTURED) ? 0 : 1;
  debug_snapshot_.estimated_id =
      (ctx_.mode == mode::TrackMode::STRUCTURED) ? ctx_.selected_panel_id : -1;
  debug_snapshot_.runtime_panel_id = ctx_.selected_panel_id;
  debug_snapshot_.bound_height_label = semantic_from_panel(ctx_.bound_panel_id);
  debug_snapshot_.obs_inferred_id = candidate.candidate_panel_id;
  debug_snapshot_.obs_inferred_id_z = binder_dbg.to_id;
  debug_snapshot_.candidate_panel_id = candidate.candidate_panel_id;
  debug_snapshot_.candidate_prob = candidate.candidate_prob;
  debug_snapshot_.candidate_margin = candidate.candidate_margin;
  debug_snapshot_.selected_xy_residual = candidate.selected_xy_residual;
  debug_snapshot_.entropy_norm = ctx_.entropy_norm;
  debug_snapshot_.max_prob = ctx_.max_prob;
  debug_snapshot_.hyp_costs = candidate.costs;
  debug_snapshot_.hyp_probs = candidate.probs;
  debug_snapshot_.center_yaw_est = ctx_.center_yaw;
  debug_snapshot_.has_observation = (obs != nullptr);
  debug_snapshot_.obs_x = (obs != nullptr) ? obs->x : kNaN;
  debug_snapshot_.obs_y = (obs != nullptr) ? obs->y : kNaN;
  debug_snapshot_.obs_z = (obs != nullptr) ? obs->z : kNaN;
  debug_snapshot_.obs_yaw = (obs != nullptr) ? obs->yaw : kNaN;
  debug_snapshot_.obs_z_jump = candidate.z_jump;
  debug_snapshot_.obs_dz_from_audit_center = kNaN;
  debug_snapshot_.obs_z_audit_costs = {kNaN, kNaN, kNaN};

  debug_snapshot_.binding_confidence = ctx_.binding_confidence;
  debug_snapshot_.switch_event = binder_out.switch_occurred ? 1 : 0;
  debug_snapshot_.switch_reason =
      mode_decision.switched ? static_cast<int>(mode_decision.reason)
                             : binder_out.switch_reason;
  debug_snapshot_.transition_state =
      (binder_dbg.fsm_state == binder::BindingFSMState::PENDING_SWITCH) ? 1 : 0;
  debug_snapshot_.z_audit_conflict_count = binder_dbg.consecutive_bad_frames;
  debug_snapshot_.z_audit_confidence = binder_dbg.health_score;
  debug_snapshot_.publish_x = ctx_.publish_pos.x();
  debug_snapshot_.publish_y = ctx_.publish_pos.y();
  debug_snapshot_.publish_z = ctx_.publish_pos.z();
  debug_snapshot_.period_confidence = binder_dbg.period_confidence;
  debug_snapshot_.period_update_applied = 0;
  debug_snapshot_.period_phase_index = binder_dbg.period_phase;
  debug_snapshot_.spin_direction = binder_dbg.spin_direction;
  debug_snapshot_.dz_small_est = binder_dbg.dz_small_est;
  debug_snapshot_.dz_large_est = binder_dbg.dz_large_est;
}

}  // namespace fyt::auto_aim
