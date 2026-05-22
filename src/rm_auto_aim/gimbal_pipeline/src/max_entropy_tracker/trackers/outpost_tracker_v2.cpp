// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_tracker_v2.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <iostream>

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
          config.outpost.mode_exit_threshold,
          config.outpost.entropy_enter,
          config.outpost.entropy_exit,
          config.outpost.max_prob_enter,
          config.outpost.max_prob_exit,
          config.outpost.stable_frames}),
      ambiguous_backend_(config),
      structured_backend_(config, dt),
      output_adapter_(config),
      maneuver_detector_(config.maneuver) {
  (void)enable_oscillation;
}

int OutpostTrackerV2::infer_init_panel(const ObservationData &obs) const {
  if (obs.panel_id.has_value()) {
    return clamp_panel(obs.panel_id.value());
  }
  // A single absolute z observation cannot identify the outpost layer unless
  // the target center height is known. Use the middle layer as a neutral seed.
  return 1;
/*
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
*/
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

void OutpostTrackerV2::reset_warmup() {
  warmup_active_ = config_.outpost.v2_warmup_enable;
  warmup_frames_ = 0;
  warmup_current_group_ = -1;
  warmup_groups_.clear();
}

void OutpostTrackerV2::update_warmup_evidence(const ObservationData &obs) {
  if (!warmup_active_) return;
  ++warmup_frames_;
  if (warmup_frames_ > config_.outpost.v2_warmup_max_frames) {
    warmup_frames_ = 1;
    warmup_current_group_ = -1;
    warmup_groups_.clear();
  }

  const Eigen::Vector3d pos(obs.x, obs.y, obs.z);
  bool start_new_group = warmup_current_group_ < 0 ||
                         warmup_current_group_ >=
                             static_cast<int>(warmup_groups_.size());
  if (!start_new_group) {
    const auto &g = warmup_groups_[warmup_current_group_];
    const double z_jump = std::abs(obs.z - g.last_pos.z());
    const double yaw_jump = std::abs(normalize_angle(obs.yaw - g.last_yaw));
    const double xyz_jump = (pos - g.last_pos).norm();
    start_new_group =
        z_jump >= config_.outpost.v2_warmup_z_jump_gate ||
        yaw_jump >= config_.outpost.v2_warmup_yaw_jump_gate ||
        xyz_jump >= config_.outpost.v2_warmup_xyz_jump_gate;
  }

  if (start_new_group) {
    WarmupGroup g;
    g.sample_count = 1;
    g.mean_z = obs.z;
    g.last_pos = pos;
    g.last_yaw = obs.yaw;
    warmup_groups_.push_back(g);
    warmup_current_group_ = static_cast<int>(warmup_groups_.size()) - 1;
    return;
  }

  auto &g = warmup_groups_[warmup_current_group_];
  ++g.sample_count;
  const double n = static_cast<double>(g.sample_count);
  g.mean_z += (obs.z - g.mean_z) / n;
  g.last_pos = pos;
  g.last_yaw = obs.yaw;
}

OutpostTrackerV2::WarmupCommit OutpostTrackerV2::try_commit_warmup() const {
  WarmupCommit out;
  if (!warmup_active_ ||
      static_cast<int>(warmup_groups_.size()) <
          config_.outpost.v2_warmup_min_groups) {
    return out;
  }

  struct Level {
    double mean_z = 0.0;
    int samples = 0;
    std::vector<int> groups;
  };

  std::vector<int> valid_groups;
  for (int i = 0; i < static_cast<int>(warmup_groups_.size()); ++i) {
    if (warmup_groups_[i].sample_count >=
        config_.outpost.v2_warmup_min_samples_per_group) {
      valid_groups.push_back(i);
    }
  }
  if (static_cast<int>(valid_groups.size()) <
      config_.outpost.v2_warmup_min_groups) {
    return out;
  }

  std::sort(valid_groups.begin(), valid_groups.end(), [this](int a, int b) {
    return warmup_groups_[a].mean_z > warmup_groups_[b].mean_z;
  });

  const double merge_gate =
      std::max(0.015, 0.6 * config_.outpost.v2_warmup_z_jump_gate);
  std::vector<Level> levels;
  for (int group_idx : valid_groups) {
    const auto &g = warmup_groups_[group_idx];
    bool merged = false;
    for (auto &level : levels) {
      if (std::abs(g.mean_z - level.mean_z) <= merge_gate) {
        const int new_samples = level.samples + g.sample_count;
        level.mean_z =
            (level.mean_z * level.samples + g.mean_z * g.sample_count) /
            static_cast<double>(new_samples);
        level.samples = new_samples;
        level.groups.push_back(group_idx);
        merged = true;
        break;
      }
    }
    if (!merged) {
      Level level;
      level.mean_z = g.mean_z;
      level.samples = g.sample_count;
      level.groups.push_back(group_idx);
      levels.push_back(level);
    }
  }

  if (levels.size() < 3) return out;
  std::sort(levels.begin(), levels.end(),
            [](const Level &a, const Level &b) { return a.mean_z > b.mean_z; });

  const double dz01 = levels[0].mean_z - levels[1].mean_z;
  const double dz12 = levels[1].mean_z - levels[2].mean_z;
  const double dz02 = levels[0].mean_z - levels[2].mean_z;
  if (!(dz01 > 1e-5 && dz12 > 1e-5 && dz02 > 1e-5)) return out;
  if (dz02 < config_.outpost.v2_warmup_min_large_diff) return out;

  const double dz_small = 0.5 * (dz01 + dz12);
  const double ratio = dz02 / std::max(1e-5, dz_small);
  const double balance = std::max(dz01, dz12) / std::max(1e-5, std::min(dz01, dz12));
  if (ratio < config_.outpost.v2_warmup_ratio_min ||
      ratio > config_.outpost.v2_warmup_ratio_max || balance > 1.8) {
    return out;
  }

  int current_level = -1;
  if (warmup_current_group_ >= 0) {
    double best_err = std::numeric_limits<double>::infinity();
    const double current_z = warmup_groups_[warmup_current_group_].mean_z;
    for (int i = 0; i < 3; ++i) {
      const double err = std::abs(current_z - levels[i].mean_z);
      if (err < best_err) {
        best_err = err;
        current_level = i;
      }
    }
  }
  if (current_level < 0 || current_level > 2) return out;

  out.ready = true;
  out.current_panel = current_level;  // sorted high/mid/low maps to panel 0/1/2
  out.dz_small = dz_small;
  out.dz_large = dz02;
  return out;
}

bool OutpostTrackerV2::run_warmup_update(const ObservationData &obs) {
  update_warmup_evidence(obs);

  outpost_v2::BackendUpdateHint hint;
  hint.panel_id = 1;
  hint.position_confidence = 0.25;
  ambiguous_backend_.update(obs, hint);

  const auto snap = ambiguous_backend_.snapshot();
  sync_runtime_from_backend(snap);
  outpost_v2::PublishStateInput pub_input;
  pub_input.mode = mode::TrackMode::AMBIGUOUS;
  pub_input.backend_snap = &snap;
  pub_input.armor_snap = &ambiguous_backend_.ambiguous_snapshot();
  output_adapter_.update_publish_state(&ctx_, pub_input);

  const auto commit = try_commit_warmup();
  if (!commit.ready) return true;

  const int panel = clamp_panel(commit.current_panel);
  warmup_active_ = false;
  ctx_.selected_panel_id = panel;
  ctx_.bound_panel_id = panel;
  ctx_.binding_confidence = 0.75;
  binder_bridge_.reset(panel, obs.z);
  structured_backend_.reset(obs, panel);
  ambiguous_backend_.update(obs, outpost_v2::BackendUpdateHint{panel, 0.5, true});
  return true;
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

  ctx_.lost_frames = lost_count();
  handle_observation_received(config_.outpost.tracking_thres);
  candidate = obs_frontend_.build_binding_candidate(*selected, ctx_);

  binder_out = binder_bridge_.step(*selected, obs, static_cast<int>(obs.size()),
                                   candidate, ctx_);
  const auto &binder_dbg = binder_bridge_.debug_snapshot();

  // Phase-3 policy: periodic signature is the primary 2dz evidence.
  // DOUBLE_DZ tag alone is no longer sufficient unless signature is also decent.
  const bool has_2dz_signature =
      (binder_dbg.signature_score >= 0.60) ||
      (binder_dbg.jump_detected &&
       binder_dbg.jump_kind == binder::JumpKind::DOUBLE_DZ &&
       binder_dbg.signature_score >= 0.45);
  std::cout << "[EvidenceFuser]:has_2dz_signature: " << has_2dz_signature
            << ",jump_detected: " << binder_dbg.jump_detected
            << ",jump_kind: " << static_cast<int>(binder_dbg.jump_kind)
            << ",signature_score: " << binder_dbg.signature_score
            << std::endl;
  mode::ModeEvidence evidence = evidence_fuser_.fuse(
      selected->timestamp.value_or(ctx_.last_timestamp.value_or(0.0)),
      static_cast<int>(obs.size()), candidate.candidate_panel_id,
      has_2dz_signature, candidate.entropy_norm, candidate.max_prob,
      candidate.candidate_margin, binder_dbg);
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
  if (binder_out.binding_conflict_for_update) {
    const double conflict_scale =
        std::clamp(config_.outpost.binding_conflict_position_scale, 0.0, 1.0);
    hint.position_confidence =
        std::clamp(hint.position_confidence * conflict_scale, 0.05, 1.0);
  }
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
  if (binder_out.bound_id >= 0) {
    ctx_.bound_panel_id = binder_out.bound_id;
  } else if (binder_out.selected_id >= 0) {
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
