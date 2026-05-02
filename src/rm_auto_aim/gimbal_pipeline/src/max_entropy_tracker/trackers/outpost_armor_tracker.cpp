// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_armor_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim {

namespace {

constexpr double kLog3 = 1.0986122886681098;
constexpr double kOutpostPitchDown = -0.2618;

double clamp01(double x) {
  return std::clamp(x, 0.0, 1.0);
}

double angle_abs_diff(double a, double b) {
  return std::abs(normalize_angle(a - b));
}

double median_of_vector(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const size_t n = v.size();
  if (n % 2 == 1) return v[n / 2];
  return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

int wrap_phase_index(int value, int modulo) {
  const int m = std::max(1, modulo);
  const int r = value % m;
  return (r < 0) ? (r + m) : r;
}

}  // namespace

OutpostArmorTracker::OutpostArmorTracker(const UnifiedConfig &config, double dt,
                                         bool enable_oscillation)
    : BaseTracker(dt),
      config_(config),
      radius_(std::max(0.05, config.outpost.radius)),
      z_offsets_{config.outpost.z_offset_0, config.outpost.z_offset_1,
                 config.outpost.z_offset_2},
      outpost_ukf_(config, dt),
      maneuver_detector_(config.maneuver),
      binding_profile_(binder::RobotBindingProfileProvider::from_robot_id(
          "outpost",
          {config.outpost.z_offset_0, config.outpost.z_offset_1,
           config.outpost.z_offset_2})) {
  (void)enable_oscillation;
  const double raw_step = (config.outpost.panel_angle_step > 1e-6)
                              ? config.outpost.panel_angle_step
                              : (2.0 * M_PI / 3.0);
  const double step = std::abs(raw_step);
  // Contract:
  //   id=0 (highest) at 0 deg,
  //   clockwise order: 0 -> 2 -> 1.
  // In CCW-positive yaw convention this is: [0, +step, -step].
  panel_angles_ = {0.0, step, -step};

  binder_pipeline_ = binder::BinderFactory::create(
      binding_profile_, build_binder_config_from_outpost());
}

void OutpostArmorTracker::initialize(const std::vector<ObservationData> &obs,
                                     double /*r1*/, double /*r2*/,
                                     double /*dza*/) {
  if (obs.empty()) {
    throw std::invalid_argument("OutpostArmorTracker requires one observation");
  }

  const ObservationData *selected = select_observation(obs);
  if (selected == nullptr) {
    throw std::invalid_argument("OutpostArmorTracker cannot select observation");
  }

  int init_panel = 0;
  double min_abs_cz = std::abs(selected->z - z_offsets_[0]);
  for (int i = 1; i < 3; ++i) {
    const double cz_i = selected->z - z_offsets_[i];
    const double abs_cz_i = std::abs(cz_i);
    if (abs_cz_i < min_abs_cz) {
      min_abs_cz = abs_cz_i;
      init_panel = i;
    }
  }

  selected_panel_id_ = init_panel;
  last_best_panel_id_ = init_panel;

  mode_ = TrackMode::AMBIGUOUS_SINGLE_ARMOR;
  stable_counter_ = 0;
  entropy_norm_ = 1.0;
  max_prob_ = 1.0 / 3.0;

  center_z_history_.clear();
  push_center_z_history(selected->z - z_offsets_[init_panel]);

  z_audit_initialized_ = false;
  z_audit_center_est_ = std::numeric_limits<double>::quiet_NaN();
  z_audit_prev_obs_z_ = std::numeric_limits<double>::quiet_NaN();
  z_audit_prev_panel_id_ = -1;

  bound_panel_id_ = -1;
  binding_transition_state_ = BindingTransitionState::LOCKED;
  transition_candidate_panel_ = -1;
  transition_confirm_count_ = 0;
  switch_event_ = 0;
  switch_reason_ = 0;
  z_audit_conflict_count_ = 0;
  z_audit_confidence_ = std::numeric_limits<double>::quiet_NaN();
  binding_conflict_for_update_ = false;
  binding_confidence_ = max_prob_;
  bound_height_label_ = static_cast<int>(HeightSemantic::UNKNOWN);
  candidate_panel_id_ = init_panel;
  candidate_prob_ = 1.0 / 3.0;
  candidate_margin_ = 0.0;
  selected_xy_residual_ = std::numeric_limits<double>::quiet_NaN();
  dz_jump_history_.clear();
  dz_small_est_ = std::numeric_limits<double>::quiet_NaN();
  dz_large_est_ = std::numeric_limits<double>::quiet_NaN();
  period_update_applied_ = 0;
  period_phase_index_ = -1;
  period_confidence_ = std::numeric_limits<double>::quiet_NaN();
  spin_direction_ = 0;

  outpost_ukf_.initialize({*selected}, radius_, radius_, 0.0, init_panel);
  if (binder_pipeline_) {
    binder_pipeline_->reset(init_panel, binder::HeightLabel::MIDDLE,
                            selected->z);
  }
  sync_internal_state_from_filter();

  if (selected->timestamp.has_value()) {
    current_time_ = selected->timestamp.value();
    last_update_time_ = selected->timestamp.value();
    last_internal_update_time_ = selected->timestamp.value();
  } else {
    last_internal_update_time_.reset();
  }

  mark_initialized();
  transition_to(TrackerState::INITIALIZING);
  increment_frame();

  update_publish_state();

  const double init_history_center_z =
      center_z_history_.empty() ? std::numeric_limits<double>::quiet_NaN()
                                : history_center_z_median();
  const auto init_hyps = evaluate_hypotheses(*selected, center_position_est_.z(),
                                             init_history_center_z);
  const auto z_audit = infer_panel_id_from_z_jump_audit(*selected);

  debug_snapshot_.valid = true;
  debug_snapshot_.track_mode = static_cast<int>(mode_);
  debug_snapshot_.estimated_id = is_ambiguous_single_mode() ? -1 : selected_panel_id_;
  debug_snapshot_.runtime_panel_id = selected_panel_id_;
  debug_snapshot_.bound_height_label = bound_height_label_;
  debug_snapshot_.obs_inferred_id = infer_panel_id_from_hypotheses(init_hyps);
  debug_snapshot_.obs_inferred_id_z = z_audit.panel_id;
  debug_snapshot_.candidate_panel_id = candidate_panel_id_;
  debug_snapshot_.candidate_prob = candidate_prob_;
  debug_snapshot_.candidate_margin = candidate_margin_;
  debug_snapshot_.selected_xy_residual = selected_xy_residual_;
  debug_snapshot_.entropy_norm = entropy_norm_;
  debug_snapshot_.max_prob = max_prob_;
  debug_snapshot_.hyp_costs = {0.0, 0.0, 0.0};
  debug_snapshot_.hyp_probs = {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0};
  debug_snapshot_.center_yaw_est = center_yaw_est_;
  debug_snapshot_.has_observation = true;
  debug_snapshot_.obs_x = selected->x;
  debug_snapshot_.obs_y = selected->y;
  debug_snapshot_.obs_z = selected->z;
  debug_snapshot_.obs_yaw = selected->yaw;
  debug_snapshot_.obs_z_jump = z_audit.z_jump;
  debug_snapshot_.obs_dz_from_audit_center = z_audit.dz_from_center;
  debug_snapshot_.obs_z_audit_costs = z_audit.costs;
  debug_snapshot_.binding_confidence = binding_confidence_;
  debug_snapshot_.switch_event = switch_event_;
  debug_snapshot_.switch_reason = switch_reason_;
  debug_snapshot_.transition_state =
      static_cast<int>(binding_transition_state_);
  debug_snapshot_.z_audit_conflict_count = z_audit_conflict_count_;
  debug_snapshot_.z_audit_confidence = z_audit_confidence_;
  debug_snapshot_.publish_x = publish_position_.x();
  debug_snapshot_.publish_y = publish_position_.y();
  debug_snapshot_.publish_z = publish_position_.z();
  debug_snapshot_.period_confidence = period_confidence_;
  debug_snapshot_.period_update_applied = period_update_applied_;
  debug_snapshot_.period_phase_index = period_phase_index_;
  debug_snapshot_.spin_direction = spin_direction_;
  debug_snapshot_.dz_small_est = dz_small_est_;
  debug_snapshot_.dz_large_est = dz_large_est_;
}

void OutpostArmorTracker::predict(std::optional<double> target_time) {
  if (!is_initialized()) return;

  const double dt = compute_dt(target_time);

  outpost_ukf_.predict(dt);
  apply_motion_constraints_from_config();
  sync_internal_state_from_filter();

  if (target_time.has_value()) {
    current_time_ = target_time.value();
  } else if (current_time_.has_value()) {
    current_time_ = current_time_.value() + dt;
  }

  update_publish_state();
}

bool OutpostArmorTracker::update(const std::vector<ObservationData> &obs) {
  // Default to "no fresh observation" for this cycle; update paths with a
  // selected observation will overwrite these fields.
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  switch_event_ = 0;
  switch_reason_ = 0;
  period_update_applied_ = 0;
  binding_conflict_for_update_ = false;
  candidate_panel_id_ = -1;
  candidate_prob_ = kNaN;
  candidate_margin_ = kNaN;
  selected_xy_residual_ = kNaN;
  debug_snapshot_.valid = true;
  debug_snapshot_.track_mode = static_cast<int>(mode_);
  debug_snapshot_.estimated_id =
      is_ambiguous_single_mode() ? -1 : selected_panel_id_;
  debug_snapshot_.runtime_panel_id = selected_panel_id_;
  debug_snapshot_.bound_height_label = bound_height_label_;
  debug_snapshot_.obs_inferred_id = -1;
  debug_snapshot_.obs_inferred_id_z = -1;
  debug_snapshot_.candidate_panel_id = -1;
  debug_snapshot_.candidate_prob = kNaN;
  debug_snapshot_.candidate_margin = kNaN;
  debug_snapshot_.selected_xy_residual = kNaN;
  debug_snapshot_.hyp_costs = {kNaN, kNaN, kNaN};
  debug_snapshot_.hyp_probs = {kNaN, kNaN, kNaN};
  debug_snapshot_.has_observation = false;
  debug_snapshot_.obs_x = kNaN;
  debug_snapshot_.obs_y = kNaN;
  debug_snapshot_.obs_z = kNaN;
  debug_snapshot_.obs_yaw = kNaN;
  debug_snapshot_.obs_z_jump = kNaN;
  debug_snapshot_.obs_dz_from_audit_center = kNaN;
  debug_snapshot_.obs_z_audit_costs = {kNaN, kNaN, kNaN};
  debug_snapshot_.binding_confidence = binding_confidence_;
  debug_snapshot_.switch_event = switch_event_;
  debug_snapshot_.switch_reason = switch_reason_;
  debug_snapshot_.transition_state =
      static_cast<int>(binding_transition_state_);
  debug_snapshot_.z_audit_conflict_count = z_audit_conflict_count_;
  debug_snapshot_.z_audit_confidence = z_audit_confidence_;
  debug_snapshot_.publish_x = publish_position_.x();
  debug_snapshot_.publish_y = publish_position_.y();
  debug_snapshot_.publish_z = publish_position_.z();
  debug_snapshot_.period_confidence = period_confidence_;
  debug_snapshot_.period_update_applied = period_update_applied_;
  debug_snapshot_.period_phase_index = period_phase_index_;
  debug_snapshot_.spin_direction = spin_direction_;
  debug_snapshot_.dz_small_est = dz_small_est_;
  debug_snapshot_.dz_large_est = dz_large_est_;

  if (!is_initialized() || obs.empty()) {
    handle_observation_loss(config_.outpost.tracking_thres,
                            config_.outpost.lost_thres);
    return false;
  }

  const ObservationData *selected = select_observation(obs);
  if (selected == nullptr) {
    handle_observation_loss(config_.outpost.tracking_thres,
                            config_.outpost.lost_thres);
    return false;
  }

  const auto z_audit = infer_panel_id_from_z_jump_audit(*selected);
  z_audit_confidence_ = 0.0;
  if (z_audit.panel_id >= 0) {
    double best_audit = std::numeric_limits<double>::infinity();
    double second_audit = std::numeric_limits<double>::infinity();
    for (double cost : z_audit.costs) {
      if (cost < best_audit) {
        second_audit = best_audit;
        best_audit = cost;
      } else if (cost < second_audit) {
        second_audit = cost;
      }
    }
    if (std::isfinite(best_audit) && std::isfinite(second_audit)) {
      z_audit_confidence_ =
          std::clamp(std::max(0.0, second_audit - best_audit) / 0.10,
                     0.0, 1.0);
    }
  }

  // Predict to observation timestamp when possible.
  if (selected->timestamp.has_value() && current_time_.has_value()) {
    const double d = selected->timestamp.value() - current_time_.value();
    if (d > min_dt_) predict(selected->timestamp.value());
  }

  handle_observation_received(config_.outpost.tracking_thres);

  double dt_for_update = dt_;
  if (selected->timestamp.has_value() && last_internal_update_time_.has_value()) {
    dt_for_update = std::clamp(selected->timestamp.value() -
                                   last_internal_update_time_.value(),
                               min_dt_, max_dt_);
  }

  if (selected->timestamp.has_value()) {
    last_internal_update_time_ = selected->timestamp.value();
  }

  const double hist_center_z = center_z_history_.empty()
                                   ? std::numeric_limits<double>::quiet_NaN()
                                   : history_center_z_median();

  auto hyps = evaluate_hypotheses(*selected, center_position_est_.z(),
                                  hist_center_z);

  // In ambiguous mode, fuse independent z-jump audit as a soft prior to
  // reduce persistent panel-id suppression (especially panel 2).
  if (mode_ == TrackMode::AMBIGUOUS_SINGLE_ARMOR && z_audit.panel_id >= 0) {
    double best_audit = z_audit.costs[0];
    double second_audit = std::numeric_limits<double>::infinity();
    for (int i = 1; i < 3; ++i) {
      if (z_audit.costs[i] < best_audit) {
        second_audit = best_audit;
        best_audit = z_audit.costs[i];
      } else {
        second_audit = std::min(second_audit, z_audit.costs[i]);
      }
    }

    const double audit_conf = z_audit_confidence_;
    double prior_weight = 0.35 * audit_conf;

    // Panel 2 is the most frequently suppressed branch in ambiguous mode.
    // Add a targeted boost only when independent z-audit selects panel 2.
    if (z_audit.panel_id == 2) {
      prior_weight += 4.0 * audit_conf;
    }

    // If the previous posterior was still highly ambiguous, mildly increase
    // prior usage to help the tracker escape long ambiguous lock-in.
    const double ambiguity_boost =
        std::clamp((entropy_norm_ - 0.55) / 0.20, 0.0, 1.0);
    prior_weight += 0.30 * ambiguity_boost * audit_conf;

    for (int i = 0; i < 3; ++i) {
      const double normalized_audit =
          std::max(0.0, z_audit.costs[i] - best_audit);
      hyps[i].cost += prior_weight * normalized_audit;
    }
  }

  compute_probabilities(hyps);

  int best_idx_pre = 0;
  int second_idx_pre = 1;
  if (hyps[second_idx_pre].probability > hyps[best_idx_pre].probability) {
    std::swap(best_idx_pre, second_idx_pre);
  }
  for (int i = 2; i < 3; ++i) {
    if (hyps[i].probability > hyps[best_idx_pre].probability) {
      second_idx_pre = best_idx_pre;
      best_idx_pre = i;
    } else if (hyps[i].probability > hyps[second_idx_pre].probability) {
      second_idx_pre = i;
    }
  }

  const double candidate_prob_pre = hyps[best_idx_pre].probability;
  const double candidate_margin_pre =
      std::max(0.0, hyps[best_idx_pre].probability -
                        hyps[second_idx_pre].probability);
  const bool allow_period_update =
      candidate_prob_pre >= config_.outpost.binding_period_update_min_confidence &&
      candidate_margin_pre >= config_.outpost.binding_min_candidate_margin;

  update_periodic_evidence(z_audit.z_jump, allow_period_update);
  apply_periodic_jump_prior(hyps, z_audit.z_jump);

  compute_probabilities(hyps);

  int best_idx = 0;
  int second_idx = 1;
  if (hyps[second_idx].probability > hyps[best_idx].probability) {
    std::swap(best_idx, second_idx);
  }
  for (int i = 2; i < 3; ++i) {
    if (hyps[i].probability > hyps[best_idx].probability) {
      second_idx = best_idx;
      best_idx = i;
    } else if (hyps[i].probability > hyps[second_idx].probability) {
      second_idx = i;
    }
  }

  candidate_panel_id_ = hyps[best_idx].panel_id;
  candidate_prob_ = hyps[best_idx].probability;
  candidate_margin_ =
      std::max(0.0, hyps[best_idx].probability - hyps[second_idx].probability);

    const int current_idx = (bound_panel_id_ >= 0)
                  ? hypothesis_index_for_panel(hyps, bound_panel_id_)
                  : best_idx;
    const double current_panel_score =
      compute_same_panel_score(hyps[current_idx], center_position_est_.z());
  const double switch_base =
      (bound_panel_id_ >= 0 && candidate_panel_id_ != bound_panel_id_)
          ? candidate_prob_
          : 0.0;
  const double z_audit_switch_support =
      (z_audit.panel_id >= 0 && candidate_panel_id_ == z_audit.panel_id &&
       candidate_panel_id_ != bound_panel_id_)
          ? 1.0
          : 0.0;
  const double switch_score =
      clamp01(0.50 * switch_base + 0.20 * period_confidence_ +
              0.10 * z_audit_switch_support + 0.20 * candidate_margin_);

  if (config_.outpost.binding_use_new_binder_pipeline && binder_pipeline_) {
    binder::BinderFrameInput binder_in;
    binder_in.timestamp =
        selected->timestamp.value_or(current_time_.value_or(0.0));
    binder_in.profile = &binding_profile_;
    binder_in.obs_count = static_cast<int>(obs.size());
    binder_in.candidate_id = candidate_panel_id_;
    binder_in.candidate_prob = candidate_prob_;
    binder_in.candidate_margin = candidate_margin_;
    binder_in.obs_z_values.push_back(selected->z);
    binder_in.obs_yaw_values.push_back(selected->yaw);
    binder_in.z_jump = z_audit.z_jump;
    binder_in.has_z_jump = std::isfinite(z_audit.z_jump);
    binder_in.yaw_rate_est = yaw_rate_est_;
    binder_in.spin_direction_hint = spin_direction_;
    binder_in.selected_yaw_err = hyps[current_idx].yaw_err;
    binder_in.cost_margin = candidate_margin_;
    binder_in.same_panel_residual = hyps[current_idx].xy_residual;
    binder_in.has_history = !center_z_history_.empty();

    const binder::BinderOutput binder_out = binder_pipeline_->step(binder_in);
    const auto &binder_dbg = binder_pipeline_->debug_snapshot();

    if (binder_out.selected_id >= 0) {
      selected_panel_id_ = binder_out.selected_id;
      bound_panel_id_ = binder_out.selected_id;
    } else {
      selected_panel_id_ = candidate_panel_id_;
      bound_panel_id_ = candidate_panel_id_;
    }
    bound_height_label_ = semantic_from_panel(selected_panel_id_);
    switch_event_ = binder_out.switch_occurred ? 1 : 0;
    switch_reason_ = binder_out.switch_reason;
    binding_confidence_ = binder_out.binding_confidence;
    binding_transition_state_ =
        (binder_out.fsm_state == binder::BindingFSMState::PENDING_SWITCH)
            ? BindingTransitionState::TRANSITION_CANDIDATE
            : BindingTransitionState::LOCKED;
    transition_candidate_panel_ = binder_dbg.target_id;
    transition_confirm_count_ = 0;
    z_audit_conflict_count_ = 0;
    z_audit_confidence_ = binder_dbg.health_score;
    period_confidence_ = binder_dbg.period_confidence;
    period_phase_index_ = binder_dbg.period_phase;
    spin_direction_ = binder_dbg.spin_direction;
    dz_small_est_ = binder_dbg.dz_small_est;
    dz_large_est_ = binder_dbg.dz_large_est;
    binding_conflict_for_update_ =
        (candidate_panel_id_ >= 0 && candidate_panel_id_ != selected_panel_id_);
  } else {
    update_binding_state_machine(candidate_panel_id_, candidate_prob_,
                     candidate_margin_, current_panel_score,
                                 switch_score);
    const bool z_audit_conflicts =
        config_.outpost.z_audit_rebind_enable && z_audit.panel_id >= 0 &&
        bound_panel_id_ >= 0 && z_audit.panel_id != bound_panel_id_;
    const double min_rebind_conf = std::clamp(
        config_.outpost.z_audit_rebind_min_confidence, 0.0, 1.0);
    const double min_rebind_jump =
        std::max(0.0, config_.outpost.z_audit_rebind_min_jump);
    const bool z_audit_has_jump =
        std::isfinite(z_audit.z_jump) && std::abs(z_audit.z_jump) >= min_rebind_jump;
    const bool z_audit_strong_level =
        z_audit_confidence_ >= std::min(1.0, min_rebind_conf + 0.25);
    if (z_audit_conflicts && z_audit_confidence_ >= min_rebind_conf &&
        (z_audit_has_jump || z_audit_strong_level)) {
      ++z_audit_conflict_count_;
    } else if (!z_audit_conflicts) {
      z_audit_conflict_count_ = 0;
    }

    const int z_rebind_required =
        std::max(1, config_.outpost.z_audit_rebind_confirm_frames);
    if (z_audit_conflict_count_ >= z_rebind_required) {
      bound_panel_id_ = z_audit.panel_id;
      binding_transition_state_ = BindingTransitionState::LOCKED;
      transition_candidate_panel_ = -1;
      transition_confirm_count_ = 0;
      z_audit_conflict_count_ = 0;
      switch_event_ = 1;
      switch_reason_ = 5;
    }
    if (bound_panel_id_ >= 0) {
      if (binding_transition_state_ == BindingTransitionState::TRANSITION_CANDIDATE &&
          transition_candidate_panel_ >= 0) {
        selected_panel_id_ = transition_candidate_panel_;
      } else {
        selected_panel_id_ = bound_panel_id_;
      }
    } else {
      selected_panel_id_ = candidate_panel_id_;
    }
    bound_height_label_ =
        (bound_panel_id_ >= 0)
            ? semantic_from_panel(bound_panel_id_)
            : semantic_from_panel(selected_panel_id_);
  }

  const int selected_idx = hypothesis_index_for_panel(hyps, selected_panel_id_);
  if (!config_.outpost.binding_use_new_binder_pipeline) {
    binding_conflict_for_update_ =
        (candidate_panel_id_ >= 0 && candidate_panel_id_ != selected_panel_id_) ||
        (z_audit.panel_id >= 0 && z_audit.panel_id != selected_panel_id_);
  }
  max_prob_ = hyps[selected_idx].probability;
  selected_xy_residual_ = hyps[selected_idx].xy_residual;
    const double selected_panel_score =
      compute_same_panel_score(hyps[selected_idx], center_position_est_.z());
  if (!config_.outpost.binding_use_new_binder_pipeline) {
    binding_confidence_ =
        binding_confidence_from_scores(max_prob_, candidate_margin_,
                       selected_panel_score, switch_score);
  }

  double entropy = 0.0;
  for (const auto &h : hyps) {
    const double p = std::max(h.probability, 1e-12);
    entropy -= p * std::log(p);
  }
  entropy_norm_ = std::clamp(entropy / kLog3, 0.0, 1.0);

  update_mode_from_entropy(selected_panel_id_, entropy_norm_, max_prob_);
  if (!update_internal_state(*selected, hyps[selected_idx], dt_for_update)) {
    debug_snapshot_.valid = true;
    debug_snapshot_.track_mode = static_cast<int>(mode_);
    debug_snapshot_.estimated_id = is_ambiguous_single_mode() ? -1 : selected_panel_id_;
    debug_snapshot_.runtime_panel_id = selected_panel_id_;
    debug_snapshot_.bound_height_label = bound_height_label_;
    debug_snapshot_.obs_inferred_id = infer_panel_id_from_hypotheses(hyps);
    debug_snapshot_.obs_inferred_id_z = z_audit.panel_id;
    debug_snapshot_.candidate_panel_id = candidate_panel_id_;
    debug_snapshot_.candidate_prob = candidate_prob_;
    debug_snapshot_.candidate_margin = candidate_margin_;
    debug_snapshot_.selected_xy_residual = selected_xy_residual_;
    debug_snapshot_.entropy_norm = entropy_norm_;
    debug_snapshot_.max_prob = max_prob_;
    debug_snapshot_.hyp_costs = {hyps[0].cost, hyps[1].cost, hyps[2].cost};
    debug_snapshot_.hyp_probs = {hyps[0].probability, hyps[1].probability,
                                 hyps[2].probability};
    debug_snapshot_.center_yaw_est = center_yaw_est_;
    debug_snapshot_.has_observation = true;
    debug_snapshot_.obs_x = selected->x;
    debug_snapshot_.obs_y = selected->y;
    debug_snapshot_.obs_z = selected->z;
    debug_snapshot_.obs_yaw = selected->yaw;
    debug_snapshot_.obs_z_jump = z_audit.z_jump;
    debug_snapshot_.obs_dz_from_audit_center = z_audit.dz_from_center;
    debug_snapshot_.obs_z_audit_costs = z_audit.costs;
    debug_snapshot_.binding_confidence = binding_confidence_;
    debug_snapshot_.switch_event = switch_event_;
    debug_snapshot_.switch_reason = switch_reason_;
    debug_snapshot_.transition_state =
        static_cast<int>(binding_transition_state_);
    debug_snapshot_.z_audit_conflict_count = z_audit_conflict_count_;
    debug_snapshot_.z_audit_confidence = z_audit_confidence_;
    debug_snapshot_.publish_x = publish_position_.x();
    debug_snapshot_.publish_y = publish_position_.y();
    debug_snapshot_.publish_z = publish_position_.z();
    debug_snapshot_.period_confidence = period_confidence_;
    debug_snapshot_.period_update_applied = period_update_applied_;
    debug_snapshot_.period_phase_index = period_phase_index_;
    debug_snapshot_.spin_direction = spin_direction_;
    debug_snapshot_.dz_small_est = dz_small_est_;
    debug_snapshot_.dz_large_est = dz_large_est_;
    return false;
  }
  push_center_z_history(hyps[selected_idx].center_z);

  if (selected->timestamp.has_value()) {
    update_time(selected->timestamp.value());
  }

  update_publish_state();

  debug_snapshot_.valid = true;
  debug_snapshot_.track_mode = static_cast<int>(mode_);
  debug_snapshot_.estimated_id = is_ambiguous_single_mode() ? -1 : selected_panel_id_;
  debug_snapshot_.runtime_panel_id = selected_panel_id_;
  debug_snapshot_.bound_height_label = bound_height_label_;
  debug_snapshot_.obs_inferred_id = infer_panel_id_from_hypotheses(hyps);
  debug_snapshot_.obs_inferred_id_z = z_audit.panel_id;
  debug_snapshot_.candidate_panel_id = candidate_panel_id_;
  debug_snapshot_.candidate_prob = candidate_prob_;
  debug_snapshot_.candidate_margin = candidate_margin_;
  debug_snapshot_.selected_xy_residual = selected_xy_residual_;
  debug_snapshot_.entropy_norm = entropy_norm_;
  debug_snapshot_.max_prob = max_prob_;
  debug_snapshot_.hyp_costs = {hyps[0].cost, hyps[1].cost, hyps[2].cost};
  debug_snapshot_.hyp_probs = {hyps[0].probability, hyps[1].probability,
                               hyps[2].probability};
  debug_snapshot_.center_yaw_est = center_yaw_est_;
  debug_snapshot_.has_observation = true;
  debug_snapshot_.obs_x = selected->x;
  debug_snapshot_.obs_y = selected->y;
  debug_snapshot_.obs_z = selected->z;
  debug_snapshot_.obs_yaw = selected->yaw;
  debug_snapshot_.obs_z_jump = z_audit.z_jump;
  debug_snapshot_.obs_dz_from_audit_center = z_audit.dz_from_center;
  debug_snapshot_.obs_z_audit_costs = z_audit.costs;
  debug_snapshot_.binding_confidence = binding_confidence_;
  debug_snapshot_.switch_event = switch_event_;
  debug_snapshot_.switch_reason = switch_reason_;
  debug_snapshot_.transition_state =
      static_cast<int>(binding_transition_state_);
  debug_snapshot_.z_audit_conflict_count = z_audit_conflict_count_;
  debug_snapshot_.z_audit_confidence = z_audit_confidence_;
  debug_snapshot_.publish_x = publish_position_.x();
  debug_snapshot_.publish_y = publish_position_.y();
  debug_snapshot_.publish_z = publish_position_.z();
  debug_snapshot_.period_confidence = period_confidence_;
  debug_snapshot_.period_update_applied = period_update_applied_;
  debug_snapshot_.period_phase_index = period_phase_index_;
  debug_snapshot_.spin_direction = spin_direction_;
  debug_snapshot_.dz_small_est = dz_small_est_;
  debug_snapshot_.dz_large_est = dz_large_est_;

  increment_frame();
  return true;
}

Eigen::Vector3d OutpostArmorTracker::get_center_position() const {
  return center_position_est_;
}

double OutpostArmorTracker::get_yaw() const {
  // Published TrackedRobot uses armors_offset profile convention where
  // offsets are expressed with a pi-shifted local frame. Keep structured
  // outpost yaw aligned with that convention to avoid 180-deg reconstruction
  // mismatch when consumers recover full armor geometry from (center,yaw,offsets).
  return normalize_angle(center_yaw_est_ + M_PI);
}

std::pair<double, double> OutpostArmorTracker::get_radii() const {
  return {radius_, radius_};
}

ManeuverResult OutpostArmorTracker::assess_maneuver() const {
  const double innov_norm = outpost_ukf_.last_innov_xyz().size() >= 3
                                ? outpost_ukf_.last_innov_xyz().norm()
                                : 0.0;
  return maneuver_detector_.detect(
      outpost_ukf_.last_nis(), innov_norm, outpost_ukf_.last_update_type());
}

bool OutpostArmorTracker::is_ambiguous_single_mode() const {
  return mode_ == TrackMode::AMBIGUOUS_SINGLE_ARMOR;
}

int OutpostArmorTracker::effective_num_armors() const {
  return is_ambiguous_single_mode() ? 1 : 3;
}

int OutpostArmorTracker::selected_panel_id() const { return selected_panel_id_; }

double OutpostArmorTracker::normalized_entropy() const { return entropy_norm_; }

double OutpostArmorTracker::max_panel_probability() const { return max_prob_; }

const OutpostArmorTracker::DebugSnapshot &
OutpostArmorTracker::debug_snapshot() const {
  return debug_snapshot_;
}

double OutpostArmorTracker::confidence_scale() const {
  if (!is_ambiguous_single_mode()) return 1.0;
  return clamp01(config_.outpost.single_mode_confidence_scale);
}

std::vector<geometry_msgs::msg::Pose>
OutpostArmorTracker::build_armors_offset_for_message() const {
  std::vector<geometry_msgs::msg::Pose> offsets;

  if (is_ambiguous_single_mode()) {
    geometry_msgs::msg::Pose pose;
    const double angle = panel_angles_[selected_panel_id_];
    pose.position.x = -radius_ * std::cos(angle);
    pose.position.y = -radius_ * std::sin(angle);
    pose.position.z = z_offsets_[selected_panel_id_];
    tf2::Quaternion q;
    q.setRPY(0.0, kOutpostPitchDown, angle + M_PI);
    pose.orientation = tf2::toMsg(q);
    offsets.push_back(pose);
    return offsets;
  }

  offsets.reserve(3);
  for (int i = 0; i < 3; ++i) {
    const double angle = panel_angles_[i];
    geometry_msgs::msg::Pose pose;
    pose.position.x = -radius_ * std::cos(angle);
    pose.position.y = -radius_ * std::sin(angle);
    pose.position.z = z_offsets_[i];

    tf2::Quaternion q;
    q.setRPY(0.0, kOutpostPitchDown, angle + M_PI);
    pose.orientation = tf2::toMsg(q);
    offsets.push_back(pose);
  }

  return offsets;
}

const ObservationData *OutpostArmorTracker::select_observation(
    const std::vector<ObservationData> &obs) const {
  if (obs.empty()) return nullptr;

  if (config_.outpost.binding_enable_multi_obs && obs.size() > 1) {
    const bool has_history = !center_z_history_.empty();
    const double history_center_z = has_history ? history_center_z_median()
                                                : center_position_est_.z();
    const double predicted_center_z = center_position_est_.z();

    const double w_yaw = std::max(0.0, config_.outpost.weight_yaw);
    const double w_z_state = std::max(0.0, config_.outpost.weight_z_state);
    const double w_z_hist = std::max(0.0, config_.outpost.weight_z_history);

    const ObservationData *best = &obs.front();
    double best_min_cost = std::numeric_limits<double>::infinity();

    for (const auto &o : obs) {
      double obs_min_cost = std::numeric_limits<double>::infinity();
      for (int i = 0; i < 3; ++i) {
        const double center_yaw = normalize_angle(o.yaw - panel_angles_[i]);
        const double center_z = o.z - z_offsets_[i];
        const double yaw_err = angle_abs_diff(center_yaw, center_yaw_est_);
        const double z_state_err = std::abs(center_z - predicted_center_z);
        const double z_hist_err = std::abs(center_z - history_center_z);
        const double cost =
            w_yaw * yaw_err + w_z_state * z_state_err + w_z_hist * z_hist_err;
        obs_min_cost = std::min(obs_min_cost, cost);
      }

      bool better = obs_min_cost < best_min_cost;
      if (!better && std::abs(obs_min_cost - best_min_cost) < 1e-6) {
        if (o.timestamp.has_value() && best->timestamp.has_value()) {
          better = o.timestamp.value() > best->timestamp.value();
        } else if (o.timestamp.has_value() && !best->timestamp.has_value()) {
          better = true;
        } else if (!o.timestamp.has_value() && !best->timestamp.has_value()) {
          better = o.confidence > best->confidence;
        }
      }

      if (better) {
        best = &o;
        best_min_cost = obs_min_cost;
      }
    }
    return best;
  }

  const ObservationData *best = &obs.front();
  for (const auto &o : obs) {
    if (o.timestamp.has_value()) {
      if (!best->timestamp.has_value() ||
          o.timestamp.value() > best->timestamp.value()) {
        best = &o;
        continue;
      }
    }

    if ((!o.timestamp.has_value() && !best->timestamp.has_value()) &&
        (o.confidence > best->confidence)) {
      best = &o;
    }
  }
  return best;
}

int OutpostArmorTracker::infer_panel_id_from_hypotheses(
    const std::array<PanelHypothesis, 3> &hyps) const {
  int best_panel = hyps[0].panel_id;
  double best_cost = hyps[0].cost;
  for (int i = 1; i < 3; ++i) {
    if (hyps[i].cost < best_cost) {
      best_cost = hyps[i].cost;
      best_panel = hyps[i].panel_id;
    }
  }
  return best_panel;
}

OutpostArmorTracker::ZJumpAuditResult
OutpostArmorTracker::infer_panel_id_from_z_jump_audit(
    const ObservationData &obs) {
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  ZJumpAuditResult result;

  if (!z_audit_initialized_ || !std::isfinite(z_audit_center_est_)) {
    // Bootstrap with panel-1 as neutral anchor (middle offset is expected near 0).
    z_audit_center_est_ = obs.z - z_offsets_[1];
    z_audit_prev_obs_z_ = obs.z;
    z_audit_prev_panel_id_ = -1;
    z_audit_initialized_ = true;
    result.dz_from_center = obs.z - z_audit_center_est_;
    return result;
  }

  const bool has_prev = std::isfinite(z_audit_prev_obs_z_);
  const double z_jump = has_prev ? (obs.z - z_audit_prev_obs_z_) : 0.0;
  result.z_jump = has_prev ? z_jump : kNaN;

  constexpr double kWeightLevel = 1.0;
  constexpr double kWeightJump = 2.5;
  constexpr double kWeightCenter = 1.0;
  constexpr double kSwitchPenalty = 0.02;

  int best_panel = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  for (int i = 0; i < 3; ++i) {
    const double center_i = obs.z - z_offsets_[i];
    const double level_err =
        std::abs(obs.z - (z_audit_center_est_ + z_offsets_[i]));

    double jump_err = 0.0;
    if (has_prev) {
      if (z_audit_prev_panel_id_ >= 0) {
        const double expected_jump =
            z_offsets_[i] - z_offsets_[z_audit_prev_panel_id_];
        jump_err = std::abs(z_jump - expected_jump);
      } else {
        // No previous z-id yet: compare against all possible prior panels.
        double min_jump_err = std::numeric_limits<double>::infinity();
        for (int j = 0; j < 3; ++j) {
          const double expected_jump = z_offsets_[i] - z_offsets_[j];
          min_jump_err = std::min(min_jump_err,
                                  std::abs(z_jump - expected_jump));
        }
        jump_err = min_jump_err;
      }
    }

    const double center_err = std::abs(center_i - z_audit_center_est_);
    const double switch_penalty =
        (z_audit_prev_panel_id_ >= 0 && i != z_audit_prev_panel_id_)
            ? kSwitchPenalty
            : 0.0;

    const double cost = kWeightLevel * level_err + kWeightJump * jump_err +
                        kWeightCenter * center_err + switch_penalty;
    result.costs[i] = cost;
    if (cost < best_cost) {
      best_cost = cost;
      best_panel = i;
    }
  }

  if (best_panel >= 0) {
    const double center_best = obs.z - z_offsets_[best_panel];
    constexpr double kAlphaCenter = 0.20;
    z_audit_center_est_ =
        (1.0 - kAlphaCenter) * z_audit_center_est_ + kAlphaCenter * center_best;

    z_audit_prev_obs_z_ = obs.z;
    z_audit_prev_panel_id_ = best_panel;

    result.panel_id = best_panel;
    result.dz_from_center = obs.z - z_audit_center_est_;
  } else {
    result.panel_id = -1;
    result.dz_from_center = kNaN;
  }

  return result;
}

void OutpostArmorTracker::update_periodic_evidence(
    double z_jump, bool allow_model_update) {
  period_update_applied_ = 0;

  const double spin_gate =
      std::max(0.0, config_.outpost.binding_period_min_spin_rate);
  if (std::abs(yaw_rate_est_) >= spin_gate) {
    spin_direction_ = (yaw_rate_est_ >= 0.0) ? 1 : -1;
  }

  if (!std::isfinite(z_jump)) {
    period_phase_index_ = -1;
    period_confidence_ = 0.0;
    return;
  }

  dz_jump_history_.push_back(z_jump);
  const int window = std::max(3, config_.outpost.binding_period_window);
  while (static_cast<int>(dz_jump_history_.size()) > window) {
    dz_jump_history_.pop_front();
  }

  const double abs_jump = std::abs(z_jump);
  const double min_jump =
      std::max(1e-5, config_.outpost.binding_period_update_min_jump);
  if (allow_model_update && abs_jump > min_jump) {
    if (!std::isfinite(dz_small_est_)) {
      dz_small_est_ = abs_jump;
      dz_large_est_ = 2.0 * dz_small_est_;
    } else {
      const double alpha =
          std::clamp(config_.outpost.binding_dz_ema_alpha, 0.01, 1.0);
      if (!std::isfinite(dz_large_est_)) {
        dz_large_est_ = 2.0 * dz_small_est_;
      }
      const bool is_large_jump =
          std::abs(abs_jump - dz_large_est_) < std::abs(abs_jump - dz_small_est_);
      const double target_small = is_large_jump ? (0.5 * abs_jump) : abs_jump;
      dz_small_est_ = (1.0 - alpha) * dz_small_est_ + alpha * target_small;
      dz_large_est_ = 2.0 * dz_small_est_;
    }
    period_update_applied_ = 1;
  }

  if (spin_direction_ == 0 || !std::isfinite(dz_small_est_) ||
      dz_small_est_ < 1e-4 || dz_jump_history_.size() < 3) {
    period_phase_index_ = -1;
    period_confidence_ = 0.0;
    return;
  }

  const auto templ = periodic_template_for_spin();
  const int sample_count =
      std::min(static_cast<int>(dz_jump_history_.size()), window);

  int best_phase = 0;
  double best_conf = -1.0;
  for (int phase = 0; phase < 3; ++phase) {
    const double conf =
        compute_period_confidence_for_phase(phase, templ, sample_count);
    if (conf > best_conf) {
      best_conf = conf;
      best_phase = phase;
    }
  }

  period_phase_index_ = best_phase;
  period_confidence_ = std::clamp(best_conf, 0.0, 1.0);
}

void OutpostArmorTracker::apply_periodic_jump_prior(
    std::array<PanelHypothesis, 3> &hyps, double z_jump) const {
  if (!std::isfinite(z_jump) || bound_panel_id_ < 0 ||
      !std::isfinite(dz_small_est_)) {
    return;
  }

  const double period_conf =
      std::isfinite(period_confidence_) ? period_confidence_ : 0.0;
  const double prior_weight =
      std::max(0.0, config_.outpost.binding_period_weight) *
      std::clamp(period_conf, 0.0, 1.0);
  if (prior_weight <= 0.0) {
    return;
  }

  const double dz_unit = std::max(0.02, dz_small_est_);
  for (int i = 0; i < 3; ++i) {
    const double expected_jump = z_offsets_[i] - z_offsets_[bound_panel_id_];
    const double normalized_err = std::abs(z_jump - expected_jump) / dz_unit;
    hyps[i].cost += prior_weight * normalized_err;
  }
}

std::array<double, 3> OutpostArmorTracker::periodic_template_for_spin() const {
  // Outpost 3-panel periodic dz template:
  //   CW  : [-2, +1, +1] * dz_small
  //   CCW : [+2, -1, -1] * dz_small
  if (spin_direction_ >= 0) {
    return {2.0, -1.0, -1.0};
  }
  return {-2.0, 1.0, 1.0};
}

double OutpostArmorTracker::compute_period_confidence_for_phase(
    int phase, const std::array<double, 3> &templ, int sample_count) const {
  if (!std::isfinite(dz_small_est_) || dz_small_est_ < 1e-4 ||
      sample_count <= 0 ||
      static_cast<int>(dz_jump_history_.size()) < sample_count) {
    return 0.0;
  }

  const int start = static_cast<int>(dz_jump_history_.size()) - sample_count;
  double err_sum = 0.0;
  for (int k = 0; k < sample_count; ++k) {
    const double obs_norm = dz_jump_history_[start + k] / dz_small_est_;
    const int idx = wrap_phase_index(phase + k, 3);
    err_sum += std::abs(obs_norm - templ[idx]);
  }

  const double mean_err = err_sum / static_cast<double>(sample_count);
  return std::exp(-0.65 * mean_err);
}

int OutpostArmorTracker::hypothesis_index_for_panel(
    const std::array<PanelHypothesis, 3> &hyps, int panel_id) const {
  for (int i = 0; i < 3; ++i) {
    if (hyps[i].panel_id == panel_id) {
      return i;
    }
  }
  return 0;
}

double OutpostArmorTracker::compute_same_panel_score(
    const PanelHypothesis &hyp, double predicted_center_z) const {
  const double yaw_gate = std::max(1e-3, config_.outpost.binding_same_panel_yaw_gate);
  const double z_gate = std::max(1e-3, config_.outpost.binding_same_panel_z_gate);
  const double xy_gate = std::max(1e-3, config_.outpost.binding_same_panel_xy_gate);

  const double yaw_err = angle_abs_diff(hyp.center_yaw, center_yaw_est_);
  const double z_err = std::abs(hyp.center_z - predicted_center_z);
  const double xy_err = hyp.xy_residual;

  const double score = 1.0 -
      (0.40 * (yaw_err / yaw_gate) +
       0.35 * (z_err / z_gate) +
       0.25 * (xy_err / xy_gate));
  return clamp01(score);
}

int OutpostArmorTracker::semantic_from_panel(int panel_id) const {
  if (panel_id == 0) return static_cast<int>(HeightSemantic::HIGH);
  if (panel_id == 1) return static_cast<int>(HeightSemantic::MIDDLE);
  if (panel_id == 2) return static_cast<int>(HeightSemantic::LOW);
  return static_cast<int>(HeightSemantic::UNKNOWN);
}

void OutpostArmorTracker::update_binding_state_machine(int candidate_panel,
                                                       double candidate_prob,
                                                       double candidate_margin,
                                                       double same_panel_score,
                                                       double switch_score) {
  switch_event_ = 0;
  switch_reason_ = 0;

  const double min_candidate_prob =
      std::clamp(config_.outpost.binding_min_candidate_prob, 0.0, 1.0);
  const double min_candidate_margin =
      std::clamp(config_.outpost.binding_min_candidate_margin, 0.0, 1.0);
  const double switch_strong_score =
      std::clamp(config_.outpost.binding_switch_strong_score, 0.0, 1.0);

  if (candidate_prob < min_candidate_prob) {
    switch_reason_ = 2;
  } else if (candidate_margin < min_candidate_margin) {
    switch_reason_ = 3;
  }

  if (bound_panel_id_ < 0) {
    if (candidate_panel < 0 || candidate_prob < min_candidate_prob ||
        candidate_margin < min_candidate_margin) {
      binding_transition_state_ = BindingTransitionState::LOCKED;
      transition_candidate_panel_ = -1;
      transition_confirm_count_ = 0;
      return;
    }
    bound_panel_id_ = candidate_panel;
    bound_height_label_ = semantic_from_panel(bound_panel_id_);
    binding_transition_state_ = BindingTransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    return;
  }

  const int confirm_required =
      std::max(1, config_.outpost.binding_transition_confirm_frames);

  const bool candidate_valid =
      candidate_prob >= min_candidate_prob &&
      candidate_margin >= min_candidate_margin;

  if (!candidate_valid) {
    binding_transition_state_ = BindingTransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    return;
  }

  if (binding_transition_state_ == BindingTransitionState::LOCKED) {
    const bool trigger_transition =
        (candidate_panel != bound_panel_id_) &&
        (switch_score > (0.50 + 0.20 * same_panel_score));
    if (trigger_transition) {
      if (confirm_required <= 1) {
        bound_panel_id_ = candidate_panel;
        bound_height_label_ = semantic_from_panel(bound_panel_id_);
        binding_transition_state_ = BindingTransitionState::LOCKED;
        transition_candidate_panel_ = -1;
        transition_confirm_count_ = 0;
        switch_event_ = 1;
        switch_reason_ = 1;
        return;
      }
      binding_transition_state_ = BindingTransitionState::TRANSITION_CANDIDATE;
      transition_candidate_panel_ = candidate_panel;
      transition_confirm_count_ = 1;
    } else {
      transition_candidate_panel_ = -1;
      transition_confirm_count_ = 0;
    }
    return;
  }

  if (candidate_panel == transition_candidate_panel_ &&
      switch_score > switch_strong_score) {
    ++transition_confirm_count_;
  } else if (candidate_panel != bound_panel_id_ &&
             switch_score > std::max(0.60, switch_strong_score)) {
    transition_candidate_panel_ = candidate_panel;
    transition_confirm_count_ = 1;
  } else {
    binding_transition_state_ = BindingTransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    switch_reason_ = 4;
    return;
  }

  if (transition_confirm_count_ >= confirm_required) {
    bound_panel_id_ = transition_candidate_panel_;
    bound_height_label_ = semantic_from_panel(bound_panel_id_);
    binding_transition_state_ = BindingTransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    switch_event_ = 1;
    switch_reason_ = 1;
  }
}

double OutpostArmorTracker::binding_confidence_from_scores(
    double candidate_prob, double candidate_margin,
    double same_panel_score, double switch_score) const {
  const double period_conf =
      std::isfinite(period_confidence_) ? period_confidence_ : 0.0;
  const double consistency_score = std::max(same_panel_score, period_conf);
  double base = clamp01(0.55 * candidate_prob +
                        0.20 * candidate_margin +
                        0.25 * consistency_score);

  if (binding_transition_state_ == BindingTransitionState::TRANSITION_CANDIDATE) {
    base *= std::clamp(1.0 - 0.25 * switch_score, 0.55, 1.0);
  }

  const double floor = std::clamp(config_.outpost.binding_confidence_floor,
                                  0.0, 0.95);
  return floor + (1.0 - floor) * clamp01(base);
}

std::array<OutpostArmorTracker::PanelHypothesis, 3>
OutpostArmorTracker::evaluate_hypotheses(const ObservationData &obs,
                                         double predicted_center_z,
                                         double history_center_z) const {
  std::array<PanelHypothesis, 3> hyps;

  const bool has_history = std::isfinite(history_center_z);
  const double w_yaw = std::max(0.0, config_.outpost.weight_yaw);
  const double w_z_state = std::max(0.0, config_.outpost.weight_z_state);
  const double w_z_hist = std::max(0.0, config_.outpost.weight_z_history);
  const double w_xy = std::max(0.0, config_.outpost.weight_xy_residual);
  const double w_switch = std::max(0.0, config_.outpost.weight_switch_penalty);

  for (int i = 0; i < 3; ++i) {
    PanelHypothesis h;
    h.panel_id = i;
    h.center_yaw = normalize_angle(obs.yaw - panel_angles_[i]);
    h.center_z = obs.z - z_offsets_[i];

    const double yaw_err = angle_abs_diff(h.center_yaw, center_yaw_est_);
    const double z_state_err = std::abs(h.center_z - predicted_center_z);
    const double z_hist_err = has_history ? std::abs(h.center_z - history_center_z)
                                          : 0.0;

    const double predicted_panel_yaw =
        normalize_angle(center_yaw_est_ + panel_angles_[i]);
    const double pred_x =
        center_position_est_.x() + radius_ * std::cos(predicted_panel_yaw);
    const double pred_y =
        center_position_est_.y() + radius_ * std::sin(predicted_panel_yaw);
    const double xy_residual = std::hypot(obs.x - pred_x, obs.y - pred_y);

    const double switch_penalty =
        (bound_panel_id_ >= 0 && i != bound_panel_id_) ? w_switch : 0.0;

    h.yaw_err = yaw_err;
    h.z_state_err = z_state_err;
    h.z_hist_err = z_hist_err;
    h.xy_residual = xy_residual;
    h.switch_penalty = switch_penalty;

    h.cost = w_yaw * yaw_err +
             w_z_state * z_state_err +
             w_z_hist * z_hist_err +
             w_xy * xy_residual +
             switch_penalty;
    hyps[i] = h;
  }

  return hyps;
}

void OutpostArmorTracker::compute_probabilities(
    std::array<PanelHypothesis, 3> &hyps) const {
  const double temp = std::max(1e-3, config_.outpost.softmax_temperature);
  double min_cost = hyps[0].cost;
  for (int i = 1; i < 3; ++i) {
    min_cost = std::min(min_cost, hyps[i].cost);
  }

  double sum = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double scaled = -(hyps[i].cost - min_cost) / temp;
    hyps[i].probability = std::exp(scaled);
    sum += hyps[i].probability;
  }

  sum = std::max(sum, 1e-12);
  for (int i = 0; i < 3; ++i) {
    hyps[i].probability /= sum;
  }
}

void OutpostArmorTracker::update_mode_from_entropy(int best_panel,
                                                   double entropy_norm,
                                                   double max_prob) {
  const double h_enter = std::clamp(config_.outpost.entropy_enter, 0.0, 1.0);
  const double h_exit = std::clamp(config_.outpost.entropy_exit, 0.0, 1.0);
  const double p_enter = std::clamp(config_.outpost.max_prob_enter, 0.0, 1.0);
  const double p_exit = std::clamp(config_.outpost.max_prob_exit, 0.0, 1.0);
  const int stable_required = std::max(1, config_.outpost.stable_frames);

  if (mode_ == TrackMode::STRUCTURED_3_ARMORS) {
    const bool to_ambiguous = (entropy_norm > h_enter) || (max_prob < p_enter);
    if (to_ambiguous) {
      mode_ = TrackMode::AMBIGUOUS_SINGLE_ARMOR;
      stable_counter_ = 0;
    }
    return;
  }

  if (best_panel == last_best_panel_id_) {
    ++stable_counter_;
  } else {
    last_best_panel_id_ = best_panel;
    stable_counter_ = 1;
  }

  const bool to_structured = (entropy_norm < h_exit) && (max_prob > p_exit) &&
                             (stable_counter_ >= stable_required);
  if (to_structured) {
    mode_ = TrackMode::STRUCTURED_3_ARMORS;
  }
}

double OutpostArmorTracker::history_center_z_median() const {
  std::vector<double> values(center_z_history_.begin(), center_z_history_.end());
  return median_of_vector(values);
}

void OutpostArmorTracker::push_center_z_history(double center_z) {
  center_z_history_.push_back(center_z);
  const int window = std::max(1, config_.outpost.z_history_window);
  while (static_cast<int>(center_z_history_.size()) > window) {
    center_z_history_.pop_front();
  }
}

bool OutpostArmorTracker::update_internal_state(const ObservationData &obs,
                                                const PanelHypothesis &best,
                                                double /*dt*/) {
  ObservationData obs_with_panel = obs;
  obs_with_panel.panel_id = best.panel_id;

  // Use fused binding confidence (probability + periodic evidence + transition
  // status) to scale measurement trust.
  const double confidence_floor =
      std::clamp(config_.outpost.binding_confidence_floor, 0.0, 0.95);
  double position_confidence =
      std::clamp(std::max(binding_confidence_, max_prob_), confidence_floor,
                 1.0);
  if (binding_conflict_for_update_) {
    const double conflict_scale =
        std::clamp(config_.outpost.binding_conflict_position_scale, 0.0, 1.0);
    position_confidence =
        std::clamp(position_confidence * conflict_scale, 0.05, 1.0);
  }
  if (!outpost_ukf_.update_with_panel(obs_with_panel, best.panel_id,
                                      position_confidence)) {
    return false;
  }

  apply_motion_constraints_from_config();
  sync_internal_state_from_filter();
  return true;
}

void OutpostArmorTracker::apply_motion_constraints_from_config() {
  auto &x = outpost_ukf_.x();
  const auto idx = outpost_ukf_.state_idx();

  if (config_.outpost.assume_static_center) {
    const double lin_damping =
        std::clamp(config_.outpost.linear_velocity_damping, 0.0, 1.0);
    x(idx.VX()) *= lin_damping;
    x(idx.VY()) *= lin_damping;
    x(idx.VZ()) *= lin_damping;
  }

  const double yaw_damping =
      std::clamp(config_.outpost.yaw_rate_damping, 0.0, 1.0);
  x(idx.DELTA_RATE()) *= yaw_damping;

  Eigen::Vector3d vel(x(idx.VX()), x(idx.VY()), x(idx.VZ()));
  const double max_center_speed =
      std::max(0.01, config_.outpost.max_center_speed);
  const double speed_norm = vel.norm();
  if (speed_norm > max_center_speed) {
    const double ratio = max_center_speed / speed_norm;
    x(idx.VX()) *= ratio;
    x(idx.VY()) *= ratio;
    x(idx.VZ()) *= ratio;
  }

  const double max_yaw_rate = std::max(0.01, config_.outpost.max_yaw_rate);
  x(idx.DELTA_RATE()) =
      std::clamp(x(idx.DELTA_RATE()), -max_yaw_rate, max_yaw_rate);
}

void OutpostArmorTracker::sync_internal_state_from_filter() {
  const auto &x = outpost_ukf_.x();
  const auto idx = outpost_ukf_.state_idx();
  center_position_est_ = outpost_ukf_.get_center_position();
  center_velocity_est_ = Eigen::Vector3d(x(idx.VX()), x(idx.VY()), x(idx.VZ()));
  center_yaw_est_ = outpost_ukf_.get_yaw();
  yaw_rate_est_ = x(idx.DELTA_RATE());
}

void OutpostArmorTracker::update_publish_state() {
  if (is_ambiguous_single_mode()) {
    const double theta = center_yaw_est_ + panel_angles_[selected_panel_id_];
    publish_position_ = center_position_est_ +
                        Eigen::Vector3d(radius_ * std::cos(theta),
                                        radius_ * std::sin(theta),
                                        z_offsets_[selected_panel_id_]);

    const Eigen::Vector3d tangential(
        -yaw_rate_est_ * radius_ * std::sin(theta),
        yaw_rate_est_ * radius_ * std::cos(theta), 0.0);
    publish_velocity_ = center_velocity_est_ + tangential;
    return;
  }

  publish_position_ = center_position_est_;
  publish_velocity_ = center_velocity_est_;
}

BinderConfig OutpostArmorTracker::build_binder_config_from_outpost() const {
  BinderConfig cfg;
  cfg.confirm_frames = std::max(1, config_.outpost.binding_transition_confirm_frames);
  cfg.lock_new_hold_frames = 2;
  cfg.force_rebind_bad_frames = std::max(1, config_.outpost.z_audit_rebind_confirm_frames);
  cfg.pending_window_frames = std::max(cfg.confirm_frames + 1, 4);
  cfg.post_jump_min_confidence =
      std::clamp(config_.outpost.binding_min_candidate_prob, 0.35, 0.70);
  cfg.confidence_floor = std::clamp(config_.outpost.binding_confidence_floor, 0.0, 0.95);

  cfg.z_jump_min = std::max(0.0, config_.outpost.binding_period_update_min_jump);
  cfg.dz_match_tolerance = 0.03;
  cfg.dz_gate = 0.010;
  cfg.yaw_err_gate = std::max(1e-3, config_.outpost.binding_same_panel_yaw_gate);
  cfg.cost_margin_min = std::clamp(config_.outpost.binding_min_candidate_margin, 0.0, 1.0);
  cfg.dz_ema_alpha = std::clamp(config_.outpost.binding_dz_ema_alpha, 0.01, 1.0);

  cfg.periodic_enable = true;
  cfg.periodic_window = std::max(3, config_.outpost.binding_period_window);
  cfg.periodic_weight = std::max(0.0, config_.outpost.binding_period_weight);
  cfg.periodic_min_spin_rate = std::max(0.0, config_.outpost.binding_period_min_spin_rate);
  cfg.periodic_update_min_jump = std::max(1e-5, config_.outpost.binding_period_update_min_jump);

  cfg.min_candidate_prob = std::clamp(config_.outpost.binding_min_candidate_prob, 0.0, 1.0);
  cfg.min_candidate_margin = std::clamp(config_.outpost.binding_min_candidate_margin, 0.0, 1.0);
  cfg.switch_strong_score = std::clamp(config_.outpost.binding_switch_strong_score, 0.0, 1.0);
  cfg.single_obs_history_window = std::max(3, config_.outpost.z_history_window);
  cfg.dual_obs_enable = config_.outpost.binding_enable_multi_obs;

  cfg.scorer_enable = true;
  cfg.same_panel_yaw_gate = std::max(1e-3, config_.outpost.binding_same_panel_yaw_gate);
  cfg.same_panel_z_gate = std::max(1e-3, config_.outpost.binding_same_panel_z_gate);
  cfg.same_panel_xy_gate = std::max(1e-3, config_.outpost.binding_same_panel_xy_gate);

  cfg.z_audit_rebind_enable = config_.outpost.z_audit_rebind_enable;
  cfg.z_audit_rebind_confirm_frames =
      std::max(1, config_.outpost.z_audit_rebind_confirm_frames);
  cfg.z_audit_rebind_min_confidence =
      std::clamp(config_.outpost.z_audit_rebind_min_confidence, 0.0, 1.0);
  cfg.z_audit_rebind_min_jump = std::max(0.0, config_.outpost.z_audit_rebind_min_jump);
  return cfg;
}

}  // namespace fyt::auto_aim
