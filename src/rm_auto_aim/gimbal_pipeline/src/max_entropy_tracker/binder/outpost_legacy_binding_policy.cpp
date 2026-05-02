// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/policy/outpost_legacy_binding_policy.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::binder {

namespace {

constexpr double kLog3 = 1.0986122886681098;

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

double angle_abs_diff(double a, double b) {
  return std::abs(fyt::auto_aim::normalize_angle(a - b));
}

int wrap_phase_index(int phase, int modulo) {
  const int m = std::max(1, modulo);
  int r = phase % m;
  if (r < 0) r += m;
  return r;
}

}  // namespace

OutpostLegacyBindingPolicy::OutpostLegacyBindingPolicy(
    const UnifiedConfig & config, const RobotBindingProfile & profile)
    : config_(config), profile_(profile) {
  radius_ = std::max(0.05, config_.outpost.radius);
  z_offsets_ = {config_.outpost.z_offset_0, config_.outpost.z_offset_1,
                config_.outpost.z_offset_2};
  const double raw_step = (config_.outpost.panel_angle_step > 1e-6)
                              ? config_.outpost.panel_angle_step
                              : (2.0 * M_PI / 3.0);
  const double step = std::abs(raw_step);
  panel_angles_ = {0.0, step, -step};
}

void OutpostLegacyBindingPolicy::reset(int init_panel_id,
                                       std::optional<double> obs_z) {
  selected_panel_id_ = std::clamp(init_panel_id, 0, 2);
  bound_panel_id_ = -1;
  bound_height_label_ = HeightLabel::UNKNOWN;
  transition_state_ = TransitionState::LOCKED;
  transition_candidate_panel_ = -1;
  transition_confirm_count_ = 0;
  switch_event_ = 0;
  switch_reason_ = 0;
  binding_conflict_for_update_ = false;
  binding_confidence_ = 1.0 / 3.0;
  entropy_norm_ = 1.0;

  z_audit_initialized_ = false;
  z_audit_center_est_ = std::numeric_limits<double>::quiet_NaN();
  z_audit_prev_obs_z_ = std::numeric_limits<double>::quiet_NaN();
  z_audit_prev_panel_id_ = -1;
  z_audit_conflict_count_ = 0;
  z_audit_confidence_ = 0.0;

  dz_jump_history_.clear();
  dz_small_est_ = std::numeric_limits<double>::quiet_NaN();
  dz_large_est_ = std::numeric_limits<double>::quiet_NaN();
  period_confidence_ = 0.0;
  period_phase_index_ = -1;
  spin_direction_ = 0;
  center_z_history_.clear();
  if (obs_z.has_value() && std::isfinite(obs_z.value())) {
    push_center_z_history(obs_z.value() - z_offsets_[selected_panel_id_]);
  }
}

OutpostLegacyBindingPolicy::ZJumpAuditResult
OutpostLegacyBindingPolicy::infer_panel_id_from_z_jump_audit(
    const ObservationData & obs) {
  const double kNaN = std::numeric_limits<double>::quiet_NaN();
  ZJumpAuditResult result;

  if (!z_audit_initialized_ || !std::isfinite(z_audit_center_est_)) {
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
        double min_jump_err = std::numeric_limits<double>::infinity();
        for (int j = 0; j < 3; ++j) {
          const double expected_jump = z_offsets_[i] - z_offsets_[j];
          min_jump_err =
              std::min(min_jump_err, std::abs(z_jump - expected_jump));
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

std::array<BindingHypothesis, 3>
OutpostLegacyBindingPolicy::evaluate_hypotheses(
    const ObservationData & obs,
    const Eigen::Vector3d & predicted_center_pos,
    double predicted_center_yaw,
    double history_center_z) const {
  std::array<BindingHypothesis, 3> hyps;

  const bool has_history = std::isfinite(history_center_z);
  const double w_yaw = std::max(0.0, config_.outpost.weight_yaw);
  const double w_z_state = std::max(0.0, config_.outpost.weight_z_state);
  const double w_z_hist = std::max(0.0, config_.outpost.weight_z_history);
  const double w_xy = std::max(0.0, config_.outpost.weight_xy_residual);
  const double w_switch = std::max(0.0, config_.outpost.weight_switch_penalty);

  for (int i = 0; i < 3; ++i) {
    BindingHypothesis h;
    h.panel_id = i;
    h.center_yaw =
        fyt::auto_aim::normalize_angle(obs.yaw - panel_angles_[i]);
    h.center_z = obs.z - z_offsets_[i];

    h.yaw_err = angle_abs_diff(h.center_yaw, predicted_center_yaw);
    h.z_state_err = std::abs(h.center_z - predicted_center_pos.z());
    h.z_hist_err =
        has_history ? std::abs(h.center_z - history_center_z) : 0.0;

    const double predicted_panel_yaw =
        fyt::auto_aim::normalize_angle(predicted_center_yaw + panel_angles_[i]);
    const double pred_x =
        predicted_center_pos.x() + radius_ * std::cos(predicted_panel_yaw);
    const double pred_y =
        predicted_center_pos.y() + radius_ * std::sin(predicted_panel_yaw);
    h.xy_residual = std::hypot(obs.x - pred_x, obs.y - pred_y);

    h.switch_penalty =
        (bound_panel_id_ >= 0 && i != bound_panel_id_) ? w_switch : 0.0;
    h.cost = w_yaw * h.yaw_err + w_z_state * h.z_state_err +
             w_z_hist * h.z_hist_err + w_xy * h.xy_residual +
             h.switch_penalty;
    hyps[i] = h;
  }

  return hyps;
}

void OutpostLegacyBindingPolicy::compute_probabilities(
    std::array<BindingHypothesis, 3> & hyps) const {
  const double temp = std::max(1e-3, config_.outpost.softmax_temperature);
  double min_cost = hyps[0].cost;
  for (int i = 1; i < 3; ++i) {
    min_cost = std::min(min_cost, hyps[i].cost);
  }

  double sum = 0.0;
  for (auto & h : hyps) {
    const double scaled = -(h.cost - min_cost) / temp;
    h.probability = std::exp(scaled);
    sum += h.probability;
  }
  sum = std::max(sum, 1e-12);
  for (auto & h : hyps) {
    h.probability /= sum;
  }
}

void OutpostLegacyBindingPolicy::update_periodic_evidence(
    double z_jump, bool allow_model_update, double yaw_rate_est) {
  const double spin_gate =
      std::max(0.0, config_.outpost.binding_period_min_spin_rate);
  if (std::abs(yaw_rate_est) >= spin_gate) {
    spin_direction_ = (yaw_rate_est >= 0.0) ? 1 : -1;
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
          std::abs(abs_jump - dz_large_est_) <
          std::abs(abs_jump - dz_small_est_);
      const double target_small = is_large_jump ? (0.5 * abs_jump) : abs_jump;
      dz_small_est_ = (1.0 - alpha) * dz_small_est_ + alpha * target_small;
      dz_large_est_ = 2.0 * dz_small_est_;
    }
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
  period_confidence_ = clamp01(best_conf);
}

void OutpostLegacyBindingPolicy::apply_periodic_jump_prior(
    std::array<BindingHypothesis, 3> & hyps, double z_jump) const {
  if (!std::isfinite(z_jump) || bound_panel_id_ < 0 ||
      !std::isfinite(dz_small_est_)) {
    return;
  }

  const double prior_weight =
      std::max(0.0, config_.outpost.binding_period_weight) *
      clamp01(period_confidence_);
  if (prior_weight <= 0.0) return;

  const double dz_unit = std::max(0.02, dz_small_est_);
  for (int i = 0; i < 3; ++i) {
    const double expected_jump = z_offsets_[i] - z_offsets_[bound_panel_id_];
    const double normalized_err = std::abs(z_jump - expected_jump) / dz_unit;
    hyps[i].cost += prior_weight * normalized_err;
  }
}

std::array<double, 3>
OutpostLegacyBindingPolicy::periodic_template_for_spin() const {
  if (spin_direction_ >= 0) {
    return {2.0, -1.0, -1.0};
  }
  return {-2.0, 1.0, 1.0};
}

double OutpostLegacyBindingPolicy::compute_period_confidence_for_phase(
    int phase, const std::array<double, 3> & templ, int sample_count) const {
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

int OutpostLegacyBindingPolicy::hypothesis_index_for_panel(
    const std::array<BindingHypothesis, 3> & hyps, int panel_id) const {
  for (int i = 0; i < 3; ++i) {
    if (hyps[i].panel_id == panel_id) return i;
  }
  return 0;
}

double OutpostLegacyBindingPolicy::compute_same_panel_score(
    const BindingHypothesis & hyp, double predicted_center_z) const {
  const double yaw_gate =
      std::max(1e-3, config_.outpost.binding_same_panel_yaw_gate);
  const double z_gate =
      std::max(1e-3, config_.outpost.binding_same_panel_z_gate);
  const double xy_gate =
      std::max(1e-3, config_.outpost.binding_same_panel_xy_gate);

  const double yaw_err = hyp.yaw_err;
  const double z_err = std::abs(hyp.center_z - predicted_center_z);
  const double xy_err = hyp.xy_residual;

  const double score =
      1.0 - (0.40 * (yaw_err / yaw_gate) + 0.35 * (z_err / z_gate) +
             0.25 * (xy_err / xy_gate));
  return clamp01(score);
}

void OutpostLegacyBindingPolicy::update_binding_state_machine(
    int candidate_panel, double candidate_prob, double candidate_margin,
    double same_panel_score, double switch_score) {
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
      transition_state_ = TransitionState::LOCKED;
      transition_candidate_panel_ = -1;
      transition_confirm_count_ = 0;
      return;
    }
    bound_panel_id_ = candidate_panel;
    bound_height_label_ = height_label_from_panel(bound_panel_id_);
    transition_state_ = TransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    return;
  }

  const int confirm_required =
      std::max(1, config_.outpost.binding_transition_confirm_frames);
  const bool candidate_valid = candidate_prob >= min_candidate_prob &&
                               candidate_margin >= min_candidate_margin;
  if (!candidate_valid) {
    transition_state_ = TransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    return;
  }

  if (transition_state_ == TransitionState::LOCKED) {
    const bool trigger_transition =
        (candidate_panel != bound_panel_id_) &&
        (switch_score > (0.50 + 0.20 * same_panel_score));
    if (trigger_transition) {
      if (confirm_required <= 1) {
        bound_panel_id_ = candidate_panel;
        bound_height_label_ = height_label_from_panel(bound_panel_id_);
        transition_state_ = TransitionState::LOCKED;
        transition_candidate_panel_ = -1;
        transition_confirm_count_ = 0;
        switch_event_ = 1;
        switch_reason_ = 1;
        return;
      }
      transition_state_ = TransitionState::TRANSITION_CANDIDATE;
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
    transition_state_ = TransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    switch_reason_ = 4;
    return;
  }

  if (transition_confirm_count_ >= confirm_required) {
    bound_panel_id_ = transition_candidate_panel_;
    bound_height_label_ = height_label_from_panel(bound_panel_id_);
    transition_state_ = TransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    switch_event_ = 1;
    switch_reason_ = 1;
  }
}

double OutpostLegacyBindingPolicy::binding_confidence_from_scores(
    double candidate_prob, double candidate_margin, double same_panel_score,
    double switch_score) const {
  const double consistency_score = std::max(same_panel_score, period_confidence_);
  double base = clamp01(0.55 * candidate_prob + 0.20 * candidate_margin +
                        0.25 * consistency_score);

  if (transition_state_ == TransitionState::TRANSITION_CANDIDATE) {
    base *= std::clamp(1.0 - 0.25 * switch_score, 0.55, 1.0);
  }

  const double floor =
      std::clamp(config_.outpost.binding_confidence_floor, 0.0, 0.95);
  return floor + (1.0 - floor) * clamp01(base);
}

double OutpostLegacyBindingPolicy::history_center_z_median() const {
  if (center_z_history_.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::vector<double> values(center_z_history_.begin(), center_z_history_.end());
  const auto mid = values.begin() + static_cast<long>(values.size() / 2);
  std::nth_element(values.begin(), mid, values.end());
  return *mid;
}

void OutpostLegacyBindingPolicy::push_center_z_history(double center_z) {
  if (!std::isfinite(center_z)) return;
  center_z_history_.push_back(center_z);
  const int window = std::max(1, config_.outpost.z_history_window);
  while (static_cast<int>(center_z_history_.size()) > window) {
    center_z_history_.pop_front();
  }
}

HeightLabel OutpostLegacyBindingPolicy::height_label_from_panel(int panel_id) const {
  if (panel_id == 0) return HeightLabel::UPPER;
  if (panel_id == 1) return HeightLabel::MIDDLE;
  if (panel_id == 2) return HeightLabel::LOWER;
  return HeightLabel::UNKNOWN;
}

OutpostLegacyBindingOutput OutpostLegacyBindingPolicy::step(
    const OutpostLegacyBindingInput & input) {
  OutpostLegacyBindingOutput out;
  if (input.obs == nullptr) {
    out.binder_output.selected_id = selected_panel_id_;
    out.binder_output.bound_id = bound_panel_id_;
    out.binder_output.pending_id = transition_candidate_panel_;
    out.binder_output.height_label = height_label_from_panel(selected_panel_id_);
    out.binder_output.binding_confidence = binding_confidence_;
    return out;
  }

  const ObservationData & obs = *input.obs;
  const auto z_audit = infer_panel_id_from_z_jump_audit(obs);
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

  const double hist_center_z = history_center_z_median();
  auto hyps = evaluate_hypotheses(obs, input.predicted_center_pos,
                                  input.predicted_center_yaw, hist_center_z);

  if (input.ambiguous_mode && z_audit.panel_id >= 0) {
    double best_audit = z_audit.costs[0];
    for (int i = 1; i < 3; ++i) {
      best_audit = std::min(best_audit, z_audit.costs[i]);
    }

    const double audit_conf = z_audit_confidence_;
    double prior_weight = 0.35 * audit_conf;
    if (z_audit.panel_id == 2) {
      prior_weight += 4.0 * audit_conf;
    }
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

  update_periodic_evidence(z_audit.z_jump, allow_period_update,
                           input.yaw_rate_est);
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

  const int candidate_panel = hyps[best_idx].panel_id;
  const double candidate_prob = hyps[best_idx].probability;
  const double candidate_margin =
      std::max(0.0, hyps[best_idx].probability - hyps[second_idx].probability);

  const int current_idx =
      (bound_panel_id_ >= 0) ? hypothesis_index_for_panel(hyps, bound_panel_id_)
                             : best_idx;
  const double same_panel_score =
      compute_same_panel_score(hyps[current_idx], input.predicted_center_pos.z());
  const double switch_base =
      (bound_panel_id_ >= 0 && candidate_panel != bound_panel_id_)
          ? candidate_prob
          : 0.0;
  const double z_audit_switch_support =
      (z_audit.panel_id >= 0 && candidate_panel == z_audit.panel_id &&
       candidate_panel != bound_panel_id_)
          ? 1.0
          : 0.0;
  const double switch_score =
      clamp01(0.50 * switch_base + 0.20 * period_confidence_ +
              0.10 * z_audit_switch_support + 0.20 * candidate_margin);

  update_binding_state_machine(candidate_panel, candidate_prob, candidate_margin,
                               same_panel_score, switch_score);

  const bool z_audit_conflicts =
      config_.outpost.z_audit_rebind_enable && z_audit.panel_id >= 0 &&
      bound_panel_id_ >= 0 && z_audit.panel_id != bound_panel_id_;
  const double min_rebind_conf =
      std::clamp(config_.outpost.z_audit_rebind_min_confidence, 0.0, 1.0);
  const double min_rebind_jump =
      std::max(0.0, config_.outpost.z_audit_rebind_min_jump);
  const bool z_audit_has_jump =
      std::isfinite(z_audit.z_jump) &&
      std::abs(z_audit.z_jump) >= min_rebind_jump;
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
    bound_height_label_ = height_label_from_panel(bound_panel_id_);
    transition_state_ = TransitionState::LOCKED;
    transition_candidate_panel_ = -1;
    transition_confirm_count_ = 0;
    z_audit_conflict_count_ = 0;
    switch_event_ = 1;
    switch_reason_ = 5;
  }

  if (bound_panel_id_ >= 0) {
    if (transition_state_ == TransitionState::TRANSITION_CANDIDATE &&
        transition_candidate_panel_ >= 0) {
      selected_panel_id_ = transition_candidate_panel_;
    } else {
      selected_panel_id_ = bound_panel_id_;
    }
  } else {
    selected_panel_id_ = candidate_panel;
  }
  bound_height_label_ =
      (bound_panel_id_ >= 0) ? height_label_from_panel(bound_panel_id_)
                             : height_label_from_panel(selected_panel_id_);

  const int selected_idx = hypothesis_index_for_panel(hyps, selected_panel_id_);
  const double selected_panel_score =
      compute_same_panel_score(hyps[selected_idx], input.predicted_center_pos.z());
  binding_conflict_for_update_ =
      (candidate_panel >= 0 && candidate_panel != selected_panel_id_) ||
      (z_audit.panel_id >= 0 && z_audit.panel_id != selected_panel_id_);
  binding_confidence_ =
      binding_confidence_from_scores(hyps[selected_idx].probability,
                                     candidate_margin, selected_panel_score,
                                     switch_score);

  double entropy = 0.0;
  for (const auto & h : hyps) {
    const double p = std::max(h.probability, 1e-12);
    entropy -= p * std::log(p);
  }
  const double entropy_norm = std::clamp(entropy / kLog3, 0.0, 1.0);
  entropy_norm_ = entropy_norm;
  push_center_z_history(hyps[selected_idx].center_z);

  out.candidate_id = candidate_panel;
  out.candidate_prob = candidate_prob;
  out.candidate_margin = candidate_margin;
  out.max_prob = hyps[selected_idx].probability;
  out.entropy_norm = entropy_norm;
  out.selected_xy_residual = hyps[selected_idx].xy_residual;
  out.hypotheses = hyps;
  out.z_audit_panel_id = z_audit.panel_id;
  out.z_audit_confidence = z_audit_confidence_;
  out.z_jump = z_audit.z_jump;
  out.dz_from_center = z_audit.dz_from_center;
  out.z_audit_costs = z_audit.costs;
  out.period_confidence = period_confidence_;
  out.period_phase = period_phase_index_;
  out.spin_direction = spin_direction_;
  out.dz_small_est = dz_small_est_;
  out.dz_large_est = dz_large_est_;
  out.transition_state =
      (transition_state_ == TransitionState::TRANSITION_CANDIDATE) ? 1 : 0;
  out.transition_candidate_id = transition_candidate_panel_;

  out.binder_output.selected_id = selected_panel_id_;
  out.binder_output.bound_id = bound_panel_id_;
  out.binder_output.pending_id = transition_candidate_panel_;
  out.binder_output.height_label = height_label_from_panel(selected_panel_id_);
  out.binder_output.fsm_state =
      (transition_state_ == TransitionState::TRANSITION_CANDIDATE)
          ? BindingFSMState::PENDING_SWITCH
          : BindingFSMState::LOCKED;
  out.binder_output.action =
      switch_event_ ? BindingAction::SWITCH
                    : (out.transition_state ? BindingAction::PENDING
                                            : BindingAction::HOLD);
  out.binder_output.switch_occurred = switch_event_ != 0;
  out.binder_output.switch_reason = switch_reason_;
  out.binder_output.binding_confidence = binding_confidence_;
  out.binder_output.binding_conflict_for_update = binding_conflict_for_update_;
  out.binder_output.same_panel_score = selected_panel_score;
  out.binder_output.switch_score = switch_score;

  return out;
}

}  // namespace fyt::auto_aim::binder
