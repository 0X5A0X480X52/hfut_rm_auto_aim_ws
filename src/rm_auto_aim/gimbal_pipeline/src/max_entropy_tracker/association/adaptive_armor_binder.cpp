// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/association/adaptive_armor_binder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace fyt::auto_aim {

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

int wrap_mod(int value, int modulo) {
  const int m = std::max(1, modulo);
  const int r = value % m;
  return (r < 0) ? (r + m) : r;
}

}  // namespace

AdaptiveArmorBinder::AdaptiveArmorBinder(const Config &config)
    : config_(config) {
  if (config_.n_panels < 2) {
    throw std::invalid_argument(
        "AdaptiveArmorBinder: n_panels must be >= 2");
  }
  if (config_.z_offsets.size() != static_cast<size_t>(config_.n_panels)) {
    throw std::invalid_argument(
        "AdaptiveArmorBinder: z_offsets size must match n_panels");
  }
}

void AdaptiveArmorBinder::reset(int init_panel_id, HeightLabel init_label,
                                std::optional<double> obs_z,
                                std::optional<double> obs_time) {
  bound_panel_id_ = init_panel_id;
  bound_height_label_ =
      (init_label == HeightLabel::UNKNOWN)
          ? default_label_for_panel(init_panel_id)
          : init_label;
  bound_confidence_ = 0.5;
  binding_state_ = BindingState::LOCKED;
  transition_candidate_ = -1;
  transition_confirm_count_ = 0;
  cooldown_remaining_ = 0;
  last_panel_id_ = init_panel_id;
  last_obs_z_ = obs_z;
  last_obs_time_ = obs_time;
  z_jump_history_.clear();
  dz_jump_est_ = std::numeric_limits<double>::quiet_NaN();

  // Periodic evidence reset
  dz_periodic_history_.clear();
  dz_small_est_ = std::numeric_limits<double>::quiet_NaN();
  dz_large_est_ = std::numeric_limits<double>::quiet_NaN();
  period_update_applied_ = 0;
  period_phase_index_ = -1;
  period_confidence_ = std::numeric_limits<double>::quiet_NaN();
  spin_direction_ = 0;
}

/* =================================================================== */
/*  PROXIMITY strategy (AdaptiveArmorTracker style)                     */
/* =================================================================== */

bool AdaptiveArmorBinder::evaluate_proximity_gates(
    int candidate_panel, double z_jump, bool has_z_jump,
    const AssociationDiagnostics &diag) const {
  const int raw_diff = std::abs(candidate_panel - bound_panel_id_);
  const int n = config_.n_panels;
  const bool adjacent_panel = (raw_diff == 1 || raw_diff == n - 1);

  const bool jump_mag_ok =
      has_z_jump &&
      (std::abs(z_jump) >= std::max(0.0, config_.z_jump_min));

  bool dz_match_ok = true;
  if (jump_mag_ok && std::isfinite(dz_jump_est_)) {
    dz_match_ok =
        std::abs(std::abs(z_jump) - dz_jump_est_) <=
        std::max(0.0, config_.dz_match_tolerance);
  }

  const double yaw_err =
      std::isfinite(diag.selected_yaw_err) ? diag.selected_yaw_err : 1e9;
  const bool yaw_ok =
      yaw_err <= std::max(1e-3, config_.yaw_err_gate);

  const double margin =
      std::isfinite(diag.cost_margin) ? diag.cost_margin : 0.0;
  const bool margin_ok =
      margin >= std::max(0.0, config_.cost_margin_min);

  return adjacent_panel && jump_mag_ok && dz_match_ok && yaw_ok && margin_ok;
}

bool AdaptiveArmorBinder::bind_proximity(
    double obs_z, double /*obs_yaw*/, int candidate_panel,
    const AssociationDiagnostics &diag, HeightLabel candidate_label,
    double candidate_h_conf, BindResult *result) {
  if (result == nullptr) return false;

  if (bound_panel_id_ < 0) {
    reset(candidate_panel, candidate_label, obs_z, std::nullopt);
  }

  const bool has_z_jump = last_obs_z_.has_value();
  const double z_jump = has_z_jump ? (obs_z - last_obs_z_.value()) : 0.0;

  const bool gate_passed =
      evaluate_proximity_gates(candidate_panel, z_jump, has_z_jump, diag);

  // Cool-down tick
  if (cooldown_remaining_ > 0) --cooldown_remaining_;

  const bool gate_open = (cooldown_remaining_ == 0 && gate_passed);

  const int confirm_required = std::max(1, config_.confirm_frames);
  const bool switch_confirmed =
      attempt_transition(candidate_panel, gate_open, confirm_required);

  if (switch_confirmed) {
    bound_panel_id_ = candidate_panel;

    // Resolve height label from jump direction
    HeightLabel fallback = default_label_for_panel(bound_panel_id_);
    if (has_z_jump) {
      const double dz_gate = std::max(0.0, config_.dz_gate);
      if (z_jump > dz_gate) {
        bound_height_label_ = HeightLabel::UPPER;
      } else if (z_jump < -dz_gate) {
        bound_height_label_ = HeightLabel::LOWER;
      } else {
        bound_height_label_ = fallback;
      }
    }
    if (bound_height_label_ == HeightLabel::UNKNOWN) {
      bound_height_label_ =
          (candidate_label == HeightLabel::UNKNOWN) ? fallback : candidate_label;
    }

    update_jump_statistics(z_jump, true);
    cooldown_remaining_ = std::max(0, config_.cooldown_frames);
  }

  // Ensure bound label is never UNKNOWN
  if (bound_height_label_ == HeightLabel::UNKNOWN) {
    bound_height_label_ =
        (candidate_label == HeightLabel::UNKNOWN)
            ? default_label_for_panel(bound_panel_id_)
            : candidate_label;
  }

  bound_confidence_ = compute_proximity_confidence(diag, gate_passed);

  result->bound_panel_id = bound_panel_id_;
  result->bound_height_label = bound_height_label_;
  result->bound_confidence = bound_confidence_;
  result->switch_occurred = switch_confirmed;
  result->switch_reason = switch_confirmed ? 1 : 0;

  last_obs_z_ = obs_z;
  last_panel_id_ = candidate_panel;
  return switch_confirmed;
}

/* =================================================================== */
/*  COST strategy (OutpostArmorTracker style)                           */
/* =================================================================== */

bool AdaptiveArmorBinder::bind_cost(
    const std::vector<HypothesisScore> &hypotheses,
    double predicted_center_z, double center_yaw_est,
    BindResult *result) {
  if (result == nullptr) return false;
  if (hypotheses.empty()) return false;

  // Find best hypothesis
  const auto *best = &hypotheses[0];
  double second_prob = 0.0;
  for (const auto &h : hypotheses) {
    if (h.probability > best->probability) best = &h;
  }
  for (const auto &h : hypotheses) {
    if (&h != best && h.probability > second_prob) second_prob = h.probability;
  }

  const double candidate_prob = best->probability;
  const double candidate_margin = candidate_prob - second_prob;

  const double same_panel_score =
      compute_same_panel_score(*best, predicted_center_z, center_yaw_est);

  const double switch_score =
      1.0 - same_panel_score;  // simplified switch score

  result->switch_occurred = false;
  result->switch_reason = 0;

  const double min_prob =
      std::clamp(config_.min_candidate_prob, 0.0, 1.0);
  const double min_margin =
      std::clamp(config_.min_candidate_margin, 0.0, 1.0);
  const double strong_score =
      std::clamp(config_.switch_strong_score, 0.0, 1.0);

  if (candidate_prob < min_prob) {
    result->switch_reason = 2;
  } else if (candidate_margin < min_margin) {
    result->switch_reason = 3;
  }

  // First-time binding
  if (bound_panel_id_ < 0) {
    bound_panel_id_ = best->panel_id;
    bound_height_label_ = default_label_for_panel(bound_panel_id_);
    binding_state_ = BindingState::LOCKED;
    transition_candidate_ = -1;
    transition_confirm_count_ = 0;
    result->bound_panel_id = bound_panel_id_;
    result->bound_height_label = bound_height_label_;
    result->bound_confidence = candidate_prob;
    return false;
  }

  const int confirm_required = std::max(1, config_.confirm_frames);
  const bool candidate_valid =
      candidate_prob >= min_prob && candidate_margin >= min_margin;

  if (!candidate_valid) {
    binding_state_ = BindingState::LOCKED;
    transition_candidate_ = -1;
    transition_confirm_count_ = 0;
    result->bound_panel_id = bound_panel_id_;
    result->bound_height_label = bound_height_label_;
    result->bound_confidence =
        compute_cost_confidence(candidate_prob, candidate_margin,
                                same_panel_score, switch_score);
    return false;
  }

  if (binding_state_ == BindingState::LOCKED) {
    const bool trigger_transition =
        (best->panel_id != bound_panel_id_) &&
        (switch_score > (0.50 + 0.20 * same_panel_score));
    if (trigger_transition) {
      if (confirm_required <= 1) {
        bound_panel_id_ = best->panel_id;
        bound_height_label_ = default_label_for_panel(bound_panel_id_);
        binding_state_ = BindingState::LOCKED;
        transition_candidate_ = -1;
        transition_confirm_count_ = 0;
        result->switch_occurred = true;
        result->switch_reason = 1;
      } else {
        binding_state_ = BindingState::TRANSITION_CANDIDATE;
        transition_candidate_ = best->panel_id;
        transition_confirm_count_ = 1;
      }
    } else {
      transition_candidate_ = -1;
      transition_confirm_count_ = 0;
    }
    result->bound_panel_id = bound_panel_id_;
    result->bound_height_label = bound_height_label_;
    result->bound_confidence =
        compute_cost_confidence(candidate_prob, candidate_margin,
                                same_panel_score, switch_score);
    return result->switch_occurred;
  }

  // Already in TRANSITION_CANDIDATE state
  if (best->panel_id == transition_candidate_ &&
      switch_score > strong_score) {
    ++transition_confirm_count_;
  } else if (best->panel_id != bound_panel_id_ &&
             switch_score > std::max(0.60, strong_score)) {
    transition_candidate_ = best->panel_id;
    transition_confirm_count_ = 1;
  } else {
    binding_state_ = BindingState::LOCKED;
    transition_candidate_ = -1;
    transition_confirm_count_ = 0;
    result->switch_reason = 4;
    result->bound_panel_id = bound_panel_id_;
    result->bound_height_label = bound_height_label_;
    result->bound_confidence =
        compute_cost_confidence(candidate_prob, candidate_margin,
                                same_panel_score, switch_score);
    return false;
  }

  if (transition_confirm_count_ >= confirm_required) {
    bound_panel_id_ = transition_candidate_;
    bound_height_label_ = default_label_for_panel(bound_panel_id_);
    binding_state_ = BindingState::LOCKED;
    transition_candidate_ = -1;
    transition_confirm_count_ = 0;
    result->switch_occurred = true;
    result->switch_reason = 1;
  }

  result->bound_panel_id = bound_panel_id_;
  result->bound_height_label = bound_height_label_;
  result->bound_confidence =
      compute_cost_confidence(candidate_prob, candidate_margin,
                              same_panel_score, switch_score);
  return result->switch_occurred;
}

/* =================================================================== */
/*  State machine core                                                 */
/* =================================================================== */

bool AdaptiveArmorBinder::attempt_transition(int candidate_panel,
                                             bool gate_passed,
                                             int confirm_required) {
  if (candidate_panel == bound_panel_id_) {
    binding_state_ = BindingState::LOCKED;
    transition_candidate_ = -1;
    transition_confirm_count_ = 0;
    return false;
  }

  if (!gate_passed) {
    binding_state_ = BindingState::LOCKED;
    transition_candidate_ = -1;
    transition_confirm_count_ = 0;
    return false;
  }

  if (binding_state_ == BindingState::LOCKED) {
    if (confirm_required <= 1) return true;
    binding_state_ = BindingState::TRANSITION_CANDIDATE;
    transition_candidate_ = candidate_panel;
    transition_confirm_count_ = 1;
    return false;
  }

  if (candidate_panel == transition_candidate_) {
    ++transition_confirm_count_;
    if (transition_confirm_count_ >= confirm_required) {
      binding_state_ = BindingState::LOCKED;
      transition_candidate_ = -1;
      transition_confirm_count_ = 0;
      return true;
    }
    return false;
  }

  // Different candidate — restart transition
  transition_candidate_ = candidate_panel;
  transition_confirm_count_ = 1;
  return false;
}

/* =================================================================== */
/*  Periodic evidence                                                  */
/* =================================================================== */

void AdaptiveArmorBinder::update_periodic_evidence(
    double z_jump, double yaw_rate_est, bool allow_model_update) {
  if (!config_.periodic_enable) return;
  period_update_applied_ = 0;

  const double spin_gate =
      std::max(0.0, config_.periodic_min_spin_rate);
  if (std::abs(yaw_rate_est) >= spin_gate) {
    spin_direction_ = (yaw_rate_est >= 0.0) ? 1 : -1;
  }

  if (!std::isfinite(z_jump)) {
    period_phase_index_ = -1;
    period_confidence_ = 0.0;
    return;
  }

  dz_periodic_history_.push_back(z_jump);
  const int window = std::max(3, config_.periodic_window);
  while (static_cast<int>(dz_periodic_history_.size()) > window) {
    dz_periodic_history_.pop_front();
  }

  const double abs_jump = std::abs(z_jump);
  const double min_jump =
      std::max(1e-5, config_.periodic_update_min_jump);
  if (allow_model_update && abs_jump > min_jump) {
    if (!std::isfinite(dz_small_est_)) {
      dz_small_est_ = abs_jump;
      dz_large_est_ = 2.0 * dz_small_est_;
    } else {
      const double alpha =
          std::clamp(config_.dz_ema_alpha, 0.01, 1.0);
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
    period_update_applied_ = 1;
  }

  // Phase matching
  const int n = config_.n_panels;
  if (spin_direction_ == 0 || !std::isfinite(dz_small_est_) ||
      dz_small_est_ < 1e-4 ||
      static_cast<int>(dz_periodic_history_.size()) < std::min(3, n)) {
    period_phase_index_ = -1;
    period_confidence_ = 0.0;
    return;
  }

  // Build template: CW=[-2, +1, +1]*dz_small, CCW=[+2, -1, -1]*dz_small
  const int sample_count =
      std::min(static_cast<int>(dz_periodic_history_.size()), window);

  int best_phase = 0;
  double best_conf = -1.0;
  for (int phase = 0; phase < n; ++phase) {
    const int start =
        static_cast<int>(dz_periodic_history_.size()) - sample_count;
    double err_sum = 0.0;
    for (int k = 0; k < sample_count; ++k) {
      const double obs_norm =
          dz_periodic_history_[start + k] / dz_small_est_;
      const int t_idx = wrap_mod(phase + k, n);
      // Template values: first element is either +2 (CCW) or -2 (CW),
      // remaining elements alternate between -1 and +1
      double t_val = 0.0;
      if (spin_direction_ >= 0) {  // CCW
        if (t_idx == 0) t_val = 2.0;
        else t_val = -1.0 / static_cast<double>(n - 1) * 2.0;
      } else {  // CW
        if (t_idx == 0) t_val = -2.0;
        else t_val = 1.0 / static_cast<double>(n - 1) * 2.0;
      }
      err_sum += std::abs(obs_norm - t_val);
    }
    const double mean_err = err_sum / static_cast<double>(sample_count);
    const double conf = std::exp(-0.65 * mean_err);
    if (conf > best_conf) {
      best_conf = conf;
      best_phase = phase;
    }
  }

  period_phase_index_ = best_phase;
  period_confidence_ = clamp01(best_conf);
}

void AdaptiveArmorBinder::apply_periodic_prior(
    std::vector<HypothesisScore> &hyps, double z_jump) const {
  if (!std::isfinite(z_jump) || bound_panel_id_ < 0 ||
      !std::isfinite(dz_small_est_)) {
    return;
  }

  const double period_conf =
      std::isfinite(period_confidence_) ? period_confidence_ : 0.0;
  const double prior_weight =
      std::max(0.0, config_.periodic_weight) *
      clamp01(period_conf);
  if (prior_weight <= 0.0) return;

  const double dz_unit = std::max(0.02, dz_small_est_);
  for (auto &hyp : hyps) {
    const double expected =
        config_.z_offsets[hyp.panel_id] -
        config_.z_offsets[bound_panel_id_];
    const double normalized_err =
        std::abs(z_jump - expected) / dz_unit;
    hyp.cost += prior_weight * normalized_err;
  }
}

/* =================================================================== */
/*  Confidence computation helpers                                     */
/* =================================================================== */

double AdaptiveArmorBinder::compute_proximity_confidence(
    const AssociationDiagnostics &diag, bool jump_gate_passed) const {
  const double margin_base =
      std::max(1e-3, config_.cost_margin_min);
  const double yaw_gate =
      std::max(1e-3, config_.yaw_err_gate);

  const double margin =
      std::isfinite(diag.cost_margin) ? diag.cost_margin : 0.0;
  const double margin_score =
      clamp01(margin / (2.0 * margin_base));

  const double yaw_err_val =
      std::isfinite(diag.selected_yaw_err)
          ? diag.selected_yaw_err
          : yaw_gate;
  const double yaw_score =
      clamp01(1.0 - yaw_err_val / yaw_gate);

  const double jump_score = jump_gate_passed ? 1.0 : 0.0;
  const double base = clamp01(
      0.50 * margin_score + 0.30 * yaw_score + 0.20 * jump_score);

  const double floor =
      std::clamp(config_.confidence_floor, 0.0, 0.95);
  return floor + (1.0 - floor) * base;
}

double AdaptiveArmorBinder::compute_cost_confidence(
    double candidate_prob, double candidate_margin,
    double same_panel_score, double /*switch_score*/) const {
  const double period_conf =
      (config_.periodic_enable && std::isfinite(period_confidence_))
          ? period_confidence_
          : 0.0;

  const double w_prob = 0.35;
  const double w_margin = 0.30;
  const double w_period = 0.20 * std::clamp(period_conf, 0.0, 1.0);
  const double w_same = 0.15 * same_panel_score;
  const double denom = w_prob + w_margin + w_period + w_same;

  double v = (w_prob * candidate_prob +
              w_margin * std::min(1.0, candidate_margin / 0.25) +
              w_period * period_conf +
              w_same * same_panel_score) /
             denom;

  const double floor =
      std::clamp(config_.confidence_floor, 0.0, 0.95);
  return floor + (1.0 - floor) * clamp01(v);
}

double AdaptiveArmorBinder::compute_same_panel_score(
    const HypothesisScore &hyp, double predicted_center_z,
    double center_yaw_est) const {
  const double yaw_gate =
      std::max(1e-3, config_.same_panel_yaw_gate);
  const double z_gate =
      std::max(1e-3, config_.same_panel_z_gate);
  const double xy_gate =
      std::max(1e-3, config_.same_panel_xy_gate);

  double yaw_diff = 0.0;
  if (std::isfinite(hyp.center_yaw) && std::isfinite(center_yaw_est)) {
    yaw_diff = std::abs(std::remainder(hyp.center_yaw - center_yaw_est,
                                       2.0 * kPi));
  }

  double z_diff = 0.0;
  if (std::isfinite(hyp.center_z) && std::isfinite(predicted_center_z)) {
    z_diff = std::abs(hyp.center_z - predicted_center_z);
  }

  const double xy_err = std::isfinite(hyp.xy_residual) ? hyp.xy_residual : 0.0;

  double score = 1.0 -
      (0.40 * (yaw_diff / yaw_gate) +
       0.35 * (z_diff / z_gate) +
       0.25 * (xy_err / xy_gate));
  return clamp01(score);
}

/* =================================================================== */
/*  Helpers                                                            */
/* =================================================================== */

AdaptiveArmorBinder::HeightLabel AdaptiveArmorBinder::default_label_for_panel(
    int panel_id) const {
  if (config_.n_panels <= 2) {
    // 2-panel: even=LOWER, odd=UPPER (same as 4-panel)
    return (panel_id % 2 == 0) ? HeightLabel::LOWER : HeightLabel::UPPER;
  }

  // For N > 2: use z_offsets to determine label
  // Panel with highest z = UPPER, lowest z = LOWER
  if (panel_id < 0 || panel_id >= config_.n_panels) return HeightLabel::UNKNOWN;

  // For standard layout: even=LOWER, odd=UPPER
  // z_offsets higher → UPPER, lower → LOWER
  const int n = config_.n_panels;
  const double z_this = config_.z_offsets[panel_id];

  // Simple heuristic: if z is above median, it's UPPER
  std::vector<double> sorted_z = config_.z_offsets;
  std::sort(sorted_z.begin(), sorted_z.end());
  const double median = sorted_z[n / 2];

  return (z_this >= median) ? HeightLabel::UPPER : HeightLabel::LOWER;
}

AdaptiveArmorBinder::DebugSnapshot AdaptiveArmorBinder::debug_snapshot() const {
  DebugSnapshot snap;
  snap.valid = true;
  snap.bound_panel_id = bound_panel_id_;
  snap.bound_height_label = bound_height_label_;
  snap.bound_confidence = bound_confidence_;
  snap.transition_state = static_cast<int>(binding_state_);
  snap.transition_candidate = transition_candidate_;
  snap.transition_confirm_count = transition_confirm_count_;
  snap.cooldown_remaining = cooldown_remaining_;
  snap.dz_jump_est = dz_jump_est_;
  snap.period_confidence = period_confidence_;
  snap.period_phase = period_phase_index_;
  snap.spin_direction = spin_direction_;
  return snap;
}

void AdaptiveArmorBinder::update_jump_statistics(
    double z_jump, bool switch_confirmed) {
  if (!switch_confirmed || !std::isfinite(z_jump)) return;

  const double abs_jump = std::abs(z_jump);
  if (abs_jump < std::max(0.0, config_.z_jump_min)) return;

  z_jump_history_.push_back(abs_jump);
  while (z_jump_history_.size() > 20) {
    z_jump_history_.pop_front();
  }

  const double alpha =
      std::clamp(config_.dz_ema_alpha, 0.01, 1.0);
  if (!std::isfinite(dz_jump_est_)) {
    dz_jump_est_ = abs_jump;
  } else {
    dz_jump_est_ = (1.0 - alpha) * dz_jump_est_ + alpha * abs_jump;
  }
}

}  // namespace fyt::auto_aim
