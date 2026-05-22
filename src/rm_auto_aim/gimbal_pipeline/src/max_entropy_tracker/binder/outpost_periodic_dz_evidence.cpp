// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/decoder/outpost_periodic_dz_evidence.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

namespace {

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

int wrap_phase_index(int phase, int modulo) {
  const int m = std::max(1, modulo);
  int r = phase % m;
  if (r < 0) r += m;
  return r;
}

}  // namespace

OutpostPeriodicDzEvidence::OutpostPeriodicDzEvidence(
    const UnifiedConfig & config, const std::array<double, 3> & z_offsets)
    : config_(config), z_offsets_(z_offsets) {}

void OutpostPeriodicDzEvidence::reset() {
  dz_jump_history_.clear();
  dz_small_est_ = std::numeric_limits<double>::quiet_NaN();
  dz_large_est_ = std::numeric_limits<double>::quiet_NaN();
  period_confidence_ = 0.0;
  period_phase_index_ = -1;
  spin_direction_ = 0;
  pending_spin_direction_ = 0;
  pending_spin_direction_count_ = 0;
}

void OutpostPeriodicDzEvidence::update(
    double z_jump, bool allow_model_update, double yaw_rate_est) {
  const double spin_gate =
      std::max(0.0, config_.outpost.binding_period_min_spin_rate);
  if (std::abs(yaw_rate_est) >= spin_gate) {
    const int observed_direction = (yaw_rate_est >= 0.0) ? 1 : -1;
    if (observed_direction == spin_direction_) {
      pending_spin_direction_ = 0;
      pending_spin_direction_count_ = 0;
    } else if (observed_direction == pending_spin_direction_) {
      ++pending_spin_direction_count_;
    } else {
      pending_spin_direction_ = observed_direction;
      pending_spin_direction_count_ = 1;
    }

    const int confirm_frames =
        std::max(1, config_.outpost.spin_direction_confirm_frames);
    if (pending_spin_direction_count_ >= confirm_frames) {
      spin_direction_ = pending_spin_direction_;
      pending_spin_direction_ = 0;
      pending_spin_direction_count_ = 0;
    }
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

void OutpostPeriodicDzEvidence::apply_prior(
    std::array<BindingHypothesis, 3> & hyps, double z_jump,
    int bound_panel_id) const {
  if (!std::isfinite(z_jump) || bound_panel_id < 0 ||
      !std::isfinite(dz_small_est_)) {
    return;
  }

  const double prior_weight =
      std::max(0.0, config_.outpost.binding_period_weight) *
      clamp01(period_confidence_);
  if (prior_weight <= 0.0) return;

  const double dz_unit = std::max(0.02, dz_small_est_);
  for (int i = 0; i < 3; ++i) {
    const double expected_jump = z_offsets_[i] - z_offsets_[bound_panel_id];
    const double normalized_err = std::abs(z_jump - expected_jump) / dz_unit;
    hyps[i].cost += prior_weight * normalized_err;
  }
}

std::array<double, 3>
OutpostPeriodicDzEvidence::periodic_template_for_spin() const {
  if (spin_direction_ >= 0) {
    return {2.0, -1.0, -1.0};
  }
  return {-2.0, 1.0, 1.0};
}

double OutpostPeriodicDzEvidence::compute_period_confidence_for_phase(
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

}  // namespace fyt::auto_aim::binder
