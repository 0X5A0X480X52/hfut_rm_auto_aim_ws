// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/decoder/outpost_trilevel_jump_decoder.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

int wrap_mod(int value, int modulo) {
  const int m = std::max(1, modulo);
  const int r = value % m;
  return (r < 0) ? (r + m) : r;
}

}  // namespace

OutpostTriLevelJumpDecoder::OutpostTriLevelJumpDecoder(
    const OutpostTriLevelJumpDecoderConfig & config)
    : config_(config) {}

JumpDecision OutpostTriLevelJumpDecoder::decode(const BinderFrameInput & input,
                                                DecoderContext & ctx) {
  // 1. Update periodic evidence
  if (config_.periodic_enable && input.has_z_jump) {
    bool allow = std::abs(input.yaw_rate_est) >=
                 std::max(0.0, config_.periodic_min_spin_rate);
    update_periodic_evidence(input.z_jump, input.yaw_rate_est, allow, ctx);
  }

  // 2. Try z-audit path (independent z-only estimator)
  if (config_.z_audit_enable) {
    JumpDecision zd = decode_from_z_audit(input, ctx);
    if (zd.detected) return zd;
  }

  // 3. Fall back to cost-based detection
  return decode_from_cost(input, ctx);
}

JumpDecision OutpostTriLevelJumpDecoder::decode_from_z_audit(
    const BinderFrameInput & input, DecoderContext & ctx) {
  JumpDecision jd;
  if (input.obs_z_values.empty()) return jd;

  const double obs_z = input.obs_z_values[0];

  // Initialize z-audit
  if (!z_audit_init_) {
    z_audit_center_ = obs_z;
    z_audit_prev_z_ = obs_z;
    z_audit_prev_panel_ = 0;
    z_audit_init_ = true;
    return jd;
  }

  const double z_jump = obs_z - z_audit_prev_z_;
  z_audit_prev_z_ = obs_z;

  // Find best panel match among 3 panels
  int best_panel = -1;
  double best_cost = 1e9;
  for (int p = 0; p < 3; ++p) {
    const double level_err =
        std::abs(obs_z - (z_audit_center_ + config_.z_offsets[p]));
    const double expected_jump =
        config_.z_offsets[p] - config_.z_offsets[z_audit_prev_panel_];
    const double jump_err = std::abs(z_jump - expected_jump);
    const double switch_penalty = (p != z_audit_prev_panel_) ? 1.0 : 0.0;
    const double cost =
        6.0 * level_err + 2.0 * jump_err + 0.5 * switch_penalty;
    if (cost < best_cost) {
      best_cost = cost;
      best_panel = p;
    }
  }

  // EMA update center estimate
  const double obs_center_z = obs_z - config_.z_offsets[best_panel];
  z_audit_center_ = 0.65 * z_audit_center_ + 0.35 * obs_center_z;

  // Check for conflict with previous panel
  if (best_panel >= 0 && best_panel != z_audit_prev_panel_) {
    const double abs_jump = std::abs(z_jump);
    if (abs_jump >= config_.z_audit_min_jump) {
      ++z_audit_conflict_count_;
    }
  } else {
    z_audit_conflict_count_ = 0;
  }

  z_audit_prev_panel_ = best_panel;

  // Trigger jump if conflict sustained
  if (z_audit_conflict_count_ >= config_.z_audit_confirm_frames) {
    z_audit_conflict_count_ = 0;
    const double conf = clamp01(1.0 / (1.0 + best_cost));
    if (conf < std::clamp(config_.z_audit_min_confidence, 0.0, 1.0)) {
      return jd;
    }
    jd.detected = true;
    const double abs_jump = std::abs(z_jump);
    if (std::isfinite(ctx.dz_small_est) && std::isfinite(ctx.dz_large_est) &&
        std::abs(abs_jump - ctx.dz_large_est) <
            std::abs(abs_jump - ctx.dz_small_est)) {
      jd.jump_kind = JumpKind::DOUBLE_DZ;
    } else {
      jd.jump_kind = JumpKind::DZ;
    }
    jd.from_id = ctx.last_panel_id;
    jd.to_id = best_panel;
    jd.confidence = conf;
    jd.reason_code = 2;
  }

  return jd;
}

JumpDecision OutpostTriLevelJumpDecoder::decode_from_cost(
    const BinderFrameInput & input, const DecoderContext & ctx) {
  JumpDecision jd;
  if (input.candidate_id < 0 || !input.profile) return jd;

  const double min_prob =
      std::clamp(config_.min_candidate_prob, 0.0, 1.0);
  const double min_margin =
      std::clamp(config_.min_candidate_margin, 0.0, 1.0);

  if (input.candidate_prob < min_prob) {
    jd.reason_code = 2;
    return jd;
  }
  if (input.candidate_margin < min_margin) {
    jd.reason_code = 3;
    return jd;
  }

  const int n = std::max(1, input.profile->panel_count);
  if (ctx.last_panel_id < 0 || ctx.last_panel_id >= n) {
    jd.reason_code = 4;
    return jd;
  }

  std::vector<double> costs(static_cast<size_t>(n), 1e9);
  const double z_jump = input.has_z_jump ? input.z_jump : 0.0;
  for (int panel = 0; panel < n; ++panel) {
    costs[panel] = (panel == input.candidate_id) ? 0.0 : 0.7;
    if (input.has_z_jump && panel < static_cast<int>(config_.z_offsets.size()) &&
        ctx.last_panel_id < static_cast<int>(config_.z_offsets.size())) {
      const double expected =
          config_.z_offsets[panel] - config_.z_offsets[ctx.last_panel_id];
      costs[panel] += std::abs(z_jump - expected);
    }
  }
  if (input.has_z_jump) {
    apply_periodic_prior(costs, z_jump, ctx.last_panel_id, ctx);
  }

  int best_panel = 0;
  double best_cost = costs[0];
  for (int i = 1; i < n; ++i) {
    if (costs[i] < best_cost) {
      best_cost = costs[i];
      best_panel = i;
    }
  }
  if (best_panel == ctx.last_panel_id) return jd;

  const int raw_diff = std::abs(best_panel - ctx.last_panel_id);
  const int cyclic_diff = std::min(raw_diff, n - raw_diff);
  jd.detected = true;
  jd.jump_kind = (cyclic_diff >= 2) ? JumpKind::DOUBLE_DZ : JumpKind::DZ;
  jd.from_id = ctx.last_panel_id;
  jd.to_id = best_panel;
  jd.confidence = clamp01(0.65 * input.candidate_prob + 0.35 / (1.0 + best_cost));
  jd.reason_code = 1;
  return jd;
}

void OutpostTriLevelJumpDecoder::update_periodic_evidence(
    double z_jump, double yaw_rate_est, bool allow_model_update,
    DecoderContext & ctx) {
  const double spin_gate =
      std::max(0.0, config_.periodic_min_spin_rate);
  if (std::abs(yaw_rate_est) >= spin_gate) {
    ctx.spin_direction = (yaw_rate_est >= 0.0) ? 1 : -1;
  }

  if (!std::isfinite(z_jump)) {
    ctx.period_phase = -1;
    ctx.period_confidence = 0.0;
    return;
  }

  ctx.dz_history.push_back(z_jump);
  const int window = std::max(3, config_.periodic_window);
  while (static_cast<int>(ctx.dz_history.size()) > window) {
    ctx.dz_history.pop_front();
  }

  const double abs_jump = std::abs(z_jump);
  const double min_jump =
      std::max(1e-5, config_.periodic_update_min_jump);
  if (allow_model_update && abs_jump > min_jump) {
    if (!std::isfinite(ctx.dz_small_est)) {
      ctx.dz_small_est = abs_jump;
      ctx.dz_large_est = 2.0 * ctx.dz_small_est;
    } else {
      const double alpha =
          std::clamp(config_.dz_ema_alpha, 0.01, 1.0);
      const bool is_large =
          std::abs(abs_jump - ctx.dz_large_est) <
          std::abs(abs_jump - ctx.dz_small_est);
      const double target_small = is_large ? (0.5 * abs_jump) : abs_jump;
      ctx.dz_small_est =
          (1.0 - alpha) * ctx.dz_small_est + alpha * target_small;
      ctx.dz_large_est = 2.0 * ctx.dz_small_est;
    }
  }

  // Phase matching
  const int n = 3;  // outpost has 3 panels
  if (ctx.spin_direction == 0 || !std::isfinite(ctx.dz_small_est) ||
      ctx.dz_small_est < 1e-4 ||
      static_cast<int>(ctx.dz_history.size()) < std::min(3, n)) {
    ctx.period_phase = -1;
    ctx.period_confidence = 0.0;
    return;
  }

  const int sample_count =
      std::min(static_cast<int>(ctx.dz_history.size()), window);
  int best_phase = 0;
  double best_conf = -1.0;
  for (int phase = 0; phase < n; ++phase) {
    const int start =
        static_cast<int>(ctx.dz_history.size()) - sample_count;
    double err_sum = 0.0;
    for (int k = 0; k < sample_count; ++k) {
      const double obs_norm =
          ctx.dz_history[start + k] / ctx.dz_small_est;
      const int t_idx = wrap_mod(phase + k, n);
      double t_val = 0.0;
      if (ctx.spin_direction >= 0) {
        t_val = (t_idx == 0) ? 2.0 : -1.0;
      } else {
        t_val = (t_idx == 0) ? -2.0 : 1.0;
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
  ctx.period_phase = best_phase;
  ctx.period_confidence = clamp01(best_conf);
}

void OutpostTriLevelJumpDecoder::apply_periodic_prior(
    std::vector<double> & costs, double z_jump, int bound_id,
    const DecoderContext & ctx) const {
  if (!std::isfinite(z_jump) || bound_id < 0 ||
      !std::isfinite(ctx.dz_small_est)) {
    return;
  }
  const double period_conf =
      std::isfinite(ctx.period_confidence) ? ctx.period_confidence : 0.0;
  const double prior_weight =
      std::max(0.0, config_.periodic_weight) * clamp01(period_conf);
  if (prior_weight <= 0.0) return;

  const double dz_unit = std::max(0.02, ctx.dz_small_est);
  for (size_t i = 0; i < costs.size() && i < 3; ++i) {
    const double expected =
        config_.z_offsets[i] - config_.z_offsets[bound_id];
    const double normalized_err =
        std::abs(z_jump - expected) / dz_unit;
    costs[i] += prior_weight * normalized_err;
  }
}

}  // namespace fyt::auto_aim::binder
