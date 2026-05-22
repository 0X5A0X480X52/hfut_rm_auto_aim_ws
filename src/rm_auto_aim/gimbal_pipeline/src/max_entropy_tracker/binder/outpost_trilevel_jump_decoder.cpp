// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/decoder/outpost_trilevel_jump_decoder.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

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
  JumpDecision jd;

  if (!input.obs_z_values.empty()) {
    ensure_active_cluster(input.obs_z_values[0]);
  }

  if (!input.obs_z_values.empty() && input.event_type == TrackEventType::CONTINUITY &&
      ctx.last_panel_id >= 0 && ctx.last_panel_id < 3) {
    update_z_cluster(ctx.last_panel_id, input.obs_z_values[0]);
  }

  // 1. Update periodic evidence
  if (config_.periodic_enable && input.has_z_jump) {
    bool allow = std::abs(input.yaw_rate_est) >=
                 std::max(0.0, config_.periodic_min_spin_rate);
    update_periodic_evidence(input.z_jump, input.yaw_rate_est, allow, ctx);
  }

  if (std::isfinite(ctx.period_confidence)) {
    jd.signature_score = clamp01(ctx.period_confidence);
  }

  // 1.5 Reacquire branch
  if ((input.event_type == TrackEventType::REACQUIRE || input.is_reacquired) &&
      (input.gap_dt >= config_.reacquire_gap_dt_gate ||
       input.lost_frames >= config_.reacquire_lost_frames_gate)) {
    jd = decode_from_reacquire(input, ctx);
    if (std::isfinite(ctx.period_confidence)) {
      jd.signature_score = std::max(jd.signature_score, clamp01(ctx.period_confidence));
    }
    if (jd.detected) {
      if (!input.obs_z_values.empty() && jd.to_id >= 0 && jd.to_id < 3) {
        rotate_cluster_after_jump(jd.to_id, input.obs_z_values[0]);
      }
      return jd;
    }
  }

  // NOTE:
  // To avoid duplicated internal jump detection loops, do not let z-audit
  // directly trigger switching in the main path. Keep it as a latent estimator
  // updated only when needed by debug/periodic context.
  if (config_.z_audit_enable) {
    (void)decode_from_z_audit(input, ctx);
  }

  // 3. Fall back to cost-based detection
  JumpDecision cd = decode_from_cost(input, ctx);
  cd.signature_score = std::max(cd.signature_score, jd.signature_score);
  if (cd.detected && !input.obs_z_values.empty() &&
      cd.to_id >= 0 && cd.to_id < 3) {
    rotate_cluster_after_jump(cd.to_id, input.obs_z_values[0]);
  }
  return cd;
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
  if (input.event_type != TrackEventType::SWITCH_CANDIDATE &&
      input.event_type != TrackEventType::REACQUIRE &&
      !input.is_reacquired) {
    jd.signature_score = clamp01(ctx.period_confidence);
    return jd;
  }
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
  const double abs_jump = std::abs(z_jump);
  jd.detected = true;
  jd.jump_kind = classify_jump_kind_by_bands(abs_jump, ctx);
  if (jd.jump_kind != JumpKind::DOUBLE_DZ && cyclic_diff >= 2) {
    jd.jump_kind = JumpKind::DOUBLE_DZ;
  }
  jd.from_id = ctx.last_panel_id;
  jd.to_id = best_panel;
  jd.confidence = clamp01(0.65 * input.candidate_prob + 0.35 / (1.0 + best_cost));
  jd.reason_code = 1;
  jd.signature_score = clamp01(ctx.period_confidence);
  return jd;
}

void OutpostTriLevelJumpDecoder::update_periodic_evidence(
    double z_jump, double yaw_rate_est, bool allow_model_update,
    DecoderContext & ctx) {
  const double spin_gate =
      std::max(0.0, config_.periodic_min_spin_rate);
  if (std::abs(yaw_rate_est) >= spin_gate) {
    const int observed_direction = (yaw_rate_est >= 0.0) ? 1 : -1;
    if (observed_direction == ctx.spin_direction) {
      ctx.pending_spin_direction = 0;
      ctx.pending_spin_direction_count = 0;
    } else if (observed_direction == ctx.pending_spin_direction) {
      ++ctx.pending_spin_direction_count;
    } else {
      ctx.pending_spin_direction = observed_direction;
      ctx.pending_spin_direction_count = 1;
    }

    constexpr int kConfirmFrames = 3;
    if (ctx.pending_spin_direction_count >= kConfirmFrames) {
      ctx.spin_direction = ctx.pending_spin_direction;
      ctx.pending_spin_direction = 0;
      ctx.pending_spin_direction_count = 0;
    }
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

JumpDecision OutpostTriLevelJumpDecoder::decode_from_reacquire(
    const BinderFrameInput & input, DecoderContext & ctx) {
  JumpDecision jd;

  std::cout << "[REACQUIRE] ===== New Frame =====" << std::endl;

  if (input.obs_z_values.empty() || ctx.last_panel_id < 0 || ctx.last_panel_id >= 3) {
    std::cout << "[REACQUIRE] Early exit: "
              << "obs_z_values.empty=" << input.obs_z_values.empty()
              << " last_panel_id=" << ctx.last_panel_id
              << std::endl;
    return jd;
  }

  const double obs_z = input.obs_z_values[0];
  const int prev_panel = ctx.last_panel_id;

  std::cout << "[REACQUIRE] obs_z=" << obs_z
            << " prev_panel=" << prev_panel
            << " period_confidence=" << ctx.period_confidence
            << std::endl;

  int best_panel = prev_panel;
  double best_cost = std::numeric_limits<double>::infinity();
  double prev_cost = std::numeric_limits<double>::infinity();

  for (int p = 0; p < 3; ++p) {
    double cost = 1e6;
    const int slot = (p >= 0 && p < 3) ? alias_to_slot_[p] : -1;
    const bool slot_ok =
        slot >= 0 && slot < kMaxClusterSlots && z_clusters_[slot].initialized;

    std::cout << "[REACQUIRE] panel=" << p
              << " slot=" << slot
              << " initialized=" << (slot_ok ? z_clusters_[slot].initialized : 0)
              << " mean=" << (slot_ok ? z_clusters_[slot].mean : 0.0)
              << " var=" << (slot_ok ? z_clusters_[slot].var : 0.02)
              << " offset=" << config_.z_offsets[p]
              << " count=" << (slot_ok ? z_clusters_[slot].count : 0);

    if (slot_ok) {
      const double pred_z = z_clusters_[slot].mean + config_.z_offsets[p];
      const double sigma = std::sqrt(std::max(1e-6, z_clusters_[slot].var));
      const double denom = std::max(0.01, sigma);
      const double residual = obs_z - pred_z;

      cost = std::abs(residual) / denom;

      std::cout << " pred_z=" << pred_z
                << " residual=" << residual
                << " sigma=" << sigma
                << " denom=" << denom
                << " cost=" << cost;
    } else {
      std::cout << " pred_z=N/A residual=N/A sigma=N/A cost=1e6";
    }

    std::cout << std::endl;

    if (p == prev_panel) {
      prev_cost = cost;
    }

    if (cost < best_cost) {
      best_cost = cost;
      best_panel = p;
    }
  }

  std::cout << "[REACQUIRE] select_result:"
            << " prev_panel=" << prev_panel
            << " best_panel=" << best_panel
            << " prev_cost=" << prev_cost
            << " best_cost=" << best_cost
            << std::endl;

  if (best_panel == prev_panel) {
    std::cout << "[REACQUIRE] No switch: best_panel == prev_panel, update cluster"
              << std::endl;
    update_z_cluster(prev_panel, obs_z);
    return jd;
  }

  const double improve = prev_cost - best_cost;
  const double assign_gate = std::max(0.10, config_.z_cluster_assign_gate);

  std::cout << "[REACQUIRE] switch_gate:"
            << " improve=" << improve
            << " assign_gate=" << assign_gate
            << " config_gate=" << config_.z_cluster_assign_gate
            << std::endl;

  if (improve < assign_gate) {
    std::cout << "[REACQUIRE] Reject switch: improve < assign_gate"
              << std::endl;
    return jd;
  }

  jd.detected = true;
  jd.from_id = prev_panel;
  jd.to_id = best_panel;
  jd.reason_code = 7;  // reacquire-switch
  jd.confidence = clamp01(0.5 + 0.2 * improve);

  const int prev_slot =
      (prev_panel >= 0 && prev_panel < 3) ? alias_to_slot_[prev_panel] : -1;
  const double prev_pred_z =
      (prev_slot >= 0 && prev_slot < kMaxClusterSlots &&
       z_clusters_[prev_slot].initialized)
          ? (z_clusters_[prev_slot].mean + config_.z_offsets[prev_panel])
          : obs_z;
  const double abs_jump = std::abs(obs_z - prev_pred_z);

  jd.jump_kind = JumpKind::DZ;

  std::cout << "[REACQUIRE] SWITCH DETECTED:"
            << " from=" << jd.from_id
            << " to=" << jd.to_id
            << " confidence=" << jd.confidence
            << " prev_pred_z=" << prev_pred_z
            << " abs_jump=" << abs_jump
            << " dz_small_est=" << ctx.dz_small_est
            << " dz_large_est=" << ctx.dz_large_est
            << std::endl;

  jd.jump_kind = classify_jump_kind_by_bands(abs_jump, ctx);
  std::cout << "[REACQUIRE] jump_kind="
            << (jd.jump_kind == JumpKind::DOUBLE_DZ ? "DOUBLE_DZ" : "DZ")
            << std::endl;

  jd.signature_score = clamp01(ctx.period_confidence);

  std::cout << "[REACQUIRE] final_decision:"
            << " detected=" << jd.detected
            << " from_id=" << jd.from_id
            << " to_id=" << jd.to_id
            << " reason_code=" << jd.reason_code
            << " confidence=" << jd.confidence
            << " signature_score=" << jd.signature_score
            << std::endl;

  return jd;
}

void OutpostTriLevelJumpDecoder::update_z_cluster(int panel_id, double obs_z) {
  std::cout << "[Z_CLUSTER] update request:"
            << " panel_id=" << panel_id
            << " obs_z=" << obs_z
            << std::endl;

  if (panel_id < 0 || panel_id >= 3 || !std::isfinite(obs_z)) {
    std::cout << "[Z_CLUSTER] skip: invalid input"
              << " panel_id=" << panel_id
              << " obs_z=" << obs_z
              << std::endl;
    return;
  }

  int slot = alias_to_slot_[panel_id];
  if (slot < 0 || slot >= kMaxClusterSlots) {
    slot = allocate_cluster_slot();
    alias_to_slot_[panel_id] = slot;
  }
  const double center_z = obs_z - config_.z_offsets[panel_id];
  auto & c = z_clusters_[slot];

  std::cout << "[Z_CLUSTER] normalized:"
              << " panel_id=" << panel_id
              << " slot=" << slot
              << " center_z=" << center_z
            << " offset=" << config_.z_offsets[panel_id]
            << " initialized=" << c.initialized
            << " old_mean=" << c.mean
            << " old_var=" << c.var
            << " old_count=" << c.count
            << std::endl;

  if (!c.initialized) {
    c.initialized = true;
    c.mean = center_z;
    c.var = 0.02;
    c.raw_mean = obs_z;
    c.raw_var = 0.02;
    c.count = 1;

    std::cout << "[Z_CLUSTER] initialized:"
              << " panel_id=" << panel_id
              << " slot=" << slot
              << " mean=" << c.mean
              << " var=" << c.var
              << " count=" << c.count
              << std::endl;
    return;
  }

  const double alpha = std::clamp(config_.z_cluster_ema_alpha, 0.01, 1.0);
  const double e = center_z - c.mean;

  std::cout << "[Z_CLUSTER] ema_before:"
              << " panel_id=" << panel_id
              << " slot=" << slot
              << " alpha=" << alpha
            << " error=" << e
            << " mean=" << c.mean
            << " var=" << c.var
            << " count=" << c.count
            << std::endl;

  c.mean = (1.0 - alpha) * c.mean + alpha * center_z;
  c.var = (1.0 - alpha) * c.var + alpha * (e * e);
  if (!std::isfinite(c.raw_mean)) {
    c.raw_mean = obs_z;
  }
  const double raw_e = obs_z - c.raw_mean;
  c.raw_mean = (1.0 - alpha) * c.raw_mean + alpha * obs_z;
  c.raw_var = (1.0 - alpha) * c.raw_var + alpha * (raw_e * raw_e);
  c.count += 1;

  std::cout << "[Z_CLUSTER] ema_after:"
              << " panel_id=" << panel_id
              << " slot=" << slot
              << " mean=" << c.mean
            << " var=" << c.var
            << " sigma=" << std::sqrt(std::max(1e-6, c.var))
            << " raw_mean=" << c.raw_mean
            << " raw_sigma=" << std::sqrt(std::max(1e-6, c.raw_var))
            << " count=" << c.count
            << std::endl;
}

int OutpostTriLevelJumpDecoder::allocate_cluster_slot() {
  const int window_slots =
      std::clamp(config_.z_cluster_window_slots, 3, kMaxClusterSlots);
  const int slot = next_cluster_slot_;
  next_cluster_slot_ = (next_cluster_slot_ + 1) % window_slots;
  z_clusters_[slot] = ZCluster{};
  return slot;
}

void OutpostTriLevelJumpDecoder::ensure_active_cluster(double obs_z) {
  if (!std::isfinite(obs_z)) return;
  if (active_cluster_slot_ >= 0 && active_cluster_slot_ < kMaxClusterSlots) return;
  const int slot = allocate_cluster_slot();
  active_cluster_slot_ = slot;
  alias_to_slot_[0] = slot;
  alias_to_slot_[1] = -1;
  alias_to_slot_[2] = -1;
}

void OutpostTriLevelJumpDecoder::rotate_cluster_after_jump(
    int to_alias_id, double obs_z) {
  if (to_alias_id < 0 || to_alias_id >= 3 || !std::isfinite(obs_z)) return;
  const int slot = allocate_cluster_slot();
  alias_to_slot_[to_alias_id] = slot;
  active_cluster_slot_ = slot;
  update_z_cluster(to_alias_id, obs_z);
}

bool OutpostTriLevelJumpDecoder::infer_dz_bands_from_raw_clusters(
    double & dz_small, double & dz_large) const {
  std::vector<double> means;
  std::vector<double> weights;
  means.reserve(3);
  weights.reserve(3);
  for (const auto & c : z_clusters_) {
    if (c.initialized && c.count >= 1 && std::isfinite(c.raw_mean)) {
      means.push_back(c.raw_mean);
      weights.push_back(static_cast<double>(std::max(1, c.count)));
    }
  }
  if (means.size() < 2) return false;

  struct WeightedDiff {
    double diff = 0.0;
    double w = 1.0;
  };
  std::vector<WeightedDiff> diffs;
  for (size_t i = 0; i < means.size(); ++i) {
    for (size_t j = i + 1; j < means.size(); ++j) {
      WeightedDiff d;
      d.diff = std::abs(means[i] - means[j]);
      d.w = std::min(weights[i], weights[j]);
      diffs.push_back(d);
    }
  }
  if (diffs.empty()) return false;
  std::sort(diffs.begin(), diffs.end(),
            [](const WeightedDiff & a, const WeightedDiff & b) {
              return a.diff < b.diff;
            });

  // Weighted robust estimate: average bottom/top halves instead of strict min/max.
  const size_t half = std::max<size_t>(1, diffs.size() / 2);
  double s_small = 0.0, w_small = 0.0;
  double s_large = 0.0, w_large = 0.0;
  for (size_t i = 0; i < half; ++i) {
    s_small += diffs[i].w * diffs[i].diff;
    w_small += diffs[i].w;
  }
  for (size_t i = diffs.size() - half; i < diffs.size(); ++i) {
    s_large += diffs[i].w * diffs[i].diff;
    w_large += diffs[i].w;
  }
  dz_small = (w_small > 1e-6) ? (s_small / w_small) : diffs.front().diff;
  dz_large = (w_large > 1e-6) ? (s_large / w_large) : diffs.back().diff;

  if (!(std::isfinite(dz_small) && std::isfinite(dz_large))) return false;
  if (dz_small < 1e-4 || dz_large < 1e-4) return false;
  // Relax ratio gate for short-window/new-slot regime.
  if (dz_large < 1.3 * dz_small) return false;
  return true;
}

JumpKind OutpostTriLevelJumpDecoder::classify_jump_kind_by_bands(
    double abs_jump, const DecoderContext & ctx) const {
  double dz_small = std::numeric_limits<double>::quiet_NaN();
  double dz_large = std::numeric_limits<double>::quiet_NaN();
  bool has_cluster_bands = infer_dz_bands_from_raw_clusters(dz_small, dz_large);

  if (!has_cluster_bands &&
      std::isfinite(ctx.dz_small_est) &&
      std::isfinite(ctx.dz_large_est)) {
    dz_small = ctx.dz_small_est;
    dz_large = ctx.dz_large_est;
  }
  if (!(std::isfinite(dz_small) && std::isfinite(dz_large))) {
    return JumpKind::DZ;
  }

  const double diff_small = std::abs(abs_jump - dz_small);
  const double diff_large = std::abs(abs_jump - dz_large);
  const bool periodic_ready =
      std::isfinite(ctx.period_confidence) &&
      clamp01(ctx.period_confidence) >= config_.periodic_signature_threshold;
  const double margin = has_cluster_bands ? 0.002 : 0.008;

  // Primary rule: closer to large band with margin.
  if ((periodic_ready || has_cluster_bands) &&
      (diff_large + margin < diff_small)) {
    return JumpKind::DOUBLE_DZ;
  }

  // Secondary relative rule (no absolute calibration):
  // if jump is much larger than the estimated small band and clearly above
  // the middle between small/large, treat as DOUBLE_DZ in reacquire-like cases.
  const double mid = 0.5 * (dz_small + dz_large);
  if (has_cluster_bands) {
    if (abs_jump > mid && abs_jump > 1.6 * dz_small) {
      return JumpKind::DOUBLE_DZ;
    }
  }

  return JumpKind::DZ;
}

}  // namespace fyt::auto_aim::binder
