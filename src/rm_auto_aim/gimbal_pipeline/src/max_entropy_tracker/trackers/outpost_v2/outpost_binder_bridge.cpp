// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v2/outpost_binder_bridge.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::outpost_v2 {

OutpostBinderBridge::OutpostBinderBridge(const UnifiedConfig & cfg)
    : cfg_(cfg),
      profile_(binder::RobotBindingProfileProvider::from_robot_id(
          "outpost", {cfg.outpost.z_offset_0, cfg.outpost.z_offset_1,
                      cfg.outpost.z_offset_2})) {
  pipeline_ = binder::BinderFactory::create(profile_, build_binder_config(cfg_));
}

void OutpostBinderBridge::reset(int init_panel_id, std::optional<double> obs_z) {
  if (!pipeline_) return;
  pipeline_->reset(init_panel_id, binder::HeightLabel::MIDDLE, obs_z);
}

binder::BinderOutput OutpostBinderBridge::step(
    const ObservationData & obs, const std::vector<ObservationData> & all_obs,
    int obs_count, const BindingCandidate & candidate,
    const OutpostRuntimeContext & ctx) {
  binder::BinderOutput out;
  if (!pipeline_) return out;

  binder::BinderFrameInput in;
  in.timestamp = obs.timestamp.value_or(ctx.last_timestamp.value_or(0.0));
  in.profile = &profile_;
  in.obs_count = obs_count;
  in.candidate_id = candidate.candidate_panel_id;
  in.candidate_prob = candidate.candidate_prob;
  in.candidate_margin = candidate.candidate_margin;
  if (!all_obs.empty()) {
    for (const auto & o : all_obs) {
      in.obs_z_values.push_back(o.z);
      in.obs_yaw_values.push_back(o.yaw);
    }
  } else {
    in.obs_z_values.push_back(obs.z);
    in.obs_yaw_values.push_back(obs.yaw);
  }
  in.z_jump = candidate.z_jump;
  in.has_z_jump = candidate.has_z_jump;
  in.yaw_rate_est = ctx.yaw_rate;
  in.spin_direction_hint = ctx.spin_direction;
  in.selected_yaw_err = candidate.selected_yaw_err;
  in.cost_margin = candidate.candidate_margin;
  in.same_panel_residual = candidate.selected_xy_residual;
  in.has_history = ctx.last_obs_z.has_value();
  in.lost_frames = std::max(0, ctx.lost_frames);
  in.is_reacquired = in.lost_frames > 0;
  in.gap_dt = 0.0;
  if (obs.timestamp.has_value() && ctx.last_timestamp.has_value()) {
    in.gap_dt = std::max(0.0, obs.timestamp.value() - ctx.last_timestamp.value());
  }

  const double reacquire_dt_gate = std::max(0.12, 3.0 * cfg_.dt);
  const bool reacquire_by_gap = in.gap_dt > reacquire_dt_gate;
  const bool continuity_gate =
      std::isfinite(in.selected_yaw_err) && std::isfinite(in.same_panel_residual) &&
      (in.selected_yaw_err <= std::max(1e-3, cfg_.outpost.binding_same_panel_yaw_gate)) &&
      (in.same_panel_residual <= std::max(1e-3, cfg_.outpost.binding_same_panel_xy_gate));

  if (in.is_reacquired || reacquire_by_gap) {
    in.event_type = binder::TrackEventType::REACQUIRE;
  } else if (continuity_gate) {
    in.event_type = binder::TrackEventType::CONTINUITY;
  } else if (candidate.candidate_margin >=
             std::max(0.01, cfg_.outpost.binding_min_candidate_margin)) {
    in.event_type = binder::TrackEventType::SWITCH_CANDIDATE;
  } else {
    in.event_type = binder::TrackEventType::AMBIGUOUS;
  }

  out = pipeline_->step(in);
  return out;
}

const binder::BinderDebugSnapshot & OutpostBinderBridge::debug_snapshot() const {
  if (!pipeline_) return empty_debug_;
  return pipeline_->debug_snapshot();
}

BinderConfig OutpostBinderBridge::build_binder_config(const UnifiedConfig & cfg) {
  BinderConfig b;
  b.confirm_frames = std::max(1, cfg.outpost.binding_transition_confirm_frames);
  b.lock_new_hold_frames = 2;
  b.force_rebind_bad_frames = std::max(1, cfg.outpost.z_audit_rebind_confirm_frames);
  b.pending_window_frames = std::max(b.confirm_frames + 1, 4);
  b.post_jump_min_confidence =
      std::clamp(cfg.outpost.binding_min_candidate_prob, 0.35, 0.70);
  b.confidence_floor =
      std::clamp(cfg.outpost.binding_confidence_floor, 0.0, 0.95);

  b.z_jump_min = std::max(0.0, cfg.outpost.binding_period_update_min_jump);
  b.dz_match_tolerance = 0.03;
  b.dz_gate = 0.010;
  b.yaw_err_gate = std::max(1e-3, cfg.outpost.binding_same_panel_yaw_gate);
  b.cost_margin_min = std::clamp(cfg.outpost.binding_min_candidate_margin, 0.0, 1.0);
  b.dz_ema_alpha = std::clamp(cfg.outpost.binding_dz_ema_alpha, 0.01, 1.0);

  b.periodic_enable = true;
  b.periodic_window = std::max(3, cfg.outpost.binding_period_window);
  b.periodic_weight = std::max(0.0, cfg.outpost.binding_period_weight);
  b.periodic_min_spin_rate = std::max(0.0, cfg.outpost.binding_period_min_spin_rate);
  b.periodic_update_min_jump = std::max(1e-5, cfg.outpost.binding_period_update_min_jump);
  b.periodic_signature_threshold = 0.60;
  b.reacquire_gap_dt_gate = std::max(0.08, 3.0 * cfg.dt);
  b.reacquire_lost_frames_gate = 1;
  b.z_cluster_ema_alpha = 0.25;
  b.z_cluster_assign_gate = 0.10;

  b.min_candidate_prob = std::clamp(cfg.outpost.binding_min_candidate_prob, 0.0, 1.0);
  b.min_candidate_margin = std::clamp(cfg.outpost.binding_min_candidate_margin, 0.0, 1.0);
  b.switch_strong_score = std::clamp(cfg.outpost.binding_switch_strong_score, 0.0, 1.0);
  b.single_obs_history_window = std::max(3, cfg.outpost.z_history_window);
  b.dual_obs_enable = cfg.outpost.binding_enable_multi_obs;

  // Debug mode for binding diagnosis:
  // disable scorer/force-rebind chain to avoid masking jump/cluster behavior.
  b.scorer_enable = false;
  b.same_panel_yaw_gate = std::max(1e-3, cfg.outpost.binding_same_panel_yaw_gate);
  b.same_panel_z_gate = std::max(1e-3, cfg.outpost.binding_same_panel_z_gate);
  b.same_panel_xy_gate = std::max(1e-3, cfg.outpost.binding_same_panel_xy_gate);

  b.z_audit_rebind_enable = cfg.outpost.z_audit_rebind_enable;
  b.z_audit_rebind_confirm_frames = std::max(1, cfg.outpost.z_audit_rebind_confirm_frames);
  b.z_audit_rebind_min_confidence =
      std::clamp(cfg.outpost.z_audit_rebind_min_confidence, 0.0, 1.0);
  b.z_audit_rebind_min_jump = std::max(0.0, cfg.outpost.z_audit_rebind_min_jump);
  return b;
}

}  // namespace fyt::auto_aim::outpost_v2
