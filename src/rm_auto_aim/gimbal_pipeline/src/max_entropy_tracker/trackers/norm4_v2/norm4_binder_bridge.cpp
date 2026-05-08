// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v2/norm4_binder_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "max_entropy_tracker/binder/factory/binder_factory.hpp"

namespace fyt::auto_aim::norm4_v2 {
namespace {

int left_neighbor(int panel_id) {
  const int p = ((panel_id % 4) + 4) % 4;
  return (p + 3) % 4;
}

int right_neighbor(int panel_id) {
  const int p = ((panel_id % 4) + 4) % 4;
  return (p + 1) % 4;
}

bool find_track_center_x_for_observation(
    const evidence::ArmorEvidenceFrame &frame, int observation_index,
    double *center_x) {
  if (observation_index < 0 || center_x == nullptr) return false;
  for (const auto &te : frame.track2d_evidence) {
    if (!te.valid) continue;
    if (te.observation_index != observation_index) continue;
    if (!std::isfinite(te.center_x)) continue;
    *center_x = te.center_x;
    return true;
  }
  return false;
}

double compute_topology_consistency_score(
    const norm4_v2::BindingCandidate &candidate,
    const evidence::ArmorEvidenceFrame &frame) {
  if (candidate.candidate_panel_id < 0) return 1.0;
  if (candidate.obs_panel_ids.size() < 2) return 1.0;

  double x0 = std::numeric_limits<double>::quiet_NaN();
  double x1 = std::numeric_limits<double>::quiet_NaN();
  if (!find_track_center_x_for_observation(frame, 0, &x0) ||
      !find_track_center_x_for_observation(frame, 1, &x1)) {
    return 1.0;
  }

  const int p0 = candidate.obs_panel_ids[0];
  const int p1 = candidate.obs_panel_ids[1];
  if (p0 < 0 || p1 < 0) return 1.0;

  if (std::abs(x0 - x1) < 1e-3) return 1.0;
  const bool obs1_on_left = (x1 < x0);
  const int expected_p1 = obs1_on_left ? left_neighbor(p0) : right_neighbor(p0);
  return (expected_p1 == p1) ? 1.0 : 0.0;
}

}  // namespace

Norm4BinderBridge::Norm4BinderBridge(const UnifiedConfig &cfg)
    : cfg_(cfg),
      profile_(binder::RobotBindingProfileProvider::from_robot_id("2")) {
  pipeline_ = binder::BinderFactory::create(profile_, build_binder_config());
}

BinderConfig Norm4BinderBridge::build_binder_config() const {
  BinderConfig c = cfg_.binder;

  c.confirm_frames = std::max(1, cfg_.tracker.jump_binding_confirm_frames);
  c.lock_new_hold_frames = std::max(0, cfg_.tracker.jump_binding_switch_cooldown);
  c.z_jump_min = std::max(0.0, cfg_.tracker.jump_binding_z_jump_min);
  c.dz_match_tolerance =
      std::max(0.0, cfg_.tracker.jump_binding_dz_match_tolerance);
  c.dz_gate = std::max(0.0, cfg_.tracker.jump_binding_dz_gate);
  c.yaw_err_gate = std::max(1e-3, cfg_.tracker.jump_binding_yaw_err_gate);
  c.cost_margin_min = std::max(0.0, cfg_.tracker.jump_binding_cost_margin_min);
  c.dz_ema_alpha = std::clamp(cfg_.tracker.jump_binding_dz_ema_alpha, 0.01, 1.0);
  c.confidence_floor =
      std::clamp(cfg_.tracker.jump_binding_confidence_floor, 0.0, 0.95);
  c.periodic_enable = cfg_.tracker.periodic_binding_enable;
  c.periodic_weight = std::max(0.0, cfg_.tracker.periodic_binding_weight);
  c.periodic_min_spin_rate =
      std::max(0.0, cfg_.tracker.periodic_binding_spin_rate_gate);
  c.pending_window_frames =
      (c.pending_window_frames > 0) ? c.pending_window_frames : (c.confirm_frames + 1);
  return c;
}

void Norm4BinderBridge::reset(int init_panel_id, binder::HeightLabel init_label,
                              std::optional<double> obs_z) {
  if (!pipeline_) return;
  pipeline_->reset(init_panel_id, init_label, obs_z);
  debug_ = pipeline_->debug_snapshot();
}

binder::TrackEventType Norm4BinderBridge::infer_event_type(
    const BindingCandidate &candidate, const Norm4RuntimeContext &ctx) {
  if (ctx.lost_frames > 0) {
    return binder::TrackEventType::REACQUIRE;
  }
  if (candidate.candidate_panel_id < 0) {
    return binder::TrackEventType::AMBIGUOUS;
  }
  if (ctx.bound_panel_id >= 0 && candidate.candidate_panel_id != ctx.bound_panel_id) {
    return binder::TrackEventType::SWITCH_CANDIDATE;
  }
  return binder::TrackEventType::CONTINUITY;
}

binder::BinderOutput Norm4BinderBridge::step(
    const ObservationData &obs, const std::vector<ObservationData> &all_obs,
    int obs_count, const BindingCandidate &candidate,
    const Norm4RuntimeContext &ctx) {
  binder::BinderOutput out;
  if (!pipeline_) return out;

  binder::BinderFrameInput in;
  in.timestamp = obs.timestamp.value_or(ctx.last_timestamp.value_or(0.0));
  in.profile = &profile_;
  in.obs_count = obs_count;
  in.candidate_id = candidate.candidate_panel_id;
  in.candidate_prob = candidate.candidate_prob;
  in.candidate_margin = candidate.candidate_margin;

  for (const auto &o : all_obs) {
    in.obs_z_values.push_back(o.z);
    in.obs_yaw_values.push_back(o.yaw);
  }
  in.obs_panel_ids = candidate.obs_panel_ids;
  in.obs_height_labels = candidate.obs_height_labels;
  if (in.obs_z_values.empty()) {
    in.obs_z_values.push_back(obs.z);
    in.obs_yaw_values.push_back(obs.yaw);
  }

  in.z_jump = candidate.z_jump;
  in.has_z_jump = candidate.has_z_jump;
  in.yaw_rate_est = ctx.yaw_rate;
  in.spin_direction_hint = ctx.spin_direction;
  in.selected_yaw_err = candidate.selected_yaw_err;
  in.cost_margin = candidate.cost_margin;
  in.same_panel_residual = candidate.selected_xy_residual;
  in.has_history = ctx.last_timestamp.has_value();
  in.event_type = infer_event_type(candidate, ctx);
  in.is_reacquired = ctx.lost_frames > 0;
  in.gap_dt = 0.0;
  in.lost_frames = std::max(0, ctx.lost_frames);

  // Phase 6: soft fusion fields from runtime context.
  in.phase_confidence = ctx.binding_confidence;
  in.ping_pong_risk = ctx.ping_pong_risk_score;
  in.topology_consistency_score =
      compute_topology_consistency_score(candidate, ctx.evidence_frame);
  if (ctx.ping_pong_pending || ctx.ping_pong_should_hold) {
    in.track_continuity_score = 1.0 - std::min(1.0, ctx.ping_pong_risk_score);
    in.kinematic_consistency = 1.0 - std::min(1.0, ctx.ping_pong_risk_score);
  }
  double vel_cos = 1.0;
  double acc_n = 0.0;
  for (const auto &pe : ctx.evidence_frame.proxy_evidence) {
    if (pe.valid) {
      vel_cos = std::max(vel_cos, pe.kin_summary.velocity_dir_cos);
      acc_n = std::max(acc_n, pe.kin_summary.acc_norm_window);
    }
  }
  in.velocity_dir_cos = vel_cos;
  in.acc_norm = acc_n;
  in.has_soft_fusion =
      (cfg_.binder.enable_soft_fusion &&
       (ctx.evidence_frame.completeness.has_proxy ||
        ctx.evidence_frame.completeness.has_2d_tracks));

  out = pipeline_->step(in);
  debug_ = pipeline_->debug_snapshot();
  return out;
}

const binder::BinderDebugSnapshot &Norm4BinderBridge::debug_snapshot() const {
  return debug_;
}

}  // namespace fyt::auto_aim::norm4_v2
