// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v2/outpost_binder_bridge.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::outpost_v2 {

namespace {

binder::JumpKind classify_jump_kind(double abs_jump, double dz_small,
                                    double dz_large) {
  if (!std::isfinite(abs_jump) || abs_jump <= 1e-6) {
    return binder::JumpKind::NONE;
  }
  if (std::isfinite(dz_small) && std::isfinite(dz_large)) {
    const double diff_small = std::abs(abs_jump - dz_small);
    const double diff_large = std::abs(abs_jump - dz_large);
    return (diff_large < diff_small) ? binder::JumpKind::DOUBLE_DZ
                                     : binder::JumpKind::DZ;
  }
  return binder::JumpKind::DZ;
}

}  // namespace

OutpostBinderBridge::OutpostBinderBridge(const UnifiedConfig & cfg)
    : cfg_(cfg),
      profile_(binder::RobotBindingProfileProvider::from_robot_id(
          "outpost", {cfg.outpost.z_offset_0, cfg.outpost.z_offset_1,
                      cfg.outpost.z_offset_2})) {
  policy_ = std::make_unique<binder::OutpostLegacyBindingPolicy>(cfg_, profile_);
}

void OutpostBinderBridge::reset(int init_panel_id, std::optional<double> obs_z) {
  if (!policy_) return;
  policy_->reset(init_panel_id, obs_z);
  debug_ = binder::BinderDebugSnapshot{};
  debug_.valid = true;
  debug_.decoder_name = "OutpostLegacyBindingPolicy";
  debug_.id_binder_name = "LegacyHypothesisBinding";
  debug_.scorer_name = "LegacyOutpostScores";
}

binder::BinderOutput OutpostBinderBridge::step(
    const ObservationData & obs, const std::vector<ObservationData> & /*all_obs*/,
    int obs_count, BindingCandidate & candidate,
    const OutpostRuntimeContext & ctx) {
  binder::BinderOutput out;
  if (!policy_) return out;

  binder::OutpostLegacyBindingInput in;
  in.obs = &obs;
  in.obs_count = obs_count;
  in.predicted_center_pos = ctx.center_pos;
  in.predicted_center_yaw = ctx.center_yaw;
  in.yaw_rate_est = ctx.yaw_rate;
  in.ambiguous_mode = (ctx.mode == mode::TrackMode::AMBIGUOUS);
  in.lost_frames = std::max(0, ctx.lost_frames);
  in.last_timestamp = ctx.last_timestamp;
  in.last_obs_z = ctx.last_obs_z;

  const auto policy_out = policy_->step(in);
  out = policy_out.binder_output;

  candidate.candidate_panel_id = policy_out.candidate_id;
  candidate.candidate_prob = policy_out.candidate_prob;
  candidate.candidate_margin = policy_out.candidate_margin;
  candidate.max_prob = policy_out.max_prob;
  candidate.entropy_norm = policy_out.entropy_norm;
  candidate.selected_xy_residual = policy_out.selected_xy_residual;
  candidate.z_jump = policy_out.z_jump;
  candidate.has_z_jump = std::isfinite(policy_out.z_jump);
  for (int i = 0; i < 3; ++i) {
    candidate.costs[i] = policy_out.hypotheses[i].cost;
    candidate.probs[i] = policy_out.hypotheses[i].probability;
    if (policy_out.hypotheses[i].panel_id == policy_out.candidate_id) {
      candidate.selected_yaw_err = policy_out.hypotheses[i].yaw_err;
    }
  }

  debug_ = binder::BinderDebugSnapshot{};
  debug_.valid = true;
  debug_.decoder_name = "OutpostLegacyBindingPolicy";
  debug_.id_binder_name = "LegacyHypothesisBinding";
  debug_.scorer_name = "LegacyOutpostScores";
  debug_.jump_detected = out.switch_occurred;
  debug_.jump_kind = classify_jump_kind(std::abs(policy_out.z_jump),
                                        policy_out.dz_small_est,
                                        policy_out.dz_large_est);
  debug_.jump_confidence = out.switch_occurred ? out.binding_confidence : 0.0;
  debug_.from_id = ctx.bound_panel_id;
  debug_.to_id = out.selected_id;
  debug_.target_id = out.selected_id;
  debug_.target_confidence = out.binding_confidence;
  debug_.fsm_state = out.fsm_state;
  debug_.action = out.action;
  debug_.switch_reason = out.switch_reason;
  debug_.health_score = policy_out.z_audit_confidence;
  debug_.consecutive_bad_frames = 0;
  debug_.force_rebind_flag = false;
  debug_.obs_count = obs_count;
  debug_.z_jump = policy_out.z_jump;
  debug_.spin_direction = policy_out.spin_direction;
  debug_.candidate_prob = policy_out.candidate_prob;
  debug_.candidate_margin = policy_out.candidate_margin;
  debug_.dz_small_est = policy_out.dz_small_est;
  debug_.dz_large_est = policy_out.dz_large_est;
  debug_.period_confidence = policy_out.period_confidence;
  debug_.period_phase = policy_out.period_phase;
  debug_.signature_score = policy_out.period_confidence;
  debug_.event_type =
      out.fsm_state == binder::BindingFSMState::PENDING_SWITCH
          ? binder::TrackEventType::SWITCH_CANDIDATE
          : binder::TrackEventType::CONTINUITY;
  debug_.is_reacquired = ctx.lost_frames > 0;
  debug_.gap_dt = 0.0;
  debug_.lost_frames = ctx.lost_frames;

  return out;
}

const binder::BinderDebugSnapshot & OutpostBinderBridge::debug_snapshot() const {
  return debug_;
}

}  // namespace fyt::auto_aim::outpost_v2
