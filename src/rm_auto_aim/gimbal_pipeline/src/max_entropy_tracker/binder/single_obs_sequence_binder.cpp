// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/id_binder/single_obs_sequence_binder.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

namespace {

int infer_step_from_jump(const JumpDecision & jump) {
  if (!jump.detected) return 0;
  if (jump.jump_kind == JumpKind::DOUBLE_DZ) return 2;
  if (jump.jump_kind == JumpKind::DZ) return 1;
  return 0;
}

int next_id_from_cyclic(const std::vector<int> & order, int from_id, int step) {
  if (order.empty() || from_id < 0 || step == 0) return -1;
  int idx = -1;
  for (size_t i = 0; i < order.size(); ++i) {
    if (order[i] == from_id) {
      idx = static_cast<int>(i);
      break;
    }
  }
  if (idx < 0) return -1;
  const int n = static_cast<int>(order.size());
  int to_idx = (idx + step) % n;
  if (to_idx < 0) to_idx += n;
  return order[to_idx];
}

HeightLabel infer_height_label(const BinderFrameInput & input,
                               const JumpDecision & jump, int target_id,
                               const SingleObsSequenceBinderConfig & config) {
  if (!input.profile) return HeightLabel::UNKNOWN;
  if (jump.detected && input.has_z_jump) {
      const double dz_gate = std::max(0.0, config.dz_gate);
      if (input.profile->height_levels == 2) {
        if (input.z_jump > dz_gate) return HeightLabel::UPPER;
        if (input.z_jump < -dz_gate) return HeightLabel::LOWER;
      }
  }
  return input.profile->height_label_for(target_id);
}

}  // namespace

SingleObsSequenceBinder::SingleObsSequenceBinder(
    const SingleObsSequenceBinderConfig & config)
    : config_(config) {}

void SingleObsSequenceBinder::activate_pending_switch(
    int target_id, const JumpDecision & jump, const BinderFrameInput & input) {
  if (target_id < 0) return;
  pending_ = PendingSwitchState{};
  pending_.active = true;
  pending_.target_id = target_id;
  pending_.ttl = std::max(1, config_.pending_confirm_window);
  pending_.seed_confidence = jump.confidence;
  if (input.has_z_jump && std::isfinite(input.z_jump)) {
    pending_.ref_abs_jump = std::abs(input.z_jump);
    pending_.ema_abs_jump = pending_.ref_abs_jump;
  } else {
    pending_.ref_abs_jump = std::max(1e-4, std::abs(config_.dz_gate));
    pending_.ema_abs_jump = pending_.ref_abs_jump;
  }
}

bool SingleObsSequenceBinder::update_pending_switch(const BinderFrameInput & input) {
  if (!pending_.active) return false;
  if (pending_.ttl <= 0) {
    pending_.active = false;
    return false;
  }

  if (input.has_z_jump && std::isfinite(input.z_jump)) {
    const double alpha = std::clamp(config_.pending_ema_alpha, 0.05, 1.0);
    const double abs_jump = std::abs(input.z_jump);
    pending_.ema_abs_jump = (1.0 - alpha) * pending_.ema_abs_jump + alpha * abs_jump;
  }

  const double denom = std::max(1e-4, pending_.ref_abs_jump);
  const double ratio = pending_.ema_abs_jump / denom;
  const bool ratio_ok = ratio >= std::max(0.1, config_.pending_ratio_min) &&
                        ratio <= std::max(config_.pending_ratio_min + 0.1,
                                          config_.pending_ratio_max);
  const bool candidate_not_contradict =
      (input.candidate_id < 0 || input.candidate_id == pending_.target_id ||
       input.candidate_prob < 0.7);
  const bool consistent = ratio_ok && candidate_not_contradict;

  if (consistent) ++pending_.consistent_hits;
  --pending_.ttl;

  if (pending_.ttl <= 0) {
    pending_.active = false;
  }
  return consistent;
}

TargetDecision SingleObsSequenceBinder::propose(
    const BinderFrameInput & input, const JumpDecision & jump,
    const BinderContext & ctx) {
  TargetDecision td;
  td.target_id = input.candidate_id;
  td.confidence = input.candidate_prob;

  if (jump.detected) {
    if (jump.to_id >= 0) {
      td.target_id = jump.to_id;
      td.confidence = std::max(td.confidence, jump.confidence);
      activate_pending_switch(td.target_id, jump, input);
    } else if (input.profile && !input.profile->cyclic_order.empty() &&
               ctx.current_bound_id >= 0) {
      int step = infer_step_from_jump(jump);
      if (step != 0) {
        if (input.spin_direction_hint < 0) step = -step;
        const int inferred =
            next_id_from_cyclic(input.profile->cyclic_order,
                                ctx.current_bound_id, step);
        if (inferred >= 0) {
          td.target_id = inferred;
          td.confidence = std::max(td.confidence, 0.55 * jump.confidence);
          activate_pending_switch(td.target_id, jump, input);
        }
      }
    }
  } else if (update_pending_switch(input)) {
    td.target_id = pending_.target_id;
    td.confidence = std::max(td.confidence, 0.55 + 0.30 * pending_.seed_confidence);
  } else if (ctx.current_bound_id >= 0 && input.candidate_prob < 0.45) {
    // Keep lock under weak evidence when no jump event is observed.
    td.target_id = ctx.current_bound_id;
    td.confidence = std::max(td.confidence, 0.4);
  }

  td.height_label = infer_height_label(input, jump, td.target_id, config_);
  if (td.height_label == HeightLabel::UNKNOWN) {
    td.height_label = input.profile ? input.profile->height_label_for(td.target_id)
                                    : HeightLabel::LOWER;
  }

  return td;
}

}  // namespace fyt::auto_aim::binder
