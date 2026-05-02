// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/id_binder/dual_obs_direct_binder.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

TargetDecision DualObsDirectBinder::propose(
    const BinderFrameInput & input, const JumpDecision & jump,
    const BinderContext & /*ctx*/) {
  TargetDecision td;

  if (input.obs_count < 2 || input.obs_z_values.size() < 2) {
    td.target_id = input.candidate_id;
    td.confidence = 0.0;  // signal fallback
    return td;
  }

  // With dual obs, prefer the jump decision's to_id if available
  if (jump.detected && jump.to_id >= 0) {
    td.target_id = jump.to_id;
    td.confidence = std::max(jump.confidence, input.candidate_prob);
  } else {
    td.target_id = input.candidate_id;
    td.confidence = input.candidate_prob;
  }

  if (input.profile) {
    td.height_label = input.profile->height_label_for(td.target_id);
  }

  return td;
}

}  // namespace fyt::auto_aim::binder
