// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/id_binder/dual_obs_direct_binder.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

TargetDecision DualObsDirectBinder::propose(
    const BinderFrameInput & input, const JumpDecision & jump,
    const BinderContext & ctx) {
  TargetDecision td;

  if (input.obs_count < 2 || input.obs_z_values.size() < 2) {
    td.target_id = input.candidate_id;
    td.confidence = 0.0;  // signal fallback
    return td;
  }

  // Norm4 dual-observation path can provide per-observation panel ids.
  // Keep the current lock if one of the two visible armors is already bound;
  // the second armor is then consumed by the structured dual update instead
  // of forcing the binder to collapse both observations into one id.
  if (input.obs_panel_ids.size() >= 2) {
    if (ctx.current_bound_id >= 0) {
      for (size_t i = 0; i < input.obs_panel_ids.size(); ++i) {
        if (input.obs_panel_ids[i] == ctx.current_bound_id) {
          td.target_id = ctx.current_bound_id;
          td.confidence = std::max(input.candidate_prob, 0.80);
          if (i < input.obs_height_labels.size()) {
            td.height_label = input.obs_height_labels[i];
          } else if (input.profile) {
            td.height_label = input.profile->height_label_for(td.target_id);
          }
          return td;
        }
      }
    }

    td.target_id = input.candidate_id;
    td.confidence = std::max(input.candidate_prob, 0.70);
    for (size_t i = 0; i < input.obs_panel_ids.size(); ++i) {
      if (input.obs_panel_ids[i] == td.target_id &&
          i < input.obs_height_labels.size()) {
        td.height_label = input.obs_height_labels[i];
        return td;
      }
    }
  }

  // With dual obs, prefer the jump decision's to_id if available.
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
