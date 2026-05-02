// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/id_binder/hybrid_id_binder.hpp"

namespace fyt::auto_aim::binder {

HybridIDBinder::HybridIDBinder(
    std::unique_ptr<SingleObsSequenceBinder> single_binder,
    std::unique_ptr<DualObsDirectBinder> dual_binder,
    bool fallback_on_dual_failure)
    : single_(std::move(single_binder)),
      dual_(std::move(dual_binder)),
      fallback_on_dual_failure_(fallback_on_dual_failure) {}

TargetDecision HybridIDBinder::propose(
    const BinderFrameInput & input, const JumpDecision & jump,
    const BinderContext & ctx) {
  if (input.obs_count >= 2 && dual_) {
    TargetDecision td = dual_->propose(input, jump, ctx);
    if (td.confidence > 0.0 || !fallback_on_dual_failure_) {
      return td;
    }
  }

  if (single_) {
    return single_->propose(input, jump, ctx);
  }

  TargetDecision td;
  td.target_id = input.candidate_id;
  td.confidence = input.candidate_prob;
  return td;
}

}  // namespace fyt::auto_aim::binder
