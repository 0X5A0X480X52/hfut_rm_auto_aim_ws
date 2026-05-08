// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_SCORER_SOFT_FUSION_SCORER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_SCORER_SOFT_FUSION_SCORER_HPP_

#include <memory>

#include "max_entropy_tracker/binder/scorer/hypothesis_scorer.hpp"

namespace fyt::auto_aim::binder {

struct SoftFusionConfig {
  double w_seq = 0.25;
  double w_geo = 0.40;
  double w_dyn = 0.20;
  double w_continuity = 0.15;
  double w_topology = 0.15;
};

/// Phase 6: wraps an inner scorer and adds track_continuity_cost and
/// phase_transition_cost from 2D/proxy soft fusion evidence.
///
/// phase_score = w_seq * sequence_consistency
///             + w_geo * yaw_xy_consistency
///             + w_dyn * kinematic_consistency
///             + w_continuity * track_continuity_score
///             - w_topology * topology_transition_cost
///
/// The resulting confidence penalty is blended into the inner scorer's
/// health score when soft fusion fields are present in BinderFrameInput.
class SoftFusionScorer : public HypothesisScorer {
 public:
  SoftFusionScorer(std::unique_ptr<HypothesisScorer> inner,
                   const SoftFusionConfig &config);

  BindingHealth evaluate(const BinderFrameInput &input,
                         const BinderOutput &output,
                         ScorerContext &ctx) override;

  const char *name() const override { return "SoftFusionScorer"; }

  /// Access the inner scorer for read-only inspection.
  const HypothesisScorer *inner() const { return inner_.get(); }

 private:
  double compute_phase_confidence_penalty(const BinderFrameInput &input) const;
  double compute_transition_cost(int current_panel, int proposed_panel) const;
  void fuse_into_health(BindingHealth &health, const BinderFrameInput &input,
                        const BinderOutput &output) const;

  std::unique_ptr<HypothesisScorer> inner_;
  SoftFusionConfig config_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_SCORER_SOFT_FUSION_SCORER_HPP_
