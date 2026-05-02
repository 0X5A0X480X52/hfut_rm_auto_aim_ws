// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_SCORER_RESIDUAL_HYPOTHESIS_SCORER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_SCORER_RESIDUAL_HYPOTHESIS_SCORER_HPP_

#include "max_entropy_tracker/binder/scorer/hypothesis_scorer.hpp"

namespace fyt::auto_aim::binder {

struct ResidualHypothesisScorerConfig {
  double same_panel_yaw_gate = 0.35;
  double same_panel_z_gate = 0.08;
  double same_panel_xy_gate = 0.18;
  int consecutive_bad_threshold = 10;
};

class ResidualHypothesisScorer : public HypothesisScorer {
 public:
  explicit ResidualHypothesisScorer(
      const ResidualHypothesisScorerConfig & config);
  BindingHealth evaluate(const BinderFrameInput & input,
                         const BinderOutput & output,
                         ScorerContext & ctx) override;
  const char * name() const override { return "ResidualHypothesisScorer"; }

 private:
  ResidualHypothesisScorerConfig config_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_SCORER_RESIDUAL_HYPOTHESIS_SCORER_HPP_
