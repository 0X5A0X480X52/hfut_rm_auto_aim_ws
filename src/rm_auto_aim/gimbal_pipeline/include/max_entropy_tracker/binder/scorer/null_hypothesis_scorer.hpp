// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_SCORER_NULL_HYPOTHESIS_SCORER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_SCORER_NULL_HYPOTHESIS_SCORER_HPP_

#include "max_entropy_tracker/binder/scorer/hypothesis_scorer.hpp"

namespace fyt::auto_aim::binder {

class NullHypothesisScorer : public HypothesisScorer {
 public:
  BindingHealth evaluate(const BinderFrameInput & /*input*/,
                         const BinderOutput & /*output*/,
                         ScorerContext & /*ctx*/) override {
    return BindingHealth{};
  }
  const char * name() const override { return "NullHypothesisScorer"; }
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_SCORER_NULL_HYPOTHESIS_SCORER_HPP_
