// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_SCORER_HYPOTHESIS_SCORER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_SCORER_HYPOTHESIS_SCORER_HPP_

#include "max_entropy_tracker/binder/model/binder_types.hpp"

namespace fyt::auto_aim::binder {

struct ScorerContext {
  int consecutive_bad_frames = 0;
};

class HypothesisScorer {
 public:
  virtual ~HypothesisScorer() = default;
  virtual BindingHealth evaluate(const BinderFrameInput & input,
                                 const BinderOutput & output,
                                 ScorerContext & ctx) = 0;
  virtual const char * name() const = 0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_SCORER_HYPOTHESIS_SCORER_HPP_
