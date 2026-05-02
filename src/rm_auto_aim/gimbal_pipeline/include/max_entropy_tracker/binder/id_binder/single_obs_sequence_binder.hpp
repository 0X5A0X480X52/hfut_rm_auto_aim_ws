// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_SINGLE_OBS_SEQUENCE_BINDER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_SINGLE_OBS_SEQUENCE_BINDER_HPP_

#include "max_entropy_tracker/binder/id_binder/id_binder.hpp"

namespace fyt::auto_aim::binder {

struct SingleObsSequenceBinderConfig {
  int history_window = 8;
  double dz_gate = 0.010;
};

class SingleObsSequenceBinder : public IDBinder {
 public:
  explicit SingleObsSequenceBinder(
      const SingleObsSequenceBinderConfig & config);
  TargetDecision propose(const BinderFrameInput & input,
                         const JumpDecision & jump,
                         const BinderContext & ctx) override;
  const char * name() const override { return "SingleObsSequenceBinder"; }

 private:
  SingleObsSequenceBinderConfig config_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_SINGLE_OBS_SEQUENCE_BINDER_HPP_
