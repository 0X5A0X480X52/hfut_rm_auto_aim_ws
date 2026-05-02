// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_DUAL_OBS_DIRECT_BINDER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_DUAL_OBS_DIRECT_BINDER_HPP_

#include "max_entropy_tracker/binder/id_binder/id_binder.hpp"

namespace fyt::auto_aim::binder {

class DualObsDirectBinder : public IDBinder {
 public:
  TargetDecision propose(const BinderFrameInput & input,
                         const JumpDecision & jump,
                         const BinderContext & ctx) override;
  const char * name() const override { return "DualObsDirectBinder"; }
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_DUAL_OBS_DIRECT_BINDER_HPP_
