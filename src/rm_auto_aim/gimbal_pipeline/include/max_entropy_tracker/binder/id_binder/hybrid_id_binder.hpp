// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_HYBRID_ID_BINDER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_HYBRID_ID_BINDER_HPP_

#include <memory>

#include "max_entropy_tracker/binder/id_binder/dual_obs_direct_binder.hpp"
#include "max_entropy_tracker/binder/id_binder/id_binder.hpp"
#include "max_entropy_tracker/binder/id_binder/single_obs_sequence_binder.hpp"

namespace fyt::auto_aim::binder {

class HybridIDBinder : public IDBinder {
 public:
  HybridIDBinder(std::unique_ptr<SingleObsSequenceBinder> single_binder,
                 std::unique_ptr<DualObsDirectBinder> dual_binder,
                 bool fallback_on_dual_failure = true);
  TargetDecision propose(const BinderFrameInput & input,
                         const JumpDecision & jump,
                         const BinderContext & ctx) override;
  const char * name() const override { return "HybridIDBinder"; }

 private:
  std::unique_ptr<SingleObsSequenceBinder> single_;
  std::unique_ptr<DualObsDirectBinder> dual_;
  bool fallback_on_dual_failure_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_HYBRID_ID_BINDER_HPP_
