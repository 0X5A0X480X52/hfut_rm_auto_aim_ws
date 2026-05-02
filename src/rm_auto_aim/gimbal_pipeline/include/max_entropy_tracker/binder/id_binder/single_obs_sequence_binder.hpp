// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_SINGLE_OBS_SEQUENCE_BINDER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_SINGLE_OBS_SEQUENCE_BINDER_HPP_

#include "max_entropy_tracker/binder/id_binder/id_binder.hpp"

namespace fyt::auto_aim::binder {

struct SingleObsSequenceBinderConfig {
  int history_window = 8;
  double dz_gate = 0.010;
  int pending_confirm_window = 5;
  double pending_ema_alpha = 0.35;
  double pending_ratio_min = 0.5;
  double pending_ratio_max = 2.5;
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
  struct PendingSwitchState {
    bool active = false;
    int target_id = -1;
    int ttl = 0;
    int consistent_hits = 0;
    double ref_abs_jump = 0.0;
    double ema_abs_jump = 0.0;
    double seed_confidence = 0.0;
  };

  void activate_pending_switch(int target_id, const JumpDecision & jump,
                               const BinderFrameInput & input);
  bool update_pending_switch(const BinderFrameInput & input);

  SingleObsSequenceBinderConfig config_;
  PendingSwitchState pending_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_SINGLE_OBS_SEQUENCE_BINDER_HPP_
