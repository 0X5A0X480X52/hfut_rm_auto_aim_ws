// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_MODE_EVIDENCE_FUSER_HPP_
#define MAX_ENTROPY_TRACKER_MODE_EVIDENCE_FUSER_HPP_

#include "max_entropy_tracker/binder/debug/binder_debug_snapshot.hpp"
#include "max_entropy_tracker/mode/mode_types.hpp"

namespace fyt::auto_aim::mode {

struct EvidenceFuserConfig {
  // Base structured-evidence mix.
  double w_dual = 0.25;
  double w_margin = 0.35;
  double w_health = 0.25;
  double w_entropy = 0.15;

  // Jump is an event pulse, not a persistent per-frame evidence.
  double jump_event_weight = 0.35;
  double jump_event_refractory_s = 0.06;
  double signature_2dz_bonus = 0.15;
};

class EvidenceFuser {
 public:
  explicit EvidenceFuser(const EvidenceFuserConfig & cfg);

  ModeEvidence fuse(
      double timestamp,
      int obs_count,
      int candidate_id,
      bool has_2dz_signature,
      double entropy_norm,
      double max_prob,
      double candidate_margin,
      const binder::BinderDebugSnapshot & binder_dbg);

 private:
  EvidenceFuserConfig cfg_;
  bool last_jump_detected_ = false;
  double last_jump_event_time_ = -1.0;
};

}  // namespace fyt::auto_aim::mode

#endif  // MAX_ENTROPY_TRACKER_MODE_EVIDENCE_FUSER_HPP_
