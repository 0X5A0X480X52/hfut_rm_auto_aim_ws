// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_BASELINE_OUTPOST_BINDER_BRIDGE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_BASELINE_OUTPOST_BINDER_BRIDGE_HPP_

#include <optional>

#include "max_entropy_tracker/binder/debug/binder_debug_snapshot.hpp"
#include "max_entropy_tracker/binder/policy/outpost_baseline_binding_policy.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/outpost_baseline/outpost_observation_frontend.hpp"
#include "max_entropy_tracker/trackers/outpost_baseline/outpost_runtime_context.hpp"

namespace fyt::auto_aim::outpost_baseline {

class OutpostBinderBridge {
 public:
  explicit OutpostBinderBridge(const UnifiedConfig & cfg);

  void reset(int init_panel_id, std::optional<double> obs_z);

  binder::BinderOutput step(
      const ObservationData & obs,
      int obs_count,
      BindingCandidate & candidate,
      const OutpostRuntimeContext & ctx);

  const binder::BinderDebugSnapshot & debug_snapshot() const;

 private:
  binder::OutpostBaselineBindingPolicy policy_;
  binder::BinderDebugSnapshot debug_;
};

}  // namespace fyt::auto_aim::outpost_baseline

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_BASELINE_OUTPOST_BINDER_BRIDGE_HPP_
