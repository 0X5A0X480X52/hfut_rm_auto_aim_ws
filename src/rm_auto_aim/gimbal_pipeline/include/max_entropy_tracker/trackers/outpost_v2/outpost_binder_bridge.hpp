// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_BINDER_BRIDGE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_BINDER_BRIDGE_HPP_

#include <memory>
#include <optional>

#include "max_entropy_tracker/binder/debug/binder_debug_snapshot.hpp"
#include "max_entropy_tracker/binder/model/robot_binding_profile.hpp"
#include "max_entropy_tracker/binder/policy/outpost_legacy_binding_policy.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_observation_frontend.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_runtime_context.hpp"

namespace fyt::auto_aim::outpost_v2 {

class OutpostBinderBridge {
 public:
  explicit OutpostBinderBridge(const UnifiedConfig & cfg);

  void reset(int init_panel_id, std::optional<double> obs_z);

  binder::BinderOutput step(
      const ObservationData & obs,
      const std::vector<ObservationData> & all_obs,
      int obs_count,
      BindingCandidate & candidate,
      const OutpostRuntimeContext & ctx);

  const binder::BinderDebugSnapshot & debug_snapshot() const;

 private:
  UnifiedConfig cfg_;
  binder::RobotBindingProfile profile_;
  std::unique_ptr<binder::OutpostLegacyBindingPolicy> policy_;
  binder::BinderDebugSnapshot debug_;
};

}  // namespace fyt::auto_aim::outpost_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_BINDER_BRIDGE_HPP_
