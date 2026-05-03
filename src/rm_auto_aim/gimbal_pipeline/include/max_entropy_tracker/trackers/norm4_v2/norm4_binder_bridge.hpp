// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_BINDER_BRIDGE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_BINDER_BRIDGE_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "max_entropy_tracker/binder/debug/binder_debug_snapshot.hpp"
#include "max_entropy_tracker/binder/model/robot_binding_profile.hpp"
#include "max_entropy_tracker/binder/pipeline/binder_pipeline.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_observation_frontend.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_runtime_context.hpp"

namespace fyt::auto_aim::norm4_v2 {

class Norm4BinderBridge {
 public:
  explicit Norm4BinderBridge(const UnifiedConfig &cfg);

  void reset(int init_panel_id, binder::HeightLabel init_label,
             std::optional<double> obs_z);

  binder::BinderOutput step(const ObservationData &obs,
                            const std::vector<ObservationData> &all_obs,
                            int obs_count, const BindingCandidate &candidate,
                            const Norm4RuntimeContext &ctx);

  const binder::BinderDebugSnapshot &debug_snapshot() const;

 private:
  BinderConfig build_binder_config() const;
  static binder::TrackEventType infer_event_type(
      const BindingCandidate &candidate, const Norm4RuntimeContext &ctx);

  UnifiedConfig cfg_;
  binder::RobotBindingProfile profile_;
  std::unique_ptr<binder::BinderPipeline> pipeline_;
  binder::BinderDebugSnapshot debug_;
};

}  // namespace fyt::auto_aim::norm4_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_BINDER_BRIDGE_HPP_
