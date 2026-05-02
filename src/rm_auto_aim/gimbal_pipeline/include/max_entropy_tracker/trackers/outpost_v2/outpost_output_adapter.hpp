// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_OUTPUT_ADAPTER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_OUTPUT_ADAPTER_HPP_

#include <array>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/mode/mode_enums.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_backend_interface.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_runtime_context.hpp"

namespace fyt::auto_aim::outpost_v2 {

// Forward declared in outpost_ambiguous_backend.hpp
struct AmbiguousArmorSnapshot;

/// Bundled input for update_publish_state so the adapter can route
/// between STRUCTURED (center-centric) and AMBIGUOUS (single-armor).
struct PublishStateInput {
  const BackendStateSnapshot *backend_snap = nullptr;
  const AmbiguousArmorSnapshot *armor_snap = nullptr;
  mode::TrackMode mode = mode::TrackMode::AMBIGUOUS;
};

class OutpostOutputAdapter {
 public:
  explicit OutpostOutputAdapter(const UnifiedConfig &cfg);

  /// Fill publish_pos/vel/yaw/yaw_rate and center_* fields in ctx.
  /// - AMBIGUOUS (with valid armor_snap): publish = armor raw state
  /// - STRUCTURED or fallback: center-centric
  void update_publish_state(OutpostRuntimeContext *ctx,
                            const PublishStateInput &input) const;

  std::vector<geometry_msgs::msg::Pose> build_armors_offset_for_message(
      const OutpostRuntimeContext &ctx) const;

 private:
  void fill_from_armor(OutpostRuntimeContext *ctx,
                       const AmbiguousArmorSnapshot &snap) const;
  void fill_from_center(OutpostRuntimeContext *ctx,
                        const BackendStateSnapshot &snap) const;

  std::array<double, 3> z_offsets_{0.06, 0.0, -0.06};
  std::array<double, 3> panel_angles_{0.0, 2.0 * M_PI / 3.0, -2.0 * M_PI / 3.0};
  double radius_ = 0.26;
  bool publish_single_semantics_ = true;
  bool ambiguous_zero_offset_ = true;
};

}  // namespace fyt::auto_aim::outpost_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_OUTPUT_ADAPTER_HPP_
