// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_OUTPUT_ADAPTER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_OUTPUT_ADAPTER_HPP_

#include <array>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/mode/mode_enums.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_ambiguous_backend.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_runtime_context.hpp"

namespace fyt::auto_aim::norm4_v2 {

struct PublishStateInput {
  const BackendStateSnapshot *backend_snap = nullptr;
  const AmbiguousArmorSnapshot *armor_snap = nullptr;
  mode::TrackMode mode = mode::TrackMode::AMBIGUOUS;
};

class Norm4OutputAdapter {
 public:
  explicit Norm4OutputAdapter(const UnifiedConfig &cfg);

  void update_publish_state(Norm4RuntimeContext *ctx,
                            const PublishStateInput &input) const;

  std::vector<geometry_msgs::msg::Pose> build_armors_offset_for_message(
      const Norm4RuntimeContext &ctx) const;

 private:
  void fill_from_armor(Norm4RuntimeContext *ctx,
                       const AmbiguousArmorSnapshot &snap) const;
  void fill_from_center(Norm4RuntimeContext *ctx,
                        const BackendStateSnapshot &snap) const;

  std::array<double, 4> panel_angles_{0.0, M_PI / 2.0, M_PI, -M_PI / 2.0};
  bool publish_single_semantics_ = true;
};

}  // namespace fyt::auto_aim::norm4_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_OUTPUT_ADAPTER_HPP_
