// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_RUNTIME_CONTEXT_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_RUNTIME_CONTEXT_HPP_

#include <optional>

#include <Eigen/Dense>

#include "max_entropy_tracker/mode/mode_enums.hpp"

namespace fyt::auto_aim::outpost_v2 {

struct OutpostRuntimeContext {
  mode::TrackMode mode = mode::TrackMode::AMBIGUOUS;

  int selected_panel_id = -1;
  int bound_panel_id = -1;
  double binding_confidence = 0.0;

  Eigen::Vector3d center_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_vel = Eigen::Vector3d::Zero();
  double center_yaw = 0.0;
  double yaw_rate = 0.0;

  Eigen::Vector3d publish_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d publish_vel = Eigen::Vector3d::Zero();

  double entropy_norm = 1.0;
  double max_prob = 0.0;
  int spin_direction = 0;

  std::optional<double> last_timestamp;
  std::optional<double> last_obs_z;
  int lost_frames = 0;
};

}  // namespace fyt::auto_aim::outpost_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_RUNTIME_CONTEXT_HPP_
