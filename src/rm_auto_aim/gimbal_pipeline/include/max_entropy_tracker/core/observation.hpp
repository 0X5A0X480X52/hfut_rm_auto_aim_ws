// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_CORE_OBSERVATION_HPP_
#define MAX_ENTROPY_TRACKER_CORE_OBSERVATION_HPP_

#include <Eigen/Dense>
#include <optional>

namespace fyt::auto_aim {

/// Observation from a single armor plate
struct ObservationData {
  // Required
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;

  // Optional metadata
  std::optional<int> panel_id;
  std::optional<std::string> layer;
  double confidence = 1.0;
  std::optional<double> timestamp;

  // 来源相机 frame_id
  std::string source_frame;

  Eigen::Vector3d position() const { return {x, y, z}; }

  Eigen::Vector4d as_4d() const { return {x, y, z, yaw}; }

  static ObservationData from_4d(const Eigen::Vector4d &v) {
    ObservationData obs;
    obs.x = v(0);
    obs.y = v(1);
    obs.z = v(2);
    obs.yaw = v(3);
    return obs;
  }
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_CORE_OBSERVATION_HPP_
