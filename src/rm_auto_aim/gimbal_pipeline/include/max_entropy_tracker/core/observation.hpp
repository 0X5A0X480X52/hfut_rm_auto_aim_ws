// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_CORE_OBSERVATION_HPP_
#define MAX_ENTROPY_TRACKER_CORE_OBSERVATION_HPP_

#include <Eigen/Dense>

#include <array>
#include <optional>
#include <string>

namespace fyt::auto_aim {

/// 2D image-domain metadata carried alongside a 3D observation.
/// All fields are optional; valid == false when the source detector
/// did not provide image geometry.
struct ImageObservation2D {
  bool valid = false;
  int detection_id = -1;

  double bbox_x = 0.0;
  double bbox_y = 0.0;
  double bbox_w = 0.0;
  double bbox_h = 0.0;

  std::array<Eigen::Vector2d, 4> corners{};
  double image_center_x = 0.0;
  double image_center_y = 0.0;

  double detection_confidence = 0.0;
  std::string number;
  std::string type;
};

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

  // 2D evidence (Phase 1: append-only, optional)
  std::optional<ImageObservation2D> image;
  std::optional<int> track2d_id;

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
