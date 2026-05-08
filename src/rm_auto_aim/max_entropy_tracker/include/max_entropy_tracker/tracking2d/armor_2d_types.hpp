// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKING2D_ARMOR_2D_TYPES_HPP_
#define MAX_ENTROPY_TRACKER_TRACKING2D_ARMOR_2D_TYPES_HPP_

#include <Eigen/Dense>

#include <array>
#include <string>

#include "max_entropy_tracker/core/ids.hpp"

namespace fyt::auto_aim {

/// 2D detection input for the image-domain tracker.
struct Armor2DDetection {
  DetectionId detection_id = -1;
  double timestamp = 0.0;

  double bbox_x = 0.0;
  double bbox_y = 0.0;
  double bbox_w = 0.0;
  double bbox_h = 0.0;
  std::array<Eigen::Vector2d, 4> corners{};

  double confidence = 0.0;
  std::string number;
  std::string type;

  int observation_index = -1;
};

/// Per-track 2D evidence produced by the 2D tracker.
struct Armor2DTrackEvidence {
  bool valid = false;
  Track2DId track_id = -1;
  int observation_index = -1;

  double association_quality = 0.0;
  bool confirmed = false;
  int age = 0;
  int hits = 0;
  int missed = 0;

  double center_x = 0.0;
  double center_y = 0.0;
  double velocity_x = 0.0;
  double velocity_y = 0.0;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKING2D_ARMOR_2D_TYPES_HPP_
