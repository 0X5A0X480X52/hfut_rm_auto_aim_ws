// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_SINGLE_SINGLE_TRACKER_TYPES_HPP_
#define MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_SINGLE_SINGLE_TRACKER_TYPES_HPP_

#include <Eigen/Dense>

#include "max_entropy_tracker/core/ids.hpp"

namespace fyt::auto_aim {

/// Rolling-window kinematic summary for anti-pingpong and mode quality scoring.
struct KinematicSummary {
  double velocity_var_window = 0.0;
  double acc_norm_window = 0.0;
  double yaw_rate_continuity_score = 1.0;
  double jerk_estimate = 0.0;
  double velocity_dir_cos = 1.0;
  bool yaw_rate_converged = false;
};

/// Per-track2d-id single-armor evidence produced by SingleArmorProxyManager.
struct SingleArmorTrackEvidence {
  bool valid = false;
  Track2DId track2d_id = -1;
  Track3DId proxy_id = -1;

  Eigen::Vector3d armor_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d armor_vel = Eigen::Vector3d::Zero();
  double armor_yaw = 0.0;
  double armor_yaw_rate = 0.0;

  int panel_id = -1;

  int age = 0;
  int hits = 0;
  int missed = 0;
  bool initialized = false;

  KinematicSummary kin_summary;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_POSE_TRACKER_BACKEND_SINGLE_SINGLE_TRACKER_TYPES_HPP_
