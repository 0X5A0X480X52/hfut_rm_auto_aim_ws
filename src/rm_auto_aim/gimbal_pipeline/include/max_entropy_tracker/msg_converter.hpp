// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_MSG_CONVERTER_HPP_
#define MAX_ENTROPY_TRACKER_MSG_CONVERTER_HPP_

#include <cmath>
#include <optional>
#include <string>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rm_interfaces/msg/armor.hpp>

#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim {

/// Extract yaw from quaternion (ZYX Euler convention).
inline double quaternion_to_yaw(const geometry_msgs::msg::Quaternion &q) {
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

/**
 * Convert Armor message to ObservationData.
 *
 * NOTE: The detector outputs orientation with the Z-axis normal pointing
 *       towards the observer. The tracker expects **radial** yaw (center →
 *       armor), so we add π.
 */
inline ObservationData armor_to_observation(
    const rm_interfaces::msg::Armor &armor,
    std::optional<double> timestamp = std::nullopt) {
  double x = armor.pose.position.x;
  double y = armor.pose.position.y;
  double z = armor.pose.position.z;

  double yaw_normal = quaternion_to_yaw(armor.pose.orientation);
  double yaw_radial = yaw_normal + M_PI;

  ObservationData obs;
  obs.x = x;
  obs.y = y;
  obs.z = z;
  obs.yaw = yaw_radial;
  obs.timestamp = timestamp;

  // Phase 1: copy 2D image geometry when available
  if (armor.has_image_geometry) {
    ImageObservation2D img;
    img.valid = true;
    img.detection_confidence = armor.detection_confidence;
    img.number = armor.number;
    img.type = armor.type;
    img.bbox_x = armor.bbox_xywh[0];
    img.bbox_y = armor.bbox_xywh[1];
    img.bbox_w = armor.bbox_xywh[2];
    img.bbox_h = armor.bbox_xywh[3];
    img.image_center_x = img.bbox_x + img.bbox_w * 0.5;
    img.image_center_y = img.bbox_y + img.bbox_h * 0.5;
    for (int i = 0; i < 4 && i < 4; ++i) {
      img.corners[i] = Eigen::Vector2d(
        armor.image_corners[i].x,
        armor.image_corners[i].y);
    }
    obs.image = std::move(img);
  }

  // Phase 1: copy BA/PnP quality metadata when any BA-relevant field is present
  const bool has_ba_meta =
      (armor.pose_estimate_mode != 0) ||
      armor.pose_covariance_valid ||
      (armor.pose_num_points > 0) ||
      (armor.pose_num_inliers > 0) ||
      (armor.pose_quality_score > 0.0f) ||
      (armor.reproj_error_refined > 0.0f) ||
      (armor.pose_condition_number > 0.0f);
  if (has_ba_meta) {
    ObservationCovarianceMeta ba;
    ba.valid = true;
    ba.cov_valid = armor.pose_covariance_valid;
    ba.confidence = static_cast<double>(armor.pose_quality_score);
    ba.reproj_rms = static_cast<double>(armor.reproj_error_refined);
    ba.condition_number = static_cast<double>(armor.pose_condition_number);
    ba.num_observations = static_cast<int>(armor.pose_num_points);
    ba.num_inliers = static_cast<int>(armor.pose_num_inliers);
    ba.pose_estimate_mode = static_cast<int>(armor.pose_estimate_mode);
    ba.frame_aligned = false;
    if (ba.cov_valid) {
      for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
          ba.cov_xyz_yaw(r, c) = static_cast<double>(
              armor.pose_covariance_xyz_yaw[r * 4 + c]);
    }
    obs.ba_pnp = std::move(ba);
  }

  return obs;
}

/// Convert geometry_msgs::Pose → ObservationData (same normal→radial logic).
inline ObservationData pose_to_observation(
    const geometry_msgs::msg::Pose &pose,
    std::optional<double> timestamp = std::nullopt) {
  double yaw_normal = quaternion_to_yaw(pose.orientation);
  double yaw_radial = yaw_normal + M_PI;

  ObservationData obs;
  obs.x = pose.position.x;
  obs.y = pose.position.y;
  obs.z = pose.position.z;
  obs.yaw = yaw_radial;
  obs.timestamp = timestamp;
  return obs;
}

/// Map armor number string to a robot ID.
inline std::string get_robot_id(const std::string &armor_number) {
  return armor_number;  // identity mapping; customise as needed
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_MSG_CONVERTER_HPP_
