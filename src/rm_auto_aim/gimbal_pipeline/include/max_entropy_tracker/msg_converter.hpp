// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_MSG_CONVERTER_HPP_
#define MAX_ENTROPY_TRACKER_MSG_CONVERTER_HPP_

#include <cmath>
#include <optional>
#include <string>

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
