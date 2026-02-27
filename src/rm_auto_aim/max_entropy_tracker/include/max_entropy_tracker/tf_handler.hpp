// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TF_HANDLER_HPP_
#define MAX_ENTROPY_TRACKER_TF_HANDLER_HPP_

#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/armor.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/msg_converter.hpp"

namespace fyt::auto_aim {

/// TF2-based coordinate transform handler (camera→odom).
class TFHandler {
 public:
  TFHandler(rclcpp::Node *node, const std::string &target_frame = "odom")
      : target_frame_(target_frame),
        tf_buffer_(node->get_clock()),
        tf_listener_(tf_buffer_) {}

  /// Transform PoseStamped into the target frame.
  std::optional<geometry_msgs::msg::PoseStamped> transform_pose(
      const geometry_msgs::msg::PoseStamped &pose_in) {
    try {
      return tf_buffer_.transform(pose_in, target_frame_,
                                  tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &) {
      // Fallback: latest
      try {
        auto ps = pose_in;
        ps.header.stamp = rclcpp::Time(0);
        return tf_buffer_.transform(ps, target_frame_,
                                    tf2::durationFromSec(0.1));
      } catch (...) {
        return std::nullopt;
      }
    }
  }

  /// Transform an Armor message to ObservationData in target frame.
  std::optional<ObservationData> transform_armor_to_observation(
      const rm_interfaces::msg::Armor &armor, const std::string &source_frame,
      const rclcpp::Time &stamp) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = source_frame;
    ps.header.stamp = stamp;
    ps.pose = armor.pose;

    auto transformed = transform_pose(ps);
    if (!transformed) return std::nullopt;

    return pose_to_observation(transformed->pose,
                               stamp.seconds());
  }

  bool can_transform(const std::string &source_frame) const {
    return tf_buffer_.canTransform(target_frame_, source_frame,
                                   tf2::TimePointZero,
                                   tf2::durationFromSec(0.01));
  }

  const std::string &target_frame() const { return target_frame_; }

 private:
  std::string target_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TF_HANDLER_HPP_
