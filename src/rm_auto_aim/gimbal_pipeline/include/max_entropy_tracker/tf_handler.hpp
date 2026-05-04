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
///
/// This class does NOT own a tf2_ros::Buffer or TransformListener.
/// Instead it shares the buffer owned by the calling node, which
/// must also have set up a tf2_ros::MessageFilter on that buffer.
/// The MessageFilter guarantees that by the time armorsCallback fires
/// the transform at the message timestamp is already in the buffer,
/// so transform_pose() can use timeout=0 and will always succeed.
class TFHandler {
 public:
  /// @param buffer  Shared tf2 buffer owned by the node (also used by
  ///                tf2_ros::MessageFilter). Must outlive this object.
  TFHandler(std::shared_ptr<tf2_ros::Buffer> buffer,
            const std::string &target_frame = "odom")
      : target_frame_(target_frame),
        tf_buffer_(std::move(buffer)) {}

  /// Transform PoseStamped into the target frame.
  /// Pre-condition: the caller's MessageFilter has already confirmed that
  /// the transform at pose_in.header.stamp is available, so no waiting.
  std::optional<geometry_msgs::msg::PoseStamped> transform_pose(
      const geometry_msgs::msg::PoseStamped &pose_in) {
    try {
      return tf_buffer_->transform(pose_in, target_frame_,
                                   tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(rclcpp::get_logger("tf_handler"),
                   "Unexpected TF error at stamp %.4f: %s",
                   rclcpp::Time(pose_in.header.stamp).seconds(), ex.what());
      return std::nullopt;
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

    auto obs = pose_to_observation(transformed->pose, stamp.seconds());
    // 使用 armor.source_frame 如果可用，否则使用传入的 source_frame
    obs.source_frame = armor.source_frame.empty() ? source_frame : armor.source_frame;

    // 传递 PnP 协方差
    Eigen::Matrix3d cov;
    cov << armor.pos_covariance[0], armor.pos_covariance[1], armor.pos_covariance[2],
           armor.pos_covariance[3], armor.pos_covariance[4], armor.pos_covariance[5],
           armor.pos_covariance[6], armor.pos_covariance[7], armor.pos_covariance[8];

    // 检查协方差是否有效
    bool cov_valid = cov.array().isFinite().all() &&
                     (Eigen::LLT<Eigen::Matrix3d>(cov).info() == Eigen::Success);

    if (cov_valid) {
      obs.pos_covariance = cov;
      obs.yaw_variance = armor.yaw_variance;
    }

    return obs;
  }

  bool can_transform(const std::string &source_frame) const {
    return tf_buffer_->canTransform(target_frame_, source_frame,
                                    tf2::TimePointZero,
                                    tf2::durationFromSec(0.01));
  }

  const std::string &target_frame() const { return target_frame_; }

 private:
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;  // shared with node & MessageFilter
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TF_HANDLER_HPP_
