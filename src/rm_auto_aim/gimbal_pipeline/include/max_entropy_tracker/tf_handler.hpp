// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TF_HANDLER_HPP_
#define MAX_ENTROPY_TRACKER_TF_HANDLER_HPP_

#include <memory>
#include <optional>
#include <string>

#include <Eigen/Geometry>
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

    // Preserve image-domain metadata from detector, and only replace the
    // 3D pose/yaw with TF-transformed values.
    auto obs = armor_to_observation(armor, stamp.seconds());
    auto tf_obs = pose_to_observation(transformed->pose, stamp.seconds());
    obs.x = tf_obs.x;
    obs.y = tf_obs.y;
    obs.z = tf_obs.z;
    obs.yaw = tf_obs.yaw;
    obs.timestamp = tf_obs.timestamp;

    // Rotate BA covariance from source frame to target frame:
    // P' = A * P * A^T, A = diag(R_tf, 1), state = [x, y, z, yaw].
    // Yaw offset is additive constant under frame transform, so yaw variance is
    // preserved and xyz-yaw cross-cov rotates with xyz block.
    if (obs.ba_pnp.has_value()) {
      auto &ba = *obs.ba_pnp;
      ba.frame_aligned = false;
      if (ba.valid && ba.cov_valid && ba.cov_xyz_yaw.allFinite()) {
        try {
          const auto tf = tf_buffer_->lookupTransform(
              target_frame_, source_frame, ps.header.stamp, tf2::durationFromSec(0.0));
          const auto &q_msg = tf.transform.rotation;
          Eigen::Quaterniond q_tf(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
          Eigen::Matrix3d R_tf = q_tf.toRotationMatrix();
          Eigen::Matrix4d A = Eigen::Matrix4d::Identity();
          A.block<3, 3>(0, 0) = R_tf;
          ba.cov_xyz_yaw = A * ba.cov_xyz_yaw * A.transpose();
          ba.cov_xyz_yaw = 0.5 * (ba.cov_xyz_yaw + ba.cov_xyz_yaw.transpose());
          ba.frame_aligned = ba.cov_xyz_yaw.allFinite();
          if (!ba.frame_aligned) {
            ba.cov_valid = false;
          }
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(rclcpp::get_logger("tf_handler"),
                      "BA covariance TF lookup failed at stamp %.4f: %s",
                      rclcpp::Time(ps.header.stamp).seconds(), ex.what());
          ba.cov_valid = false;
        }
      }
    }


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
