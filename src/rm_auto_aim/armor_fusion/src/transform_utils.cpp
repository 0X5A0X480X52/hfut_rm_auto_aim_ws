#include "armor_fusion/transform_utils.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fyt::auto_aim {

std::optional<ArmorMeasurement> transformMeasurement(
  tf2_ros::Buffer & tf_buffer,
  const rm_interfaces::msg::Armor & armor,
  const std::string & source_frame,
  const std::string & target_frame,
  const builtin_interfaces::msg::Time & stamp,
  double tf_lookup_timeout,
  bool console_debug,
  const rclcpp::Logger & logger)
{
  geometry_msgs::msg::PoseStamped source_pose;
  source_pose.header.frame_id = source_frame;
  source_pose.header.stamp = stamp;
  source_pose.pose = armor.pose;

  geometry_msgs::msg::PoseStamped transformed_pose;
  try {
    tf_buffer.transform(source_pose, transformed_pose, target_frame, tf2::durationFromSec(tf_lookup_timeout));
  } catch (const tf2::TransformException & ex) {
    // Fallback to latest transform to avoid dropping all measurements on short TF delays.
    try {
      source_pose.header.stamp = builtin_interfaces::msg::Time();
      tf_buffer.transform(source_pose, transformed_pose, target_frame, tf2::durationFromSec(tf_lookup_timeout));
    } catch (const tf2::TransformException & ex_latest) {
      if (console_debug) {
        RCLCPP_WARN(
          logger,
          "Transform failed from [%s] to [%s]: %s; latest fallback failed: %s",
          source_frame.c_str(),
          target_frame.c_str(),
          ex.what(),
          ex_latest.what());
      }
      return std::nullopt;
    }
  }

  ArmorMeasurement measurement;
  measurement.position = Eigen::Vector3d(
    transformed_pose.pose.position.x,
    transformed_pose.pose.position.y,
    transformed_pose.pose.position.z);
  measurement.orientation = Eigen::Quaterniond(
    transformed_pose.pose.orientation.w,
    transformed_pose.pose.orientation.x,
    transformed_pose.pose.orientation.y,
    transformed_pose.pose.orientation.z);
  measurement.number = armor.number;
  measurement.armor_type = armor.type;
  measurement.source_frame = source_frame;
  measurement.stamp = stamp;
  measurement.distance_to_image_center = armor.distance_to_image_center;
  return measurement;
}

}  // namespace fyt::auto_aim
