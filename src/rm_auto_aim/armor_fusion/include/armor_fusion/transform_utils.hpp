#ifndef ARMOR_FUSION__TRANSFORM_UTILS_HPP_
#define ARMOR_FUSION__TRANSFORM_UTILS_HPP_

#include <optional>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "armor_fusion/measurement_types.hpp"
#include "rm_interfaces/msg/armor.hpp"

namespace fyt::auto_aim {

// Transform one armor measurement to target_frame and normalize into ArmorMeasurement.
std::optional<ArmorMeasurement> transformMeasurement(
  tf2_ros::Buffer & tf_buffer,
  const rm_interfaces::msg::Armor & armor,
  const std::string & source_frame,
  const std::string & target_frame,
  const builtin_interfaces::msg::Time & stamp,
  double tf_lookup_timeout,
  bool console_debug,
  const rclcpp::Logger & logger);

}  // namespace fyt::auto_aim

#endif  // ARMOR_FUSION__TRANSFORM_UTILS_HPP_
