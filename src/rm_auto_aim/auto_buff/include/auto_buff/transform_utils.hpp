#pragma once

#include <tf2/time.h>
#include <tf2_ros/buffer.h>

#include <exception>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace auto_buff {

inline bool transformPointPair(tf2_ros::Buffer &buffer,
                               const geometry_msgs::msg::PointStamped &first,
                               const geometry_msgs::msg::PointStamped &second,
                               const std::string &target_frame,
                               double timeout_seconds,
                               geometry_msgs::msg::PointStamped &first_out,
                               geometry_msgs::msg::PointStamped &second_out,
                               std::string &error) {
  try {
    const auto timeout = tf2::durationFromSec(timeout_seconds);
    first_out = buffer.transform(first, target_frame, timeout);
    second_out = buffer.transform(second, target_frame, timeout);
    error.clear();
    return true;
  } catch (const std::exception &exception) {
    error = exception.what();
    return false;
  }
}

}  // namespace auto_buff
