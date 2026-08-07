// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0.

#ifndef GIMBAL_CONTROLLER__GIMBAL_CONTROL_TYPES_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CONTROL_TYPES_HPP_

#include <cstdint>
#include <string>

#include <rclcpp/time.hpp>
#include <rm_interfaces/msg/tracked_robot.hpp>

namespace gimbal_controller
{

struct GimbalControlContext
{
  rm_interfaces::msg::TrackedRobot target_robot;
  double current_yaw{0.0};
  double current_pitch{0.0};
  double bullet_speed{20.0};
  rclcpp::Time current_time;
  rclcpp::Time target_stamp;
  bool is_tracking{false};
  bool is_temp_lost{false};
  bool is_maneuvering{false};
};

struct DelayAuditSnapshot
{
  bool valid{false};
  bool tracking{false};
  std::string strategy_name;
  double processing_delay_s{0.0};
  double prediction_extra_s{0.0};
  double flight_time_s{0.0};
  double total_prediction_time_s{0.0};
  double control_latency_s{0.0};
  double fire_control_compensation_s{0.0};
  std::int32_t control_delay_steps{0};
  bool uses_delayed_b{false};
  bool double_compensation_risk{false};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__GIMBAL_CONTROL_TYPES_HPP_
