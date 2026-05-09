// Copyright (C) FYT Vision Group. All rights reserved.
#ifndef GIMBAL_PIPELINE__ADAPTERS__BUFF_TARGET_ADAPTER_HPP_
#define GIMBAL_PIPELINE__ADAPTERS__BUFF_TARGET_ADAPTER_HPP_

#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/tracked_robot.hpp>

namespace fyt::auto_aim::adapters
{

class BuffTargetAdapter
{
public:
  struct Config
  {
    bool enable{false};
    std::string topic{"/auto_buff/tracked_robot"};
    double timeout_s{0.3};
    std::string target_frame{"odom"};
  };

  BuffTargetAdapter(rclcpp::Node & node, const Config & config);

  std::optional<rm_interfaces::msg::TrackedRobot> latestValid(const rclcpp::Time & now) const;
  void setEnabled(bool enable);

private:
  void onTrackedRobot(const rm_interfaces::msg::TrackedRobot::SharedPtr msg);
  rm_interfaces::msg::TrackedRobot normalizeBuffRobot(
    const rm_interfaces::msg::TrackedRobot & msg) const;

private:
  rclcpp::Node & node_;
  Config config_;
  bool enabled_{false};

  rclcpp::Subscription<rm_interfaces::msg::TrackedRobot>::SharedPtr sub_;

  mutable std::mutex mutex_;
  std::optional<rm_interfaces::msg::TrackedRobot> latest_robot_;
  rclcpp::Time latest_stamp_{0, 0, RCL_ROS_TIME};
};

}  // namespace fyt::auto_aim::adapters

#endif  // GIMBAL_PIPELINE__ADAPTERS__BUFF_TARGET_ADAPTER_HPP_
