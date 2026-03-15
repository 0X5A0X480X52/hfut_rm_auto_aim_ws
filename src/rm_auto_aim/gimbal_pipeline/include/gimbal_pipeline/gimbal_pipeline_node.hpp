// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GIMBAL_PIPELINE_NODE_HPP_
#define GIMBAL_PIPELINE_NODE_HPP_

// std
#include <memory>
#include <string>
// ros2
#include <message_filters/subscriber.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>
// project
#include "gimbal_pipeline/gimbal_pipeline.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

// ROS2 component node that owns the GimbalPipeline.
//
// Subscriptions
//   /armor_solver/target  (rm_interfaces/msg/Target)
//
// Publications
//   /gimbal_pipeline/cmd_gimbal  (rm_interfaces/msg/GimbalCmd)
//
// Timer
//   Publishes at ~250 Hz (4 ms period).  When no target has ever been
//   detected, a zero-movement command is published so that the serial driver
//   always receives a message.
class GimbalPipelineNode : public rclcpp::Node {
public:
  explicit GimbalPipelineNode(const rclcpp::NodeOptions &options);

private:
  // Callback: cache the latest target message
  void targetCallback(const rm_interfaces::msg::Target::SharedPtr target_msg);

  // Timer callback: run the pipeline and publish GimbalCmd
  void timerCallback();

  // Service callback: enable / disable the node
  void setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                       std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

  // TF infrastructure
  std::string target_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // Latest target state from armor_solver
  rm_interfaces::msg::Target latest_target_;
  bool is_temp_lost_ = false;

  // Pipeline
  std::unique_ptr<GimbalPipeline> pipeline_;

  // ROS2 interfaces
  rclcpp::Subscription<rm_interfaces::msg::Target>::SharedPtr target_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;

  bool enable_ = true;

  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_;
};

}  // namespace fyt::auto_aim

#endif  // GIMBAL_PIPELINE_NODE_HPP_
