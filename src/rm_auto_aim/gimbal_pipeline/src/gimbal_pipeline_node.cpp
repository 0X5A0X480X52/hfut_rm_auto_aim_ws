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

#include "gimbal_pipeline/gimbal_pipeline_node.hpp"

// std
#include <memory>
// ros2
#include <tf2_ros/create_timer_ros.h>
// project
#include "rm_utils/common.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::auto_aim {

GimbalPipelineNode::GimbalPipelineNode(const rclcpp::NodeOptions &options)
: Node("gimbal_pipeline", options) {
  FYT_REGISTER_LOGGER("gimbal_pipeline", "~/fyt2024-log", INFO);
  FYT_INFO("gimbal_pipeline", "Starting GimbalPipelineNode!");

  target_frame_ = this->declare_parameter("target_frame", "odom");

  // TF buffer & listener
  tf2_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_if = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_if);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Initialise latest_target_ with an empty frame_id so the timer can detect
  // that no target has been received yet.
  latest_target_.header.frame_id = "";
  latest_target_.header.stamp    = this->now();
  is_temp_lost_                  = false;

  // Pipeline (lazy init so that weak_from_this() is safe inside constructor)
  pipeline_ = std::make_unique<GimbalPipeline>(weak_from_this());

  // Subscriber
  target_sub_ = this->create_subscription<rm_interfaces::msg::Target>(
    "armor_solver/target",
    rclcpp::SensorDataQoS(),
    std::bind(&GimbalPipelineNode::targetCallback, this, std::placeholders::_1));

  // Publisher
  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>(
    "gimbal_pipeline/cmd_gimbal", rclcpp::SensorDataQoS());

  // Timer ~250 Hz
  pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                       std::bind(&GimbalPipelineNode::timerCallback, this));

  // Enable / disable service
  enable_       = true;
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "gimbal_pipeline/set_mode",
    std::bind(&GimbalPipelineNode::setModeCallback,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);
}

// ---------------------------------------------------------------------------
// targetCallback
// ---------------------------------------------------------------------------
void GimbalPipelineNode::targetCallback(
  const rm_interfaces::msg::Target::SharedPtr target_msg) {
  latest_target_ = *target_msg;

  // Detect TEMP_LOST: the armor_solver publishes target.tracking = true even
  // in TEMP_LOST state, but the target frame_id transitions from empty to a
  // valid frame when tracking begins.  The armor_solver_node sets tracking=true
  // for both TRACKING and TEMP_LOST states, so we rely on whether an actual
  // velocity update was received – conservatively we mark is_temp_lost as
  // false whenever we receive a new target message with tracking=true.
  is_temp_lost_ = !target_msg->tracking;
}

// ---------------------------------------------------------------------------
// timerCallback
// ---------------------------------------------------------------------------
void GimbalPipelineNode::timerCallback() {
  if (!enable_) {
    return;
  }

  rm_interfaces::msg::GimbalCmd cmd;

  // No target ever detected: publish a safe zero-movement command
  if (latest_target_.header.frame_id.empty() || !latest_target_.tracking) {
    cmd.yaw_diff   = 0.0;
    cmd.pitch_diff = 0.0;
    cmd.distance   = -1.0;
    // Absolute yaw/pitch set to 0 to indicate "hold current position";
    // the serial driver acts only on yaw_diff/pitch_diff in this state.
    cmd.pitch      = 0.0;
    cmd.yaw        = 0.0;
    cmd.fire_advice = false;
    gimbal_pub_->publish(cmd);
    return;
  }

  try {
    GimbalControlContext context =
      pipeline_->buildContext(latest_target_, is_temp_lost_, tf2_buffer_);
    cmd = pipeline_->solve(context, this->now());
  } catch (const std::exception &e) {
    FYT_ERROR("gimbal_pipeline", "Pipeline error: {}", e.what());
    cmd.yaw_diff   = 0.0;
    cmd.pitch_diff = 0.0;
    cmd.distance   = -1.0;
    cmd.fire_advice = false;
  } catch (...) {
    FYT_ERROR("gimbal_pipeline", "Unknown pipeline error");
    cmd.yaw_diff   = 0.0;
    cmd.pitch_diff = 0.0;
    cmd.distance   = -1.0;
    cmd.fire_advice = false;
  }

  gimbal_pub_->publish(cmd);
}

// ---------------------------------------------------------------------------
// setModeCallback
// ---------------------------------------------------------------------------
void GimbalPipelineNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;

  VisionMode mode      = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    FYT_ERROR("gimbal_pipeline", "Invalid mode: {}", request->mode);
    return;
  }

  switch (mode) {
    case VisionMode::AUTO_AIM_RED:
    case VisionMode::AUTO_AIM_BLUE:
      enable_ = true;
      break;
    default:
      enable_ = false;
      break;
  }

  FYT_WARN("gimbal_pipeline", "Set Mode to {}", visionModeToString(mode));
}

}  // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::GimbalPipelineNode)
