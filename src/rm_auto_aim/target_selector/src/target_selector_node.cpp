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

#include "target_selector/target_selector_node.hpp"

#include <chrono>
#include <functional>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fyt::auto_aim {

TargetSelectorNode::TargetSelectorNode(const rclcpp::NodeOptions& options)
  : Node("target_selector", options)
  , current_target_id_("")
  , is_tracking_active_(false) {
  
  // 注册日志器
  try {
    FYT_REGISTER_LOGGER("target_selector", "logs/target_selector", INFO);
  } catch (...) {
    // Logger may already be registered, ignore
  }
  
  FYT_INFO("target_selector", "Initializing Target Selector Node...");
  
  // 声明和加载参数
  declareParameters();
  
  // 构建配置
  config_ = buildConfig();
  topic_config_ = buildTopicConfig();
  
  // 初始化选择配置
  selection_config_.reference_yaw = config_.reference_yaw;
  selection_config_.max_yaw_deviation = config_.max_yaw_deviation;
  selection_config_.max_distance = config_.max_distance;
  selection_config_.min_confidence = config_.min_confidence;
  selection_config_.hysteresis_threshold = config_.hysteresis_threshold;
  
  // 初始化策略
  initStrategy();
  
  // 初始化 TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // 创建订阅者
  robots_sub_ = this->create_subscription<TrackedRobots>(
    topic_config_.robots_sub,
    rclcpp::SensorDataQoS(),
    std::bind(&TargetSelectorNode::robotsCallback, this, std::placeholders::_1)
  );
  
  // 创建发布者
  selected_target_pub_ = this->create_publisher<SelectedTarget>(
    topic_config_.selected_target_pub,
    rclcpp::SensorDataQoS()
  );
  
  if (config_.debug) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      topic_config_.markers_pub,
      10
    );
  }
  
  // 创建服务客户端
  set_target_client_ = this->create_client<SetTargetRobot>(
    topic_config_.set_target_service
  );
  
  // 创建 Action 客户端
  trajectory_action_client_ = rclcpp_action::create_client<TrajectoryPlan>(
    this,
    topic_config_.trajectory_action
  );
  
  // 创建心跳
  heartbeat_ = HeartBeatPublisher::create(this);
  
  FYT_INFO("target_selector", "Target Selector Node initialized");
  FYT_INFO("target_selector", "  Strategy: {}", config_.strategy_name);
  FYT_INFO("target_selector", "  Subscribing to: {}", topic_config_.robots_sub);
  FYT_INFO("target_selector", "  Publishing to: {}", topic_config_.selected_target_pub);
  FYT_INFO("target_selector", "  Service client: {}", topic_config_.set_target_service);
  FYT_INFO("target_selector", "  Action client: {}", topic_config_.trajectory_action);
}

void TargetSelectorNode::declareParameters() {
  // 策略配置
  this->declare_parameter("strategy", "min_yaw_deviation");
  this->declare_parameter("reference_yaw", 0.0);
  
  // 选择参数
  this->declare_parameter("max_yaw_deviation", M_PI);
  this->declare_parameter("max_distance", 10.0);
  this->declare_parameter("min_confidence", 0.3);
  this->declare_parameter("hysteresis_threshold", 0.1);
  
  // TF 坐标系
  this->declare_parameter("gimbal_frame", "gimbal_link");
  this->declare_parameter("odom_frame", "odom");
  
  // 话题配置
  this->declare_parameter("topics.robots_sub", "/robot_pose_estimator/robots");
  this->declare_parameter("topics.selected_target_pub", "/target_selector/selected_target");
  this->declare_parameter("topics.markers_pub", "/target_selector/markers");
  this->declare_parameter("topics.set_target_service", "/trajectory_planner/set_target_robot");
  this->declare_parameter("topics.trajectory_action", "/trajectory_planner/trajectory_plan");
  
  // 调试模式
  this->declare_parameter("debug", false);
}

SelectorConfig TargetSelectorNode::buildConfig() {
  SelectorConfig config;
  
  config.strategy_name = this->get_parameter("strategy").as_string();
  config.reference_yaw = this->get_parameter("reference_yaw").as_double();
  config.max_yaw_deviation = this->get_parameter("max_yaw_deviation").as_double();
  config.max_distance = this->get_parameter("max_distance").as_double();
  config.min_confidence = this->get_parameter("min_confidence").as_double();
  config.hysteresis_threshold = this->get_parameter("hysteresis_threshold").as_double();
  config.gimbal_frame = this->get_parameter("gimbal_frame").as_string();
  config.odom_frame = this->get_parameter("odom_frame").as_string();
  config.debug = this->get_parameter("debug").as_bool();
  
  return config;
}

SelectorTopicConfig TargetSelectorNode::buildTopicConfig() {
  SelectorTopicConfig config;
  
  config.robots_sub = this->get_parameter("topics.robots_sub").as_string();
  config.selected_target_pub = this->get_parameter("topics.selected_target_pub").as_string();
  config.markers_pub = this->get_parameter("topics.markers_pub").as_string();
  config.set_target_service = this->get_parameter("topics.set_target_service").as_string();
  config.trajectory_action = this->get_parameter("topics.trajectory_action").as_string();
  
  return config;
}

void TargetSelectorNode::initStrategy() {
  if (config_.strategy_name == "min_yaw_deviation") {
    strategy_ = std::make_unique<MinYawDeviationStrategy>();
  } else {
    FYT_WARN("target_selector", "Unknown strategy '{}', using min_yaw_deviation", 
             config_.strategy_name);
    strategy_ = std::make_unique<MinYawDeviationStrategy>();
  }
  
  FYT_INFO("target_selector", "Using strategy: {} - {}", 
           strategy_->getName(), strategy_->getDescription());
}

void TargetSelectorNode::robotsCallback(const TrackedRobots::SharedPtr msg) {
  FYT_DEBUG("target_selector", "robotsCallback invoked. header stamp: {} | robots count: {}", 
            msg->header.stamp.sec, msg->robots.size());
  
  if (msg->robots.empty()) {
    // 没有检测到机器人,停止跟踪
    if (is_tracking_active_) {
      FYT_INFO("target_selector", "No robots detected, stopping tracking");
      stopTrajectoryPlanning();
      current_target_id_ = "";
    }
    return;
  }
  
  // 打印机器人信息
  for (const auto& robot : msg->robots) {
    FYT_DEBUG("target_selector", "Robot detected: id={}, confidence={:.3f}, position=({:.2f},{:.2f},{:.2f})",
              robot.robot_id, robot.confidence, 
              robot.center_position.x, robot.center_position.y, robot.center_position.z);
  }
  
  // 处理目标选择
  processTargetSelection(*msg);
}

void TargetSelectorNode::processTargetSelection(const TrackedRobots& robots) {
  FYT_DEBUG("target_selector", "processTargetSelection called with {} robots", robots.robots.size());
  
  // 更新选择配置
  selection_config_.current_target_id = current_target_id_;
  FYT_DEBUG("target_selector", "Current target ID: '{}'", current_target_id_);
  
  // 使用策略选择目标
  auto result = strategy_->selectTarget(robots, selection_config_);
  
  FYT_DEBUG("target_selector", "Strategy selectTarget returned: {}", result.has_value() ? "valid result" : "nullopt");
  
  if (!result.has_value()) {
    // 没有合适的目标
    FYT_DEBUG("target_selector", "No suitable target found by strategy");
    if (is_tracking_active_) {
      FYT_INFO("target_selector", "No suitable target, stopping tracking");
      stopTrajectoryPlanning();
      current_target_id_ = "";
    }
    
    // 发布空的选择结果
    SelectedTarget empty_target;
    empty_target.header.stamp = this->now();
    empty_target.header.frame_id = config_.gimbal_frame;
    empty_target.robot_id = "";
    empty_target.confidence = 0.0;
    empty_target.selection_strategy = strategy_->getName();
    selected_target_pub_->publish(empty_target);
    
    FYT_DEBUG("target_selector", "Published empty target with robot_id='{}'", empty_target.robot_id);
    
    if (config_.debug && marker_pub_) {
      publishMarkers(robots, std::nullopt);
    }
    return;
  }
  
  // 检查是否需要切换目标
  bool target_changed = (result->robot_id != current_target_id_);
  
  if (target_changed) {
    FYT_INFO("target_selector", "Target changed: {} -> {}, yaw_dev={:.3f}, dist={:.2f}",
             current_target_id_.empty() ? "none" : current_target_id_,
             result->robot_id,
             result->yaw_deviation,
             result->distance);
    
    current_target_id_ = result->robot_id;
    
    // 调用服务设置新目标
    callSetTargetService(result->robot_id);
    
    // 如果还没有启动跟踪,启动 Action
    if (!is_tracking_active_) {
      startTrajectoryPlanning(result->robot_id);
    }
  }
  
  // 发布选中的目标
  publishSelectedTarget(*result);
  
  // 发布可视化标记
  if (config_.debug && marker_pub_) {
    publishMarkers(robots, result);
  }
}

void TargetSelectorNode::startTrajectoryPlanning(const std::string& robot_id) {
  // 快速检查 action server 是否可用（不阻塞）
  if (!trajectory_action_client_->wait_for_action_server(std::chrono::milliseconds(10))) {
    FYT_DEBUG("target_selector", "Trajectory action server not available, skipping action call");
    return;
  }
  
  auto goal_msg = TrajectoryPlan::Goal();
  goal_msg.robot_id = robot_id;
  goal_msg.reference_yaw = selection_config_.reference_yaw;
  goal_msg.enable_tracking = true;
  
  auto send_goal_options = rclcpp_action::Client<TrajectoryPlan>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&TargetSelectorNode::trajectoryGoalResponseCallback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&TargetSelectorNode::trajectoryFeedbackCallback, this,
              std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&TargetSelectorNode::trajectoryResultCallback, this, std::placeholders::_1);
  
  FYT_INFO("target_selector", "Sending trajectory planning goal for robot: {}", robot_id);
  trajectory_action_client_->async_send_goal(goal_msg, send_goal_options);
}

void TargetSelectorNode::stopTrajectoryPlanning() {
  if (current_goal_handle_ != nullptr) {
    FYT_INFO("target_selector", "Canceling trajectory planning");
    trajectory_action_client_->async_cancel_goal(current_goal_handle_);
    current_goal_handle_ = nullptr;
  }
  is_tracking_active_ = false;
}

void TargetSelectorNode::callSetTargetService(const std::string& robot_id) {
  // 快速检查 service 是否可用（不阻塞）
  if (!set_target_client_->wait_for_service(std::chrono::milliseconds(10))) {
    FYT_DEBUG("target_selector", "SetTargetRobot service not available, skipping service call");
    return;
  }
  
  auto request = std::make_shared<SetTargetRobot::Request>();
  request->robot_id = robot_id;
  
  auto result_future = set_target_client_->async_send_request(request,
    [this, robot_id](rclcpp::Client<SetTargetRobot>::SharedFuture future) {
      try {
        auto result = future.get();
        if (result->success) {
          FYT_DEBUG("target_selector", "Successfully set target to: {}", robot_id);
        } else {
          FYT_WARN("target_selector", "Failed to set target: {}", result->message);
        }
      } catch (const std::exception& e) {
        FYT_ERROR("target_selector", "Service call failed: {}", e.what());
      }
    });
}

void TargetSelectorNode::trajectoryGoalResponseCallback(
    const TrajectoryPlanGoalHandle::SharedPtr& goal_handle) {
  if (!goal_handle) {
    FYT_WARN("target_selector", "Trajectory goal was rejected by server");
    is_tracking_active_ = false;
    return;
  }
  
  FYT_INFO("target_selector", "Trajectory goal accepted");
  current_goal_handle_ = goal_handle;
  is_tracking_active_ = true;
}

void TargetSelectorNode::trajectoryFeedbackCallback(
    TrajectoryPlanGoalHandle::SharedPtr /*goal_handle*/,
    const std::shared_ptr<const TrajectoryPlan::Feedback> feedback) {
  FYT_DEBUG("target_selector", "Trajectory feedback: robot={}, yaw_err={:.3f}, locked={}",
            feedback->current_robot_id,
            feedback->yaw_error,
            feedback->target_locked);
}

void TargetSelectorNode::trajectoryResultCallback(
    const TrajectoryPlanGoalHandle::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      FYT_INFO("target_selector", "Trajectory planning completed: {}",
               result.result->message);
      break;
    case rclcpp_action::ResultCode::ABORTED:
      FYT_WARN("target_selector", "Trajectory planning aborted: {}",
               result.result->message);
      break;
    case rclcpp_action::ResultCode::CANCELED:
      FYT_INFO("target_selector", "Trajectory planning canceled");
      break;
    default:
      FYT_WARN("target_selector", "Unknown trajectory result code");
      break;
  }
  
  current_goal_handle_ = nullptr;
  is_tracking_active_ = false;
}

void TargetSelectorNode::publishSelectedTarget(const SelectionResult& result) {
  SelectedTarget msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = config_.gimbal_frame;
  msg.robot_id = result.robot_id;
  msg.confidence = result.confidence;
  msg.selection_strategy = strategy_->getName();
  
  FYT_DEBUG("target_selector", "Publishing SelectedTarget: robot_id='{}', confidence={:.2f}, strategy='{}'",
           msg.robot_id, msg.confidence, msg.selection_strategy);
  
  selected_target_pub_->publish(msg);
  
  FYT_DEBUG("target_selector", "SelectedTarget published successfully");
}

void TargetSelectorNode::publishMarkers(
    const TrackedRobots& robots,
    const std::optional<SelectionResult>& selected) {
  
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;
  
  for (const auto& robot : robots.robots) {
    // 机器人中心标记
    visualization_msgs::msg::Marker marker;
    marker.header = robots.header;
    marker.ns = "robots";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = robot.center_position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    
    // 选中的目标用绿色,其他用黄色
    if (selected.has_value() && robot.robot_id == selected->robot_id) {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    marker.color.a = 0.8;
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    marker_array.markers.push_back(marker);
    
    // 机器人 ID 文字标记
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = robots.header;
    text_marker.ns = "robot_ids";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position = robot.center_position;
    text_marker.pose.position.z += 0.3;
    text_marker.scale.z = 0.15;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    text_marker.text = robot.robot_id;
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    marker_array.markers.push_back(text_marker);
  }
  
  // 选中目标的连线
  if (selected.has_value()) {
    visualization_msgs::msg::Marker line_marker;
    line_marker.header = robots.header;
    line_marker.ns = "selection_line";
    line_marker.id = id++;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.02;
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 0.8;
    
    // 从原点到选中目标
    geometry_msgs::msg::Point origin;
    origin.x = 0.0;
    origin.y = 0.0;
    origin.z = 0.0;
    line_marker.points.push_back(origin);
    
    // 找到选中的机器人位置
    for (const auto& robot : robots.robots) {
      if (robot.robot_id == selected->robot_id) {
        line_marker.points.push_back(robot.center_position);
        break;
      }
    }
    
    line_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker_array.markers.push_back(line_marker);
  }
  
  marker_pub_->publish(marker_array);
}

}  // namespace fyt::auto_aim
