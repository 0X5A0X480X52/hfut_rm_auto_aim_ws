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

#include "robot_pose_estimator/robot_pose_estimator_node.hpp"

#include <functional>
#include <chrono>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

RobotPoseEstimatorNode::RobotPoseEstimatorNode(const rclcpp::NodeOptions& options)
  : Node("robot_pose_estimator", options)
  , debug_mode_(false)
  , predict_rate_(100.0) {
  
  FYT_INFO("robot_pose_estimator", "Initializing Robot Pose Estimator Node...");
  
  // 声明和加载参数
  declareParameters();
  
  // 构建配置
  auto config = buildConfig();
  topic_config_ = buildTopicConfig();
  
  // 创建核心估计器
  estimator_core_ = std::make_unique<RobotPoseEstimatorCore>(config);
  
  // 创建订阅者
  tracked_armors_sub_ = this->create_subscription<rm_interfaces::msg::TrackedArmors>(
    topic_config_.tracked_armors_sub,
    rclcpp::SensorDataQoS(),
    std::bind(&RobotPoseEstimatorNode::trackedArmorsCallback, this, std::placeholders::_1)
  );
  
  // 创建发布者
  robots_pub_ = this->create_publisher<rm_interfaces::msg::TrackedRobots>(
    topic_config_.robots_pub,
    rclcpp::SensorDataQoS()
  );
  
  virtual_armors_pub_ = this->create_publisher<rm_interfaces::msg::Armors>(
    topic_config_.virtual_armors_pub,
    rclcpp::SensorDataQoS()
  );
  
  target_pub_ = this->create_publisher<rm_interfaces::msg::Target>(
    topic_config_.target_pub,
    rclcpp::SensorDataQoS()
  );
  
  if (debug_mode_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      topic_config_.markers_pub,
      10
    );
  }
  
  // 创建预测定时器
  double predict_period_ms = 1000.0 / predict_rate_;
  predict_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(predict_period_ms)),
    std::bind(&RobotPoseEstimatorNode::predictTimerCallback, this)
  );
  
  // 初始化时间戳
  last_update_time_ = this->now();
  
  // 创建心跳
  heartbeat_ = HeartBeatPublisher::create(this);
  
  FYT_INFO("robot_pose_estimator", "Robot Pose Estimator Node initialized");
  FYT_INFO("robot_pose_estimator", "  Subscribing to: {}", topic_config_.tracked_armors_sub);
  FYT_INFO("robot_pose_estimator", "  Publishing robots to: {}", topic_config_.robots_pub);
  FYT_INFO("robot_pose_estimator", "  Publishing virtual armors to: {}", topic_config_.virtual_armors_pub);
  FYT_INFO("robot_pose_estimator", "  Publishing target to: {}", topic_config_.target_pub);
}

void RobotPoseEstimatorNode::declareParameters() {
  // 调试模式
  debug_mode_ = this->declare_parameter("debug", false);
  predict_rate_ = this->declare_parameter("predict_rate", 100.0);
  
  // 话题配置
  this->declare_parameter("topics.tracked_armors_sub", "/armor_tracker/tracked_armors");
  this->declare_parameter("topics.robots_pub", "/robot_pose_estimator/robots");
  this->declare_parameter("topics.virtual_armors_pub", "/robot_pose_estimator/virtual_armors");
  this->declare_parameter("topics.target_pub", "/robot_pose_estimator/target");
  this->declare_parameter("topics.markers_pub", "/robot_pose_estimator/markers");
  
  // EKF参数
  this->declare_parameter("ekf.sigma2_q_xyz", 0.05);
  this->declare_parameter("ekf.sigma2_q_yaw", 1.0);
  this->declare_parameter("ekf.sigma2_q_r", 0.05);
  this->declare_parameter("ekf.r_xyz", 0.05);
  this->declare_parameter("ekf.r_yaw", 0.02);
  
  // 关联参数
  this->declare_parameter("tracking.max_match_distance", 0.5);
  this->declare_parameter("tracking.max_match_yaw_diff", 1.0);
  this->declare_parameter("tracking.tracking_threshold", 5);
  this->declare_parameter("tracking.lost_threshold", 100);
  this->declare_parameter("tracking.robot_timeout", 2.0);
  
  // 机器人参数
  this->declare_parameter("robot.balance_radius", 0.15);
  this->declare_parameter("robot.standard_radius", 0.23);
  this->declare_parameter("robot.hero_radius", 0.28);
  this->declare_parameter("robot.outpost_radius", 0.26);
  
  // 虚拟装甲板生成
  this->declare_parameter("virtual_armor_generation", true);
}

PoseEstimatorConfig RobotPoseEstimatorNode::buildConfig() {
  PoseEstimatorConfig config;
  
  // EKF参数
  config.sigma2_q_xyz = this->get_parameter("ekf.sigma2_q_xyz").as_double();
  config.sigma2_q_yaw = this->get_parameter("ekf.sigma2_q_yaw").as_double();
  config.sigma2_q_r = this->get_parameter("ekf.sigma2_q_r").as_double();
  config.r_xyz = this->get_parameter("ekf.r_xyz").as_double();
  config.r_yaw = this->get_parameter("ekf.r_yaw").as_double();
  
  // 关联参数
  config.max_match_distance = this->get_parameter("tracking.max_match_distance").as_double();
  config.max_match_yaw_diff = this->get_parameter("tracking.max_match_yaw_diff").as_double();
  config.tracking_threshold = this->get_parameter("tracking.tracking_threshold").as_int();
  config.lost_threshold = this->get_parameter("tracking.lost_threshold").as_int();
  config.robot_timeout = this->get_parameter("tracking.robot_timeout").as_double();
  
  // 机器人参数
  config.robot_params.balance_radius = this->get_parameter("robot.balance_radius").as_double();
  config.robot_params.standard_radius = this->get_parameter("robot.standard_radius").as_double();
  config.robot_params.hero_radius = this->get_parameter("robot.hero_radius").as_double();
  config.robot_params.outpost_radius = this->get_parameter("robot.outpost_radius").as_double();
  
  // 虚拟装甲板生成
  config.virtual_armor_generation = this->get_parameter("virtual_armor_generation").as_bool();
  
  return config;
}

EstimatorTopicConfig RobotPoseEstimatorNode::buildTopicConfig() {
  EstimatorTopicConfig config;
  
  config.tracked_armors_sub = this->get_parameter("topics.tracked_armors_sub").as_string();
  config.robots_pub = this->get_parameter("topics.robots_pub").as_string();
  config.virtual_armors_pub = this->get_parameter("topics.virtual_armors_pub").as_string();
  config.target_pub = this->get_parameter("topics.target_pub").as_string();
  config.markers_pub = this->get_parameter("topics.markers_pub").as_string();
  
  return config;
}

void RobotPoseEstimatorNode::trackedArmorsCallback(
    const rm_interfaces::msg::TrackedArmors::SharedPtr msg) {
  
  // 计算时间间隔
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  last_update_time_ = current_time;
  
  // 限制 dt 范围
  dt = std::clamp(dt, 0.001, 0.1);
  
  // 更新估计器
  estimator_core_->update(*msg, dt);
  
  // 获取机器人状态
  auto robots = estimator_core_->getTrackingRobots();
  
  // 发布机器人状态
  publishRobots(robots, msg->header);
  
  // 发布虚拟装甲板
  if (estimator_core_->getConfig().virtual_armor_generation) {
    auto virtual_armors = estimator_core_->generateVirtualArmors();
    publishVirtualArmors(virtual_armors, msg->header);
  }
  
  // 发布 Target 消息（选择最佳机器人）
  const RobotState* best_robot = selectBestRobot(robots);
  if (best_robot != nullptr) {
    publishTarget(*best_robot, msg->header);
  }
  
  // 发布可视化标记
  if (debug_mode_) {
    publishMarkers(robots);
  }
}

void RobotPoseEstimatorNode::predictTimerCallback() {
  // 此回调用于在没有观测时维持预测
  // 实际预测在 trackedArmorsCallback 中执行
  // 这里可以用于发布心跳等
}

void RobotPoseEstimatorNode::publishRobots(
    const std::vector<RobotState>& robots,
    const std_msgs::msg::Header& header) {
  
  rm_interfaces::msg::TrackedRobots msg;
  msg.header = header;
  
  for (const auto& robot : robots) {
    msg.robots.push_back(robotStateToMsg(robot, header));
  }
  
  robots_pub_->publish(msg);
}

void RobotPoseEstimatorNode::publishVirtualArmors(
    const std::vector<rm_interfaces::msg::Armor>& armors,
    const std_msgs::msg::Header& header) {
  
  rm_interfaces::msg::Armors msg;
  msg.header = header;
  msg.armors = armors;
  
  virtual_armors_pub_->publish(msg);
}

void RobotPoseEstimatorNode::publishTarget(
    const RobotState& robot,
    const std_msgs::msg::Header& header) {
  
  auto target_msg = robotStateToTarget(robot, header);
  target_pub_->publish(target_msg);
}

rm_interfaces::msg::TrackedRobot RobotPoseEstimatorNode::robotStateToMsg(
    const RobotState& state,
    const std_msgs::msg::Header& header) {
  
  rm_interfaces::msg::TrackedRobot msg;
  msg.header = header;
  
  msg.robot_id = state.robot_id;
  msg.robot_type = static_cast<uint8_t>(state.robot_type);
  
  msg.center_position.x = state.center_position.x();
  msg.center_position.y = state.center_position.y();
  msg.center_position.z = state.center_position.z();
  
  msg.center_velocity.x = state.center_velocity.x();
  msg.center_velocity.y = state.center_velocity.y();
  msg.center_velocity.z = state.center_velocity.z();
  
  msg.yaw = state.yaw;
  msg.yaw_velocity = state.yaw_velocity;
  msg.radius = state.radius_1;
  msg.num_armors = state.num_armors;
  msg.confidence = state.confidence;
  
  // 绑定的装甲板ID
  for (const auto& armor_state : state.armor_states) {
    msg.bound_armor_ids.push_back(armor_state.armor_id);
  }
  
  return msg;
}

rm_interfaces::msg::Target RobotPoseEstimatorNode::robotStateToTarget(
    const RobotState& state,
    const std_msgs::msg::Header& header) {
  
  rm_interfaces::msg::Target msg;
  msg.header = header;
  
  msg.tracking = state.is_tracking;
  msg.id = state.robot_id;
  msg.armors_num = state.num_armors;
  
  msg.position.x = state.center_position.x();
  msg.position.y = state.center_position.y();
  msg.position.z = state.center_position.z();
  
  msg.velocity.x = state.center_velocity.x();
  msg.velocity.y = state.center_velocity.y();
  msg.velocity.z = state.center_velocity.z();
  
  msg.yaw = state.yaw;
  msg.v_yaw = state.yaw_velocity;
  
  msg.radius_1 = state.radius_1;
  msg.radius_2 = state.radius_2;
  msg.d_za = state.d_za;
  msg.d_zc = state.d_zc;
  
  // 计算 yaw_diff 和 position_diff（相对于上一帧）
  msg.yaw_diff = 0.0;
  msg.position_diff = 0.0;
  
  return msg;
}

const RobotState* RobotPoseEstimatorNode::selectBestRobot(
    const std::vector<RobotState>& robots) {
  
  if (robots.empty()) {
    return nullptr;
  }
  
  // 选择最近的正在跟踪的机器人
  double min_distance = std::numeric_limits<double>::max();
  const RobotState* best = nullptr;
  
  for (const auto& robot : robots) {
    if (robot.is_tracking) {
      double distance = robot.center_position.head<2>().norm();
      if (distance < min_distance) {
        min_distance = distance;
        best = &robot;
      }
    }
  }
  
  // 如果没有正在跟踪的，选择置信度最高的
  if (best == nullptr) {
    double max_confidence = -1.0;
    for (const auto& robot : robots) {
      if (robot.confidence > max_confidence) {
        max_confidence = robot.confidence;
        best = &robot;
      }
    }
  }
  
  return best;
}

void RobotPoseEstimatorNode::publishMarkers(const std::vector<RobotState>& robots) {
  if (!marker_pub_) {
    return;
  }
  
  visualization_msgs::msg::MarkerArray marker_array;
  int id = 0;
  
  for (const auto& robot : robots) {
    // 机器人中心球体
    visualization_msgs::msg::Marker center_marker;
    center_marker.header.frame_id = "odom";
    center_marker.header.stamp = this->now();
    center_marker.ns = "robot_centers";
    center_marker.id = id++;
    center_marker.type = visualization_msgs::msg::Marker::SPHERE;
    center_marker.action = visualization_msgs::msg::Marker::ADD;
    
    center_marker.pose.position.x = robot.center_position.x();
    center_marker.pose.position.y = robot.center_position.y();
    center_marker.pose.position.z = robot.center_position.z();
    center_marker.pose.orientation.w = 1.0;
    
    center_marker.scale.x = 0.1;
    center_marker.scale.y = 0.1;
    center_marker.scale.z = 0.1;
    
    center_marker.color.r = 0.0;
    center_marker.color.g = 1.0;
    center_marker.color.b = 0.0;
    center_marker.color.a = 0.8;
    
    center_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    marker_array.markers.push_back(center_marker);
    
    // 机器人ID文本
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "odom";
    text_marker.header.stamp = this->now();
    text_marker.ns = "robot_ids";
    text_marker.id = id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    
    text_marker.pose.position.x = robot.center_position.x();
    text_marker.pose.position.y = robot.center_position.y();
    text_marker.pose.position.z = robot.center_position.z() + 0.2;
    text_marker.pose.orientation.w = 1.0;
    
    text_marker.text = robot.robot_id + " (" + getRobotTypeName(robot.robot_type) + ")";
    text_marker.scale.z = 0.1;
    
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    marker_array.markers.push_back(text_marker);
  }
  
  marker_pub_->publish(marker_array);
}

}  // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::RobotPoseEstimatorNode)
