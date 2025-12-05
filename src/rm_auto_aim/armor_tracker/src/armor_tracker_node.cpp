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

#include "armor_tracker/armor_tracker_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <algorithm>
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions& options)
: Node("armor_tracker", options)
, last_detection_time_(this->now())
, last_estimate_time_(this->now())
, last_predict_time_(this->now())
{
  FYT_REGISTER_LOGGER("armor_tracker", "~/fyt2024-log", INFO);
  FYT_INFO("armor_tracker", "Starting ArmorTrackerNode (v2.0 - muit_obj_tracker based)!");

  // 声明参数
  declareParameters();
  
  // 构建配置并创建核心跟踪器
  auto config = buildConfig();
  tracker_core_ = std::make_unique<ArmorTrackerCore>(config);
  
  // 创建策略管理器
  strategy_manager_ = std::make_shared<TrackingStrategyManager>();
  tracker_core_->setStrategyManager(strategy_manager_);

  // 订阅者 - 检测结果
  armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
    "/armor_detector/armors",
    rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::armorsCallback, this, std::placeholders::_1)
  );
  
  // 订阅者 - 估计装甲板（预留接口）
  estimated_armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
    "/robot_pose_estimator/virtual_armors",
    rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::estimatedArmorsCallback, this, std::placeholders::_1)
  );

  // 发布者
  tracked_armors_pub_ = this->create_publisher<rm_interfaces::msg::TrackedArmors>(
    "/armor_tracker/tracked_armors",
    rclcpp::SensorDataQoS()
  );

  if (debug_mode_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/armor_tracker/markers",
      10
    );
  }
  
  // 创建预测更新定时器
  auto predict_period = std::chrono::duration<double>(1.0 / predict_rate_);
  predict_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(predict_period),
    std::bind(&ArmorTrackerNode::predictTimerCallback, this)
  );
  
  // 创建发布定时器
  auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
    std::bind(&ArmorTrackerNode::publishTimerCallback, this)
  );

  // 心跳
  heartbeat_ = HeartBeatPublisher::create(this);

  FYT_INFO("armor_tracker", "ArmorTrackerNode initialized successfully!");
  FYT_INFO("armor_tracker", "  - Predict rate: {} Hz", predict_rate_);
  FYT_INFO("armor_tracker", "  - Publish rate: {} Hz", publish_rate_);
  FYT_INFO("armor_tracker", "  - Model: {}", config.model_name);
}

void ArmorTrackerNode::declareParameters() {
  // 调试模式
  debug_mode_ = this->declare_parameter("debug", true);
  
  // 异步更新参数
  predict_rate_ = this->declare_parameter("predict_rate", 100.0);
  publish_rate_ = this->declare_parameter("publish_rate", 100.0);
  detection_timeout_ = this->declare_parameter("detection_timeout", 0.5);
  
  // 跟踪器参数
  this->declare_parameter("max_match_distance", 0.5);
  this->declare_parameter("max_match_yaw_diff", 1.0);
  this->declare_parameter("tracking_threshold", 3);
  this->declare_parameter("lost_threshold", 30);
  this->declare_parameter("max_trackers", 20);
  
  // 模型配置
  this->declare_parameter("model.name", "CV_KF");
  this->declare_parameter("model.config_file", "");
}

TrackerConfig ArmorTrackerNode::buildConfig() {
  TrackerConfig config;
  
  config.max_match_distance = this->get_parameter("max_match_distance").as_double();
  config.max_match_yaw_diff = this->get_parameter("max_match_yaw_diff").as_double();
  config.tracking_threshold = this->get_parameter("tracking_threshold").as_int();
  config.lost_threshold = this->get_parameter("lost_threshold").as_int();
  config.max_trackers = this->get_parameter("max_trackers").as_int();
  config.predict_rate = predict_rate_;
  config.model_name = this->get_parameter("model.name").as_string();
  config.model_config_file = this->get_parameter("model.config_file").as_string();
  
  return config;
}

void ArmorTrackerNode::armorsCallback(
  const rm_interfaces::msg::Armors::SharedPtr msg) {
  
  std::lock_guard<std::mutex> lock(callback_mutex_);
  
  // 转换为内部观测格式
  auto observations = armorsToObservations(*msg, ArmorSourceType::DETECT);
  
  // 更新跟踪器
  if (!observations.empty()) {
    tracker_core_->update(observations);
    detection_received_ = true;
    last_detection_time_ = this->now();
    
    FYT_DEBUG("armor_tracker", "Received {} detections", observations.size());
  }
}

void ArmorTrackerNode::estimatedArmorsCallback(
  const rm_interfaces::msg::Armors::SharedPtr msg) {
  
  std::lock_guard<std::mutex> lock(callback_mutex_);
  
  // 转换为内部观测格式（来源为估计）
  auto observations = armorsToObservations(*msg, ArmorSourceType::ESTIMATE);
  
  // 使用估计数据更新
  if (!observations.empty()) {
    tracker_core_->update(observations);
    estimate_received_ = true;
    last_estimate_time_ = this->now();
    
    FYT_DEBUG("armor_tracker", "Received {} estimated armors", observations.size());
  }
}

void ArmorTrackerNode::predictTimerCallback() {
  std::lock_guard<std::mutex> lock(callback_mutex_);
  
  auto current_time = this->now();
  double time_since_detection = (current_time - last_detection_time_).seconds();
  double time_since_estimate = (current_time - last_estimate_time_).seconds();
  
  // 如果超过超时时间没有收到任何观测，执行纯预测
  if (time_since_detection > detection_timeout_ && 
      time_since_estimate > detection_timeout_) {
    tracker_core_->predictUpdate();
    FYT_DEBUG("armor_tracker", "No observation, performing predict-only update");
  } else {
    // 正常执行预测步骤
    tracker_core_->predict();
  }
  
  last_predict_time_ = current_time;
}

void ArmorTrackerNode::publishTimerCallback() {
  // 获取跟踪结果
  auto tracks = tracker_core_->getTracks();
  
  // 发布跟踪结果
  rm_interfaces::msg::TrackedArmors tracked_msg;
  tracked_msg.header.stamp = this->now();
  tracked_msg.header.frame_id = "odom";
  
  for (const auto& track : tracks) {
    tracked_msg.armors.push_back(stateToMessage(track, tracked_msg.header.stamp));
  }
  
  tracked_armors_pub_->publish(tracked_msg);
  
  // 发布可视化
  if (debug_mode_ && marker_pub_) {
    publishMarkers(tracks);
  }
}

std::vector<ArmorObservation> ArmorTrackerNode::armorsToObservations(
  const rm_interfaces::msg::Armors& msg,
  ArmorSourceType source) {
  
  std::vector<ArmorObservation> observations;
  observations.reserve(msg.armors.size());
  
  for (const auto& armor : msg.armors) {
    ArmorObservation obs;
    obs.armor_id = armor.number;
    obs.armor_type = armor.type;
    obs.position = Eigen::Vector3d(
      armor.pose.position.x,
      armor.pose.position.y,
      armor.pose.position.z
    );
    obs.yaw = quaternionToYaw(armor.pose.orientation);
    obs.confidence = 1.0f - armor.distance_to_image_center;  // 简化的置信度计算
    obs.source = source;
    obs.timestamp = msg.header.stamp;
    
    observations.push_back(obs);
  }
  
  return observations;
}

rm_interfaces::msg::TrackedArmor ArmorTrackerNode::stateToMessage(
  const TrackedArmorState& state,
  const builtin_interfaces::msg::Time& stamp) {
  
  rm_interfaces::msg::TrackedArmor msg;
  
  msg.header.stamp = stamp;
  msg.header.frame_id = "odom";
  
  msg.track_id = state.track_id;
  msg.armor_id = state.armor_id;
  msg.armor_type = state.armor_type;
  
  msg.position.x = state.position.x();
  msg.position.y = state.position.y();
  msg.position.z = state.position.z();
  
  msg.velocity.x = state.velocity.x();
  msg.velocity.y = state.velocity.y();
  msg.velocity.z = state.velocity.z();
  
  msg.yaw = state.yaw;
  msg.yaw_velocity = state.yaw_velocity;
  
  msg.confidence = state.confidence;
  msg.source_type = static_cast<uint8_t>(state.source_type);
  msg.tracking_state = static_cast<uint8_t>(state.tracking_state);
  msg.tracking_count = state.tracking_count;
  msg.lost_count = state.lost_count;
  msg.time_since_update = state.time_since_update;
  msg.last_detected_time = state.last_detected_time;
  
  return msg;
}

double ArmorTrackerNode::quaternionToYaw(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

void ArmorTrackerNode::publishMarkers(const std::vector<TrackedArmorState>& tracks) {
  visualization_msgs::msg::MarkerArray marker_array;
  
  for (size_t i = 0; i < tracks.size(); ++i) {
    const auto& track = tracks[i];
    
    // 位置标记
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "armor_tracker";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = track.position.x();
    marker.pose.position.y = track.position.y();
    marker.pose.position.z = track.position.z();
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    // 根据状态和来源设置颜色
    switch (track.tracking_state) {
      case TrackingState::TRACKING:
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case TrackingState::DETECTING:
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        break;
      case TrackingState::TEMP_LOST:
        marker.color.r = 1.0;
        marker.color.g = 0.5;
        marker.color.b = 0.0;
        break;
      default:
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
    }
    
    // 根据来源类型调整透明度
    switch (track.source_type) {
      case ArmorSourceType::DETECT:
        marker.color.a = 1.0;
        break;
      case ArmorSourceType::ESTIMATE:
        marker.color.a = 0.7;
        break;
      case ArmorSourceType::PREDICT:
        marker.color.a = 0.4;
        break;
    }
    
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker_array.markers.push_back(marker);
    
    // 速度箭头
    visualization_msgs::msg::Marker vel_marker;
    vel_marker.header = marker.header;
    vel_marker.ns = "velocity";
    vel_marker.id = static_cast<int>(i);
    vel_marker.type = visualization_msgs::msg::Marker::ARROW;
    vel_marker.action = visualization_msgs::msg::Marker::ADD;
    
    geometry_msgs::msg::Point start, end;
    start.x = track.position.x();
    start.y = track.position.y();
    start.z = track.position.z();
    end.x = track.position.x() + track.velocity.x() * 0.5;
    end.y = track.position.y() + track.velocity.y() * 0.5;
    end.z = track.position.z() + track.velocity.z() * 0.5;
    
    vel_marker.points.push_back(start);
    vel_marker.points.push_back(end);
    
    vel_marker.scale.x = 0.02;
    vel_marker.scale.y = 0.04;
    vel_marker.scale.z = 0.0;
    
    vel_marker.color.r = 0.0;
    vel_marker.color.g = 0.0;
    vel_marker.color.b = 1.0;
    vel_marker.color.a = 0.8;
    
    vel_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker_array.markers.push_back(vel_marker);
    
    // 文本标签 - 显示跟踪信息
    visualization_msgs::msg::Marker text_marker;
    text_marker.header = marker.header;
    text_marker.ns = "info";
    text_marker.id = static_cast<int>(i);
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    
    text_marker.pose.position.x = track.position.x();
    text_marker.pose.position.y = track.position.y();
    text_marker.pose.position.z = track.position.z() + 0.15;
    
    text_marker.scale.z = 0.08;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;
    
    text_marker.text = track.armor_id + " [" + 
                       sourceTypeToString(track.source_type) + "/" +
                       trackingStateToString(track.tracking_state) + "]";
    
    text_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    marker_array.markers.push_back(text_marker);
  }
  
  marker_pub_->publish(marker_array);
}

}  // namespace fyt::auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorTrackerNode)
