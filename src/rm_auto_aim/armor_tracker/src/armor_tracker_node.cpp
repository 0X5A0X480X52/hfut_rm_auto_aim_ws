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

// Include model factories for manual registration
#include "basic_models/basic_model_factories.h"
#include "models/model_factory.h"

// Manual factory registration to ensure models are available at runtime
// Static initialization across shared libraries is unreliable, so we register explicitly
namespace {
void registerModelFactories() {
  auto& registry = ModelFactoryRegistry::getInstance();
  
  // Register basic model factories if not already registered
  if (!registry.hasFactory("CV_KF")) {
    registry.registerFactory("CV_KF", std::make_shared<CV_KF_Factory>());
  }
  if (!registry.hasFactory("CA_KF")) {
    registry.registerFactory("CA_KF", std::make_shared<CA_KF_Factory>());
  }
  if (!registry.hasFactory("CS_KF")) {
    registry.registerFactory("CS_KF", std::make_shared<CS_KF_Factory>());
  }
  if (!registry.hasFactory("CTRV_EKF")) {
    registry.registerFactory("CTRV_EKF", std::make_shared<CTRV_EKF_Factory>());
  }
  if (!registry.hasFactory("Singer_KF")) {
    registry.registerFactory("Singer_KF", std::make_shared<Singer_KF_Factory>());
  }
}

// Static initializer to guarantee registration before node construction
struct ModelFactoryInitializer {
  ModelFactoryInitializer() {
    registerModelFactories();
  }
} g_model_factory_initializer;
}  // namespace

namespace fyt::auto_aim
{

ArmorTrackerNode::ArmorTrackerNode(const rclcpp::NodeOptions & options)
: Node("armor_tracker", options)
, last_detection_time_(this->now())
, last_estimate_time_(this->now())
, last_predict_time_(this->now())
{
  FYT_REGISTER_LOGGER("armor_tracker", "~/fyt2024-log", INFO);
  FYT_INFO("armor_tracker", "Starting ArmorTrackerNode (v2.0 - muit_obj_tracker based)!");

  // 声明参数
  declareParameters();
  
  // 构建配置
  auto config = buildConfig();
  topic_config_ = buildTopicConfig();
  auto history_config = buildHistoryConfig();
  auto prediction_config = buildPredictionConfig();
  
  // 创建核心跟踪器
  tracker_core_ = std::make_unique<ArmorTrackerCore>(config);
  
  // 创建策略管理器
  strategy_manager_ = std::make_shared<TrackingStrategyManager>();
  tracker_core_->setStrategyManager(strategy_manager_);
  
  // 创建历史窗口管理器
  history_manager_ = std::make_unique<TrackHistoryManager>(history_config);
  
  // 创建预测窗口管理器
  prediction_manager_ = std::make_unique<TrackPredictionManager>(prediction_config);

  // 订阅者 - 检测结果（话题名从配置读取）
  armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
    topic_config_.armors_sub_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::armorsCallback, this, std::placeholders::_1)
  );
  
  // 订阅者 - 估计装甲板（话题名从配置读取）
  estimated_armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
    topic_config_.estimated_armors_sub_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::estimatedArmorsCallback, this, std::placeholders::_1)
  );

  // 发布者（话题名从配置读取）
  tracked_armors_pub_ = this->create_publisher<rm_interfaces::msg::TrackedArmors>(
    topic_config_.tracked_armors_pub_topic,
    rclcpp::SensorDataQoS()
  );

  if (debug_mode_) {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      topic_config_.markers_pub_topic,
      10
    );
  }
  
  // 历史窗口发布者
  if (enable_history_window_) {
    history_windows_pub_ = this->create_publisher<rm_interfaces::msg::TrackHistoryWindows>(
      topic_config_.history_windows_pub_topic,
      rclcpp::SensorDataQoS()
    );
      FYT_INFO(
        "armor_tracker", "History window publisher enabled: {}",
        topic_config_.history_windows_pub_topic);
  }
  
  // 预测窗口发布者
  if (enable_prediction_window_) {
    prediction_windows_pub_ = this->create_publisher<rm_interfaces::msg::TrackPredictionWindows>(
      topic_config_.prediction_windows_pub_topic,
      rclcpp::SensorDataQoS()
    );
    FYT_INFO(
      "armor_tracker", "Prediction window publisher enabled: {}",
      topic_config_.prediction_windows_pub_topic);
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
  FYT_INFO("armor_tracker", "  - Subscribe armors from: {}", topic_config_.armors_sub_topic);
  FYT_INFO("armor_tracker", "  - Publish tracked to: {}", topic_config_.tracked_armors_pub_topic);
}

void ArmorTrackerNode::declareParameters()
{
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
  
  // ==================== 话题名称配置 ====================
  // 订阅话题
  this->declare_parameter("topics.armors_sub", "/armor_detector/armors");
  this->declare_parameter("topics.estimated_armors_sub", "/robot_pose_estimator/virtual_armors");
  // 发布话题
  this->declare_parameter("topics.tracked_armors_pub", "/armor_tracker/tracked_armors");
  this->declare_parameter("topics.markers_pub", "/armor_tracker/markers");
  this->declare_parameter("topics.history_windows_pub", "/armor_tracker/history_windows");
  this->declare_parameter("topics.prediction_windows_pub", "/armor_tracker/prediction_windows");
  
  // ==================== 历史窗口配置 ====================
  enable_history_window_ = this->declare_parameter("history_window.enable", true);
  this->declare_parameter("history_window.max_size", 100);
  this->declare_parameter("history_window.record_interval", 1);
  
  // ==================== 预测窗口配置 ====================
  enable_prediction_window_ = this->declare_parameter("prediction_window.enable", true);
  this->declare_parameter("prediction_window.steps", 30);
  this->declare_parameter("prediction_window.interval", 1);
  this->declare_parameter("prediction_window.dt", 0.01);

  // no test-specific disable flag here; keep original implementation
}

TrackerConfig ArmorTrackerNode::buildConfig()
{
  TrackerConfig config;
  
  config.max_match_distance = this->get_parameter("max_match_distance").as_double();
  config.max_match_yaw_diff = this->get_parameter("max_match_yaw_diff").as_double();
  config.tracking_threshold = this->get_parameter("tracking_threshold").as_int();
  config.lost_threshold = this->get_parameter("lost_threshold").as_int();
  config.max_trackers = this->get_parameter("max_trackers").as_int();
  config.predict_rate = predict_rate_;
  config.model_name = this->get_parameter("model.name").as_string();
  config.model_config_file = this->get_parameter("model.config_file").as_string();
  // Debug: print out model configuration path for diagnosis
  FYT_DEBUG("armor_tracker", "Model: {} Config file: {}", config.model_name, config.model_config_file);
  
  return config;
}

TopicConfig ArmorTrackerNode::buildTopicConfig()
{
  TopicConfig config;
  
  config.armors_sub_topic = this->get_parameter("topics.armors_sub").as_string();
  config.estimated_armors_sub_topic = this->get_parameter("topics.estimated_armors_sub").as_string();
  config.tracked_armors_pub_topic = this->get_parameter("topics.tracked_armors_pub").as_string();
  config.markers_pub_topic = this->get_parameter("topics.markers_pub").as_string();
  config.history_windows_pub_topic = this->get_parameter("topics.history_windows_pub").as_string();
  config.prediction_windows_pub_topic = this->get_parameter("topics.prediction_windows_pub").as_string();
  
  return config;
}

HistoryWindowConfig ArmorTrackerNode::buildHistoryConfig()
{
  HistoryWindowConfig config;
  
  config.max_window_size = static_cast<uint32_t>(
    this->get_parameter("history_window.max_size").as_int());
  config.record_interval = static_cast<uint32_t>(
    this->get_parameter("history_window.record_interval").as_int());
  
  return config;
}

PredictionWindowConfig ArmorTrackerNode::buildPredictionConfig()
{
  PredictionWindowConfig config;
  
  config.prediction_steps = static_cast<uint32_t>(
    this->get_parameter("prediction_window.steps").as_int());
  config.predict_interval = static_cast<uint32_t>(
    this->get_parameter("prediction_window.interval").as_int());
  config.dt = this->get_parameter("prediction_window.dt").as_double();
  
  return config;
}

void ArmorTrackerNode::armorsCallback(
  const rm_interfaces::msg::Armors::SharedPtr msg)
{
  FYT_DEBUG("armor_tracker", "armorsCallback invoked. header stamp: {} | armors count: {}", msg->header.stamp.sec, msg->armors.size());
  if (!msg->armors.empty()) {
    std::string ids;
    for (const auto& a : msg->armors) {
      ids += a.number + ",";
    }
    FYT_DEBUG("armor_tracker", "Armors received: {}", ids);
  }
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
  const rm_interfaces::msg::Armors::SharedPtr msg)
{
  FYT_DEBUG("armor_tracker", "estimatedArmorsCallback invoked. header stamp: {} | armors count: {}", msg->header.stamp.sec, msg->armors.size());
  if (!msg->armors.empty()) {
    std::string ids;
    for (const auto& a : msg->armors) {
      ids += a.number + ",";
    }
    FYT_DEBUG("armor_tracker", "Estimated armors received: {}", ids);
  }
  
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

void ArmorTrackerNode::predictTimerCallback()
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  
  auto current_time = this->now();
  double time_since_detection = (current_time - last_detection_time_).seconds();
  double time_since_estimate = (current_time - last_estimate_time_).seconds();
  
  // 如果超过超时时间没有收到任何观测，执行纯预测
  if (time_since_detection > detection_timeout_ &&
    time_since_estimate > detection_timeout_)
  {
    tracker_core_->predictUpdate();
    FYT_DEBUG("armor_tracker", "No observation, performing predict-only update");
  } else {
    // 正常执行预测步骤
    tracker_core_->predict();
  }
  
  last_predict_time_ = current_time;
}

void ArmorTrackerNode::publishTimerCallback()
{
  // 获取跟踪结果
  auto tracks = tracker_core_->getTracks();
  auto current_stamp = this->now();
  
  // 发布跟踪结果
  rm_interfaces::msg::TrackedArmors tracked_msg;
  tracked_msg.header.stamp = current_stamp;
  tracked_msg.header.frame_id = "odom";
  
  for (const auto & track : tracks) {
    tracked_msg.armors.push_back(stateToMessage(track, tracked_msg.header.stamp));
  }
  
  tracked_armors_pub_->publish(tracked_msg);
  
  // 更新历史窗口
  if (enable_history_window_ && history_manager_) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = current_stamp.seconds();
    stamp.nanosec = current_stamp.nanoseconds() % 1000000000UL;
    
    history_manager_->update(tracks, stamp);
    
    // 发布历史窗口
    if (history_windows_pub_) {
      auto history_msg = history_manager_->getAllHistoryWindows();
      history_msg.header.stamp = current_stamp;
      history_msg.header.frame_id = "odom";
      history_windows_pub_->publish(history_msg);
    }
  }
  
  // 生成并发布预测窗口
  if (enable_prediction_window_ && prediction_manager_) {
    builtin_interfaces::msg::Time stamp;
    stamp.sec = current_stamp.seconds();
    stamp.nanosec = current_stamp.nanoseconds() % 1000000000UL;
    
    prediction_manager_->generatePredictions(tracks, stamp);
    
    // 发布预测窗口
    if (prediction_windows_pub_) {
      auto prediction_msg = prediction_manager_->getAllPredictionWindows();
      prediction_msg.header.stamp = current_stamp;
      prediction_msg.header.frame_id = "odom";
      prediction_windows_pub_->publish(prediction_msg);
    }
  }
  
  // 发布可视化
  if (debug_mode_ && marker_pub_) {
    publishMarkers(tracks);
  }
}

std::vector<ArmorObservation> ArmorTrackerNode::armorsToObservations(
  const rm_interfaces::msg::Armors & msg,
  ArmorSourceType source)
{
  
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

  // Debug: print constructed observations
  if (!observations.empty()) {
    std::string obs_info;
    for (const auto& o : observations) {
      char buf[128];
      snprintf(buf, sizeof(buf), "%s(%.2f,%.2f,%.2f)", o.armor_id.c_str(), o.position.x(), o.position.y(), o.position.z());
      obs_info += std::string(buf) + ",";
    }
    FYT_DEBUG("armor_tracker", "Converted {} armors to observations: {}", observations.size(), obs_info);
  }
  
  return observations;
}

rm_interfaces::msg::TrackedArmor ArmorTrackerNode::stateToMessage(
  const TrackedArmorState & state,
  const builtin_interfaces::msg::Time & stamp)
{
  
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

double ArmorTrackerNode::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
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
