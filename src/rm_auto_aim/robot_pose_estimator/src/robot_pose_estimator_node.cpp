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
  
  // 注册日志器（如果尚未注册）
  try {
    FYT_REGISTER_LOGGER("robot_pose_estimator", "logs/robot_pose_estimator", INFO);
  } catch (...) {
    // Logger may already be registered, ignore
  }
  
  FYT_INFO("robot_pose_estimator", "Initializing Robot Pose Estimator Node...");
  
  // 初始化 TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  // 声明和加载参数
  declareParameters();
  
  // 构建配置
  auto config = buildConfig();
  topic_config_ = buildTopicConfig();
  
  // 创建核心估计器
  estimator_core_ = std::make_unique<RobotPoseEstimatorCore>(config);
  
  // 创建订阅者 - 直接订阅armor_detector的检测结果
  armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
    topic_config_.armors_sub,
    rclcpp::SensorDataQoS(),
    std::bind(&RobotPoseEstimatorNode::armorsCallback, this, std::placeholders::_1)
  );
  
  // 创建订阅者 - 订阅armor_tracker的历史窗口，用于建立track_id与armor_id的绑定关系
  history_windows_sub_ = this->create_subscription<rm_interfaces::msg::TrackHistoryWindows>(
    topic_config_.history_windows_sub,
    rclcpp::SensorDataQoS(),
    std::bind(&RobotPoseEstimatorNode::historyWindowsCallback, this, std::placeholders::_1)
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
  FYT_INFO("robot_pose_estimator", "  Subscribing to: {} (from detector, avoids feedback loop)", topic_config_.armors_sub);
  FYT_INFO("robot_pose_estimator", "  Subscribing to: {} (for track_id binding)", topic_config_.history_windows_sub);
  FYT_INFO("robot_pose_estimator", "  Publishing robots to: {}", topic_config_.robots_pub);
  FYT_INFO("robot_pose_estimator", "  Publishing virtual armors to: {} (-> armor_tracker)", topic_config_.virtual_armors_pub);
  FYT_INFO("robot_pose_estimator", "  Publishing target to: {}", topic_config_.target_pub);
}

void RobotPoseEstimatorNode::declareParameters() {
  // 调试模式
  debug_mode_ = this->declare_parameter("debug", false);
  predict_rate_ = this->declare_parameter("predict_rate", 100.0);
  
  // 坐标系配置 - 与 armor_detector 保持一致
  target_frame_ = this->declare_parameter("target_frame", "odom");
  
  // 话题配置 - 订阅armor_detector的检测结果，避免与armor_tracker形成反馈回路
  this->declare_parameter("topics.armors_sub", "/armor_detector/armors");
  this->declare_parameter("topics.history_windows_sub", "/armor_tracker/history_windows");
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
  
  config.armors_sub = this->get_parameter("topics.armors_sub").as_string();
  config.history_windows_sub = this->get_parameter("topics.history_windows_sub").as_string();
  config.robots_pub = this->get_parameter("topics.robots_pub").as_string();
  config.virtual_armors_pub = this->get_parameter("topics.virtual_armors_pub").as_string();
  config.target_pub = this->get_parameter("topics.target_pub").as_string();
  config.markers_pub = this->get_parameter("topics.markers_pub").as_string();
  
  return config;
}

void RobotPoseEstimatorNode::armorsCallback(
    const rm_interfaces::msg::Armors::SharedPtr msg) {
  
  // 计算时间间隔
  rclcpp::Time current_time = this->now();
  double dt = (current_time - last_update_time_).seconds();
  last_update_time_ = current_time;
  
  // 限制 dt 范围
  dt = std::clamp(dt, 0.001, 0.1);
  
  // 获取源坐标系
  const std::string& source_frame = msg->header.frame_id;
  bool need_transform = (source_frame != target_frame_);
  
  // 转换装甲板位置到目标坐标系
  rm_interfaces::msg::Armors transformed_msg;
  transformed_msg.header.stamp = msg->header.stamp;
  transformed_msg.header.frame_id = target_frame_;  // 使用目标坐标系
  
  for (const auto& armor : msg->armors) {
    rm_interfaces::msg::Armor transformed_armor = armor;
    
    if (need_transform) {
      try {
        // 创建源坐标系中的位姿
        geometry_msgs::msg::PoseStamped source_pose;
        source_pose.header = msg->header;
        source_pose.pose = armor.pose;
        
        // 转换到目标坐标系
        geometry_msgs::msg::PoseStamped target_pose;
        tf_buffer_->transform(source_pose, target_pose, target_frame_);
        
        // 使用转换后的位姿
        transformed_armor.pose = target_pose.pose;
        
      } catch (tf2::TransformException &ex) {
        FYT_WARN("robot_pose_estimator", "TF transform failed: {}", ex.what());
        continue;  // 跳过这个装甲板
      }
    }
    
    transformed_msg.armors.push_back(transformed_armor);
  }
  
  // 使用转换后的消息更新估计器
  estimator_core_->update(transformed_msg, dt);
  
  // 获取机器人状态
  auto robots = estimator_core_->getTrackingRobots();
  
  // 使用目标坐标系的 header 发布
  std_msgs::msg::Header output_header;
  output_header.stamp = msg->header.stamp;
  output_header.frame_id = target_frame_;
  
  // 发布机器人状态
  publishRobots(robots, output_header);
  
  // 发布虚拟装甲板
  if (estimator_core_->getConfig().virtual_armor_generation) {
    auto virtual_armors = estimator_core_->generateVirtualArmors();
    publishVirtualArmors(virtual_armors, output_header);
  }
  
  // 发布 Target 消息（选择最佳机器人）
  const RobotState* best_robot = selectBestRobot(robots);
  if (best_robot != nullptr) {
    publishTarget(*best_robot, output_header);
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

void RobotPoseEstimatorNode::historyWindowsCallback(
    const rm_interfaces::msg::TrackHistoryWindows::SharedPtr msg) {
  
  std::map<std::string, std::vector<int>> armor_id_to_track_ids_copy;
  
  {
    std::lock_guard<std::mutex> lock(binding_mutex_);
    
    // 清空旧的映射
    track_id_to_armor_id_.clear();
    armor_id_to_track_ids_.clear();
    
    // 从历史窗口建立 track_id <-> armor_id 的绑定关系
    for (const auto& window : msg->windows) {
      int track_id = window.track_id;
      const std::string& armor_id = window.armor_id;
      
      // track_id -> armor_id
      track_id_to_armor_id_[track_id] = armor_id;
      
      // armor_id -> track_ids（同一个armor_id可能有多个track_id）
      armor_id_to_track_ids_[armor_id].push_back(track_id);
    }
    
    armor_id_to_track_ids_copy = armor_id_to_track_ids_;
    
    FYT_DEBUG("robot_pose_estimator", "Updated armor binding: {} tracks, {} unique armor_ids", 
              track_id_to_armor_id_.size(), armor_id_to_track_ids_.size());
  }
  
  // 更新 estimator_core 中各 tracker 的绑定数量（用于置信度计算）
  estimator_core_->updateBindingCounts(armor_id_to_track_ids_copy);
}

void RobotPoseEstimatorNode::publishRobots(
    const std::vector<RobotState>& robots,
    const std_msgs::msg::Header& header) {
  
  rm_interfaces::msg::TrackedRobots msg;
  msg.header = header;
  
  // 获取当前的绑定映射（加锁）
  std::map<std::string, std::vector<int>> armor_id_to_track_ids_copy;
  {
    std::lock_guard<std::mutex> lock(binding_mutex_);
    armor_id_to_track_ids_copy = armor_id_to_track_ids_;
  }
  
  for (const auto& robot : robots) {
    auto robot_msg = robotStateToMsg(robot, header);
    
    // 使用绑定映射填充 bound_armor_ids
    // robot.robot_id 就是 armor_id（例如 "3"）
    auto it = armor_id_to_track_ids_copy.find(robot.robot_id);
    if (it != armor_id_to_track_ids_copy.end()) {
      // 将 track_id 转换为字符串
      for (int track_id : it->second) {
        robot_msg.bound_armor_ids.push_back(std::to_string(track_id));
      }
      FYT_DEBUG("robot_pose_estimator", "Robot {} bound to {} tracks", 
                robot.robot_id, robot_msg.bound_armor_ids.size());
    }
    
    msg.robots.push_back(robot_msg);
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
  msg.bound_armor_ids = state.bound_armor_ids;
  
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
    center_marker.header.frame_id = target_frame_;
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
    text_marker.header.frame_id = target_frame_;
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
    
    // 绘制装甲板 CUBE (类似 armor_solver 的可视化方式)
    if (robot.is_tracking) {
      double yaw = robot.yaw;
      double r1 = robot.radius_1;
      double r2 = robot.radius_2;
      double xc = robot.center_position.x();
      double yc = robot.center_position.y();
      double zc = robot.center_position.z();
      double d_za = robot.d_za;
      double d_zc = robot.d_zc;
      int a_n = robot.num_armors;
      
      // 判断装甲板尺寸
      bool is_large = (robot.robot_type == RobotType::BALANCE_2 ||
                       robot.robot_type == RobotType::HERO_4 ||
                       robot.robot_type == RobotType::OUTPOST_3 ||
                       robot.robot_type == RobotType::BASE);
      double armor_width = is_large ? 0.23 : 0.135;
      
      bool is_current_pair = true;
      for (int i = 0; i < a_n; ++i) {
        double tmp_yaw = yaw + i * (2.0 * M_PI / a_n);
        double r = 0.0;
        double p_z = 0.0;
        
        // 只有4装甲板有两个半径和高度
        if (a_n == 4) {
          r = is_current_pair ? r1 : r2;
          p_z = zc + d_zc + (is_current_pair ? 0.0 : d_za);
          is_current_pair = !is_current_pair;
        } else {
          r = r1;
          p_z = zc + d_zc;
        }
        
        double p_x = xc - r * std::cos(tmp_yaw);
        double p_y = yc - r * std::sin(tmp_yaw);
        
        visualization_msgs::msg::Marker armor_marker;
        armor_marker.header.frame_id = target_frame_;
        armor_marker.header.stamp = this->now();
        armor_marker.ns = "predicted_armors";
        armor_marker.id = id++;
        armor_marker.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker.action = visualization_msgs::msg::Marker::ADD;
        
        armor_marker.pose.position.x = p_x;
        armor_marker.pose.position.y = p_y;
        armor_marker.pose.position.z = p_z;
        
        // 设置朝向（装甲板朝外）
        tf2::Quaternion q;
        q.setRPY(0, robot.robot_id == "outpost" ? -0.2618 : 0.2618, tmp_yaw);
        armor_marker.pose.orientation = tf2::toMsg(q);
        
        // 装甲板尺寸
        armor_marker.scale.x = 0.03;          // 厚度
        armor_marker.scale.y = armor_width;    // 宽度
        armor_marker.scale.z = 0.125;          // 高度
        
        // 蓝色表示预测装甲板
        armor_marker.color.r = 0.0;
        armor_marker.color.g = 0.5;
        armor_marker.color.b = 1.0;
        armor_marker.color.a = 0.8;
        
        armor_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
        
        marker_array.markers.push_back(armor_marker);
      }
    }
  }
  
  marker_pub_->publish(marker_array);
}

}  // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::RobotPoseEstimatorNode)
