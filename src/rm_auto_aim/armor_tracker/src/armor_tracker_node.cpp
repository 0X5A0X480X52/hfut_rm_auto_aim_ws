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
{
  FYT_REGISTER_LOGGER("armor_tracker", "~/fyt2024-log", INFO);
  FYT_INFO("armor_tracker", "Starting ArmorTrackerNode!");

  // 声明参数
  debug_mode_ = this->declare_parameter("debug", true);
  max_match_distance_ = this->declare_parameter("max_match_distance", 0.2);
  max_match_yaw_diff_ = this->declare_parameter("max_match_yaw_diff", 1.0);
  tracking_threshold_ = this->declare_parameter("tracking_threshold", 5);
  lost_threshold_ = this->declare_parameter("lost_threshold", 10);
  max_trackers_ = this->declare_parameter("max_trackers", 20);
  
  // EKF参数
  sigma2_q_xyz_ = this->declare_parameter("ekf.sigma2_q_xyz", 20.0);
  sigma2_q_yaw_ = this->declare_parameter("ekf.sigma2_q_yaw", 1.0);
  r_xyz_ = this->declare_parameter("ekf.r_xyz", 0.05);
  r_yaw_ = this->declare_parameter("ekf.r_yaw", 0.02);

  // 订阅者
  armors_sub_ = this->create_subscription<rm_interfaces::msg::Armors>(
    "/armor_detector/armors",
    rclcpp::SensorDataQoS(),
    std::bind(&ArmorTrackerNode::armorsCallback, this, std::placeholders::_1)
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

  // 心跳
  heartbeat_ = HeartBeatPublisher::create(this);

  last_time_ = this->now();

  FYT_INFO("armor_tracker", "ArmorTrackerNode initialized successfully!");
}

void ArmorTrackerNode::armorsCallback(
  const rm_interfaces::msg::Armors::SharedPtr msg) {
  
  auto current_time = msg->header.stamp;
  double dt = (rclcpp::Time(current_time) - last_time_).seconds();
  
  if (dt <= 0.0 || dt > 1.0) {
    dt = 0.01;  // 默认值
  }

  // 数据关联
  auto associations = associateDetections(msg->armors);

  // 更新已有的跟踪器
  std::vector<bool> armor_matched(msg->armors.size(), false);
  
  for (size_t i = 0; i < trackers_.size(); ++i) {
    const rm_interfaces::msg::Armor* matched_armor = nullptr;
    
    if (associations.find(i) != associations.end()) {
      size_t armor_idx = associations[i];
      matched_armor = &msg->armors[armor_idx];
      armor_matched[armor_idx] = true;
    }
    
    trackers_[i]->update(matched_armor, dt);
  }

  // 为未匹配的检测创建新跟踪器
  for (size_t i = 0; i < msg->armors.size(); ++i) {
    if (!armor_matched[i] && trackers_.size() < static_cast<size_t>(max_trackers_)) {
      const auto& armor = msg->armors[i];
      auto tracker = std::make_unique<SingleArmorTracker>(
        armor.number,
        armor.type,
        max_match_distance_,
        max_match_yaw_diff_,
        tracking_threshold_,
        lost_threshold_
      );
      tracker->initEKF(armor);
      trackers_.push_back(std::move(tracker));
      
      FYT_INFO("armor_tracker", "Created new tracker for armor: {}", armor.number);
    }
  }

  // 清理丢失的跟踪器
  pruneTrackers();

  // 发布跟踪结果
  rm_interfaces::msg::TrackedArmors tracked_msg;
  tracked_msg.header = msg->header;
  
  for (const auto& tracker : trackers_) {
    auto tracked_armor = tracker->getTrackedArmor(current_time);
    tracked_msg.armors.push_back(tracked_armor);
  }
  
  tracked_armors_pub_->publish(tracked_msg);

  // 发布可视化
  if (debug_mode_ && marker_pub_) {
    publishMarkers();
  }

  last_time_ = rclcpp::Time(current_time);
}

std::map<size_t, size_t> ArmorTrackerNode::associateDetections(
  const std::vector<rm_interfaces::msg::Armor>& armors) {
  
  std::map<size_t, size_t> associations;
  
  // 简单的贪心匹配算法
  // 可以后续升级为匈牙利算法
  std::vector<bool> armor_used(armors.size(), false);
  
  for (size_t t = 0; t < trackers_.size(); ++t) {
    double best_distance = std::numeric_limits<double>::max();
    int best_armor_idx = -1;
    
    for (size_t a = 0; a < armors.size(); ++a) {
      if (armor_used[a]) continue;
      
      if (trackers_[t]->isMatched(armors[a])) {
        // 计算距离作为匹配代价
        auto predicted_pos = trackers_[t]->getTrackedArmor(rclcpp::Time(0)).position;
        Eigen::Vector3d pred(predicted_pos.x, predicted_pos.y, predicted_pos.z);
        Eigen::Vector3d det(
          armors[a].pose.position.x,
          armors[a].pose.position.y,
          armors[a].pose.position.z
        );
        double distance = (pred - det).norm();
        
        if (distance < best_distance) {
          best_distance = distance;
          best_armor_idx = a;
        }
      }
    }
    
    if (best_armor_idx >= 0) {
      associations[t] = best_armor_idx;
      armor_used[best_armor_idx] = true;
    }
  }
  
  return associations;
}

void ArmorTrackerNode::pruneTrackers() {
  trackers_.erase(
    std::remove_if(
      trackers_.begin(),
      trackers_.end(),
      [](const std::unique_ptr<SingleArmorTracker>& tracker) {
        return tracker->shouldBeRemoved();
      }
    ),
    trackers_.end()
  );
}

void ArmorTrackerNode::publishMarkers() {
  visualization_msgs::msg::MarkerArray marker_array;
  
  for (size_t i = 0; i < trackers_.size(); ++i) {
    auto tracked = trackers_[i]->getTrackedArmor(this->now());
    
    // 位置标记
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "armor_tracker";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position = tracked.position;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    
    // 根据状态设置颜色
    if (tracked.tracking_state == rm_interfaces::msg::TrackedArmor::TRACKING) {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else if (tracked.tracking_state == rm_interfaces::msg::TrackedArmor::DETECTING) {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }
    marker.color.a = tracked.confidence;
    
    marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    
    marker_array.markers.push_back(marker);
    
    // 速度箭头
    visualization_msgs::msg::Marker vel_marker;
    vel_marker.header = marker.header;
    vel_marker.ns = "velocity";
    vel_marker.id = i;
    vel_marker.type = visualization_msgs::msg::Marker::ARROW;
    vel_marker.action = visualization_msgs::msg::Marker::ADD;
    
    geometry_msgs::msg::Point start = tracked.position;
    geometry_msgs::msg::Point end = tracked.position;
    end.x += tracked.velocity.x * 0.5;
    end.y += tracked.velocity.y * 0.5;
    end.z += tracked.velocity.z * 0.5;
    
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
  }
  
  marker_pub_->publish(marker_array);
}

}  // namespace fyt::auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorTrackerNode)
