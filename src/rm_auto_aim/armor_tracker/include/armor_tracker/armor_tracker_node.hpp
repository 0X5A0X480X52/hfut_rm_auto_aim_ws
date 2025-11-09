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

#ifndef ARMOR_TRACKER__ARMOR_TRACKER_NODE_HPP_
#define ARMOR_TRACKER__ARMOR_TRACKER_NODE_HPP_

#include <memory>
#include <vector>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/tracked_armors.hpp"
#include "armor_tracker/single_armor_tracker.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::auto_aim {

/**
 * @brief 多装甲板跟踪节点
 * 管理多个装甲板跟踪器,进行数据关联和状态更新
 */
class ArmorTrackerNode : public rclcpp::Node {
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions& options);

private:
  /**
   * @brief 装甲板检测回调
   */
  void armorsCallback(const rm_interfaces::msg::Armors::SharedPtr msg);

  /**
   * @brief 数据关联 - 将检测结果与现有跟踪器匹配
   * @param armors 检测到的装甲板列表
   * @return 匹配结果 map<tracker_index, armor_index>
   */
  std::map<size_t, size_t> associateDetections(
    const std::vector<rm_interfaces::msg::Armor>& armors);

  /**
   * @brief 清理丢失的跟踪器
   */
  void pruneTrackers();

  /**
   * @brief 发布可视化标记
   */
  void publishMarkers();

  /**
   * @brief 初始化可视化标记
   */
  void initMarkers();

  // 参数
  double max_match_distance_;
  double max_match_yaw_diff_;
  int tracking_threshold_;
  int lost_threshold_;
  int max_trackers_;
  
  // EKF参数
  double sigma2_q_xyz_;
  double sigma2_q_yaw_;
  double r_xyz_;
  double r_yaw_;

  // 跟踪器列表
  std::vector<std::unique_ptr<SingleArmorTracker>> trackers_;

  // 上一次更新时间
  rclcpp::Time last_time_;

  // 订阅者和发布者
  rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Publisher<rm_interfaces::msg::TrackedArmors>::SharedPtr tracked_armors_pub_;
  
  // 可视化
  bool debug_mode_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  // 心跳
  HeartBeatPublisher::SharedPtr heartbeat_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__ARMOR_TRACKER_NODE_HPP_
