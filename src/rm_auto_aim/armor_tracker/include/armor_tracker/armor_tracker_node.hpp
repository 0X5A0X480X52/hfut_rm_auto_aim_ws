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
#include <string>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/tracked_armors.hpp"
#include "rm_interfaces/msg/tracked_armor.hpp"
#include "rm_interfaces/msg/track_history_windows.hpp"
#include "rm_interfaces/msg/track_prediction_windows.hpp"

#include "armor_tracker/armor_types.hpp"
#include "armor_tracker/armor_tracker_core.hpp"
#include "armor_tracker/strategies/tracking_strategy_manager.hpp"
#include "armor_tracker/track_history_manager.hpp"
#include "armor_tracker/track_prediction_manager.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::auto_aim {

/**
 * @brief 话题名称配置结构
 */
struct TopicConfig {
  // 订阅话题
  std::string armors_sub_topic = "/armor_detector/armors";
  std::string estimated_armors_sub_topic = "/robot_pose_estimator/virtual_armors";
  
  // 发布话题
  std::string tracked_armors_pub_topic = "/armor_tracker/tracked_armors";
  std::string markers_pub_topic = "/armor_tracker/markers";
  std::string history_windows_pub_topic = "/armor_tracker/history_windows";
  std::string prediction_windows_pub_topic = "/armor_tracker/prediction_windows";
};

/**
 * @brief 多装甲板跟踪节点
 * 
 * 基于 muit_obj_tracker 的多目标跟踪实现：
 * - 支持异步预测更新（定时器触发）
 * - 支持多来源输入（检测/估计）
 * - 使用策略模式处理不同来源数据
 * - 帧率不稳定时使用预测维持跟踪
 * - 维护历史窗口和预测窗口
 * - 所有话题名称可通过配置文件配置
 */
class ArmorTrackerNode : public rclcpp::Node {
public:
  explicit ArmorTrackerNode(const rclcpp::NodeOptions& options);
  ~ArmorTrackerNode() override = default;

private:
  // ==================== 回调函数 ====================
  
  /**
   * @brief 装甲板检测回调（来源：armor_detector）
   */
  void armorsCallback(const rm_interfaces::msg::Armors::SharedPtr msg);
  
  /**
   * @brief 估计装甲板回调（来源：robot_pose_estimator）
   * 预留接口，用于接收虚拟装甲板估计
   */
  void estimatedArmorsCallback(const rm_interfaces::msg::Armors::SharedPtr msg);
  
  /**
   * @brief 定时器回调 - 异步预测更新
   * 即使没有检测输入也会执行预测
   */
  void predictTimerCallback();
  
  /**
   * @brief 发布定时器回调
   * 按固定频率发布跟踪结果
   */
  void publishTimerCallback();

  // ==================== 辅助函数 ====================
  
  /**
   * @brief 将 ROS 消息转换为内部观测结构
   */
  std::vector<ArmorObservation> armorsToObservations(
    const rm_interfaces::msg::Armors& msg,
    ArmorSourceType source);
  
  /**
   * @brief 将内部状态转换为 ROS 消息
   */
  rm_interfaces::msg::TrackedArmor stateToMessage(
    const TrackedArmorState& state,
    const builtin_interfaces::msg::Time& stamp);
  
  /**
   * @brief 从四元数提取 yaw 角
   */
  double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);
  
  /**
   * @brief 发布可视化标记
   */
  void publishMarkers(const std::vector<TrackedArmorState>& tracks);
  
  /**
   * @brief 声明和加载参数
   */
  void declareParameters();
  
  /**
   * @brief 从参数构建配置
   */
  TrackerConfig buildConfig();
  
  /**
   * @brief 从参数构建话题配置
   */
  TopicConfig buildTopicConfig();
  
  /**
   * @brief 从参数构建历史窗口配置
   */
  HistoryWindowConfig buildHistoryConfig();
  
  /**
   * @brief 从参数构建预测窗口配置
   */
  PredictionWindowConfig buildPredictionConfig();

  // ==================== 成员变量 ====================
  
  // 核心跟踪器
  std::unique_ptr<ArmorTrackerCore> tracker_core_;
  
  // 策略管理器
  std::shared_ptr<TrackingStrategyManager> strategy_manager_;
  
  // 历史窗口管理器
  std::unique_ptr<TrackHistoryManager> history_manager_;
  
  // 预测窗口管理器
  std::unique_ptr<TrackPredictionManager> prediction_manager_;
  
  // 订阅者
  rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr armors_sub_;
  rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr estimated_armors_sub_;
  
  // 发布者
  rclcpp::Publisher<rm_interfaces::msg::TrackedArmors>::SharedPtr tracked_armors_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<rm_interfaces::msg::TrackHistoryWindows>::SharedPtr history_windows_pub_;
  rclcpp::Publisher<rm_interfaces::msg::TrackPredictionWindows>::SharedPtr prediction_windows_pub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr predict_timer_;   // 预测更新定时器
  rclcpp::TimerBase::SharedPtr publish_timer_;   // 发布定时器
  
  // 心跳
  HeartBeatPublisher::SharedPtr heartbeat_;
  
  // 状态标志
  std::atomic<bool> detection_received_{false};  // 是否收到检测数据
  std::atomic<bool> estimate_received_{false};   // 是否收到估计数据
  
  // 时间戳
  rclcpp::Time last_detection_time_;
  rclcpp::Time last_estimate_time_;
  rclcpp::Time last_predict_time_;
  
  // 参数
  bool debug_mode_;
  double predict_rate_;      // 预测更新频率 (Hz)
  double publish_rate_;      // 发布频率 (Hz)
  double detection_timeout_; // 检测超时时间 (s)
  
  // 话题配置
  TopicConfig topic_config_;
  
  // 是否启用历史/预测窗口发布
  bool enable_history_window_;
  bool enable_prediction_window_;
  
  // 线程安全
  std::mutex callback_mutex_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__ARMOR_TRACKER_NODE_HPP_
