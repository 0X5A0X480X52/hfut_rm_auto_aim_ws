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

#ifndef ROBOT_POSE_ESTIMATOR__ROBOT_POSE_ESTIMATOR_NODE_HPP_
#define ROBOT_POSE_ESTIMATOR__ROBOT_POSE_ESTIMATOR_NODE_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "robot_pose_estimator/robot_pose_estimator_core.hpp"
#include "rm_interfaces/msg/tracked_armors.hpp"
#include "rm_interfaces/msg/tracked_robots.hpp"
#include "rm_interfaces/msg/tracked_robot.hpp"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::auto_aim {

/**
 * @brief 话题配置结构
 */
struct EstimatorTopicConfig {
  // 订阅话题 - 直接订阅armor_detector的检测结果，避免与armor_tracker形成反馈回路
  std::string armors_sub = "/armor_detector/armors";
  
  // 发布话题
  std::string robots_pub = "/robot_pose_estimator/robots";
  std::string virtual_armors_pub = "/robot_pose_estimator/virtual_armors";
  std::string target_pub = "/robot_pose_estimator/target";  // 兼容 armor_solver 的 Target 消息
  std::string markers_pub = "/robot_pose_estimator/markers";
};

/**
 * @brief 机器人姿态估计节点
 * 
 * 订阅: /armor_detector/armors (直接从检测器获取，避免反馈回路)
 * 发布: 
 *   - /robot_pose_estimator/robots (TrackedRobots)
 *   - /robot_pose_estimator/virtual_armors (Armors) -> 发往armor_tracker
 *   - /robot_pose_estimator/target (Target) - 兼容 armor_solver
 *   - /robot_pose_estimator/markers (MarkerArray) - 可视化
 */
class RobotPoseEstimatorNode : public rclcpp::Node {
public:
  explicit RobotPoseEstimatorNode(const rclcpp::NodeOptions& options);
  ~RobotPoseEstimatorNode() override = default;

private:
  // ==================== 回调函数 ====================
  
  /**
   * @brief 检测装甲板回调（来自 armor_detector）
   */
  void armorsCallback(const rm_interfaces::msg::Armors::SharedPtr msg);
  
  /**
   * @brief 定时器回调 - 预测更新
   */
  void predictTimerCallback();

  // ==================== 发布函数 ====================
  
  /**
   * @brief 发布机器人状态
   */
  void publishRobots(const std::vector<RobotState>& robots,
                     const std_msgs::msg::Header& header);
  
  /**
   * @brief 发布虚拟装甲板
   */
  void publishVirtualArmors(const std::vector<rm_interfaces::msg::Armor>& armors,
                            const std_msgs::msg::Header& header);
  
  /**
   * @brief 发布 Target 消息 (兼容 armor_solver)
   */
  void publishTarget(const RobotState& robot,
                     const std_msgs::msg::Header& header);
  
  /**
   * @brief 发布可视化标记
   */
  void publishMarkers(const std::vector<RobotState>& robots);

  // ==================== 辅助函数 ====================
  
  /**
   * @brief 声明和加载参数
   */
  void declareParameters();
  
  /**
   * @brief 从参数构建配置
   */
  PoseEstimatorConfig buildConfig();
  
  /**
   * @brief 从参数构建话题配置
   */
  EstimatorTopicConfig buildTopicConfig();
  
  /**
   * @brief 将 RobotState 转换为 TrackedRobot 消息
   */
  rm_interfaces::msg::TrackedRobot robotStateToMsg(
    const RobotState& state,
    const std_msgs::msg::Header& header);
  
  /**
   * @brief 将 RobotState 转换为 Target 消息
   */
  rm_interfaces::msg::Target robotStateToTarget(
    const RobotState& state,
    const std_msgs::msg::Header& header);
  
  /**
   * @brief 选择最佳机器人用于生成 Target
   */
  const RobotState* selectBestRobot(const std::vector<RobotState>& robots);

  // ==================== 成员变量 ====================
  
  // 核心估计器
  std::unique_ptr<RobotPoseEstimatorCore> estimator_core_;
  
  // 订阅者 - 订阅armor_detector的检测结果
  rclcpp::Subscription<rm_interfaces::msg::Armors>::SharedPtr armors_sub_;
  
  // 发布者
  rclcpp::Publisher<rm_interfaces::msg::TrackedRobots>::SharedPtr robots_pub_;
  rclcpp::Publisher<rm_interfaces::msg::Armors>::SharedPtr virtual_armors_pub_;
  rclcpp::Publisher<rm_interfaces::msg::Target>::SharedPtr target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr predict_timer_;
  
  // 心跳
  HeartBeatPublisher::SharedPtr heartbeat_;
  
  // 时间戳
  rclcpp::Time last_update_time_;
  
  // 参数
  bool debug_mode_;
  double predict_rate_;
  std::string target_frame_;  // 目标坐标系
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // 话题配置
  EstimatorTopicConfig topic_config_;
};

}  // namespace fyt::auto_aim

#endif  // ROBOT_POSE_ESTIMATOR__ROBOT_POSE_ESTIMATOR_NODE_HPP_
