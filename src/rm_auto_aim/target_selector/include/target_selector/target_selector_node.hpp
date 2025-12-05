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

#ifndef TARGET_SELECTOR__TARGET_SELECTOR_NODE_HPP_
#define TARGET_SELECTOR__TARGET_SELECTOR_NODE_HPP_

#include <memory>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "target_selector/selection_strategy.hpp"
#include "target_selector/strategies/min_yaw_deviation_strategy.hpp"
#include "rm_interfaces/msg/tracked_robots.hpp"
#include "rm_interfaces/msg/selected_target.hpp"
#include "rm_interfaces/srv/set_target_robot.hpp"
#include "rm_interfaces/action/trajectory_plan.hpp"
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

/**
 * @brief 话题配置结构
 */
struct SelectorTopicConfig {
  // 订阅话题
  std::string robots_sub = "/robot_pose_estimator/robots";
  
  // 发布话题
  std::string selected_target_pub = "/target_selector/selected_target";
  std::string markers_pub = "/target_selector/markers";
  
  // Service 名称
  std::string set_target_service = "/trajectory_planner/set_target_robot";
  
  // Action 名称
  std::string trajectory_action = "/trajectory_planner/trajectory_plan";
};

/**
 * @brief 选板器配置
 */
struct SelectorConfig {
  // 策略名称
  std::string strategy_name = "min_yaw_deviation";
  
  // 参考 yaw 方向 (rad), 0.0 表示云台正前方
  double reference_yaw = 0.0;
  
  // 选择限制
  double max_yaw_deviation = M_PI;     // 最大 yaw 偏差 (rad)
  double max_distance = 10.0;          // 最大距离 (m)
  double min_confidence = 0.3;         // 最小置信度
  
  // 滞回参数
  double hysteresis_threshold = 0.1;   // 切换目标的滞回阈值
  
  // TF 坐标系
  std::string gimbal_frame = "gimbal_link";
  std::string odom_frame = "odom";
  
  // 调试模式
  bool debug = false;
};

/**
 * @brief 目标选择器节点
 * 
 * 功能:
 * 1. 订阅机器人姿态估计器发布的机器人列表
 * 2. 使用策略模式选择打击目标
 * 3. 发布选中的目标
 * 4. 通过 Service 和 Action 与 trajectory_planner 交互
 * 
 * 订阅:
 *   - /robot_pose_estimator/robots (TrackedRobots)
 * 
 * 发布:
 *   - /target_selector/selected_target (SelectedTarget)
 *   - /target_selector/markers (MarkerArray) - 可视化
 * 
 * 服务客户端:
 *   - /trajectory_planner/set_target_robot (SetTargetRobot)
 * 
 * Action 客户端:
 *   - /trajectory_planner/trajectory_plan (TrajectoryPlan)
 */
class TargetSelectorNode : public rclcpp::Node {
public:
  using TrackedRobots = rm_interfaces::msg::TrackedRobots;
  using SelectedTarget = rm_interfaces::msg::SelectedTarget;
  using SetTargetRobot = rm_interfaces::srv::SetTargetRobot;
  using TrajectoryPlan = rm_interfaces::action::TrajectoryPlan;
  using TrajectoryPlanGoalHandle = rclcpp_action::ClientGoalHandle<TrajectoryPlan>;

  explicit TargetSelectorNode(const rclcpp::NodeOptions& options);
  ~TargetSelectorNode() override = default;

private:
  // ==================== 初始化函数 ====================
  
  /**
   * @brief 声明 ROS 参数
   */
  void declareParameters();
  
  /**
   * @brief 构建配置
   */
  SelectorConfig buildConfig();
  
  /**
   * @brief 构建话题配置
   */
  SelectorTopicConfig buildTopicConfig();
  
  /**
   * @brief 初始化选择策略
   */
  void initStrategy();
  
  // ==================== 回调函数 ====================
  
  /**
   * @brief 机器人状态回调
   */
  void robotsCallback(const TrackedRobots::SharedPtr msg);
  
  // ==================== Action 回调 ====================
  
  /**
   * @brief Action 目标响应回调
   */
  void trajectoryGoalResponseCallback(
    const TrajectoryPlanGoalHandle::SharedPtr& goal_handle);
  
  /**
   * @brief Action 反馈回调
   */
  void trajectoryFeedbackCallback(
    TrajectoryPlanGoalHandle::SharedPtr goal_handle,
    const std::shared_ptr<const TrajectoryPlan::Feedback> feedback);
  
  /**
   * @brief Action 结果回调
   */
  void trajectoryResultCallback(
    const TrajectoryPlanGoalHandle::WrappedResult& result);
  
  // ==================== 控制函数 ====================
  
  /**
   * @brief 处理目标选择逻辑
   */
  void processTargetSelection(const TrackedRobots& robots);
  
  /**
   * @brief 启动轨迹规划 Action
   */
  void startTrajectoryPlanning(const std::string& robot_id);
  
  /**
   * @brief 停止轨迹规划 Action
   */
  void stopTrajectoryPlanning();
  
  /**
   * @brief 调用设置目标服务
   */
  void callSetTargetService(const std::string& robot_id);
  
  // ==================== 发布函数 ====================
  
  /**
   * @brief 发布选中的目标
   */
  void publishSelectedTarget(const SelectionResult& result);
  
  /**
   * @brief 发布可视化标记
   */
  void publishMarkers(const TrackedRobots& robots, 
                      const std::optional<SelectionResult>& selected);
  
  // ==================== 成员变量 ====================
  
  // 配置
  SelectorConfig config_;
  SelectorTopicConfig topic_config_;
  SelectionConfig selection_config_;
  
  // 选择策略
  SelectionStrategyPtr strategy_;
  
  // 当前状态
  std::string current_target_id_;
  bool is_tracking_active_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // 订阅者
  rclcpp::Subscription<TrackedRobots>::SharedPtr robots_sub_;
  
  // 发布者
  rclcpp::Publisher<SelectedTarget>::SharedPtr selected_target_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  // 服务客户端
  rclcpp::Client<SetTargetRobot>::SharedPtr set_target_client_;
  
  // Action 客户端
  rclcpp_action::Client<TrajectoryPlan>::SharedPtr trajectory_action_client_;
  TrajectoryPlanGoalHandle::SharedPtr current_goal_handle_;
  
  // 心跳
  HeartBeatPublisher::SharedPtr heartbeat_;
};

}  // namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::TargetSelectorNode)

#endif  // TARGET_SELECTOR__TARGET_SELECTOR_NODE_HPP_
