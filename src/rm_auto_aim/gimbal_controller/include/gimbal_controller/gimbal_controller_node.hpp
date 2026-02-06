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

#ifndef GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_NODE_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "rm_interfaces/msg/tracked_robot.hpp"
#include "rm_interfaces/msg/tracked_robots.hpp"
#include "rm_interfaces/msg/selected_target.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/srv/set_mode.hpp"

#include "gimbal_controller/gimbal_control_strategy.hpp"
#include "gimbal_controller/armor_position_calculator.hpp"
#include "gimbal_controller/armor_selector.hpp"
#include "gimbal_controller/ballistic_solver_client.hpp"
#include "gimbal_controller/local_trajectory_compensator.hpp"
#include "gimbal_controller/fire_advisor.hpp"

namespace gimbal_controller
{

/**
 * @brief 云台控制器节点
 * 
 * 订阅 TrackedRobots/SelectedTarget/JointState，发布 GimbalCmd
 */
class GimbalControllerNode : public rclcpp::Node
{
public:
  explicit GimbalControllerNode(const rclcpp::NodeOptions & options);
  ~GimbalControllerNode() override = default;

private:
  // 回调函数
  void trackedRobotsCallback(const rm_interfaces::msg::TrackedRobots::SharedPtr msg);
  void selectedTargetCallback(const rm_interfaces::msg::SelectedTarget::SharedPtr msg);
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void timerCallback();

  // 服务回调
  void setModeCallback(
    const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
    std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);

  // 辅助函数
  void initializeComponents();
  void initializeStrategies();
  void updateGimbalState();
  GimbalControlStrategy::SharedPtr getStrategy(const std::string & name) const;

  // 可视化函数
  void initMarkers();
  void publishMarkers(
    const rm_interfaces::msg::TrackedRobot & target_robot,
    const rm_interfaces::msg::GimbalCmd & cmd);
  std::array<float, 4> hsvToRgb(float h, float s, float v);

  // ROS2 通信
  rclcpp::Subscription<rm_interfaces::msg::TrackedRobots>::SharedPtr tracked_robots_sub_;
  rclcpp::Subscription<rm_interfaces::msg::SelectedTarget>::SharedPtr selected_target_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // 组件
  std::shared_ptr<ArmorPositionCalculator> position_calculator_;
  std::shared_ptr<ArmorSelector> armor_selector_;
  std::shared_ptr<BallisticSolverClient> ballistic_client_;
  std::shared_ptr<LocalTrajectoryCompensator> local_compensator_;
  std::shared_ptr<FireAdvisor> fire_advisor_;

  // 策略
  std::unordered_map<std::string, GimbalControlStrategy::SharedPtr> strategies_;
  std::string current_strategy_name_{"predicted"};

  // 状态
  rm_interfaces::msg::TrackedRobots::SharedPtr latest_robots_;
  rm_interfaces::msg::SelectedTarget::SharedPtr selected_target_;
  double current_yaw_{0.0};
  double current_pitch_{0.0};
  bool enable_{true};

  // 参数
  std::string target_frame_{"odom"};
  double bullet_speed_{20.0};
  double control_rate_{250.0};
  std::string ballistic_mode_{"service"};  // "service" or "local"
  bool debug_mode_{true};

  // 可视化 Markers
  visualization_msgs::msg::Marker position_marker_;
  visualization_msgs::msg::Marker target_velocity_marker_;
  visualization_msgs::msg::Marker armors_marker_;
  visualization_msgs::msg::Marker selection_marker_;
  visualization_msgs::msg::Marker predicted_marker_;
  visualization_msgs::msg::Marker trajectory_marker_;
  std::vector<std::array<float, 4>> color_palette_;
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__GIMBAL_CONTROLLER_NODE_HPP_
