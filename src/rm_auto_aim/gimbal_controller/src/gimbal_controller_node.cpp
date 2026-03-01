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

#include "gimbal_controller/gimbal_controller_node.hpp"
#include "gimbal_controller/strategies/current_position_strategy.hpp"
#include "gimbal_controller/strategies/predicted_position_strategy.hpp"
#include "gimbal_controller/strategies/mpc_control_strategy.hpp"
#include "gimbal_controller/strategies/state_machine_strategy.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace gimbal_controller
{

GimbalControllerNode::GimbalControllerNode(const rclcpp::NodeOptions & options)
: Node("gimbal_controller", options)
{
  RCLCPP_INFO(get_logger(), "Starting GimbalControllerNode!");

  // 声明参数
  target_frame_ = declare_parameter("target_frame", "odom");
  bullet_speed_ = declare_parameter("bullet_speed", 20.0);
  control_rate_ = declare_parameter("control_rate", 250.0);
  current_strategy_name_ = declare_parameter("strategy", "current");
  ballistic_mode_ = declare_parameter("ballistic_mode", "service");
  debug_mode_ = declare_parameter("debug", true);

  // Solver 参数
  double shooting_range_w = declare_parameter("solver.shooting_range_width", 0.135);
  double shooting_range_h = declare_parameter("solver.shooting_range_height", 0.135);
  double side_angle = declare_parameter("solver.side_angle", 15.0);
  double min_switching_v_yaw = declare_parameter("solver.min_switching_v_yaw", 1.0);
  double prediction_delay = declare_parameter("solver.prediction_delay", 0.0);
  double max_tracking_v_yaw = declare_parameter("solver.max_tracking_v_yaw", 6.0);
  int transfer_thresh = declare_parameter("solver.transfer_thresh", 5);

  // 弹道参数
  double gravity = declare_parameter("solver.gravity", 9.8);
  double resistance = declare_parameter("solver.resistance", 0.001);
  int iteration_times = declare_parameter("solver.iteration_times", 20);

  // 手动补偿参数
  double pitch_offset = declare_parameter("solver.pitch_offset", 0.0);
  double yaw_offset = declare_parameter("solver.yaw_offset", 0.0);

  // Facing 过滤参数 (方案一)
  double facing_enter_angle = declare_parameter("solver.facing_enter_angle", 40.0);
  double facing_exit_angle = declare_parameter("solver.facing_exit_angle", 55.0);

  // 状态机策略参数 (方案二)
  double sm_facing_enter = declare_parameter("state_machine.facing_enter_angle", 40.0);
  double sm_facing_exit = declare_parameter("state_machine.facing_exit_angle", 55.0);
  double sm_spin_thresh = declare_parameter("state_machine.spin_v_yaw_thresh", 4.0);
  double sm_calm_thresh = declare_parameter("state_machine.calm_v_yaw_thresh", 2.0);
  int sm_spin_enter = declare_parameter("state_machine.spin_enter_count", 5);
  int sm_spin_exit = declare_parameter("state_machine.spin_exit_count", 5);
  double sm_side_angle = declare_parameter("state_machine.side_angle", 15.0);
  double sm_prediction_delay = declare_parameter("state_machine.prediction_delay", 0.0);
  double sm_max_prediction = declare_parameter("state_machine.max_prediction_time", 0.5);

  // 初始化 TF2
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // 初始化组件
  initializeComponents();

  // 配置组件参数
  armor_selector_->setParameters(side_angle, min_switching_v_yaw);
  armor_selector_->setFacingParameters(facing_enter_angle, facing_exit_angle);
  fire_advisor_->setParameters(shooting_range_w, shooting_range_h);
  local_compensator_->setParameters(bullet_speed_, gravity, resistance, iteration_times);

  // 初始化策略
  initializeStrategies();

  // 配置策略参数
  auto predicted_strategy = std::dynamic_pointer_cast<PredictedPositionStrategy>(
    strategies_["predicted"]);
  if (predicted_strategy) {
    predicted_strategy->setPredictionParameters(prediction_delay, 0.5);
    predicted_strategy->setManualOffset(pitch_offset, yaw_offset);
    predicted_strategy->setTrackingCenterParams(max_tracking_v_yaw, transfer_thresh);
  }

  auto current_strategy = std::dynamic_pointer_cast<CurrentPositionStrategy>(
    strategies_["current"]);
  if (current_strategy) {
    current_strategy->setManualOffset(pitch_offset, yaw_offset);
  }

  // 配置状态机策略参数
  auto sm_strategy_ptr = std::dynamic_pointer_cast<StateMachineStrategy>(
    strategies_["state_machine"]);
  if (sm_strategy_ptr) {
    sm_strategy_ptr->setFacingParameters(sm_facing_enter, sm_facing_exit);
    sm_strategy_ptr->setSpinParameters(
      sm_spin_thresh, sm_calm_thresh, sm_spin_enter, sm_spin_exit, sm_side_angle);
    sm_strategy_ptr->setPredictionParameters(sm_prediction_delay, sm_max_prediction);
    sm_strategy_ptr->setManualOffset(pitch_offset, yaw_offset);
  }

  // 创建订阅者
  tracked_robots_sub_ = create_subscription<rm_interfaces::msg::TrackedRobots>(
    "tracked_robots",
    rclcpp::SensorDataQoS(),
    std::bind(&GimbalControllerNode::trackedRobotsCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to tracked_robots topic: %s", tracked_robots_sub_->get_topic_name());

  selected_target_sub_ = create_subscription<rm_interfaces::msg::SelectedTarget>(
    "selected_target",
    rclcpp::SensorDataQoS(),
    std::bind(&GimbalControllerNode::selectedTargetCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Subscribed to selected_target topic: %s", selected_target_sub_->get_topic_name());

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&GimbalControllerNode::jointStateCallback, this, std::placeholders::_1));

  // 创建发布者
  gimbal_cmd_pub_ = create_publisher<rm_interfaces::msg::GimbalCmd>(
    "cmd_gimbal",
    rclcpp::SensorDataQoS());

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/marker",
    10);

  // 创建服务
  set_mode_srv_ = create_service<rm_interfaces::srv::SetMode>(
    "~/set_mode",
    std::bind(&GimbalControllerNode::setModeCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // 创建定时器
  auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&GimbalControllerNode::timerCallback, this));

  // 初始化可视化
  if (debug_mode_) {
    initMarkers();
  }

  RCLCPP_INFO(get_logger(), "GimbalControllerNode initialized with strategy: %s",
              current_strategy_name_.c_str());
}

void GimbalControllerNode::initializeComponents()
{
  position_calculator_ = std::make_shared<ArmorPositionCalculator>();
  armor_selector_ = std::make_shared<ArmorSelector>();
  ballistic_client_ = std::make_shared<BallisticSolverClient>(this);
  local_compensator_ = std::make_shared<LocalTrajectoryCompensator>();
  fire_advisor_ = std::make_shared<FireAdvisor>();
}

void GimbalControllerNode::initializeStrategies()
{
  // 创建当前位置策略
  auto current_strategy = std::make_shared<CurrentPositionStrategy>();
  current_strategy->setComponents(
    position_calculator_, armor_selector_, ballistic_client_,
    local_compensator_, fire_advisor_);
  strategies_["current"] = current_strategy;

  // 创建预测位置策略
  auto predicted_strategy = std::make_shared<PredictedPositionStrategy>();
  predicted_strategy->setComponents(
    position_calculator_, armor_selector_, ballistic_client_,
    local_compensator_, fire_advisor_);
  strategies_["predicted"] = predicted_strategy;

  // 创建 MPC 策略 (预留)
  auto mpc_strategy = std::make_shared<MpcControlStrategy>();
  mpc_strategy->setComponents(
    position_calculator_, armor_selector_, ballistic_client_,
    local_compensator_, fire_advisor_);
  strategies_["mpc"] = mpc_strategy;

  // 创建状态机策略
  auto sm_strategy = std::make_shared<StateMachineStrategy>();
  sm_strategy->setComponents(
    position_calculator_, armor_selector_, ballistic_client_,
    local_compensator_, fire_advisor_);
  strategies_["state_machine"] = sm_strategy;
}

void GimbalControllerNode::trackedRobotsCallback(
  const rm_interfaces::msg::TrackedRobots::SharedPtr msg)
{
  latest_robots_ = msg;
  RCLCPP_DEBUG(get_logger(), "Received TrackedRobots: %zu robots", msg->robots.size());
  if (!msg->robots.empty()) {
    RCLCPP_DEBUG(get_logger(), "  First robot: id=%s, track_state=%d",
                msg->robots[0].robot_id.c_str(), msg->robots[0].track_state);
    // 添加INFO级别日志以便追踪
    static int callback_counter = 0;
    callback_counter++;
    if (callback_counter % 25 == 0) {  // 约每秒打印一次
      RCLCPP_INFO(get_logger(), "TrackedRobots #%d: %zu robots, first robot id=%s pos=(%.2f,%.2f,%.2f) stamp=%d.%09d",
                  callback_counter, msg->robots.size(),
                  msg->robots[0].robot_id.c_str(),
                  msg->robots[0].center_position.x,
                  msg->robots[0].center_position.y,
                  msg->robots[0].center_position.z,
                  msg->robots[0].header.stamp.sec,
                  msg->robots[0].header.stamp.nanosec);
    }
  }
}

void GimbalControllerNode::selectedTargetCallback(
  const rm_interfaces::msg::SelectedTarget::SharedPtr msg)
{
  selected_target_ = msg;
  static int callback_counter = 0;
  callback_counter++;
  if (callback_counter % 25 == 0) {  // 约每秒打印一次
    RCLCPP_INFO(get_logger(), "SelectedTarget #%d: robot_id=%s confidence=%.2f",
                callback_counter, msg->robot_id.c_str(), msg->confidence);
  }
  RCLCPP_DEBUG(get_logger(), "Received SelectedTarget: robot_id=%s", msg->robot_id.c_str());
}

void GimbalControllerNode::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 从 joint_states 中提取云台角度
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "yaw_joint") {
      current_yaw_ = msg->position[i];
    } else if (msg->name[i] == "pitch_joint") {
      current_pitch_ = msg->position[i];
    }
  }
}

void GimbalControllerNode::updateGimbalState()
{
  // 尝试从 TF2 获取云台姿态
  try {
    auto gimbal_tf = tf2_buffer_->lookupTransform(
      target_frame_, "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
    current_pitch_ = -pitch;  // 注意符号
  } catch (const tf2::TransformException &) {
    // 使用 joint_states 中的值
  }
}

void GimbalControllerNode::timerCallback()
{
  static int counter = 0;
  counter++;
  
  if (counter % 50 == 0) {  // 每秒打印一次 (250Hz / 50 = 5Hz)
    RCLCPP_INFO(get_logger(), "⏰ Timer tick #%d: enable=%d, robots=%s, target=%s",
                counter, enable_,
                (latest_robots_ && !latest_robots_->robots.empty()) ? "yes" : "no",
                selected_target_ ? "yes" : "no");
    
    // 显示 latest_robots_ 的时间戳和数量
    if (latest_robots_ && !latest_robots_->robots.empty()) {
      RCLCPP_INFO(get_logger(), "   📊 latest_robots_: %zu robots, stamp=%d.%09d",
                  latest_robots_->robots.size(),
                  latest_robots_->robots[0].header.stamp.sec,
                  latest_robots_->robots[0].header.stamp.nanosec);
    }
    
    // 显示 selected_target_ 信息
    if (selected_target_) {
      RCLCPP_INFO(get_logger(), "   🎯 selected_target_: robot_id=%s",
                  selected_target_->robot_id.c_str());
      
      // 在 latest_robots_ 中查找对应的机器人
      if (latest_robots_ && !latest_robots_->robots.empty()) {
        bool found = false;
        for (const auto & robot : latest_robots_->robots) {
          if (robot.robot_id == selected_target_->robot_id) {
            RCLCPP_INFO(get_logger(), "   ✓ Found in latest_robots_: id=%s, track_state=%d, pos=(%.2f,%.2f,%.2f) stamp=%d.%09d",
                        robot.robot_id.c_str(), robot.track_state,
                        robot.center_position.x, robot.center_position.y, robot.center_position.z,
                        robot.header.stamp.sec, robot.header.stamp.nanosec);
            found = true;
            break;
          }
        }
        if (!found) {
          RCLCPP_WARN(get_logger(), "   ✗ Selected target id=%s NOT FOUND in latest_robots_",
                      selected_target_->robot_id.c_str());
        }
      }
    }
  }
  
  if (!enable_) {
    rm_interfaces::msg::GimbalCmd idle_cmd;
    idle_cmd.yaw_diff = 0;
    idle_cmd.pitch_diff = 0;
    idle_cmd.distance = -1;
    idle_cmd.fire_advice = false;
    gimbal_cmd_pub_->publish(idle_cmd);
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Controller disabled, publishing idle cmd");
    return;
  }

  // 更新云台状态
  updateGimbalState();

  // 获取当前策略
  auto strategy = getStrategy(current_strategy_name_);
  if (!strategy) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Strategy '%s' not found", current_strategy_name_.c_str());
    return;
  }

  // 构建控制上下文
  GimbalControlContext context;
  context.current_yaw = current_yaw_;
  context.current_pitch = current_pitch_;
  context.bullet_speed = bullet_speed_;
  context.current_time = now();
  context.is_tracking = false;

  // 查找目标机器人
  if (latest_robots_ && !latest_robots_->robots.empty()) {
    // 如果有选择目标，查找对应机器人
    if (selected_target_) {
      for (const auto & robot : latest_robots_->robots) {
        if (robot.robot_id == selected_target_->robot_id) {
          context.target_robot = robot;
          context.target_stamp = rclcpp::Time(robot.header.stamp);
          context.is_tracking = (robot.track_state == rm_interfaces::msg::TrackedRobot::TRACKING ||
                                 robot.track_state == rm_interfaces::msg::TrackedRobot::TEMP_LOST);
          RCLCPP_DEBUG(get_logger(), "Found target robot: id=%s, is_tracking=%d, pos=(%.2f,%.2f,%.2f)",
                      robot.robot_id.c_str(), context.is_tracking,
                      robot.center_position.x, robot.center_position.y, robot.center_position.z);
          break;
        }
      }
    } else {
      // 否则使用第一个机器人
      context.target_robot = latest_robots_->robots[0];
      context.target_stamp = rclcpp::Time(context.target_robot.header.stamp);
      context.is_tracking = (context.target_robot.track_state ==
                             rm_interfaces::msg::TrackedRobot::TRACKING ||
                             context.target_robot.track_state ==
                             rm_interfaces::msg::TrackedRobot::TEMP_LOST);
      RCLCPP_DEBUG(get_logger(), "Using first robot: id=%s, is_tracking=%d",
                  context.target_robot.robot_id.c_str(), context.is_tracking);
    }
  } else {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "No robots available");
  }

  // 执行策略
  RCLCPP_DEBUG(get_logger(), "Executing strategy '%s' with is_tracking=%d",
              current_strategy_name_.c_str(), context.is_tracking);
  auto cmd = strategy->solve(context);

  // 记录关键命令值
  RCLCPP_DEBUG(get_logger(), "Strategy output: yaw=%.2f, pitch=%.2f, yaw_diff=%.2f, pitch_diff=%.2f, dist=%.2f, fire=%d",
              cmd.yaw, cmd.pitch, cmd.yaw_diff, cmd.pitch_diff, cmd.distance, cmd.fire_advice);
  
  if (counter % 50 == 0) {
    RCLCPP_INFO(get_logger(), "Publishing cmd: yaw_diff=%.3f, pitch_diff=%.3f, dist=%.2f, fire=%d",
               cmd.yaw_diff, cmd.pitch_diff, cmd.distance, cmd.fire_advice);
  }

  // 发布控制命令
  gimbal_cmd_pub_->publish(cmd);

  // 发布可视化
  if (debug_mode_ && context.is_tracking) {
    publishMarkers(context.target_robot, cmd);
  }
}

void GimbalControllerNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
{
  response->success = true;

  // 根据模式设置 enable_
  // 模式定义参考原 armor_solver
  int mode = request->mode;
  if (mode == 1 || mode == 2) {  // AUTO_AIM_RED or AUTO_AIM_BLUE
    enable_ = true;
    RCLCPP_INFO(get_logger(), "GimbalController enabled");
  } else {
    enable_ = false;
    RCLCPP_INFO(get_logger(), "GimbalController disabled");
  }
}

GimbalControlStrategy::SharedPtr GimbalControllerNode::getStrategy(
  const std::string & name) const
{
  auto it = strategies_.find(name);
  if (it != strategies_.end()) {
    return it->second;
  }
  return nullptr;
}

void GimbalControllerNode::initMarkers()
{
  // 目标位置标记
  position_marker_.ns = "target_position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.15;
  position_marker_.color.a = 1.0;
  position_marker_.color.r = 1.0;
  position_marker_.color.g = 0.0;
  position_marker_.color.b = 0.0;

  // 目标速度箭头
  target_velocity_marker_.type = visualization_msgs::msg::Marker::ARROW;
  target_velocity_marker_.ns = "target_velocity";
  target_velocity_marker_.scale.x = 0.03;
  target_velocity_marker_.scale.y = 0.05;
  target_velocity_marker_.color.a = 1.0;
  target_velocity_marker_.color.r = 0.0;
  target_velocity_marker_.color.g = 1.0;
  target_velocity_marker_.color.b = 1.0;

  // 装甲板标记
  armors_marker_.ns = "armors";
  armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armors_marker_.scale.x = 0.03;
  armors_marker_.scale.y = 0.23;
  armors_marker_.scale.z = 0.125;
  armors_marker_.color.a = 0.7;
  armors_marker_.color.r = 0.0;
  armors_marker_.color.g = 0.5;
  armors_marker_.color.b = 1.0;

  // 选中目标标记
  selection_marker_.ns = "selection";
  selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.12;
  selection_marker_.color.a = 1.0;
  selection_marker_.color.r = 1.0;
  selection_marker_.color.g = 1.0;
  selection_marker_.color.b = 0.0;

  // 预测打击点标记
  predicted_marker_.ns = "predicted_hit";
  predicted_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  predicted_marker_.scale.x = predicted_marker_.scale.y = predicted_marker_.scale.z = 0.1;
  predicted_marker_.color.a = 1.0;
  predicted_marker_.color.r = 0.0;
  predicted_marker_.color.g = 1.0;
  predicted_marker_.color.b = 0.0;

  // 弹道轨迹标记
  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  trajectory_marker_.scale.x = 0.02;
  trajectory_marker_.color.a = 0.8;
  trajectory_marker_.color.r = 1.0;
  trajectory_marker_.color.g = 0.75;
  trajectory_marker_.color.b = 0.79;

  // 初始化颜色调色板
  color_palette_.clear();
  for (int i = 0; i < 10; ++i) {
    float hue = i * 36.0;
    auto c = hsvToRgb(hue, 1.0, 1.0);
    color_palette_.push_back(c);
  }

  RCLCPP_INFO(get_logger(), "Markers initialized");
}

void GimbalControllerNode::publishMarkers(
  const rm_interfaces::msg::TrackedRobot & target_robot,
  const rm_interfaces::msg::GimbalCmd & cmd)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // 目标中心位置
  position_marker_.header = target_robot.header;
  position_marker_.id = 0;
  position_marker_.action = visualization_msgs::msg::Marker::ADD;
  position_marker_.pose.position = target_robot.center_position;
  position_marker_.pose.orientation.w = 1.0;
  marker_array.markers.push_back(position_marker_);

  // 目标速度箭头
  target_velocity_marker_.header = target_robot.header;
  target_velocity_marker_.id = 0;
  target_velocity_marker_.action = visualization_msgs::msg::Marker::ADD;
  target_velocity_marker_.points.clear();
  
  geometry_msgs::msg::Point vel_start = target_robot.center_position;
  geometry_msgs::msg::Point vel_end = target_robot.center_position;
  vel_end.x += target_robot.center_velocity.x * 0.5;
  vel_end.y += target_robot.center_velocity.y * 0.5;
  vel_end.z += target_robot.center_velocity.z * 0.5;
  
  target_velocity_marker_.points.push_back(vel_start);
  target_velocity_marker_.points.push_back(vel_end);
  marker_array.markers.push_back(target_velocity_marker_);

  // 装甲板位置（根据 armors_offset）
  if (!target_robot.armors_offset.empty()) {
    for (size_t i = 0; i < target_robot.armors_offset.size(); ++i) {
      visualization_msgs::msg::Marker armor_marker = armors_marker_;
      armor_marker.header = target_robot.header;
      armor_marker.id = static_cast<int>(i);
      armor_marker.action = visualization_msgs::msg::Marker::ADD;

      // 计算装甲板世界坐标
      double cos_yaw = std::cos(target_robot.yaw);
      double sin_yaw = std::sin(target_robot.yaw);
      
      const auto & offset = target_robot.armors_offset[i];
      armor_marker.pose.position.x = target_robot.center_position.x + 
                                      offset.position.x * cos_yaw - offset.position.y * sin_yaw;
      armor_marker.pose.position.y = target_robot.center_position.y + 
                                      offset.position.x * sin_yaw + offset.position.y * cos_yaw;
      armor_marker.pose.position.z = target_robot.center_position.z + offset.position.z;

      // 装甲板朝向
      tf2::Quaternion q;
      q.setRPY(0, 0.2618, target_robot.yaw + i * (2 * M_PI / target_robot.num_armors));
      armor_marker.pose.orientation.x = q.x();
      armor_marker.pose.orientation.y = q.y();
      armor_marker.pose.orientation.z = q.z();
      armor_marker.pose.orientation.w = q.w();

      marker_array.markers.push_back(armor_marker);
    }
  }

  // 选中的打击目标
  if (cmd.distance > 0) {
    selection_marker_.header = target_robot.header;
    selection_marker_.id = 0;
    selection_marker_.action = visualization_msgs::msg::Marker::ADD;
    
    double yaw_rad = cmd.yaw * M_PI / 180.0;
    double pitch_rad = cmd.pitch * M_PI / 180.0;
    
    selection_marker_.pose.position.x = cmd.distance * std::cos(pitch_rad) * std::cos(yaw_rad);
    selection_marker_.pose.position.y = cmd.distance * std::cos(pitch_rad) * std::sin(yaw_rad);
    selection_marker_.pose.position.z = cmd.distance * std::sin(pitch_rad);
    selection_marker_.pose.orientation.w = 1.0;
    
    marker_array.markers.push_back(selection_marker_);
  }

  // 预测打击点（使用 predicted 策略时）
  if (current_strategy_name_ == "predicted" && cmd.distance > 0) {
    predicted_marker_.header = target_robot.header;
    predicted_marker_.id = 0;
    predicted_marker_.action = visualization_msgs::msg::Marker::ADD;
    predicted_marker_.pose = selection_marker_.pose;
    predicted_marker_.color.a = 0.6;
    marker_array.markers.push_back(predicted_marker_);
  }

  // 弹道轨迹（从云台到目标）
  if (cmd.distance > 0) {
    trajectory_marker_.header.frame_id = "gimbal_link";
    trajectory_marker_.header.stamp = target_robot.header.stamp;
    trajectory_marker_.id = 0;
    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();

    // 生成简化的弹道轨迹点
    int num_points = 20;
    double yaw_rad = cmd.yaw * M_PI / 180.0;
    double pitch_rad = cmd.pitch * M_PI / 180.0;
    
    for (int i = 0; i <= num_points; ++i) {
      double t = static_cast<double>(i) / num_points;
      double distance = cmd.distance * t;
      
      geometry_msgs::msg::Point p;
      p.x = distance * std::cos(pitch_rad) * std::cos(yaw_rad);
      p.y = distance * std::cos(pitch_rad) * std::sin(yaw_rad);
      
      // 简化的抛物线下落
      double flight_time = cmd.distance / bullet_speed_ * t;
      p.z = distance * std::sin(pitch_rad) - 0.5 * 9.8 * flight_time * flight_time;
      
      trajectory_marker_.points.push_back(p);
    }

    // 根据开火建议改变颜色
    if (cmd.fire_advice) {
      trajectory_marker_.color.r = 0.0;
      trajectory_marker_.color.g = 1.0;
      trajectory_marker_.color.b = 0.0;
    } else {
      trajectory_marker_.color.r = 1.0;
      trajectory_marker_.color.g = 0.75;
      trajectory_marker_.color.b = 0.79;
    }

    marker_array.markers.push_back(trajectory_marker_);
  }

  marker_pub_->publish(marker_array);
}

std::array<float, 4> GimbalControllerNode::hsvToRgb(float h, float s, float v)
{
  float c = v * s;
  float x = c * (1 - std::abs(std::fmod(h / 60.0, 2) - 1));
  float m = v - c;

  float r, g, b;
  if (h < 60) {
    r = c; g = x; b = 0;
  } else if (h < 120) {
    r = x; g = c; b = 0;
  } else if (h < 180) {
    r = 0; g = c; b = x;
  } else if (h < 240) {
    r = 0; g = x; b = c;
  } else if (h < 300) {
    r = x; g = 0; b = c;
  } else {
    r = c; g = 0; b = x;
  }

  return {r + m, g + m, b + m, 1.0f};
}

}  // namespace gimbal_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gimbal_controller::GimbalControllerNode)
