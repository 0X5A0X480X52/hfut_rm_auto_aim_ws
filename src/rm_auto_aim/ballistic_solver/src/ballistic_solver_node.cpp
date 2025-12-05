// Copyright (C) FYT Vision Group. All rights reserved.
// Created by Amatrix
// Maintained by HFUT RM SKT GROUP
// Copyright (C) HFUT RM SKT GROUP. All rights reserved.
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

#include "ballistic_solver/ballistic_solver_node.hpp"

#include <functional>

namespace ballistic_solver {

BallisticSolverNode::BallisticSolverNode(const rclcpp::NodeOptions &options)
: Node("ballistic_solver", options) {
  RCLCPP_INFO(get_logger(), "Initializing Ballistic Solver Node...");

  // 声明参数
  declareParameters();

  // 从参数加载配置
  BallisticConfig config = loadConfigFromParameters();

  // 创建弹道解算器
  solver_ = std::make_unique<BallisticSolver>(config);

  // 创建服务
  solve_service_ = create_service<rm_interfaces::srv::SolveBallistic>(
    "~/solve",
    std::bind(&BallisticSolverNode::handleSolveRequest,
              this,
              std::placeholders::_1,
              std::placeholders::_2));

  // 注册参数回调
  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&BallisticSolverNode::parametersCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Ballistic Solver Node initialized successfully");
  RCLCPP_INFO(get_logger(), "Service available at: %s/solve", get_namespace());
  RCLCPP_INFO(get_logger(), "Bullet speed: %.2f m/s", config.bullet_speed);
  RCLCPP_INFO(get_logger(), "Gravity: %.2f m/s^2", config.gravity);
  RCLCPP_INFO(get_logger(), "Air resistance: %.6f", config.air_resistance);
  RCLCPP_INFO(get_logger(), "Compensator type: %s", config.compensator_type.c_str());
}

void BallisticSolverNode::declareParameters() {
  // 弹道参数
  declare_parameter("bullet_speed", 28.0);
  declare_parameter("gravity", 9.8);
  declare_parameter("air_resistance", 0.001);
  declare_parameter("max_iterations", 50);
  declare_parameter("convergence_threshold", 0.001);
  declare_parameter("compensator_type", "resistance");
}

BallisticConfig BallisticSolverNode::loadConfigFromParameters() {
  BallisticConfig config;

  config.bullet_speed = get_parameter("bullet_speed").as_double();
  config.gravity = get_parameter("gravity").as_double();
  config.air_resistance = get_parameter("air_resistance").as_double();
  config.max_iterations = get_parameter("max_iterations").as_int();
  config.convergence_threshold = get_parameter("convergence_threshold").as_double();
  config.compensator_type = get_parameter("compensator_type").as_string();

  return config;
}

rcl_interfaces::msg::SetParametersResult BallisticSolverNode::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  bool need_update = false;
  BallisticConfig config = solver_->getConfig();

  for (const auto &param : parameters) {
    if (param.get_name() == "bullet_speed") {
      config.bullet_speed = param.as_double();
      need_update = true;
      RCLCPP_INFO(get_logger(), "Updated bullet_speed to: %.2f m/s", config.bullet_speed);
    } else if (param.get_name() == "gravity") {
      config.gravity = param.as_double();
      need_update = true;
      RCLCPP_INFO(get_logger(), "Updated gravity to: %.2f m/s^2", config.gravity);
    } else if (param.get_name() == "air_resistance") {
      config.air_resistance = param.as_double();
      need_update = true;
      RCLCPP_INFO(get_logger(), "Updated air_resistance to: %.6f", config.air_resistance);
    } else if (param.get_name() == "max_iterations") {
      config.max_iterations = param.as_int();
      need_update = true;
      RCLCPP_INFO(get_logger(), "Updated max_iterations to: %d", config.max_iterations);
    } else if (param.get_name() == "convergence_threshold") {
      config.convergence_threshold = param.as_double();
      need_update = true;
      RCLCPP_INFO(
        get_logger(), "Updated convergence_threshold to: %.6f", config.convergence_threshold);
    } else if (param.get_name() == "compensator_type") {
      std::string type = param.as_string();
      if (type != "ideal" && type != "resistance") {
        result.successful = false;
        result.reason = "Invalid compensator_type. Must be 'ideal' or 'resistance'";
        return result;
      }
      config.compensator_type = type;
      need_update = true;
      RCLCPP_INFO(get_logger(), "Updated compensator_type to: %s", config.compensator_type.c_str());
    }
  }

  if (need_update) {
    solver_->updateConfig(config);
  }

  return result;
}

void BallisticSolverNode::handleSolveRequest(
  const rm_interfaces::srv::SolveBallistic::Request::SharedPtr request,
  rm_interfaces::srv::SolveBallistic::Response::SharedPtr response) {
  // 提取目标位置
  Eigen::Vector3d target_position(
    request->target_position.x, request->target_position.y, request->target_position.z);

  // 提取目标速度
  Eigen::Vector3d target_velocity(
    request->target_velocity.x, request->target_velocity.y, request->target_velocity.z);

  // 获取子弹速度 (如果请求中指定了，使用请求中的值)
  double bullet_speed =
    request->bullet_speed > 0 ? request->bullet_speed : solver_->getBulletSpeed();

  RCLCPP_DEBUG(get_logger(),
               "Solving ballistic for target at (%.3f, %.3f, %.3f) with velocity (%.3f, %.3f, "
               "%.3f), bullet speed: %.2f",
               target_position.x(),
               target_position.y(),
               target_position.z(),
               target_velocity.x(),
               target_velocity.y(),
               target_velocity.z(),
               bullet_speed);

  // 根据是否有速度选择不同的求解方法
  BallisticResult result;
  if (target_velocity.norm() < 1e-6) {
    // 静态目标
    result = solver_->solve(target_position);
  } else {
    // 移动目标
    result = solver_->solveMovingTarget(target_position, target_velocity, bullet_speed);
  }

  // 填充响应
  response->pitch = result.pitch;
  response->yaw = result.yaw;
  response->flight_time = result.flight_time;
  response->success = result.success;
  response->message = result.message;

  if (result.success) {
    RCLCPP_DEBUG(
      get_logger(),
      "Solution found: pitch=%.4f rad (%.2f deg), yaw=%.4f rad (%.2f deg), flight_time=%.4f s",
      result.pitch,
      result.pitch * 180.0 / M_PI,
      result.yaw,
      result.yaw * 180.0 / M_PI,
      result.flight_time);
  } else {
    RCLCPP_WARN(get_logger(), "Failed to solve ballistic: %s", result.message.c_str());
  }
}

}  // namespace ballistic_solver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ballistic_solver::BallisticSolverNode)
