// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Modified by Amatrix (HFUT RM SKT GROUP)
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

#ifndef BALLISTIC_SOLVER__BALLISTIC_SOLVER_NODE_HPP_
#define BALLISTIC_SOLVER__BALLISTIC_SOLVER_NODE_HPP_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "ballistic_solver/ballistic_solver.hpp"
#include "rm_interfaces/srv/solve_ballistic.hpp"

namespace ballistic_solver {

/**
 * @brief 弹道解算服务节点
 * 
 * 提供 ROS2 服务接口，用于计算击中目标所需的 pitch 和 yaw
 */
class BallisticSolverNode : public rclcpp::Node {
public:
  /**
   * @brief 构造函数
   * @param options 节点配置选项
   */
  explicit BallisticSolverNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  /**
   * @brief 析构函数
   */
  ~BallisticSolverNode() override = default;

private:
  /**
   * @brief 声明节点参数
   */
  void declareParameters();

  /**
   * @brief 从参数加载配置
   * @return 弹道配置
   */
  BallisticConfig loadConfigFromParameters();

  /**
   * @brief 参数回调函数
   * @param parameters 更新的参数列表
   * @return 设置结果
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters);

  /**
   * @brief 弹道解算服务回调
   * @param request 服务请求
   * @param response 服务响应
   */
  void handleSolveRequest(const rm_interfaces::srv::SolveBallistic::Request::SharedPtr request,
                          rm_interfaces::srv::SolveBallistic::Response::SharedPtr response);

  // 弹道解算器
  std::unique_ptr<BallisticSolver> solver_;

  // 服务
  rclcpp::Service<rm_interfaces::srv::SolveBallistic>::SharedPtr solve_service_;

  // 参数回调句柄
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace ballistic_solver

#endif  // BALLISTIC_SOLVER__BALLISTIC_SOLVER_NODE_HPP_
