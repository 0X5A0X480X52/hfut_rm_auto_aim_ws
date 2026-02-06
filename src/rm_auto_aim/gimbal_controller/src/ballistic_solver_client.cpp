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

#include "gimbal_controller/ballistic_solver_client.hpp"
#include <algorithm>

namespace gimbal_controller
{

BallisticSolverClient::BallisticSolverClient(rclcpp::Node * node)
: node_(node)
{
  if (node_ != nullptr) {
    client_ = node_->create_client<SolveBallistic>(service_name_);
  }
}

BallisticResult BallisticSolverClient::solve(
  const Eigen::Vector3d & target_position,
  const Eigen::Vector3d & target_velocity,
  double bullet_speed,
  std::chrono::milliseconds timeout)
{
  BallisticResult result;
  result.success = false;
  result.message = "Service not available";

  // 检查服务是否可用（短超时）
  if (!client_->wait_for_service(std::chrono::milliseconds(5))) {
    if (node_) {
      RCLCPP_DEBUG(node_->get_logger(), "Ballistic solver service not available");
    }
    return result;
  }

  auto request = std::make_shared<SolveBallistic::Request>();
  request->target_position.x = target_position.x();
  request->target_position.y = target_position.y();
  request->target_position.z = target_position.z();
  request->target_velocity.x = target_velocity.x();
  request->target_velocity.y = target_velocity.y();
  request->target_velocity.z = target_velocity.z();
  request->bullet_speed = bullet_speed;

  if (node_) {
    RCLCPP_DEBUG(node_->get_logger(), "Calling ballistic solver: pos=(%.2f,%.2f,%.2f), vel=(%.2f,%.2f,%.2f)",
                target_position.x(), target_position.y(), target_position.z(),
                target_velocity.x(), target_velocity.y(), target_velocity.z());
  }

  auto future = client_->async_send_request(request);

  // 缩短超时时间以避免阻塞 timer 回调（从默认 100ms 改为 10ms）
  auto wait_timeout = std::min(timeout, std::chrono::milliseconds(10));
  if (future.wait_for(wait_timeout) == std::future_status::ready) {
    auto response = future.get();
    result.pitch = response->pitch;
    result.yaw = response->yaw;
    result.flight_time = response->flight_time;
    result.success = response->success;
    result.message = response->message;
    
    if (node_) {
      RCLCPP_DEBUG(node_->get_logger(), "Ballistic solver result: success=%d, pitch=%.3f, yaw=%.3f",
                  result.success, result.pitch, result.yaw);
    }
  } else {
    result.message = "Service call timeout";
    if (node_) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                          "Ballistic solver timeout after %ld ms", wait_timeout.count());
    }
  }

  return result;
}

bool BallisticSolverClient::isServiceAvailable() const
{
  return client_->wait_for_service(std::chrono::milliseconds(1));
}

void BallisticSolverClient::setServiceName(const std::string & service_name)
{
  service_name_ = service_name;
  if (node_ != nullptr) {
    client_ = node_->create_client<SolveBallistic>(service_name_);
  }
}

}  // namespace gimbal_controller
