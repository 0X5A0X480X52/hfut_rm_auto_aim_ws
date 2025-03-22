// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
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

#ifndef ARMOR_SOLVER_SOLVER_HPP_
#define ARMOR_SOLVER_SOLVER_HPP_

// std
#include <memory>
// ros2
#include <angles/angles.h>
#include <tf2_ros/buffer.h>

#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 3rd party
#include <Eigen/Dense>
// project
#include "armor_solver/armorFliter.hpp"
#include "models/models.h"
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/manual_compensator.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"

namespace fyt::auto_aim {
// Solver class used to solve the gimbal command from tracked target
class Solver {
public:
  using Armors = rm_interfaces::msg::Armors;
  using Armor = rm_interfaces::msg::Armor;

  explicit Solver(std::weak_ptr<rclcpp::Node> node);
  // explicit Solver(std::string trajectory_compensator_type, float max_tracking_v_yaw);
  ~Solver() = default;

  // Solve the gimbal command from tracked target
  // Throw: tf2::TransformException if the transform from "odom" to "gimbal_link" is not available
  rm_interfaces::msg::GimbalCmd solve(const rm_interfaces::msg::Target &target_msg,
                                      const rclcpp::Time &current_time,
                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

  // Solve the gimbal command from tracked target
  // 使用 ArmorFliter 进行了修正
  // Throw: tf2::TransformException if the transform from "odom" to "gimbal_link" is not available
  rm_interfaces::msg::GimbalCmd solve_withArmorFliter(const rm_interfaces::msg::Target &target,
                                                      const rclcpp::Time &current_time,
                                                      std::shared_ptr<tf2_ros::Buffer> tf2_buffer_);

  enum State { TRACKING_ARMOR = 0, TRACKING_CENTER = 1 } state;

  std::vector<std::pair<double, double>> getTrajectory() const noexcept;

private:
  // Get the armor positions from the target robot
  std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &target_center,
                                                 const double yaw,
                                                 const double r1,
                                                 const double r2,
                                                 const double d_zc,
                                                 const double d_za,
                                                 const std::size_t armors_num) const noexcept;

  // Get the armor positions from the target robot
  // 使用 ArmorFliter 进行了修正
  std::vector<Eigen::Vector3d> getArmorPositions_withArmorFliter(
    const Eigen::Vector3d &target_center,
    const double yaw,
    const double r1,
    const double r2,
    const double d_zc,
    const double d_za,
    const std::string id,
    const std::size_t armors_num) const noexcept;

public:
  std::shared_ptr<ArmorFliter> armorFliter = std::make_shared<ArmorFliter>();

private:
  // Select the best armor to shoot
  // Return: selected idx in {0, 1, ..., armors_num - 1}
  int selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                      const Eigen::Vector3d &target_center,
                      const double target_yaw,
                      const double target_v_yaw,
                      const std::size_t armors_num) const noexcept;
  // 当前装甲板的位置预测迭代次数
  int armor_current_positions_predicted_iter = 1;
  /* 基于云台角度和目标位置计算移动最小的角度 */
  // 预测装甲板位置的迭代次数
  std::vector<int> armor_predicted_iter_list = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  // std::vector<int> armor_predicted_iter_list = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 , 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};
  double diff_threshold_to_use_minDist = 1;  // 差异小于该阈值，使用最近距离轩板
  // 基于当前位置及预测位置的云台控制偏差均大于该值时，使用原始的默认选板方式
  double selectBestArmor_useDefault_threshold = 40;
  // 偏差小于该阈值，使用基于当前位置的选板方案
  double selectBestArmor_useCurrent_threshold = 20;

  /* 基于云台角度和目标位置计算移动最小的角度 */
  // Return: std::pair<Eigen::Vector3d, Eigen::Vector3d>，前者为选中板的当前位置，后者为选中板的预测位置
  std::pair<Eigen::Vector3d, Eigen::Vector3d> selectBestArmor_withArmorFliter(
    const rm_interfaces::msg::Target &target,
    const std::vector<Eigen::Vector3d> &armor_positions,
    const Eigen::Vector3d &target_center,
    const double target_yaw,
    const double target_v_yaw,
    const std::size_t armors_num) const noexcept;

  void calcYawAndPitch(const Eigen::Vector3d &p,
                       const std::array<double, 3> rpy,
                       double &yaw,
                       double &pitch) const noexcept;

  bool isOnTarget(const double cur_yaw,
                  const double cur_pitch,
                  const double target_yaw,
                  const double target_pitch,
                  const double distance) const noexcept;

  std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
  std::unique_ptr<ManualCompensator> manual_compensator_;

  std::array<double, 3> rpy_;

  double prediction_delay_;
  double controller_delay_;

  double shooting_range_w_;
  double shooting_range_h_;

  double max_tracking_v_yaw_;
  int overflow_count_;
  int transfer_thresh_;

  double side_angle_;
  double min_switching_v_yaw_;

  std::weak_ptr<rclcpp::Node> node_;
};
}  // namespace fyt::auto_aim
#endif  // ARMOR_SOLVER_SOLVER_HPP_
