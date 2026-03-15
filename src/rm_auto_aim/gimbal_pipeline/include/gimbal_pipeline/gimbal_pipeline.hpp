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

#ifndef GIMBAL_PIPELINE_HPP_
#define GIMBAL_PIPELINE_HPP_

// std
#include <array>
#include <memory>
#include <vector>
// ros2
#include <angles/angles.h>
#include <tf2_ros/buffer.h>

#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// 3rd party
#include <Eigen/Dense>
// project
#include "gimbal_pipeline/gimbal_control_context.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/manual_compensator.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"

namespace fyt::auto_aim {

// GimbalPipeline
// ==============
// Turns a GimbalControlContext (current gimbal pose + tracked-target state)
// into a GimbalCmd that is ready to be sent to the serial driver.
//
// Angle definitions
// -----------------
// All internal calculations use radians.  The output GimbalCmd fields are in
// degrees so that the serial protocol layer receives values in the same unit
// as the existing armor_solver.
//
//   cmd.yaw   [deg] – absolute target yaw  in the world frame
//                     = atan2(p_target.y, p_target.x) + yaw_offset
//   cmd.pitch [deg] – absolute target pitch in the world frame
//                     = atan2(p_target.z, |p_target.xy|) + trajectory_comp
//                       + pitch_offset
//   cmd.yaw_diff   [deg] – cmd.yaw   - context.current_yaw   (×180/π)
//   cmd.pitch_diff [deg] – cmd.pitch - context.current_pitch (×180/π)
//
// The serial driver physically moves the gimbal by yaw_diff / pitch_diff; the
// absolute yaw / pitch fields are provided for debugging and visualization.
class GimbalPipeline {
public:
  explicit GimbalPipeline(std::weak_ptr<rclcpp::Node> node);
  ~GimbalPipeline() = default;

  // Build a GimbalControlContext by reading the current TF transform of
  // "gimbal_link" into the target frame stored in target.header.frame_id.
  //
  // Throws tf2::TransformException when the transform is unavailable.
  GimbalControlContext buildContext(const rm_interfaces::msg::Target &target,
                                   bool is_temp_lost,
                                   std::shared_ptr<tf2_ros::Buffer> tf2_buffer);

  // Run the full pipeline: select the best armor, compute aim angles, apply
  // compensations and return a populated GimbalCmd.
  //
  // The distance field is set to -1 when context.is_temp_lost is true because
  // no detector observation is available in that state.
  //
  // Throws std::runtime_error when no valid armor position can be computed.
  rm_interfaces::msg::GimbalCmd solve(const GimbalControlContext &context,
                                      const rclcpp::Time &current_time);

private:
  // Compute the absolute yaw and pitch angles [rad] that point the gimbal at
  // position p (expressed in the world frame relative to the gimbal origin).
  // Trajectory compensation is applied to the pitch axis.
  void calcYawAndPitch(const Eigen::Vector3d &p,
                       double &yaw,
                       double &pitch) const noexcept;

  // Return true when the gimbal is already within the shooting window.
  bool isOnTarget(double cur_yaw,
                  double cur_pitch,
                  double target_yaw,
                  double target_pitch,
                  double distance) const noexcept;

  // Generate all armor plate positions for a spinning robot.
  std::vector<Eigen::Vector3d> getArmorPositions(const Eigen::Vector3d &center,
                                                 double yaw,
                                                 double r1,
                                                 double r2,
                                                 double d_zc,
                                                 double d_za,
                                                 std::size_t armors_num) const noexcept;

  // Select the index of the armor that requires the least gimbal movement.
  int selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                      const Eigen::Vector3d &target_center,
                      double target_yaw,
                      double target_v_yaw,
                      std::size_t armors_num) const noexcept;

  std::unique_ptr<TrajectoryCompensator> trajectory_compensator_;
  std::unique_ptr<ManualCompensator> manual_compensator_;

  double shooting_range_w_;
  double shooting_range_h_;
  double prediction_delay_;
  double controller_delay_;
  double max_tracking_v_yaw_;
  double side_angle_;
  double min_switching_v_yaw_;

  std::weak_ptr<rclcpp::Node> node_;
};

}  // namespace fyt::auto_aim

#endif  // GIMBAL_PIPELINE_HPP_
