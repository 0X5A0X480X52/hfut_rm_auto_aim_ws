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

#ifndef GIMBAL_PIPELINE_CONTROL_CONTEXT_HPP_
#define GIMBAL_PIPELINE_CONTROL_CONTEXT_HPP_

// project
#include "rm_interfaces/msg/target.hpp"

namespace fyt::auto_aim {

// Context passed through the gimbal control pipeline at every control cycle.
//
// Coordinate conventions (all angles in radians unless noted otherwise):
//
//   current_yaw
//     The current yaw angle of the gimbal in the world (odom) frame.
//     Obtained from the TF transform of "gimbal_link" relative to the target
//     frame via:
//         tf2::Matrix3x3(tf_q).getRPY(rpy[0], rpy[1], rpy[2]);
//         current_yaw = rpy[2];   // positive = counter-clockwise from X-axis
//
//   current_pitch
//     The current pitch angle of the gimbal in the world (odom) frame.
//     Because the robot's pitch axis points downward when the URDF pitch joint
//     is at zero, the raw RPY pitch is **negated** before storing:
//         current_pitch = -rpy[1];  // positive = barrel pointing upward
//
//   target_robot
//     The latest EKF state message for the tracked robot target.
//     Its header carries the time-stamp and coordinate frame used for
//     subsequent angle calculations.
//
//   is_temp_lost
//     True when the tracker is in TEMP_LOST state.  In this state no new
//     detector observation has been received for a short interval, so the
//     distance field of the outgoing GimbalCmd is set to -1 to signal that
//     no valid range measurement is available.
struct GimbalControlContext {
  // Tracked target (EKF state); header.frame_id is the world frame name
  rm_interfaces::msg::Target target_robot;

  // True while the tracker is temporarily lost (no detector observations)
  bool is_temp_lost = false;

  // Current gimbal yaw   [rad], world frame, from TF: rpy[2]
  double current_yaw = 0.0;

  // Current gimbal pitch [rad], world frame, from TF: -rpy[1]
  // Positive value means the barrel is pointing upward.
  double current_pitch = 0.0;
};

}  // namespace fyt::auto_aim

#endif  // GIMBAL_PIPELINE_CONTROL_CONTEXT_HPP_
