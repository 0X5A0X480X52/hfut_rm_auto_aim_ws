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

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fyt::auto_aim::robot_description
{

std::vector<Eigen::Vector3d> TrackedRobotUsage::resolveOffsets(
  const rm_interfaces::msg::TrackedRobot & robot,
  const OffsetFallbackGenerator & fallback_generator)
{
  std::vector<Eigen::Vector3d> offsets;

  if (!robot.armors_offset.empty()) {
    offsets.reserve(robot.armors_offset.size());
    for (const auto & pose : robot.armors_offset) {
      offsets.emplace_back(pose.position.x, pose.position.y, pose.position.z);
    }
    return offsets;
  }

  if (fallback_generator) {
    return fallback_generator(robot);
  }

  return offsets;
}

std::vector<geometry_msgs::msg::Pose> TrackedRobotUsage::generateArmorsOffsetFromProfile(
  int num_armors,
  double r1,
  double r2,
  double d_za,
  double d_zc)
{
  std::vector<geometry_msgs::msg::Pose> offsets;
  offsets.reserve(static_cast<size_t>(std::max(0, num_armors)));

  bool is_current_pair = true;
  for (int i = 0; i < num_armors; ++i) {
    const double angle = i * (2.0 * M_PI / num_armors);
    double r = r1;
    double dz = d_zc;

    if (num_armors == 4) {
      r = is_current_pair ? r1 : r2;
      dz = d_zc + (is_current_pair ? -d_za : d_za);
      is_current_pair = !is_current_pair;
    }

    geometry_msgs::msg::Pose pose;
    pose.position.x = -r * std::cos(angle);
    pose.position.y = -r * std::sin(angle);
    pose.position.z = dz;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, angle + M_PI);
    pose.orientation = tf2::toMsg(q);

    offsets.push_back(pose);
  }

  return offsets;
}

}  // namespace fyt::auto_aim::robot_description