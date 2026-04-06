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

#include "gimbal_pipeline/common/robot_description/Strategy/fixed_profile_tracked_robot_builder.hpp"

#include <algorithm>
#include <utility>

#include "max_entropy_tracker/trackers/adaptive_armor_tracker.hpp"
#include "max_entropy_tracker/utils/output_smoother.hpp"

namespace fyt::auto_aim::robot_description
{
namespace
{

class FixedProfileTrackedRobotBuilder final : public ITrackedRobotBuilderStrategy
{
public:
  FixedProfileTrackedRobotBuilder(
    std::string strategy_name,
    uint8_t robot_type,
    int num_armors)
  : strategy_name_(std::move(strategy_name)),
    robot_type_(robot_type),
    num_armors_(num_armors)
  {
  }

  std::string strategyName() const override
  {
    return strategy_name_;
  }

  uint8_t robotType() const override
  {
    return robot_type_;
  }

  int numArmors() const override
  {
    return num_armors_;
  }

  rm_interfaces::msg::TrackedRobot buildTrackedRobot(
    const TrackedRobotBuildInput & input) const override
  {
    rm_interfaces::msg::TrackedRobot msg;
    msg.header = input.header;
    msg.header.frame_id = input.target_frame;
    msg.robot_id = input.robot_id;
    msg.robot_type = robot_type_;

    if (input.tracker.is_tracking()) {
      msg.track_state = rm_interfaces::msg::TrackedRobot::TRACKING;
    } else if (input.tracker.is_temp_lost()) {
      msg.track_state = rm_interfaces::msg::TrackedRobot::TEMP_LOST;
    } else {
      msg.track_state = rm_interfaces::msg::TrackedRobot::DETECTING;
    }

    auto idx = input.tracker.ukf().state_idx();
    const auto & x = input.tracker.ukf().x();

    if (input.smoothed != nullptr) {
      msg.center_position.x = input.smoothed->center_position.x();
      msg.center_position.y = input.smoothed->center_position.y();
      msg.center_position.z = input.smoothed->center_position.z();
      msg.center_velocity.x = input.smoothed->velocity.x();
      msg.center_velocity.y = input.smoothed->velocity.y();
      msg.center_velocity.z = input.smoothed->velocity.z();
      msg.yaw = input.smoothed->yaw;
      msg.yaw_velocity = input.smoothed->yaw_velocity;
      msg.radius = input.smoothed->r1;
      msg.radius_2 = input.smoothed->r2;
      msg.d_za = input.smoothed->dza;
    } else {
      auto pos = input.tracker.get_center_position();
      msg.center_position.x = pos.x();
      msg.center_position.y = pos.y();
      msg.center_position.z = pos.z();
      msg.center_velocity.x = x(idx.VX());
      msg.center_velocity.y = x(idx.VY());
      msg.center_velocity.z = x(idx.VZ());
      msg.yaw = input.tracker.get_yaw();
      msg.yaw_velocity = x(idx.DELTA_RATE());
      auto [r1, r2] = input.tracker.get_radii();
      msg.radius = r1;
      msg.radius_2 = r2;
      msg.d_za = input.tracker.get_dza();
    }

    if (idx.has("AX")) {
      msg.center_acceleration.x = x(idx.AX());
      msg.center_acceleration.y = x(idx.AY());
      msg.center_acceleration.z = x(idx.AZ());
    } else {
      msg.center_acceleration.x = 0.0;
      msg.center_acceleration.y = 0.0;
      msg.center_acceleration.z = 0.0;
    }

    msg.yaw_acceleration = idx.has("DELTA_ACC") ? x(idx.get("DELTA_ACC")) : 0.0;
    msg.d_zc = 0.0;
    msg.num_armors = num_armors_;

    const double off_r1 = input.smoothed ? input.smoothed->r1 : msg.radius;
    const double off_r2 = input.smoothed ? input.smoothed->r2 : msg.radius_2;
    const double off_dza = input.smoothed ? input.smoothed->dza : msg.d_za;

    msg.armors_offset = TrackedRobotUsage::generateArmorsOffsetFromProfile(
      msg.num_armors,
      off_r1,
      off_r2,
      off_dza,
      msg.d_zc);

    try {
      const auto & P = input.tracker.ukf().P();
      int dim = static_cast<int>(P.rows());
      msg.covariance_dim = dim;
      msg.state_covariance.resize(dim * dim);
      for (int r = 0; r < dim; ++r) {
        for (int c = 0; c < dim; ++c) {
          msg.state_covariance[r * dim + c] = P(r, c);
        }
      }
    } catch (...) {
      msg.state_covariance.clear();
      msg.covariance_dim = 0;
    }

    msg.bound_armor_ids = {input.robot_id};

    if (input.tracker.is_tracking()) {
      msg.confidence = 1.0;
    } else if (input.tracker.is_temp_lost()) {
      msg.confidence = 0.7;
    } else {
      msg.confidence = 0.3;
    }

    msg.is_visible = input.tracker.is_tracking() || input.tracker.is_temp_lost();
    msg.visible_armor_count = msg.is_visible ? std::max(0, input.visible_armor_count) : 0;

    // Publish full-state fields and keep legacy fields synchronized.
    TrackedRobotUsage::syncFullStateFromLegacy(msg);

    return msg;
  }

private:
  std::string strategy_name_;
  uint8_t robot_type_;
  int num_armors_;
};

}  // namespace

std::shared_ptr<ITrackedRobotBuilderStrategy> createFixedProfileTrackedRobotBuilder(
  std::string strategy_name,
  uint8_t robot_type,
  int num_armors)
{
  return std::make_shared<FixedProfileTrackedRobotBuilder>(
    std::move(strategy_name),
    robot_type,
    num_armors);
}

}  // namespace fyt::auto_aim::robot_description