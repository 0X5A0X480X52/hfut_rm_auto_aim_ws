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

#include "gimbal_pipeline/common/robot_description/Strategy/fixed_profile_tracked_robot_builder.hpp"

namespace fyt::auto_aim::robot_description
{
RobotDescriptionFacade::RobotDescriptionFacade()
{
  registerDefaultBuilders();
}

void RobotDescriptionFacade::setStrictUnknownReject(bool strict_unknown_reject)
{
  strict_unknown_reject_ = strict_unknown_reject;
}

bool RobotDescriptionFacade::strictUnknownReject() const
{
  return strict_unknown_reject_;
}

void RobotDescriptionFacade::registerBuilder(
  const std::string & robot_id,
  const std::shared_ptr<ITrackedRobotBuilderStrategy> & builder)
{
  builder_registry_.registerBuilder(robot_id, builder);
}

bool RobotDescriptionFacade::isSupportedRobotId(const std::string & robot_id) const
{
  return builder_registry_.supports(robot_id);
}

std::vector<std::string> RobotDescriptionFacade::supportedRobotIds() const
{
  return builder_registry_.supportedRobotIds();
}

TrackedRobotBuildResult RobotDescriptionFacade::tryBuildTrackedRobot(
  const TrackedRobotBuildInput & input) const
{
  TrackedRobotBuildResult result;

  auto builder = builder_registry_.findBuilder(input.robot_id);
  if (!builder) {
    if (strict_unknown_reject_) {
      result.status = TrackedRobotBuildStatus::REJECTED_UNSUPPORTED_ID;
      result.reason = "Unsupported robot_id in strict mode: " + input.robot_id;
      return result;
    }

    auto fallback_builder = builder_registry_.findBuilder("2");
    if (!fallback_builder) {
      result.status = TrackedRobotBuildStatus::INTERNAL_ERROR;
      result.reason = "Fallback builder for STANDARD_4 is not registered";
      return result;
    }
    builder = fallback_builder;
  }

  try {
    result.robot = builder->buildTrackedRobot(input);
    result.status = TrackedRobotBuildStatus::SUCCESS;
    result.reason = builder->strategyName();
  } catch (...) {
    result.status = TrackedRobotBuildStatus::INTERNAL_ERROR;
    result.reason = "Exception while building TrackedRobot";
  }

  return result;
}

void RobotDescriptionFacade::registerDefaultBuilders()
{
  using T = rm_interfaces::msg::TrackedRobot;

  auto hero_builder = createFixedProfileTrackedRobotBuilder(
    "hero_4_builder",
    static_cast<uint8_t>(T::HERO_4),
    4);

  auto standard_builder = createFixedProfileTrackedRobotBuilder(
    "standard_4_builder",
    static_cast<uint8_t>(T::STANDARD_4),
    4);

  auto outpost_builder = createFixedProfileTrackedRobotBuilder(
    "outpost_3_builder",
    static_cast<uint8_t>(T::OUTPOST_3),
    3);

  auto base_builder = createFixedProfileTrackedRobotBuilder(
    "base_3_builder",
    static_cast<uint8_t>(T::BASE),
    3);

  auto sentry_builder = createFixedProfileTrackedRobotBuilder(
    "sentry_4_builder",
    static_cast<uint8_t>(T::SENTRY),
    4);

  auto buff_single_builder = createFixedProfileTrackedRobotBuilder(
    "buff_single_builder",
    static_cast<uint8_t>(T::UNKNOWN),
    1);

  registerBuilder("1", hero_builder);

  registerBuilder("2", standard_builder);
  registerBuilder("3", standard_builder);
  registerBuilder("4", standard_builder);
  registerBuilder("5", standard_builder);

  registerBuilder("outpost", outpost_builder);
  registerBuilder("base", base_builder);
  registerBuilder("sentry", sentry_builder);
  registerBuilder("big_buff", buff_single_builder);
  registerBuilder("small_buff", buff_single_builder);
}

}  // namespace fyt::auto_aim::robot_description
