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

#ifndef GIMBAL_PIPELINE__COMMON__ROBOT_DESCRIPTION__STRATEGY__FIXED_PROFILE_TRACKED_ROBOT_BUILDER_HPP_
#define GIMBAL_PIPELINE__COMMON__ROBOT_DESCRIPTION__STRATEGY__FIXED_PROFILE_TRACKED_ROBOT_BUILDER_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "gimbal_pipeline/common/robot_description/robot_description_facade.hpp"

namespace fyt::auto_aim::robot_description
{

std::shared_ptr<ITrackedRobotBuilderStrategy> createFixedProfileTrackedRobotBuilder(
  std::string strategy_name,
  uint8_t robot_type,
  int num_armors);

}  // namespace fyt::auto_aim::robot_description

#endif  // GIMBAL_PIPELINE__COMMON__ROBOT_DESCRIPTION__STRATEGY__FIXED_PROFILE_TRACKED_ROBOT_BUILDER_HPP_