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

namespace fyt::auto_aim::robot_description
{

void TrackedRobotBuilderRegistry::registerBuilder(
  const std::string & robot_id,
  const std::shared_ptr<ITrackedRobotBuilderStrategy> & builder)
{
  builders_[robot_id] = builder;
}

std::shared_ptr<const ITrackedRobotBuilderStrategy> TrackedRobotBuilderRegistry::findBuilder(
  const std::string & robot_id) const
{
  auto it = builders_.find(robot_id);
  return it == builders_.end() ? nullptr : it->second;
}

bool TrackedRobotBuilderRegistry::supports(const std::string & robot_id) const
{
  return builders_.find(robot_id) != builders_.end();
}

std::vector<std::string> TrackedRobotBuilderRegistry::supportedRobotIds() const
{
  std::vector<std::string> ids;
  ids.reserve(builders_.size());
  for (const auto & item : builders_) {
    ids.push_back(item.first);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

}  // namespace fyt::auto_aim::robot_description