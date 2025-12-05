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

#include "armor_tracker/armor_types.hpp"

namespace fyt::auto_aim
{

std::string sourceTypeToString(ArmorSourceType type)
{
  switch (type) {
    case ArmorSourceType::DETECT: return "DETECT";
    case ArmorSourceType::ESTIMATE: return "ESTIMATE";
    case ArmorSourceType::PREDICT: return "PREDICT";
    default: return "UNKNOWN";
  }
}

std::string trackingStateToString(TrackingState state)
{
  switch (state) {
    case TrackingState::LOST: return "LOST";
    case TrackingState::DETECTING: return "DETECTING";
    case TrackingState::TRACKING: return "TRACKING";
    case TrackingState::TEMP_LOST: return "TEMP_LOST";
    default: return "UNKNOWN";
  }
}

ArmorObservation::ArmorObservation()
  : position(Eigen::Vector3d::Zero())
  , yaw(0.0)
  , confidence(0.0f)
  , source(ArmorSourceType::DETECT) {}

TrackedArmorState::TrackedArmorState()
  : track_id(-1)
  , position(Eigen::Vector3d::Zero())
  , velocity(Eigen::Vector3d::Zero())
  , yaw(0.0)
  , yaw_velocity(0.0)
  , tracking_state(TrackingState::LOST)
  , source_type(ArmorSourceType::PREDICT)
  , confidence(0.0)
  , tracking_count(0)
  , lost_count(0)
  , time_since_update(0) {}

} // namespace fyt::auto_aim
