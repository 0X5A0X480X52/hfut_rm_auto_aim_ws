#include "armor_tracker/armor_types.hpp"

namespace fyt::auto_aim {

std::string sourceTypeToString(ArmorSourceType type) {
  switch (type) {
    case ArmorSourceType::DETECT: return "DETECT";
    case ArmorSourceType::ESTIMATE: return "ESTIMATE";
    case ArmorSourceType::PREDICT: return "PREDICT";
    default: return "UNKNOWN";
  }
}

std::string trackingStateToString(TrackingState state) {
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
