#ifndef ARMOR_DETECTOR_NN_ITRACKER_STRATEGY_HPP_
#define ARMOR_DETECTOR_NN_ITRACKER_STRATEGY_HPP_

#include <rclcpp/time.hpp>

#include <vector>

#include "armor_detector_nn/core/detection_types.hpp"
#include "armor_detector_nn/core/tracker/tracker_types.hpp"

namespace fyt::auto_aim {

struct TrackerStats {
  int total_tracks{0};
  int confirmed_tracks{0};
  int matched_this_frame{0};
  int new_this_frame{0};
};

class ITrackerStrategy {
public:
  virtual ~ITrackerStrategy() = default;

  // Core association: each ArmorDetection gets a track_id assigned.
  // Returns the same detections wrapped with track_id info.
  virtual std::vector<TrackedDetection> associate(
    const std::vector<ArmorDetection>& detections,
    const rclcpp::Time& stamp) = 0;

  virtual std::vector<TrackState> getActiveTracks() const = 0;
  virtual void reset() = 0;
  virtual TrackerStats getStats() const = 0;
};

}  // namespace fyt::auto_aim

#endif
