#ifndef ARMOR_DETECTOR_NN_INTERNAL_IOU_TRACKER_STRATEGY_HPP_
#define ARMOR_DETECTOR_NN_INTERNAL_IOU_TRACKER_STRATEGY_HPP_

#include <opencv2/core.hpp>

#include <vector>

#include "armor_detector_nn/core/detection_types.hpp"
#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/core/tracker/itracker_strategy.hpp"
#include "armor_detector_nn/core/tracker/tracker_types.hpp"

namespace fyt::auto_aim {

class InternalIoUTrackerStrategy : public ITrackerStrategy {
public:
  explicit InternalIoUTrackerStrategy(const TrackerConfig& config);

  std::vector<TrackedDetection> associate(
    const std::vector<ArmorDetection>& detections,
    const rclcpp::Time& stamp) override;

  std::vector<TrackState> getActiveTracks() const override;
  void reset() override;
  TrackerStats getStats() const override;

private:
  TrackerConfig config_;
  std::vector<TrackState> tracks_;
  int next_id_{0};

  static double computeIoU(const cv::Rect2f& a, const cv::Rect2f& b);

  // Hungarian algorithm on square cost matrix.
  // Returns pairs (row, col) = (track_idx, detection_idx).
  static std::vector<cv::Point> hungarianMatch(const cv::Mat& cost_matrix);
};

}  // namespace fyt::auto_aim

#endif
