#ifndef ARMOR_DETECTOR_NN_TRACKER_TYPES_HPP_
#define ARMOR_DETECTOR_NN_TRACKER_TYPES_HPP_

#include <opencv2/core/types.hpp>
#include <rclcpp/time.hpp>

#include <string>
#include <vector>

#include "armor_detector_nn/core/detection_types.hpp"

namespace fyt::auto_aim {

struct TrackedDetection {
  ArmorDetection det;
  int track_id{-1};
  bool matched{false};
};

struct TrackState {
  int id{-1};
  cv::Rect2f predicted_bbox;
  cv::Point2f center;
  int age{0};
  int hits{0};
  int missed{0};
  rclcpp::Time last_stamp;
  std::string publish_type;
  std::string publish_number;
  bool confirmed{false};

  // Motion prediction for next frame
  cv::Point2f velocity{0.0f, 0.0f};
  std::vector<cv::Point2f> recent_centers;
};

}  // namespace fyt::auto_aim

#endif
