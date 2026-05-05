#ifndef ARMOR_DETECTOR_NN_DETECTION_TYPES_HPP_
#define ARMOR_DETECTOR_NN_DETECTION_TYPES_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/header.hpp>

#include "rm_utils/common.hpp"

namespace fyt::auto_aim {

struct RawDetection {
  int class_id{-1};
  float class_score{0.0F};
  float object_score{1.0F};
  float confidence{0.0F};
  cv::Rect2f bbox;
  std::array<cv::Point2f, 4> keypoints;
};

struct ArmorDetection {
  std::string model_label;
  std::string publish_number;
  std::string publish_type;
  fyt::EnemyColor color{fyt::EnemyColor::RED};
  float confidence{0.0F};
  cv::Rect2f bbox;
  std::array<cv::Point2f, 4> keypoints;
  cv::Point2f center;

  // Tracker fields (Phase 2+)
  int track_id{-1};
  int track_age{0};
  int track_hits{0};

  // Reserved for future: per-keypoint confidence
  std::array<float, 4> keypoint_conf{1.0f, 1.0f, 1.0f, 1.0f};

  // Observation timestamp for downstream temporal modules (e.g., sliding BA)
  rclcpp::Time stamp{};
};

struct FrameDetections {
  std_msgs::msg::Header header;
  std::vector<ArmorDetection> detections;
};

struct TensorInfo {
  std::string name;
  std::vector<int64_t> shape;
  enum class DType { FLOAT32, FLOAT16, INT8, INT32 };
  DType dtype{DType::FLOAT32};
};

struct TensorInput {
  TensorInfo info;
  std::vector<float> host_data;
};

struct TensorOutput {
  TensorInfo info;
  std::vector<float> host_data;
};

}  // namespace fyt::auto_aim

#endif
