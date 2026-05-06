#ifndef ARMOR_DETECTOR_NN_SLIDING_WINDOW_STATE_HPP_
#define ARMOR_DETECTOR_NN_SLIDING_WINDOW_STATE_HPP_

#include <array>
#include <deque>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <rclcpp/time.hpp>

#include <string>

namespace fyt::auto_aim {

struct TrackWindowFrame {
  rclcpp::Time stamp;
  int track_id{-1};

  std::array<cv::Point2f, 4> keypoints;
  std::array<float, 4> keypoint_conf{1.0f, 1.0f, 1.0f, 1.0f};

  Eigen::Vector3d t_init{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d R_imu_camera{Eigen::Matrix3d::Identity()};
  double yaw_init{0.0};
  double pitch{0.0};
  double roll{0.0};

  std::string publish_type;
  std::string publish_number;
};

class SlidingWindowState {
public:
  explicit SlidingWindowState(size_t window_size = 8)
    : window_size_(window_size) {}

  void pushFrame(const TrackWindowFrame& frame) {
    frames_.push_back(frame);
    if (frames_.size() > window_size_) {
      frames_.pop_front();
    }
  }

  size_t size() const { return frames_.size(); }
  bool isReady() const { return frames_.size() >= min_frames_; }

  void clear() { frames_.clear(); }

  const std::deque<TrackWindowFrame>& frames() const { return frames_; }

  void setMinFrames(size_t n) { min_frames_ = n; }
  const TrackWindowFrame& latest() const { return frames_.back(); }

  // Keep the newest frames so that total time span stays within threshold.
  void trimToMaxSpanMs(double max_span_ms) {
    if (max_span_ms <= 0.0) return;
    while (frames_.size() >= 2) {
      double span_ms = (frames_.back().stamp - frames_.front().stamp).seconds() * 1000.0;
      if (span_ms <= max_span_ms) break;
      frames_.pop_front();
    }
  }

  double timeSpanMs() const {
    if (frames_.size() < 2) return 0.0;
    return (frames_.back().stamp - frames_.front().stamp).seconds() * 1000.0;
  }

private:
  size_t window_size_{8};
  size_t min_frames_{4};
  std::deque<TrackWindowFrame> frames_;
};

}  // namespace fyt::auto_aim

#endif
