#ifndef ARMOR_DETECTOR_NN_ARMOR_POSE_ESTIMATOR_ADAPTER_HPP_
#define ARMOR_DETECTOR_NN_ARMOR_POSE_ESTIMATOR_ADAPTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "armor_detector_nn/core/detection_types.hpp"
#include "armor_detector_nn/core/detector_config.hpp"

namespace fyt::auto_aim {

class IBundleAdjuster;

struct PoseEstimate {
  bool valid{false};
  Eigen::Vector3d translation{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation{Eigen::Quaterniond::Identity()};
  cv::Mat rvec;
  cv::Mat tvec;
  double reprojection_error{0.0};
};

class ArmorPoseEstimatorAdapter {
public:
  explicit ArmorPoseEstimatorAdapter(const PoseConfig& config);
  ~ArmorPoseEstimatorAdapter();

  // Set a custom BA adjuster. If not set and use_ba is true, a default
  // implementation (if available) is used. Takes ownership.
  void setBundleAdjuster(std::unique_ptr<IBundleAdjuster> adjuster);

  PoseEstimate estimate(
    const ArmorDetection& detection,
    const sensor_msgs::msg::CameraInfo& camera_info);

  std::vector<PoseEstimate> estimateBatch(
    const std::vector<ArmorDetection>& detections,
    const sensor_msgs::msg::CameraInfo& camera_info);

  static std::vector<cv::Point3f>
  getObjectPoints(const std::string& publish_type,
                  double small_w, double small_h,
                  double large_w, double large_h);

  static double distanceToImageCenter(
    const cv::Point2f& center,
    const cv::Point2f& image_center);

  const PoseConfig& config() const { return config_; }

private:
  PoseEstimate solvePnP(
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Point3f>& object_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs);

  PoseConfig config_;
  std::unique_ptr<IBundleAdjuster> ba_adjuster_;
};

}  // namespace fyt::auto_aim

#endif
