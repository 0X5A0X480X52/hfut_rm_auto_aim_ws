#pragma once

#include <memory>
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"

namespace armor_pnp_refiner {

class ArmorPnpRefiner;

// Adapter to integrate the public refiner into traditional armor_detector.
// Provides a simple interface to convert armor_detector's Armor type
// to PnpRefineInput and back.
class ArmorDetectorAdapter {
public:
  explicit ArmorDetectorAdapter(std::shared_ptr<ArmorPnpRefiner> refiner);
  ~ArmorDetectorAdapter();

  // Build a PnpRefineInput from raw detector PnP results.
  // Caller provides the PnP result and detection metadata.
  static PnpRefineInput buildInput(
      const Eigen::Vector3d& t_camera_armor,
      const Eigen::Quaterniond& q_camera_armor,
      double yaw_rad, double pitch_rad, double roll_rad,
      const cv::Mat& rvec, const cv::Mat& tvec,
      const std::vector<cv::Point2f>& image_points,
      const std::vector<cv::Point3f>& object_points,
      const cv::Mat& K, const cv::Mat& D,
      const std::string& armor_number,
      ArmorSizeType armor_type,
      double stamp_sec = 0.0,
      const Eigen::Matrix3d& R_imu_camera = Eigen::Matrix3d::Identity());

  // Run refinement through the refiner.
  PnpRefineOutput refine(const PnpRefineInput& input);

  std::shared_ptr<ArmorPnpRefiner> refiner() { return refiner_; }

private:
  std::shared_ptr<ArmorPnpRefiner> refiner_;
};

}  // namespace armor_pnp_refiner
