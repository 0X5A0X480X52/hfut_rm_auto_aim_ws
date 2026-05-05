#ifndef ARMOR_DETECTOR_NN_POSE_REFINER_HPP_
#define ARMOR_DETECTOR_NN_POSE_REFINER_HPP_

#include <array>
#include <memory>

#include <opencv2/core.hpp>

#include "armor_detector_nn/core/armor_pose_estimator_adapter.hpp"
#include "armor_detector_nn/core/detector_config.hpp"

namespace fyt::auto_aim {

// High-level refiner interface (Phase 1+).
// Prefer this over IBundleAdjuster for new implementations.
class IPoseRefiner {
public:
  virtual ~IPoseRefiner() = default;

  virtual PoseEstimate refine(
    const PoseEstimate& pnp_result,
    const std::array<cv::Point2f, 4>& image_points,
    const std::array<cv::Point3f, 4>& object_points,
    const cv::Mat& K,
    const cv::Mat& D) = 0;
};

// Single-frame yaw-only BA refiner.
// Fixed: tvec (from PnP), pitch, roll.
// Optimized: yaw.
class SingleYawRefiner : public IPoseRefiner {
public:
  explicit SingleYawRefiner(const SingleYawConfig& config,
                            const GateConfig& gate);

  PoseEstimate refine(
    const PoseEstimate& pnp_result,
    const std::array<cv::Point2f, 4>& image_points,
    const std::array<cv::Point3f, 4>& object_points,
    const cv::Mat& K,
    const cv::Mat& D) override;

private:
  SingleYawConfig config_;
  GateConfig gate_;

  static double angleWrap(double angle);

  double computeReprojError(
    double yaw,
    const cv::Vec3d& tvec,
    double pitch,
    double roll,
    const std::array<cv::Point2f, 4>& image_points,
    const std::array<cv::Point3f, 4>& object_points,
    const cv::Mat& K);

  bool optimizeYaw(
    double& yaw,
    const cv::Vec3d& tvec,
    double pitch,
    double roll,
    const std::array<cv::Point2f, 4>& image_points,
    const std::array<cv::Point3f, 4>& object_points,
    const cv::Mat& K);
};

}  // namespace fyt::auto_aim

#endif
