#ifndef ARMOR_DETECTOR_NN_BA_ADJUSTER_HPP_
#define ARMOR_DETECTOR_NN_BA_ADJUSTER_HPP_

#include <vector>

#include <opencv2/core.hpp>

#include "armor_detector_nn/core/armor_pose_estimator_adapter.hpp"

namespace fyt::auto_aim {

class IBundleAdjuster {
public:
  virtual ~IBundleAdjuster() = default;

  virtual PoseEstimate refine(
    const PoseEstimate& initial,
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Point3f>& object_points,
    const cv::Mat& K,
    const cv::Mat& D) = 0;

protected:
  IBundleAdjuster() = default;
  IBundleAdjuster(const IBundleAdjuster&) = delete;
  IBundleAdjuster& operator=(const IBundleAdjuster&) = delete;
};

}  // namespace fyt::auto_aim

#endif
