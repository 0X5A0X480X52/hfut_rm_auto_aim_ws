#ifndef ARMOR_DETECTOR_NN_ROI_PCA_CORNER_REFINER_HPP_
#define ARMOR_DETECTOR_NN_ROI_PCA_CORNER_REFINER_HPP_

#include <array>

#include <opencv2/core.hpp>

#include "armor_detector_nn/core/corner_refine/icorner_refiner.hpp"
#include "armor_detector_nn/core/detector_config.hpp"

namespace fyt::auto_aim {

class RoiPcaCornerRefiner : public ICornerRefiner {
public:
  explicit RoiPcaCornerRefiner(const CornerRefineConfig& config,
                               int frame_cols = 640,
                               int frame_rows = 480);

  RefineResult refine(
    const cv::Mat& frame,
    const ArmorDetection& detection) override;

private:
  CornerRefineConfig config_;
  int frame_cols_;
  int frame_rows_;

  struct LightBarEndpoints {
    bool ok{false};
    cv::Point2f top;
    cv::Point2f bottom;
    RefineFailReason reason{RefineFailReason::NONE};
  };

  // Extract left and right light-bar ROIs from the 4 keypoints.
  // Canonical keypoint order in this package:
  // [0]=left_bottom, [1]=left_top, [2]=right_top, [3]=right_bottom.
  std::pair<cv::Rect2f, cv::Rect2f> extractLightBarROIs(
    const std::array<cv::Point2f, 4>& corners);

  // PCA-based light bar endpoint refinement within a single ROI.
  LightBarEndpoints refineLightBar(
    const cv::Mat& frame,
    const cv::Rect2f& roi,
    fyt::EnemyColor color);

  // Validate refined quadrilateral geometry.
  bool validateGeometry(const std::array<cv::Point2f, 4>& corners);

  // Compute a 0–1 quality score.
  static double computeQuality(
    const std::array<cv::Point2f, 4>& refined,
    const std::array<cv::Point2f, 4>& original);
};

}  // namespace fyt::auto_aim

#endif
