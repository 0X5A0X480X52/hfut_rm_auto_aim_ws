#ifndef ARMOR_DETECTOR_NN_ROBOTPILOTS_LANDMARK_DECODE_STRATEGY_HPP_
#define ARMOR_DETECTOR_NN_ROBOTPILOTS_LANDMARK_DECODE_STRATEGY_HPP_

#include "armor_detector_nn/postprocess/decode_strategy.hpp"

namespace fyt::auto_aim {

class RobotPilotsLandmarkDecodeStrategy : public IDecodeStrategy {
public:
  RobotPilotsLandmarkDecodeStrategy() = default;
  ~RobotPilotsLandmarkDecodeStrategy() override = default;

  std::vector<RawDetection> decode(
    const std::vector<TensorOutput>& outputs,
    const ImageMeta& image_meta,
    const PostprocessConfig& config) override;

private:
  static float sigmoid(float x);
  static int mapToClassId(int color_idx, int number_idx);
  static cv::Rect2f bboxFromKeypoints(const std::array<cv::Point2f, 4>& keypoints);
  static void restoreKeypoints(
    std::array<cv::Point2f, 4>& keypoints,
    const ImageMeta& meta);
};

}  // namespace fyt::auto_aim

#endif
