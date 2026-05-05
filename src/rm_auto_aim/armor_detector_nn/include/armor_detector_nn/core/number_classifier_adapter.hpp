#ifndef ARMOR_DETECTOR_NN_NUMBER_CLASSIFIER_ADAPTER_HPP_
#define ARMOR_DETECTOR_NN_NUMBER_CLASSIFIER_ADAPTER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "armor_detector_nn/core/detection_types.hpp"

namespace fyt::auto_aim {

struct ClassifyResult {
  std::string number;
  double confidence{0.0};
};

class NumberClassifierAdapter {
public:
  NumberClassifierAdapter(const std::string& model_path,
                          const std::string& label_path,
                          double threshold,
                          const std::vector<std::string>& ignore_classes = {});

  // Extract the number ROI from the source image using armor keypoints
  // for perspective warp (replaces light-bar-based warp in legacy classifier).
  cv::Mat extractNumber(const cv::Mat& src,
                        const ArmorDetection& detection) const;

  // Run inference on a pre-extracted number image.
  ClassifyResult classify(const cv::Mat& number_img);

  // Full pipeline: extract + classify, then override detection fields
  // (publish_number, confidence) if confidence exceeds threshold and
  // the result is not in ignore_classes.
  void classifyAndOverride(const cv::Mat& src, ArmorDetection& detection);

  double threshold;

private:
  std::mutex mutex_;
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};

}  // namespace fyt::auto_aim

#endif
