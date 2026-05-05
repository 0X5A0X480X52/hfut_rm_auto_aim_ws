#ifndef ARMOR_DETECTOR_NN_PREPROCESSOR_HPP_
#define ARMOR_DETECTOR_NN_PREPROCESSOR_HPP_

#include <vector>

#include <opencv2/core.hpp>

#include "armor_detector_nn/core/detection_types.hpp"
#include "armor_detector_nn/core/detector_config.hpp"
#include "armor_detector_nn/postprocess/decode_strategy.hpp"

namespace fyt::auto_aim {

struct PreprocessResult {
  TensorInput tensor;
  ImageMeta image_meta;
};

class Preprocessor {
public:
  explicit Preprocessor(const PreprocessConfig& config);

  PreprocessResult process(const cv::Mat& bgr_image);

  PreprocessResult processBatch(const std::vector<cv::Mat>& bgr_images);

  const PreprocessConfig& config() const { return config_; }

private:
  cv::Mat letterbox(const cv::Mat& src, ImageMeta& meta) const;
  cv::Mat convertColor(const cv::Mat& src) const;
  void toTensor(const cv::Mat& src, std::vector<float>& data) const;

  PreprocessConfig config_;
  cv::Size input_size_;
};

}  // namespace fyt::auto_aim

#endif
