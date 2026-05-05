#include "armor_detector_nn/core/preprocessor.hpp"

#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace fyt::auto_aim {

Preprocessor::Preprocessor(const PreprocessConfig& config)
  : config_(config)
  , input_size_(config.input_width, config.input_height)
{
}

PreprocessResult Preprocessor::process(const cv::Mat& bgr_image) {
  PreprocessResult result;

  ImageMeta& meta = result.image_meta;
  meta.original_width  = bgr_image.cols;
  meta.original_height = bgr_image.rows;

  // 1. Color conversion (BGR → RGB if needed)
  cv::Mat converted = convertColor(bgr_image);

  // 2. Letterbox resize
  cv::Mat resized = letterbox(converted, meta);

  // 3. To tensor (HWC → CHW, normalize)
  std::vector<float> data;
  toTensor(resized, data);

  // 4. Build TensorInput
  TensorInput& tensor = result.tensor;
  tensor.info.name = "images";
  tensor.info.shape = {1, config_.input_width == resized.cols ? 3 : 3,
                       resized.rows, resized.cols};
  // Correct NCHW shape
  tensor.info.shape = {1, 3, input_size_.height, input_size_.width};
  tensor.info.dtype = TensorInfo::DType::FLOAT32;
  tensor.host_data = std::move(data);

  return result;
}

PreprocessResult Preprocessor::processBatch(const std::vector<cv::Mat>& bgr_images) {
  if (bgr_images.empty()) {
    throw std::invalid_argument("processBatch: empty image vector");
  }
  if (bgr_images.size() > 1) {
    throw std::runtime_error("processBatch: batch > 1 not supported yet");
  }
  return process(bgr_images[0]);
}

cv::Mat Preprocessor::letterbox(const cv::Mat& src, ImageMeta& meta) const {
  float scale = std::min(
    static_cast<float>(input_size_.width)  / src.cols,
    static_cast<float>(input_size_.height) / src.rows);

  int new_w = static_cast<int>(src.cols * scale);
  int new_h = static_cast<int>(src.rows * scale);

  meta.scale_x = scale;
  meta.scale_y = scale;

  meta.pad_left = (input_size_.width  - new_w) / 2.0F;
  meta.pad_top  = (input_size_.height - new_h) / 2.0F;

  cv::Mat resized;
  cv::resize(src, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

  cv::Mat out(input_size_, CV_8UC3, cv::Scalar(
    static_cast<int>(config_.pad_value),
    static_cast<int>(config_.pad_value),
    static_cast<int>(config_.pad_value)));
  resized.copyTo(out(cv::Rect(
    static_cast<int>(meta.pad_left),
    static_cast<int>(meta.pad_top),
    new_w, new_h)));

  return out;
}

cv::Mat Preprocessor::convertColor(const cv::Mat& src) const {
  if (config_.input_color == "rgb") {
    cv::Mat rgb;
    cv::cvtColor(src, rgb, cv::COLOR_BGR2RGB);
    return rgb;
  }
  return src;
}

void Preprocessor::toTensor(const cv::Mat& src, std::vector<float>& data) const {
  int c = src.channels();
  int h = src.rows;
  int w = src.cols;
  data.resize(c * h * w);

  // NCHW layout: for each channel, iterate rows then cols
  for (int ch = 0; ch < c; ++ch) {
    for (int row = 0; row < h; ++row) {
      const uint8_t* src_row = src.ptr<uint8_t>(row);
      float* dst = data.data() + ch * h * w + row * w;
      for (int col = 0; col < w; ++col) {
        float val = static_cast<float>(src_row[col * c + ch]);
        if (config_.normalize) {
          val = static_cast<float>(val / config_.std[ch] - config_.mean[ch]);
        }
        dst[col] = val;
      }
    }
  }
}

}  // namespace fyt::auto_aim
