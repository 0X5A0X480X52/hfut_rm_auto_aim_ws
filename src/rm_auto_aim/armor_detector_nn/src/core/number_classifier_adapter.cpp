#include "armor_detector_nn/core/number_classifier_adapter.hpp"

#include <algorithm>
#include <fstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/format.h>
#include <opencv2/imgproc.hpp>

#include "rm_utils/logger/log.hpp"

namespace fyt::auto_aim {

namespace {

std::string resolvePath(const std::string& raw) {
  if (raw.compare(0, 10, "package://") == 0) {
    auto slash = raw.find('/', 10);
    std::string pkg = raw.substr(10, slash - 10);
    std::string rel = raw.substr(slash + 1);
    return ament_index_cpp::get_package_share_directory(pkg) + "/" + rel;
  }
  return raw;
}

}  // namespace

NumberClassifierAdapter::NumberClassifierAdapter(
    const std::string& model_path,
    const std::string& label_path,
    const double thre,
    const std::vector<std::string>& ignore_classes)
  : threshold(thre)
  , ignore_classes_(ignore_classes)
{
  std::string resolved_model = resolvePath(model_path);
  std::string resolved_label = resolvePath(label_path);

  net_ = cv::dnn::readNetFromONNX(resolved_model);

  std::ifstream label_file(resolved_label);
  std::string line;
  while (std::getline(label_file, line)) {
    if (!line.empty()) {
      class_names_.push_back(line);
    }
  }

  if (class_names_.empty()) {
    throw std::runtime_error("NumberClassifierAdapter: no labels found in " + label_path);
  }

  FYT_INFO("armor_detector_nn",
           "NumberClassifierAdapter loaded: {} classes, model={}",
           class_names_.size(), model_path.c_str());
}

cv::Mat NumberClassifierAdapter::extractNumber(
    const cv::Mat& src,
    const ArmorDetection& detection) const
{
  static const int light_length = 12;
  static const int warp_height = 28;
  static const int small_armor_width = 32;
  static const int large_armor_width = 54;
  static const cv::Size roi_size(20, 28);
  static const cv::Size input_size(28, 28);

  // Source vertices from canonical keypoints:
  // kpt0=left_bottom, kpt1=left_top, kpt2=right_top, kpt3=right_bottom
  cv::Point2f src_vertices[4] = {
    detection.keypoints[0],  // left_bottom
    detection.keypoints[1],  // left_top
    detection.keypoints[2],  // right_top
    detection.keypoints[3],  // right_bottom
  };

  const int top_light_y = (warp_height - light_length) / 2 - 1;
  const int bottom_light_y = top_light_y + light_length;
  const int warp_width = (detection.publish_type == "large")
    ? large_armor_width : small_armor_width;

  cv::Point2f dst_vertices[4] = {
    cv::Point(0, bottom_light_y),
    cv::Point(0, top_light_y),
    cv::Point(warp_width - 1, top_light_y),
    cv::Point(warp_width - 1, bottom_light_y),
  };

  cv::Mat number_image;
  auto M = cv::getPerspectiveTransform(src_vertices, dst_vertices);
  cv::warpPerspective(src, number_image, M, cv::Size(warp_width, warp_height));

  // Crop ROI from center
  number_image = number_image(
    cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

  // Binarize
  cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
  cv::threshold(number_image, number_image, 0, 255,
                cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::resize(number_image, number_image, input_size);

  return number_image;
}

ClassifyResult NumberClassifierAdapter::classify(const cv::Mat& number_img) {
  ClassifyResult result;

  cv::Mat input = number_img / 255.0;
  cv::Mat blob;
  cv::dnn::blobFromImage(input, blob);

  cv::Mat outputs;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    net_.setInput(blob);
    outputs = net_.forward().clone();
  }

  double confidence = 0.0;
  cv::Point class_id_point;
  cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
  int label_id = class_id_point.x;

  if (label_id >= 0 && static_cast<size_t>(label_id) < class_names_.size()) {
    result.number = class_names_[label_id];
    result.confidence = confidence;
  }

  return result;
}

void NumberClassifierAdapter::classifyAndOverride(
    const cv::Mat& src, ArmorDetection& detection)
{
  auto number_img = extractNumber(src, detection);
  auto result = classify(number_img);

  bool is_ignored = false;
  for (const auto& ig : ignore_classes_) {
    if (result.number == ig) {
      is_ignored = true;
      break;
    }
  }

  if (is_ignored || result.confidence < threshold) {
    return;
  }

  // Override with classifier result
  detection.publish_number = result.number;
  detection.confidence = static_cast<float>(result.confidence);
  detection.model_label = result.number;  // best-effort; label_map lookup may refine
}

}  // namespace fyt::auto_aim
