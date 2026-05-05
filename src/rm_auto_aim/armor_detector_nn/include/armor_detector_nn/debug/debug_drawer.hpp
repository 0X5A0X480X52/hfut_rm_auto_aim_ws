#ifndef ARMOR_DETECTOR_NN_DEBUG_DRAWER_HPP_
#define ARMOR_DETECTOR_NN_DEBUG_DRAWER_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "armor_detector_nn/core/detection_types.hpp"

namespace fyt::auto_aim {

class DebugDrawer {
public:
  DebugDrawer();

  void drawDetections(
    cv::Mat& image,
    const std::vector<ArmorDetection>& detections,
    bool show_confidence = true) const;

  void drawProfiler(
    cv::Mat& image,
    double fps,
    double latency_ms,
    const std::string& backend_name,
    const std::string& precision) const;

  void drawArmorsCount(cv::Mat& image, int count) const;

  void setClassColor(const std::string& label, const cv::Scalar& color);

private:
  static cv::Scalar generateColor(const std::string& label);

  std::unordered_map<std::string, cv::Scalar> class_colors_;
};

}  // namespace fyt::auto_aim

#endif
