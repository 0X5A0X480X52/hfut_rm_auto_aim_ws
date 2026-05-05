#include "armor_detector_nn/geometry/nms.hpp"

#include <algorithm>

namespace fyt::auto_aim {

namespace {

float computeIoU(const cv::Rect2f& a, const cv::Rect2f& b) {
  float x1 = std::max(a.x, b.x);
  float y1 = std::max(a.y, b.y);
  float x2 = std::min(a.x + a.width, b.x + b.width);
  float y2 = std::min(a.y + a.height, b.y + b.height);

  float inter_w = std::max(0.0F, x2 - x1);
  float inter_h = std::max(0.0F, y2 - y1);
  float inter = inter_w * inter_h;

  float area_a = a.width * a.height;
  float area_b = b.width * b.height;
  float iou = inter / (area_a + area_b - inter + 1e-6F);
  return iou;
}

}  // namespace

std::vector<RawDetection> classAwareNMS(
  const std::vector<RawDetection>& detections,
  float iou_threshold)
{
  std::vector<RawDetection> sorted = detections;
  std::sort(sorted.begin(), sorted.end(),
    [](const RawDetection& a, const RawDetection& b) {
      return a.confidence > b.confidence;
    });

  std::vector<bool> suppressed(sorted.size(), false);
  std::vector<RawDetection> result;
  result.reserve(sorted.size());

  for (size_t i = 0; i < sorted.size(); ++i) {
    if (suppressed[i]) continue;
    result.push_back(sorted[i]);
    for (size_t j = i + 1; j < sorted.size(); ++j) {
      if (suppressed[j]) continue;
      if (sorted[i].class_id != sorted[j].class_id) continue;
      if (computeIoU(sorted[i].bbox, sorted[j].bbox) > iou_threshold) {
        suppressed[j] = true;
      }
    }
  }

  return result;
}

std::vector<RawDetection> classAgnosticNMS(
  const std::vector<RawDetection>& detections,
  float iou_threshold)
{
  std::vector<RawDetection> sorted = detections;
  std::sort(sorted.begin(), sorted.end(),
    [](const RawDetection& a, const RawDetection& b) {
      return a.confidence > b.confidence;
    });

  std::vector<bool> suppressed(sorted.size(), false);
  std::vector<RawDetection> result;
  result.reserve(sorted.size());

  for (size_t i = 0; i < sorted.size(); ++i) {
    if (suppressed[i]) continue;
    result.push_back(sorted[i]);
    for (size_t j = i + 1; j < sorted.size(); ++j) {
      if (suppressed[j]) continue;
      if (computeIoU(sorted[i].bbox, sorted[j].bbox) > iou_threshold) {
        suppressed[j] = true;
      }
    }
  }

  return result;
}

}  // namespace fyt::auto_aim
