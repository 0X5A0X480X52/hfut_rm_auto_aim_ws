#ifndef ARMOR_DETECTOR_NN_NMS_HPP_
#define ARMOR_DETECTOR_NN_NMS_HPP_

#include <vector>

#include "armor_detector_nn/core/detection_types.hpp"

namespace fyt::auto_aim {

std::vector<RawDetection> classAwareNMS(
  const std::vector<RawDetection>& detections,
  float iou_threshold);

std::vector<RawDetection> classAgnosticNMS(
  const std::vector<RawDetection>& detections,
  float iou_threshold);

inline std::vector<RawDetection> applyNMS(
  const std::vector<RawDetection>& detections,
  float iou_threshold,
  bool class_agnostic)
{
  return class_agnostic
    ? classAgnosticNMS(detections, iou_threshold)
    : classAwareNMS(detections, iou_threshold);
}

}  // namespace fyt::auto_aim

#endif
