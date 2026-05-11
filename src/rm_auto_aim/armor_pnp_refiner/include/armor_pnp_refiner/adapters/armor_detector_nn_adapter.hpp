#pragma once

#include <memory>
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"

namespace armor_pnp_refiner {

class ArmorPnpRefiner;

// Adapter to bridge armor_detector_nn's IPoseRefiner interface with
// the public armor_pnp_refiner module.
//
// This adapter wraps ArmorPnpRefiner and can be registered via
// ArmorPoseEstimatorAdapter::setRefiner() in armor_detector_nn.
//
// Usage in armor_detector_nn:
//   auto pnp_refiner = std::make_shared<ArmorPnpRefiner>(config);
//   auto adapter = std::make_shared<G2oPnpRefinerAdapter>(pnp_refiner);
//   pose_estimator_adapter->setRefiner(adapter);
class G2oPnpRefinerAdapter {
public:
  explicit G2oPnpRefinerAdapter(std::shared_ptr<ArmorPnpRefiner> refiner);
  ~G2oPnpRefinerAdapter();

  // Convert armor_detector_nn types to PnpRefineInput, call refiner,
  // convert PnpRefineOutput back to armor_detector_nn PoseEstimate.
  // Returns refined pose; caller checks validity.
  PnpRefineOutput refineFromDetectorNn(
      const void* pose_estimate,   // armor_detector_nn::PoseEstimate*
      const void* detection,       // armor_detector_nn::ArmorDetection*
      const void* camera_info);    // sensor_msgs::msg::CameraInfo* or similar

  // Direct access to underlying refiner.
  std::shared_ptr<ArmorPnpRefiner> refiner() { return refiner_; }

private:
  std::shared_ptr<ArmorPnpRefiner> refiner_;
};

}  // namespace armor_pnp_refiner
