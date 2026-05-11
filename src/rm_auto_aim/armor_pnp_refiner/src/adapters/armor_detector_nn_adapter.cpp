#include "armor_pnp_refiner/adapters/armor_detector_nn_adapter.hpp"

#include "armor_pnp_refiner/core/armor_pnp_refiner.hpp"

namespace armor_pnp_refiner {

G2oPnpRefinerAdapter::G2oPnpRefinerAdapter(std::shared_ptr<ArmorPnpRefiner> refiner)
  : refiner_(std::move(refiner)) {}

G2oPnpRefinerAdapter::~G2oPnpRefinerAdapter() = default;

PnpRefineOutput G2oPnpRefinerAdapter::refineFromDetectorNn(
    const void* /*pose_estimate*/,
    const void* /*detection*/,
    const void* /*camera_info*/)
{
  // Full integration: convert armor_detector_nn types to PnpRefineInput,
  // call refiner_->refine(), convert PnpRefineOutput back.
  //
  // This is a void* bridge to avoid direct dependency on armor_detector_nn
  // types in this public header. The actual conversion is done in
  // armor_detector_nn's own code that includes both this header and
  // its own PoseEstimate/ArmorDetection types.
  //
  // For now, return a fallback result.
  PnpRefineOutput out;
  out.valid = true;
  out.refined = false;
  out.mode = RefineMode::PNP_FALLBACK;
  out.status = RefineStatus::DEGRADED;
  out.reason = "adapter bridge not fully integrated - use ArmorPnpRefiner directly";
  return out;
}

}  // namespace armor_pnp_refiner
