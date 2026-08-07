#include "armor_pnp_refiner/core/armor_pnp_refiner.hpp"

#include "armor_pnp_refiner/graph/single_xyz_yaw/single_xyz_yaw_optimizer.hpp"
#include "armor_pnp_refiner/quality/confidence_estimator.hpp"
#include "armor_pnp_refiner/quality/fallback_manager.hpp"
#include "armor_pnp_refiner/quality/quality_gate.hpp"

namespace armor_pnp_refiner {

ArmorPnpRefiner::ArmorPnpRefiner(const PnpRefinerConfig &config) : config_(config) {
  single_xyz_yaw_optimizer_ = std::make_unique<SingleXyzYawOptimizer>(config_);
  quality_gate_ = std::make_unique<QualityGate>(config_);
  fallback_manager_ = std::make_unique<FallbackManager>();
}

ArmorPnpRefiner::~ArmorPnpRefiner() = default;

PnpRefineOutput ArmorPnpRefiner::refine(const PnpRefineInput &input) {
  // "none" mode: pass-through raw PnP.
  if (config_.mode == "none") {
    auto out = FallbackManager::buildRawPnpOutput(input);
    out.reason = "refiner disabled (mode=none)";
    return out;
  }

  // Validate input.
  if (input.image_points.size() < 4) {
    return fallback_manager_->fallbackToRawPnp(
      input, "insufficient image points (" + std::to_string(input.image_points.size()) + ")");
  }
  if (input.image_points.size() != input.object_points.size()) {
    return fallback_manager_->fallbackToRawPnp(input, "image/object point count mismatch");
  }

  if (config_.mode != "single_xyz_yaw") {
    return fallback_manager_->fallbackToRawPnp(input, "unsupported mode: " + config_.mode);
  }

  auto output = single_xyz_yaw_optimizer_->refine(input);
  if (!output.refined || output.status == RefineStatus::REJECTED) {
    return fallback_manager_->fallbackToRawPnp(input, "single_xyz_yaw failed: " + output.reason);
  }

  ConfidenceEstimator confidence_estimator;
  output.confidence = confidence_estimator.compute(output);
  output.status = quality_gate_->evaluate(output);
  if (output.status == RefineStatus::REJECTED) {
    return fallback_manager_->fallbackToRawPnp(input, "quality gate rejected: " + output.reason);
  }
  if (output.status == RefineStatus::DEGRADED) {
    return fallback_manager_->fallbackToDegraded(output, "quality gate degraded");
  }
  return output;
}

}  // namespace armor_pnp_refiner
