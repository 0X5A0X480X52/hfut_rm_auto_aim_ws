#include "armor_pnp_refiner/core/armor_pnp_refiner.hpp"

#include "armor_pnp_refiner/graph/legacy_single_yaw/armor_detector_ba_solver_proxy.hpp"
#include "armor_pnp_refiner/graph/single_yaw/single_yaw_optimizer.hpp"
#include "armor_pnp_refiner/graph/single_xyz_yaw/single_xyz_yaw_optimizer.hpp"
#include "armor_pnp_refiner/graph/sliding_window/sliding_window_optimizer.hpp"
#include "armor_pnp_refiner/association/short_term_associator.hpp"
#include "armor_pnp_refiner/association/track2d_bridge.hpp"
#include "armor_pnp_refiner/covariance/covariance_estimator.hpp"
#include "armor_pnp_refiner/quality/quality_gate.hpp"
#include "armor_pnp_refiner/quality/confidence_estimator.hpp"
#include "armor_pnp_refiner/quality/fallback_manager.hpp"

namespace armor_pnp_refiner {

ArmorPnpRefiner::ArmorPnpRefiner(const PnpRefinerConfig& config)
  : config_(config)
{
  legacy_ba_proxy_ = std::make_unique<LegacySingleYawBaProxy>(config_);
  single_yaw_optimizer_ = std::make_unique<SingleYawOptimizer>(config_);
  single_xyz_yaw_optimizer_ = std::make_unique<SingleXyzYawOptimizer>(config_);
  associator_ = std::make_unique<ShortTermAssociator>(config_);
  window_manager_ = std::make_unique<PnpRefineWindowManager>(config_);
  sliding_window_optimizer_ = std::make_unique<SlidingWindowOptimizer>(config_);
  quality_gate_ = std::make_unique<QualityGate>(config_);
  fallback_manager_ = std::make_unique<FallbackManager>(config_);
  covariance_estimator_ = std::make_unique<CovarianceEstimator>(config_);
}

ArmorPnpRefiner::~ArmorPnpRefiner() = default;

PnpRefineOutput ArmorPnpRefiner::refine(const PnpRefineInput& input)
{
  // "none" mode: pass-through raw PnP.
  if (config_.mode == "none") {
    auto out = FallbackManager::buildRawPnpOutput(input);
    out.reason = "refiner disabled (mode=none)";
    return out;
  }

  // Validate input.
  if (input.image_points.size() < 4) {
    return fallback_manager_->fallbackToRawPnp(input,
        "insufficient image points (" + std::to_string(input.image_points.size()) + ")");
  }
  if (input.image_points.size() != input.object_points.size()) {
    return fallback_manager_->fallbackToRawPnp(input,
        "image/object point count mismatch");
  }

  PnpRefineOutput output;

  // Dispatch based on mode.
  if (config_.mode == "single_yaw") {
    output = refineSingleYaw(input);
  } else if (config_.mode == "single_xyz_yaw") {
    output = refineSingleXyzYaw(input);
  } else if (config_.mode == "sliding_window") {
    int track_id = resolveRefineTrackId(input);
    output = refineSlidingWindow(input, track_id);
  } else {
    // Unknown mode: fall back to raw PnP.
    output = fallback_manager_->fallbackToRawPnp(input,
        "unknown mode: " + config_.mode);
    return output;
  }

  // Post-refinement quality evaluation.
  if (output.refined) {
    ConfidenceEstimator conf_est;
    output.confidence = conf_est.compute(output);

    RefineStatus status = quality_gate_->evaluate(output, input, config_);
    output.status = status;

    if (status == RefineStatus::REJECTED) {
      return fallback_manager_->fallbackToRawPnp(input,
          "quality gate rejected: " + output.reason);
    } else if (status == RefineStatus::DEGRADED) {
      return fallback_manager_->fallbackToDegraded(output, input,
          "quality gate degraded");
    }
    // GOOD: return as-is.
  } else {
    // Optimization failed; fall back.
    return fallback_manager_->fallbackToRawPnp(input,
        "refinement failed: " + output.reason);
  }

  return output;
}

void ArmorPnpRefiner::reset()
{
  associator_->reset();
  window_manager_->resetAll();
}

void ArmorPnpRefiner::resetRefineTrack(int refine_track_id)
{
  window_manager_->resetTrack(refine_track_id);
}

int ArmorPnpRefiner::resolveRefineTrackId(const PnpRefineInput& input)
{
  // Use external track id if available and enabled.
  if (input.external_track_id.has_value() && config_.use_external_track_id_if_available) {
    return input.external_track_id.value();
  }

  // Use internal association.
  if (config_.enable_internal_association) {
    return associator_->associate(input);
  }

  return -1;
}

PnpRefineOutput ArmorPnpRefiner::refineSingleYaw(const PnpRefineInput& input)
{
  // Phase0 policy: prefer legacy behavior path first to minimize semantic drift.
  if (config_.prefer_legacy_single_yaw) {
    auto legacy_out = legacy_ba_proxy_->refine(input);
    if (legacy_out.refined && legacy_out.status != RefineStatus::REJECTED) {
      return legacy_out;
    }
    // If legacy path is unavailable in this build/runtime context, fall through
    // to the internal optimizer so the pipeline can still run.
  }

  auto output = single_yaw_optimizer_->refine(input);
  if (!output.refined || output.status == RefineStatus::REJECTED) {
    return fallback_manager_->fallbackToRawPnp(
        input, "single_yaw failed in both legacy and internal paths");
  }
  return output;
}

PnpRefineOutput ArmorPnpRefiner::refineSingleXyzYaw(const PnpRefineInput& input)
{
  auto output = single_xyz_yaw_optimizer_->refine(input);

  if (!output.refined || output.status == RefineStatus::REJECTED) {
    // Fall back to single_yaw, then raw PnP.
    auto yaw_output = refineSingleYaw(input);
    if (yaw_output.refined && yaw_output.status != RefineStatus::REJECTED) {
      return yaw_output;
    }
    return fallback_manager_->fallbackToRawPnp(input,
        "single_xyz_yaw and single_yaw both failed");
  }

  return output;
}

PnpRefineOutput ArmorPnpRefiner::refineSlidingWindow(
    const PnpRefineInput& input, int refine_track_id)
{
  // Push current frame to window.
  window_manager_->push(refine_track_id, input);

  auto window = window_manager_->getWindow(refine_track_id);

  // Window too small: fall back to single_xyz_yaw.
  if (static_cast<int>(window.size()) < config_.min_window_size) {
    auto single_out = refineSingleXyzYaw(input);
    single_out.window_size = static_cast<int>(window.size());
    single_out.refine_track_id = refine_track_id;
    return single_out;
  }

  auto output = sliding_window_optimizer_->refine(window);

  output.refine_track_id = refine_track_id;

  if (!output.refined || output.status == RefineStatus::REJECTED) {
    // Fall back through the chain.
    window_manager_->resetTrack(refine_track_id);  // Reset contaminated window.
    return refineSingleXyzYaw(input);
  }

  return output;
}

PnpRefineOutput ArmorPnpRefiner::applyFallback(
    const PnpRefineInput& input,
    const std::string& reason,
    RefineMode attempted_mode)
{
  auto out = fallback_manager_->fallbackToRawPnp(input,
      "[" + std::to_string(static_cast<int>(attempted_mode)) + "] " + reason);
  return out;
}

}  // namespace armor_pnp_refiner
