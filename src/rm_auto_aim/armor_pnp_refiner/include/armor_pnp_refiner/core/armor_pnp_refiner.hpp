#pragma once

#include <memory>

#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"

namespace armor_pnp_refiner {

class ShortTermAssociator;
class PnpRefineWindowManager;
class SingleYawOptimizer;
class SingleXyzYawOptimizer;
class SlidingWindowOptimizer;
class QualityGate;
class FallbackManager;
class LegacySingleYawBaProxy;
class CovarianceEstimator;

// Main facade for armor PnP refinement.
// Receives raw PnP results and returns refined PnP-like output with
// covariance diagnostics.
//
// Usage:
//   auto refiner = std::make_unique<ArmorPnpRefiner>(config);
//   PnpRefineOutput out = refiner->refine(input);
class ArmorPnpRefiner {
public:
  explicit ArmorPnpRefiner(const PnpRefinerConfig& config);
  ~ArmorPnpRefiner();

  // Main entry point: PnP in -> PnP out.
  PnpRefineOutput refine(const PnpRefineInput& input);

  // Reset all internal state (windows, tracks).
  void reset();

  // Reset a specific refine track.
  void resetRefineTrack(int refine_track_id);

  // Accessors for diagnostics.
  const PnpRefinerConfig& config() const { return config_; }

private:
  PnpRefinerConfig config_;

  // Phase0: legacy yaw-only BA proxy.
  std::unique_ptr<LegacySingleYawBaProxy> legacy_ba_proxy_;

  // Phase1: single-frame optimizers.
  std::unique_ptr<SingleYawOptimizer> single_yaw_optimizer_;
  std::unique_ptr<SingleXyzYawOptimizer> single_xyz_yaw_optimizer_;

  // Phase2: sliding window components.
  std::unique_ptr<ShortTermAssociator> associator_;
  std::unique_ptr<PnpRefineWindowManager> window_manager_;
  std::unique_ptr<SlidingWindowOptimizer> sliding_window_optimizer_;

  // Quality and fallback.
  std::unique_ptr<QualityGate> quality_gate_;
  std::unique_ptr<FallbackManager> fallback_manager_;

  // Covariance.
  std::unique_ptr<CovarianceEstimator> covariance_estimator_;

  // Internal helpers.
  int resolveRefineTrackId(const PnpRefineInput& input);
  PnpRefineOutput refineSingleYaw(const PnpRefineInput& input);
  PnpRefineOutput refineSingleXyzYaw(const PnpRefineInput& input);
  PnpRefineOutput refineSlidingWindow(const PnpRefineInput& input,
                                       int refine_track_id);
  PnpRefineOutput applyFallback(const PnpRefineInput& input,
                                 const std::string& reason,
                                 RefineMode attempted_mode);
};

}  // namespace armor_pnp_refiner
