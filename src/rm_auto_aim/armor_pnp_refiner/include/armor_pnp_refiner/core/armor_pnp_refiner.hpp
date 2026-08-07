#pragma once

#include <memory>

#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"

namespace armor_pnp_refiner {

class SingleXyzYawOptimizer;
class QualityGate;
class FallbackManager;

// Main facade for armor PnP refinement.
// Receives raw PnP results and returns refined PnP-like output with
// covariance diagnostics.
//
// Usage:
//   auto refiner = std::make_unique<ArmorPnpRefiner>(config);
//   PnpRefineOutput out = refiner->refine(input);
class ArmorPnpRefiner {
public:
  explicit ArmorPnpRefiner(const PnpRefinerConfig &config);
  ~ArmorPnpRefiner();

  // Main entry point: PnP in -> PnP out.
  PnpRefineOutput refine(const PnpRefineInput &input);

  const PnpRefinerConfig &config() const { return config_; }

private:
  PnpRefinerConfig config_;
  std::unique_ptr<SingleXyzYawOptimizer> single_xyz_yaw_optimizer_;
  std::unique_ptr<QualityGate> quality_gate_;
  std::unique_ptr<FallbackManager> fallback_manager_;
};

}  // namespace armor_pnp_refiner
