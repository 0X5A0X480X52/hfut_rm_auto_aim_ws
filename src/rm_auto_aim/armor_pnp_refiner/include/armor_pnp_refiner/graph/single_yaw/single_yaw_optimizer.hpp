#pragma once

#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"

namespace armor_pnp_refiner {

// Phase0/Phase1: Single-frame yaw-only g2o optimizer.
// Replaces legacy BaSolver with the same yaw-only formulation
// but using this package's g2o edges.
class SingleYawOptimizer {
public:
  explicit SingleYawOptimizer(const PnpRefinerConfig& config);

  PnpRefineOutput refine(const PnpRefineInput& input);

private:
  PnpRefinerConfig config_;
};

}  // namespace armor_pnp_refiner
