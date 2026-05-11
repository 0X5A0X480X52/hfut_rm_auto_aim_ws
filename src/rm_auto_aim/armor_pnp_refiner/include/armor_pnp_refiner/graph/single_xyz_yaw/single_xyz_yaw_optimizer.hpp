#pragma once

#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"

namespace armor_pnp_refiner {

// Phase1: Single-frame xyz-yaw g2o optimizer.
// State: [tx, ty, tz, yaw]
// Residuals: reprojection + PnP translation prior + yaw prior
class SingleXyzYawOptimizer {
public:
  explicit SingleXyzYawOptimizer(const PnpRefinerConfig& config);

  PnpRefineOutput refine(const PnpRefineInput& input);

private:
  PnpRefinerConfig config_;
};

}  // namespace armor_pnp_refiner
