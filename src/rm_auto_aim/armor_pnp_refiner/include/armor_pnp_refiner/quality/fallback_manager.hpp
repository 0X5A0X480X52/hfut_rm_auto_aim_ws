#pragma once

#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"

namespace armor_pnp_refiner {

// Converts rejected refinements back to the raw PnP result.
class FallbackManager {
public:
  FallbackManager() = default;

  // Given a refined output that was rejected, produce a fallback output.
  // Returns the raw PnP output with valid=true, refined=false.
  PnpRefineOutput fallbackToRawPnp(const PnpRefineInput &input, const std::string &reason);

  // Produce a conservative (degraded) output when optimization partially succeeds.
  PnpRefineOutput fallbackToDegraded(const PnpRefineOutput &refined, const std::string &reason);

  // Build a raw PnP output from input (no optimization applied).
  static PnpRefineOutput buildRawPnpOutput(const PnpRefineInput &input);

  // Compute conservative diagonal covariance for fallback scenarios.
  static Eigen::Matrix4d conservativeCovariance(double var_xy = 0.02 * 0.02,
                                                double var_z = 0.05 * 0.05,
                                                double var_yaw = 0.05 * 0.05);
};

}  // namespace armor_pnp_refiner
