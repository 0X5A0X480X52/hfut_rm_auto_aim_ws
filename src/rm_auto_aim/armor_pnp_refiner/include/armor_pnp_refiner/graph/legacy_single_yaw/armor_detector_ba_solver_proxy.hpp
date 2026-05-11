#pragma once

#include <memory>
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"

namespace armor_pnp_refiner {

// Proxy that wraps armor_detector's existing BaSolver.
// Phase0: reuses old yaw-only BA with zero code duplication.
// Only adapts PnpRefineInput -> PnpRefineOutput.
class LegacySingleYawBaProxy {
public:
  explicit LegacySingleYawBaProxy(const PnpRefinerConfig& config);
  ~LegacySingleYawBaProxy();

  // Run yaw-only refinement by delegating to armor_detector::BaSolver.
  // The proxy does NOT duplicate any yaw math or g2o edge implementations.
  PnpRefineOutput refine(const PnpRefineInput& input);

private:
  PnpRefinerConfig config_;

  // Forward declaration; actual BaSolver instance is wrapped via PIMPL
  // to avoid exposing armor_detector headers in this public header.
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace armor_pnp_refiner
