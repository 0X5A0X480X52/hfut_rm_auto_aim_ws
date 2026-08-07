#pragma once

#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"

namespace armor_pnp_refiner {

class QualityGate {
public:
  explicit QualityGate(const PnpRefinerConfig &config);

  RefineStatus evaluate(const PnpRefineOutput &refined);

  // Individual checks.
  bool checkFinite(const PnpRefineOutput &out) const;
  bool checkPositiveDepth(const PnpRefineOutput &out) const;
  bool checkReprojectionError(const PnpRefineOutput &out) const;
  bool checkPoseDelta(const PnpRefineOutput &out) const;
  bool checkYawDelta(const PnpRefineOutput &out) const;
  bool checkConditionNumber(const PnpRefineOutput &out) const;
  bool checkChi2(const PnpRefineOutput &out) const;

  // Decide final status from all checks.
  RefineStatus decideStatus(bool all_passed, bool marginal, double confidence);

private:
  PnpRefinerConfig config_;
};

}  // namespace armor_pnp_refiner
