#pragma once

#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"

namespace armor_pnp_refiner {

class QualityGate {
public:
  explicit QualityGate(const PnpRefinerConfig& config);

  // Evaluate whether a refined output passes quality checks.
  // Returns the status (GOOD / DEGRADED / REJECTED) and fills reason.
  RefineStatus evaluate(const PnpRefineOutput& refined,
                         const PnpRefineInput& input,
                         const PnpRefinerConfig& config);

  // Individual checks.
  bool checkFinite(const PnpRefineOutput& out) const;
  bool checkPositiveDepth(const PnpRefineOutput& out, const PnpRefineInput& in) const;
  bool checkReprojectionError(const PnpRefineOutput& out) const;
  bool checkPoseDelta(const PnpRefineOutput& out) const;
  bool checkYawDelta(const PnpRefineOutput& out) const;
  bool checkConditionNumber(const PnpRefineOutput& out) const;
  bool checkChi2(const PnpRefineOutput& out) const;

  // Decide final status from all checks.
  RefineStatus decideStatus(bool all_passed, bool marginal, double confidence);

private:
  PnpRefinerConfig config_;
};

}  // namespace armor_pnp_refiner
