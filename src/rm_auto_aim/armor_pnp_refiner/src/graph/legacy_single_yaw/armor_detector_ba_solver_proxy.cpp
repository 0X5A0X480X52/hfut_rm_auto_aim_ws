#include "armor_pnp_refiner/graph/legacy_single_yaw/armor_detector_ba_solver_proxy.hpp"

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "armor_pnp_refiner/geometry/camera_model.hpp"
#include "armor_pnp_refiner/geometry/pose_parameterization.hpp"
#include "armor_pnp_refiner/quality/fallback_manager.hpp"

namespace armor_pnp_refiner {

// PIMPL: this package intentionally does not link against armor_detector::BaSolver
// directly to avoid package dependency cycles.
struct LegacySingleYawBaProxy::Impl {
  // Intentionally empty.
};

LegacySingleYawBaProxy::LegacySingleYawBaProxy(const PnpRefinerConfig& config)
  : config_(config), impl_(std::make_unique<Impl>()) {}

LegacySingleYawBaProxy::~LegacySingleYawBaProxy() = default;

PnpRefineOutput LegacySingleYawBaProxy::refine(const PnpRefineInput& input)
{
  // Without direct BaSolver linkage, this proxy must not claim "refined".
  // Return a soft failure and let caller choose fallback behavior.
  (void)input;
  PnpRefineOutput output;
  output.valid = false;
  output.refined = false;
  output.mode = RefineMode::G2O_SINGLE_YAW;
  output.status = RefineStatus::REJECTED;
  output.covariance_valid = false;
  output.covariance_xyz_yaw = FallbackManager::conservativeCovariance();
  output.reason =
      "legacy BaSolver proxy unavailable in this package; use armor_detector integration path";
  return output;
}

}  // namespace armor_pnp_refiner
