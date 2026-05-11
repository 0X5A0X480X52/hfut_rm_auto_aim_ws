#include "armor_pnp_refiner/quality/fallback_manager.hpp"

#include "armor_pnp_refiner/geometry/pose_parameterization.hpp"

namespace armor_pnp_refiner {

FallbackManager::FallbackManager(const PnpRefinerConfig& config)
  : config_(config) {}

PnpRefineOutput FallbackManager::fallbackToRawPnp(const PnpRefineInput& input,
                                                    const std::string& reason)
{
  auto out = buildRawPnpOutput(input);
  out.reason = reason;
  return out;
}

PnpRefineOutput FallbackManager::fallbackToDegraded(const PnpRefineOutput& refined,
                                                      const PnpRefineInput& input,
                                                      const std::string& reason)
{
  PnpRefineOutput out = refined;
  out.status = RefineStatus::DEGRADED;
  out.reason = (out.reason.empty() ? "" : out.reason + "; ") + reason;
  // Inflate covariance.
  out.covariance_xyz_yaw *= 2.0;
  return out;
}

PnpRefineOutput FallbackManager::buildRawPnpOutput(const PnpRefineInput& input)
{
  PnpRefineOutput out;
  out.valid = true;
  out.refined = false;
  out.mode = RefineMode::PNP_FALLBACK;
  out.status = RefineStatus::GOOD;

  out.t_camera_armor = input.t_camera_armor;
  out.q_camera_armor = input.q_camera_armor;
  out.yaw_rad = input.yaw_rad;
  out.pitch_rad = input.pitch_rad;
  out.roll_rad = input.roll_rad;

  out.covariance_xyz_yaw = conservativeCovariance();
  out.covariance_valid = false;
  out.confidence = 0.5;

  out.reproj_error_raw_px = 0.0;   // not computed
  out.reproj_error_refined_px = 0.0;
  out.num_points = static_cast<int>(input.image_points.size());

  geometry::eigenToCv(out.q_camera_armor, out.t_camera_armor,
                       out.rvec, out.tvec);

  return out;
}

Eigen::Matrix4d FallbackManager::conservativeCovariance(
    double var_xy, double var_z, double var_yaw)
{
  Eigen::Matrix4d cov = Eigen::Matrix4d::Identity();
  cov(0, 0) = var_xy;
  cov(1, 1) = var_xy;
  cov(2, 2) = var_z;
  cov(3, 3) = var_yaw;
  return cov;
}

}  // namespace armor_pnp_refiner
