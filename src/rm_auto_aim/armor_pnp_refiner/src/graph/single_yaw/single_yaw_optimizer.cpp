#include "armor_pnp_refiner/graph/single_yaw/single_yaw_optimizer.hpp"

#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>

#include "armor_pnp_refiner/graph/common/reprojection_edge.hpp"
#include "armor_pnp_refiner/graph/common/pnp_prior_edge.hpp"
#include "armor_pnp_refiner/graph/common/graph_diagnostics.hpp"
#include "armor_pnp_refiner/geometry/camera_model.hpp"
#include "armor_pnp_refiner/geometry/pose_parameterization.hpp"
#include "armor_pnp_refiner/geometry/structural_priors.hpp"
#include "armor_pnp_refiner/quality/fallback_manager.hpp"

namespace armor_pnp_refiner {

G2O_USE_OPTIMIZATION_LIBRARY(dense)

SingleYawOptimizer::SingleYawOptimizer(const PnpRefinerConfig& config)
  : config_(config) {}

PnpRefineOutput SingleYawOptimizer::refine(const PnpRefineInput& input)
{
  PnpRefineOutput output;
  output.valid = true;

  // Compute raw reprojection error.
  double raw_error = 0.0;
  {
    Eigen::Matrix3d R_ca = geometry::buildCameraArmorRotation(
        input.yaw_rad, input.pitch_rad, input.roll_rad);
    for (size_t j = 0; j < input.image_points.size(); ++j) {
      Eigen::Vector3d p_obj(input.object_points[j].x,
                             input.object_points[j].y,
                             input.object_points[j].z);
      Eigen::Vector3d p_cam = R_ca * p_obj + input.t_camera_armor;
      Eigen::Vector2d proj = geometry::projectPoint(p_cam,
          input.camera_matrix, input.dist_coeffs);
      double dx = input.image_points[j].x - proj.x();
      double dy = input.image_points[j].y - proj.y();
      raw_error += std::sqrt(dx * dx + dy * dy);
    }
    if (!input.image_points.empty())
      raw_error /= static_cast<double>(input.image_points.size());
  }
  output.reproj_error_raw_px = raw_error;
  output.num_points = static_cast<int>(input.image_points.size());

  // Build g2o optimizer.
  g2o::SparseOptimizer optimizer;
  g2o::OptimizationAlgorithmProperty solver_property;
  optimizer.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(
          "lm_dense", solver_property));

  // Add yaw vertex.
  auto* v_yaw = new VertexYaw();
  v_yaw->setId(0);
  v_yaw->setEstimate(input.yaw_rad);
  optimizer.addVertex(v_yaw);

  // Use explicit fixed pitch/roll when requested by caller.
  // NOTE: structural prior conversion between odom/camera should be handled
  // before filling input.fixed_* values.
  double pitch = input.use_fixed_pitch_roll
    ? input.fixed_pitch_rad
    : input.pitch_rad;
  double roll = input.use_fixed_pitch_roll
    ? input.fixed_roll_rad
    : input.roll_rad;

  // Information matrix for pixel observations.
  double sigma_px = config_.pixel_sigma;
  Eigen::Matrix2d info = Eigen::Matrix2d::Identity() * (1.0 / (sigma_px * sigma_px));

  // Add reprojection edges.
  for (size_t j = 0; j < input.image_points.size(); ++j) {
    auto* e = new EdgeYawReprojection();
    e->setVertex(0, v_yaw);
    Eigen::Vector3d p_obj(input.object_points[j].x,
                           input.object_points[j].y,
                           input.object_points[j].z);
    e->setObjectPoint(p_obj);
    e->setFixedPose(input.t_camera_armor, pitch, roll);
    e->setCameraParams(input.camera_matrix, input.dist_coeffs);
    e->setImuCameraRotation(input.R_imu_camera);
    e->setMeasurement(Eigen::Vector2d(input.image_points[j].x,
                                       input.image_points[j].y));
    e->setInformation(info);

    if (config_.use_robust_kernel) {
      auto* rk = g2o::RobustKernelFactory::instance()->construct("Huber");
      if (rk) {
        rk->setDelta(config_.huber_delta_px);
        e->setRobustKernel(rk);
      }
    }
    optimizer.addEdge(e);
  }

  // Add yaw prior.
  {
    auto* e = new EdgeYawPrior();
    e->setVertex(0, v_yaw);
    e->setPnpYaw(input.yaw_rad);
    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
                      (1.0 / (config_.prior_sigma_yaw_rad * config_.prior_sigma_yaw_rad)));
    optimizer.addEdge(e);
  }

  // Optimize.
  optimizer.initializeOptimization();
  double cost_before = optimizer.chi2();
  optimizer.optimize(config_.max_iterations);
  double cost_after = optimizer.chi2();

  double yaw_refined = v_yaw->estimate();
  double yaw_delta = std::abs(geometry::normalizeAngle(yaw_refined - input.yaw_rad));

  // Compute refined reprojection error.
  double refined_error = 0.0;
  {
    Eigen::Matrix3d R_ca = geometry::buildCameraArmorRotation(
        yaw_refined, pitch, roll);
    for (size_t j = 0; j < input.image_points.size(); ++j) {
      Eigen::Vector3d p_obj(input.object_points[j].x,
                             input.object_points[j].y,
                             input.object_points[j].z);
      Eigen::Vector3d p_cam = R_ca * p_obj + input.t_camera_armor;
      Eigen::Vector2d proj = geometry::projectPoint(p_cam,
          input.camera_matrix, input.dist_coeffs);
      double dx = input.image_points[j].x - proj.x();
      double dy = input.image_points[j].y - proj.y();
      refined_error += std::sqrt(dx * dx + dy * dy);
    }
    if (!input.image_points.empty())
      refined_error /= static_cast<double>(input.image_points.size());
  }

  // Fill output.
  output.refined = true;
  output.mode = RefineMode::G2O_SINGLE_YAW;
  output.t_camera_armor = input.t_camera_armor;
  Eigen::Matrix3d R_new = geometry::buildCameraArmorRotation(yaw_refined, pitch, roll);
  output.q_camera_armor = Eigen::Quaterniond(R_new);
  output.yaw_rad = yaw_refined;
  output.pitch_rad = pitch;
  output.roll_rad = roll;
  output.reproj_error_refined_px = refined_error;
  output.yaw_delta_rad = yaw_delta;
  output.pose_delta_m = 0.0;
  output.cost_before = cost_before;
  output.cost_after = cost_after;
  output.covariance_valid = false;
  output.covariance_xyz_yaw = FallbackManager::conservativeCovariance();

  // Fill OpenCV rvec/tvec.
  geometry::eigenToCv(output.q_camera_armor, output.t_camera_armor,
                       output.rvec, output.tvec);

  // Quality checks.
  if (yaw_delta > config_.max_yaw_delta_rad) {
    output.status = RefineStatus::DEGRADED;
    output.reason = "yaw delta exceeded threshold";
  } else if (refined_error > config_.max_reproj_error_px) {
    output.status = RefineStatus::DEGRADED;
    output.reason = "refined reprojection error too high";
  } else {
    output.status = RefineStatus::GOOD;
  }

  return output;
}

}  // namespace armor_pnp_refiner
