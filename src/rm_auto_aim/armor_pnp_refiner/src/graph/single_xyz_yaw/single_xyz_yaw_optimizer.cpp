#include "armor_pnp_refiner/graph/single_xyz_yaw/single_xyz_yaw_optimizer.hpp"

#include <g2o/core/optimization_algorithm_factory.h>
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

namespace armor_pnp_refiner {

G2O_USE_OPTIMIZATION_LIBRARY(dense)

SingleXyzYawOptimizer::SingleXyzYawOptimizer(const PnpRefinerConfig& config)
  : config_(config) {}

PnpRefineOutput SingleXyzYawOptimizer::refine(const PnpRefineInput& input)
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

  // Add xyz-yaw vertex.
  auto* v = new VertexXyzYaw();
  v->setId(0);
  v->setEstimate(Eigen::Vector4d(input.t_camera_armor.x(),
                                  input.t_camera_armor.y(),
                                  input.t_camera_armor.z(),
                                  input.yaw_rad));
  optimizer.addVertex(v);

  // Use explicit fixed pitch/roll when requested by caller.
  // NOTE: structural prior conversion between odom/camera should be handled
  // before filling input.fixed_* values.
  double pitch = input.use_fixed_pitch_roll
    ? input.fixed_pitch_rad
    : input.pitch_rad;
  double roll = input.use_fixed_pitch_roll
    ? input.fixed_roll_rad
    : input.roll_rad;

  // Information for reprojection edges.
  double sigma_px = config_.pixel_sigma;
  Eigen::Matrix2d info_proj = Eigen::Matrix2d::Identity() *
      (1.0 / (sigma_px * sigma_px));

  // Add reprojection edges.
  for (size_t j = 0; j < input.image_points.size(); ++j) {
    auto* e = new EdgeXyzYawReprojection();
    e->setVertex(0, v);
    Eigen::Vector3d p_obj(input.object_points[j].x,
                           input.object_points[j].y,
                           input.object_points[j].z);
    e->setObjectPoint(p_obj);
    e->setCameraParams(input.camera_matrix, input.dist_coeffs);
    e->setFixedOrientation(pitch, roll);
    e->setMeasurement(Eigen::Vector2d(input.image_points[j].x,
                                       input.image_points[j].y));
    e->setInformation(info_proj);

    if (config_.use_robust_kernel) {
      auto* rk = g2o::RobustKernelFactory::instance()->construct("Huber");
      if (rk) {
        rk->setDelta(config_.huber_delta_px);
        e->setRobustKernel(rk);
      }
    }
    optimizer.addEdge(e);
  }

  // Translation prior.
  Eigen::Matrix3d info_t = Eigen::Matrix3d::Identity();
  info_t(0, 0) = 1.0 / (config_.prior_sigma_xy * config_.prior_sigma_xy);
  info_t(1, 1) = 1.0 / (config_.prior_sigma_xy * config_.prior_sigma_xy);
  info_t(2, 2) = 1.0 / (config_.prior_sigma_z  * config_.prior_sigma_z);

  {
    auto* e = new EdgeTranslationPrior();
    e->setVertex(0, v);
    e->setPnpTranslation(input.t_camera_armor);
    e->setInformation(info_t);
    optimizer.addEdge(e);
  }

  // Yaw prior.
  {
    auto* e = new EdgeXyzYawPrior();
    e->setVertex(0, v);
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

  // Extract result.
  const Eigen::Vector4d& est = v->estimate();
  Eigen::Vector3d t_refined(est[0], est[1], est[2]);
  double yaw_refined = est[3];

  // Compute refined reprojection error.
  double refined_error = 0.0;
  int num_inliers = 0;
  {
    Eigen::Matrix3d R_ca = geometry::buildCameraArmorRotation(
        yaw_refined, pitch, roll);
    for (size_t j = 0; j < input.image_points.size(); ++j) {
      Eigen::Vector3d p_obj(input.object_points[j].x,
                             input.object_points[j].y,
                             input.object_points[j].z);
      Eigen::Vector3d p_cam = R_ca * p_obj + t_refined;
      Eigen::Vector2d proj = geometry::projectPoint(p_cam,
          input.camera_matrix, input.dist_coeffs);
      double dx = input.image_points[j].x - proj.x();
      double dy = input.image_points[j].y - proj.y();
      double err = std::sqrt(dx * dx + dy * dy);
      refined_error += err;
      if (err < config_.huber_delta_px) ++num_inliers;
    }
    if (!input.image_points.empty())
      refined_error /= static_cast<double>(input.image_points.size());
  }

  double pose_delta = (t_refined - input.t_camera_armor).norm();
  double yaw_delta = std::abs(geometry::normalizeAngle(yaw_refined - input.yaw_rad));

  int num_residuals = static_cast<int>(input.image_points.size()) * 2 + 3 + 1;
  int num_params = 4;
  int dof = num_residuals - num_params;
  double chi2_per_dof = (dof > 0) ? cost_after / dof : cost_after;

  // Fill output.
  output.refined = true;
  output.mode = RefineMode::G2O_SINGLE_XYZ_YAW;
  output.t_camera_armor = t_refined;
  Eigen::Matrix3d R_new = geometry::buildCameraArmorRotation(yaw_refined, pitch, roll);
  output.q_camera_armor = Eigen::Quaterniond(R_new);
  output.yaw_rad = yaw_refined;
  output.pitch_rad = pitch;
  output.roll_rad = roll;
  output.reproj_error_refined_px = refined_error;
  output.yaw_delta_rad = yaw_delta;
  output.pose_delta_m = pose_delta;
  output.cost_before = cost_before;
  output.cost_after = cost_after;
  output.chi2_per_dof = chi2_per_dof;
  output.num_inliers = num_inliers;

  // Conservative diagonal covariance (placeholder, not Hessian marginal).
  double scale = std::max(1.0, chi2_per_dof);
  output.covariance_xyz_yaw.setIdentity();
  output.covariance_xyz_yaw(0, 0) = scale * config_.min_var_x;
  output.covariance_xyz_yaw(1, 1) = scale * config_.min_var_y;
  output.covariance_xyz_yaw(2, 2) = scale * config_.min_var_z;
  output.covariance_xyz_yaw(3, 3) = scale * config_.min_var_yaw;
  output.covariance_valid = false;

  geometry::eigenToCv(output.q_camera_armor, output.t_camera_armor,
                       output.rvec, output.tvec);

  // Quality.
  if (yaw_delta > config_.max_yaw_delta_rad || pose_delta > config_.max_pose_delta_m) {
    output.status = RefineStatus::DEGRADED;
    output.reason = "pose/yaw delta exceeded threshold";
  } else if (refined_error > config_.max_reproj_error_px) {
    output.status = RefineStatus::DEGRADED;
    output.reason = "refined reprojection error too high";
  } else {
    output.status = RefineStatus::GOOD;
  }

  return output;
}

}  // namespace armor_pnp_refiner
