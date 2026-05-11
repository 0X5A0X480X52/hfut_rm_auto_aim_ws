#include "armor_pnp_refiner/graph/sliding_window/sliding_window_optimizer.hpp"

#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>

#include "armor_pnp_refiner/graph/common/reprojection_edge.hpp"
#include "armor_pnp_refiner/graph/common/pnp_prior_edge.hpp"
#include "armor_pnp_refiner/graph/common/graph_diagnostics.hpp"
#include "armor_pnp_refiner/graph/sliding_window/temporal_prior_edges.hpp"
#include "armor_pnp_refiner/geometry/camera_model.hpp"
#include "armor_pnp_refiner/geometry/pose_parameterization.hpp"
#include "armor_pnp_refiner/geometry/structural_priors.hpp"

namespace armor_pnp_refiner {

G2O_USE_OPTIMIZATION_LIBRARY(dense)

// ── SlidingWindowOptimizer ──────────────────────────────

SlidingWindowOptimizer::SlidingWindowOptimizer(const PnpRefinerConfig& config)
  : config_(config) {}

PnpRefineOutput SlidingWindowOptimizer::refine(const std::vector<WindowFrame>& window)
{
  PnpRefineOutput output;
  output.valid = true;
  int N = static_cast<int>(window.size());

  if (N < config_.min_window_size) {
    output.refined = false;
    output.mode = RefineMode::PNP_FALLBACK;
    output.reason = "window too small";
    return output;
  }

  // Build optimizer.
  g2o::SparseOptimizer optimizer;
  g2o::OptimizationAlgorithmProperty solver_property;
  optimizer.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(
          "lm_dense", solver_property));

  const double sigma_px = config_.pixel_sigma;
  const Eigen::Matrix2d info_proj = Eigen::Matrix2d::Identity() *
      (1.0 / (sigma_px * sigma_px));

  // Add vertices and per-frame edges.
  for (int i = 0; i < N; ++i) {
    auto* v = new VertexXyzYaw();
    v->setId(i);
    const auto& frame = window[i];
    double pitch = frame.input.use_fixed_pitch_roll
      ? frame.input.fixed_pitch_rad
      : frame.input.pitch_rad;
    double roll = frame.input.use_fixed_pitch_roll
      ? frame.input.fixed_roll_rad
      : frame.input.roll_rad;

    v->setEstimate(Eigen::Vector4d(
        frame.input.t_camera_armor.x(),
        frame.input.t_camera_armor.y(),
        frame.input.t_camera_armor.z(),
        frame.input.yaw_rad));
    optimizer.addVertex(v);

    // Reprojection edges.
    for (size_t j = 0; j < frame.input.image_points.size(); ++j) {
      auto* e = new EdgeXyzYawReprojection();
      e->setVertex(0, v);
      Eigen::Vector3d p_obj(frame.input.object_points[j].x,
                             frame.input.object_points[j].y,
                             frame.input.object_points[j].z);
      e->setObjectPoint(p_obj);
      e->setCameraParams(frame.input.camera_matrix, frame.input.dist_coeffs);
      e->setFixedOrientation(pitch, roll);
      e->setMeasurement(Eigen::Vector2d(
          frame.input.image_points[j].x,
          frame.input.image_points[j].y));
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

    // PnP translation prior.
    {
      Eigen::Matrix3d info_t = Eigen::Matrix3d::Identity();
      info_t(0, 0) = 1.0 / (config_.prior_sigma_xy * config_.prior_sigma_xy);
      info_t(1, 1) = 1.0 / (config_.prior_sigma_xy * config_.prior_sigma_xy);
      info_t(2, 2) = 1.0 / (config_.prior_sigma_z  * config_.prior_sigma_z);

      auto* e = new EdgeTranslationPrior();
      e->setVertex(0, v);
      e->setPnpTranslation(frame.input.t_camera_armor);
      e->setInformation(info_t);
      optimizer.addEdge(e);
    }

    // Yaw prior.
    {
      auto* e = new EdgeXyzYawPrior();
      e->setVertex(0, v);
      e->setPnpYaw(frame.input.yaw_rad);
      e->setInformation(Eigen::Matrix<double, 1, 1>::Identity() *
          (1.0 / (config_.prior_sigma_yaw_rad * config_.prior_sigma_yaw_rad)));
      optimizer.addEdge(e);
    }
  }

  // Second-order smooth edges.
  if (N >= 3) {
    Eigen::Matrix4d info_acc = Eigen::Matrix4d::Identity();
    info_acc(0, 0) = 1.0 / (config_.acc_sigma_xy  * config_.acc_sigma_xy);
    info_acc(1, 1) = 1.0 / (config_.acc_sigma_xy  * config_.acc_sigma_xy);
    info_acc(2, 2) = 1.0 / (config_.acc_sigma_z   * config_.acc_sigma_z);
    info_acc(3, 3) = 1.0 / (config_.acc_sigma_yaw_rad * config_.acc_sigma_yaw_rad);

    for (int i = 2; i < N; ++i) {
      auto* e = new EdgeSecondOrderSmooth();
      e->setVertex(0, optimizer.vertex(i - 2));
      e->setVertex(1, optimizer.vertex(i - 1));
      e->setVertex(2, optimizer.vertex(i));
      e->setInformation(info_acc);
      optimizer.addEdge(e);
    }
  }

  // Third-order smooth (optional).
  if (config_.enable_jerk_smooth && N >= 4) {
    Eigen::Matrix4d info_jerk = Eigen::Matrix4d::Identity();
    info_jerk(0, 0) = 1.0 / (config_.jerk_sigma_xy  * config_.jerk_sigma_xy);
    info_jerk(1, 1) = 1.0 / (config_.jerk_sigma_xy  * config_.jerk_sigma_xy);
    info_jerk(2, 2) = 1.0 / (config_.jerk_sigma_z   * config_.jerk_sigma_z);
    info_jerk(3, 3) = 1.0 / (config_.jerk_sigma_yaw_rad * config_.jerk_sigma_yaw_rad);

    for (int i = 3; i < N; ++i) {
      auto* e = new EdgeThirdOrderSmooth();
      e->setVertex(0, optimizer.vertex(i - 3));
      e->setVertex(1, optimizer.vertex(i - 2));
      e->setVertex(2, optimizer.vertex(i - 1));
      e->setVertex(3, optimizer.vertex(i));
      e->setInformation(info_jerk);
      optimizer.addEdge(e);
    }
  }

  // Optimize.
  optimizer.initializeOptimization();
  double cost_before = optimizer.chi2();
  optimizer.optimize(config_.max_iterations);
  double cost_after = optimizer.chi2();

  // Extract current (last) frame result.
  auto* v_current = dynamic_cast<VertexXyzYaw*>(optimizer.vertex(N - 1));
  const Eigen::Vector4d& est = v_current->estimate();
  Eigen::Vector3d t_refined(est[0], est[1], est[2]);
  double yaw_refined = est[3];

  const auto& last_input = window[N - 1].input;
  double pitch = last_input.use_fixed_pitch_roll
    ? last_input.fixed_pitch_rad
    : last_input.pitch_rad;
  double roll = last_input.use_fixed_pitch_roll
    ? last_input.fixed_roll_rad
    : last_input.roll_rad;

  // Compute refined reprojection error for current frame.
  double refined_error = 0.0;
  int num_inliers = 0;
  {
    Eigen::Matrix3d R_ca = geometry::buildCameraArmorRotation(yaw_refined, pitch, roll);
    for (size_t j = 0; j < last_input.image_points.size(); ++j) {
      Eigen::Vector3d p_obj(last_input.object_points[j].x,
                             last_input.object_points[j].y,
                             last_input.object_points[j].z);
      Eigen::Vector3d p_cam = R_ca * p_obj + t_refined;
      Eigen::Vector2d proj = geometry::projectPoint(p_cam,
          last_input.camera_matrix, last_input.dist_coeffs);
      double dx = last_input.image_points[j].x - proj.x();
      double dy = last_input.image_points[j].y - proj.y();
      double err = std::sqrt(dx * dx + dy * dy);
      refined_error += err;
      if (err < config_.huber_delta_px) ++num_inliers;
    }
    if (!last_input.image_points.empty())
      refined_error /= static_cast<double>(last_input.image_points.size());
  }

  double pose_delta = (t_refined - last_input.t_camera_armor).norm();
  double yaw_delta = std::abs(geometry::normalizeAngle(yaw_refined - last_input.yaw_rad));

  output.refined = true;
  output.mode = RefineMode::G2O_WINDOW_XYZ_YAW;
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
  output.window_size = N;
  output.num_points = static_cast<int>(last_input.image_points.size());
  output.num_inliers = num_inliers;

  // Conservative covariance placeholder (not Schur-marginal covariance yet).
  double scale = std::max(1.0, cost_after / std::max(1.0, static_cast<double>(N * 4)));
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
    output.reason = "pose/yaw delta exceeded threshold in window";
  } else if (refined_error > config_.max_reproj_error_px) {
    output.status = RefineStatus::DEGRADED;
    output.reason = "window refined reprojection error too high";
  } else {
    output.status = RefineStatus::GOOD;
  }

  return output;
}

// ── PnpRefineWindowManager ──────────────────────────────

PnpRefineWindowManager::PnpRefineWindowManager(const PnpRefinerConfig& config)
  : config_(config) {}

void PnpRefineWindowManager::push(int refine_track_id, const PnpRefineInput& input)
{
  auto& win = getOrCreateWindow(refine_track_id);

  WindowFrame frame;
  frame.frame_index = win.size();
  frame.stamp_sec = input.stamp_sec;
  frame.input = input;
  frame.xyz_yaw_initial = Eigen::Vector4d(
      input.t_camera_armor.x(),
      input.t_camera_armor.y(),
      input.t_camera_armor.z(),
      input.yaw_rad);

  win.frames.push_back(frame);
  win.hits++;
  win.misses = 0;
  win.last_stamp_sec = input.stamp_sec;

  if (win.hits >= config_.min_confirm_hits) {
    win.confirmed = true;
  }

  while (win.size() > config_.window_size) {
    win.frames.erase(win.frames.begin());
  }
}

std::vector<WindowFrame> PnpRefineWindowManager::getWindow(int refine_track_id) const
{
  const auto* win = findWindow(refine_track_id);
  if (!win) return {};
  return win->frames;
}

void PnpRefineWindowManager::resetTrack(int refine_track_id)
{
  auto it = std::find_if(windows_.begin(), windows_.end(),
      [refine_track_id](const RefineWindow& w) {
        return w.refine_track_id == refine_track_id;
      });
  if (it != windows_.end()) {
    windows_.erase(it);
  }
}

void PnpRefineWindowManager::pruneExpired(double now_sec)
{
  windows_.erase(
      std::remove_if(windows_.begin(), windows_.end(),
          [this, now_sec](const RefineWindow& w) {
            return w.isExpired(config_.max_time_gap_ms, now_sec);
          }),
      windows_.end());
}

void PnpRefineWindowManager::resetAll()
{
  windows_.clear();
}

int PnpRefineWindowManager::windowSize(int refine_track_id) const
{
  const auto* win = findWindow(refine_track_id);
  return win ? win->size() : 0;
}

RefineWindow* PnpRefineWindowManager::findWindow(int refine_track_id)
{
  auto it = std::find_if(windows_.begin(), windows_.end(),
      [refine_track_id](const RefineWindow& w) {
        return w.refine_track_id == refine_track_id;
      });
  return (it != windows_.end()) ? &(*it) : nullptr;
}

const RefineWindow* PnpRefineWindowManager::findWindow(int refine_track_id) const
{
  auto it = std::find_if(windows_.begin(), windows_.end(),
      [refine_track_id](const RefineWindow& w) {
        return w.refine_track_id == refine_track_id;
      });
  return (it != windows_.end()) ? &(*it) : nullptr;
}

RefineWindow& PnpRefineWindowManager::getOrCreateWindow(int refine_track_id)
{
  auto* win = findWindow(refine_track_id);
  if (!win) {
    RefineWindow new_win;
    new_win.refine_track_id = refine_track_id;
    windows_.push_back(new_win);
    win = &windows_.back();
  }
  return *win;
}

}  // namespace armor_pnp_refiner
