#include "armor_detector_nn/core/pose_refine/pose_refiner.hpp"

#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <rm_utils/logger/log.hpp>

namespace fyt::auto_aim {

namespace {

// Convert yaw/pitch/roll (rad) to a 3×3 rotation matrix.
// R = Rz(yaw) * Ry(pitch) * Rx(roll)
// X forward, Y left, Z up (ROS camera frame convention).
cv::Mat yawPitchRollToMatrix(double yaw, double pitch, double roll) {
  double cy = std::cos(yaw), sy = std::sin(yaw);
  double cp = std::cos(pitch), sp = std::sin(pitch);
  double cr = std::cos(roll), sr = std::sin(roll);

  cv::Mat R = (cv::Mat_<double>(3, 3)
    << cy * cp,  cy * sp * sr - sy * cr,  cy * sp * cr + sy * sr,
       sy * cp,  sy * sp * sr + cy * cr,  sy * sp * cr - cy * sr,
       -sp,      cp * sr,                 cp * cr);
  return R;
}

// Convert yaw/pitch/roll to a Rodrigues rotation vector.
cv::Mat yawPitchRollToRvec(double yaw, double pitch, double roll) {
  cv::Mat R = yawPitchRollToMatrix(yaw, pitch, roll);
  cv::Mat rvec;
  cv::Rodrigues(R, rvec);
  return rvec;
}

// Project a 3D point into the image plane.
inline cv::Point2d projectPoint(const cv::Vec3d& P_cam, double fx, double fy,
                                double cx, double cy) {
  double inv_z = 1.0 / P_cam[2];
  return {fx * P_cam[0] * inv_z + cx,
          fy * P_cam[1] * inv_z + cy};
}

}  // namespace

SingleYawRefiner::SingleYawRefiner(const SingleYawConfig& config,
                                   const GateConfig& gate)
  : config_(config), gate_(gate)
{
}

double SingleYawRefiner::angleWrap(double angle) {
  while (angle > M_PI)  angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

double SingleYawRefiner::computeReprojError(
    double yaw,
    const cv::Vec3d& tvec,
    double pitch,
    double roll,
    const std::array<cv::Point2f, 4>& image_points,
    const std::array<cv::Point3f, 4>& object_points,
    const cv::Mat& K)
{
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  cv::Mat R_wc = yawPitchRollToMatrix(yaw, pitch, roll);
  double total = 0.0;

  for (int i = 0; i < 4; ++i) {
    cv::Vec3d P_obj(object_points[i].x, object_points[i].y, object_points[i].z);
    cv::Mat P_cam_mat = R_wc * cv::Mat(P_obj) + cv::Mat(tvec);
    cv::Vec3d P_cam(P_cam_mat.at<double>(0), P_cam_mat.at<double>(1), P_cam_mat.at<double>(2));

    if (P_cam[2] <= 1e-6) return 1e9;

    cv::Point2d proj = projectPoint(P_cam, fx, fy, cx, cy);
    double dx = image_points[i].x - proj.x;
    double dy = image_points[i].y - proj.y;
    double r2 = dx * dx + dy * dy;

    // Huber loss
    if (r2 > config_.huber_delta * config_.huber_delta) {
      r2 = 2.0 * config_.huber_delta * std::sqrt(r2)
           - config_.huber_delta * config_.huber_delta;
    }
    total += r2;
  }

  return total;
}

bool SingleYawRefiner::optimizeYaw(
    double& yaw,
    const cv::Vec3d& tvec,
    double pitch,
    double roll,
    const std::array<cv::Point2f, 4>& image_points,
    const std::array<cv::Point3f, 4>& object_points,
    const cv::Mat& K)
{
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);

  for (int iter = 0; iter < config_.max_iterations; ++iter) {
    double J_sum = 0.0;
    double b_sum = 0.0;
    double cost = 0.0;

    cv::Mat R_wc = yawPitchRollToMatrix(yaw, pitch, roll);

    for (int i = 0; i < 4; ++i) {
      cv::Vec3d P_obj(object_points[i].x, object_points[i].y, object_points[i].z);
      cv::Mat P_cam_mat = R_wc * cv::Mat(P_obj) + cv::Mat(tvec);
      cv::Vec3d P_cam(P_cam_mat.at<double>(0), P_cam_mat.at<double>(1), P_cam_mat.at<double>(2));

      if (P_cam[2] <= 1e-6) return false;

      double inv_z = 1.0 / P_cam[2];
      double u_proj = fx * P_cam[0] * inv_z + cx;
      double v_proj = fy * P_cam[1] * inv_z + cy;

      double du = image_points[i].x - u_proj;
      double dv = image_points[i].y - v_proj;
      double r2 = du * du + dv * dv;

      // Huber weight
      double rho = 1.0;
      if (r2 > config_.huber_delta * config_.huber_delta) {
        rho = config_.huber_delta / std::sqrt(r2);
      }
      cost += rho * r2;

      // Jacobian: dP_cam / dyaw.
      // dR/dyaw = Rz'(yaw) * Ry * Rx = skew([0,0,1]) * R(yaw,pitch,roll)
      // So dP_cam/dyaw = [-P_cam.y, P_cam.x, 0]^T  in camera frame
      double dX_dyaw = -P_cam[1];
      double dY_dyaw =  P_cam[0];
      double dZ_dyaw = 0.0;

      double du_dyaw = fx * inv_z * (dX_dyaw - P_cam[0] * inv_z * dZ_dyaw);
      double dv_dyaw = fy * inv_z * (dY_dyaw - P_cam[1] * inv_z * dZ_dyaw);

      J_sum += rho * (du_dyaw * du_dyaw + dv_dyaw * dv_dyaw);
      b_sum += rho * (du_dyaw * du + dv_dyaw * dv);
    }

    if (std::abs(J_sum) < 1e-10) return true;

    double delta = b_sum / J_sum;

    // Line search with step decay
    double alpha = 1.0;
    for (int ls = 0; ls < 10; ++ls) {
      double yaw_try = yaw - alpha * delta;
      double cost_try = computeReprojError(yaw_try, tvec, pitch, roll,
                                           image_points, object_points, K);
      if (cost_try < cost) {
        yaw = yaw_try;
        break;
      }
      alpha *= 0.5;
    }

    if (std::abs(delta) < 1e-6) return true;
  }

  return true;
}

PoseEstimate SingleYawRefiner::refine(
    const PoseEstimate& pnp_result,
    const std::array<cv::Point2f, 4>& image_points,
    const std::array<cv::Point3f, 4>& object_points,
    const cv::Mat& K,
    const cv::Mat& /*D*/)
{
  PoseEstimate result = pnp_result;

  // Step 1: determine pitch / roll
  double pitch = pnp_result.pitch;
  double roll  = pnp_result.roll;
  if (std::isnan(pitch) || std::abs(pitch) < 1e-4) {
    pitch = config_.pitch_deg_default * M_PI / 180.0;
  }
  if (std::isnan(roll) || std::abs(roll) < 1e-4) {
    roll = config_.roll_deg_default * M_PI / 180.0;
  }

  // Step 2: yaw initial value from PnP
  double yaw_init = pnp_result.yaw;
  double yaw_opt  = yaw_init;

  // Step 3: 1-D yaw optimization
  cv::Vec3d tvec(pnp_result.tvec.at<double>(0),
                 pnp_result.tvec.at<double>(1),
                 pnp_result.tvec.at<double>(2));
  bool ok = optimizeYaw(yaw_opt, tvec, pitch, roll,
                        image_points, object_points, K);

  // Step 4: validity checks
  if (!ok || std::isnan(yaw_opt) || !std::isfinite(yaw_opt)) {
    FYT_DEBUG("armor_detector_nn",
              "SingleYawRefiner: optimization failed or produced NaN");
    result.mode = EstimateMode::PNP_VALID;
    result.quality_score = 0.5;
    result.reproj_error_refined = result.reproj_error_raw;
    return result;
  }

  // Step 5: check yaw delta
  double delta_yaw = std::abs(angleWrap(yaw_opt - yaw_init));
  if (delta_yaw > gate_.max_yaw_delta_deg * M_PI / 180.0) {
    FYT_DEBUG("armor_detector_nn",
              "SingleYawRefiner: yaw delta too large (%.2f deg)",
              delta_yaw * 180.0 / M_PI);
    result.mode = EstimateMode::PNP_VALID;
    result.quality_score = 0.5;
    result.reproj_error_refined = result.reproj_error_raw;
    return result;
  }

  // Step 6: quality gate
  double refined_error = computeReprojError(yaw_opt, tvec, pitch, roll,
                                             image_points, object_points, K);
  if (gate_.require_finite && !std::isfinite(refined_error)) {
    result.mode = EstimateMode::PNP_VALID;
    result.quality_score = 0.5;
    result.reproj_error_refined = result.reproj_error_raw;
    return result;
  }
  if (refined_error > gate_.max_reproj_error) {
    result.mode = EstimateMode::PNP_VALID;
    result.quality_score =
      std::max(0.0, 1.0 - refined_error / gate_.max_reproj_error);
    result.reproj_error_refined = result.reproj_error_raw;
    return result;
  }

  // Step 7: success — update result
  result.yaw = yaw_opt;
  result.rvec = yawPitchRollToRvec(yaw_opt, pitch, roll);
  result.mode = EstimateMode::SINGLE_BA_VALID;
  result.reproj_error_refined = refined_error / 4.0;  // per-point
  result.reproj_error_raw = pnp_result.reproj_error_raw;  // preserve
  result.quality_score =
    std::max(0.0, 1.0 - refined_error / (4.0 * gate_.max_reproj_error));

  // Recompute rotation quaternion and translation
  result.translation = Eigen::Vector3d(tvec[0], tvec[1], tvec[2]);
  cv::Mat R;
  cv::Rodrigues(result.rvec, R);
  Eigen::Matrix3d eigen_R;
  cv::cv2eigen(R, eigen_R);
  result.rotation = Eigen::Quaterniond(eigen_R);

  return result;
}

}  // namespace fyt::auto_aim
