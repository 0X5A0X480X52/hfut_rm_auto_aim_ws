#include "armor_detector_nn/core/armor_pose_estimator_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <rm_utils/logger/log.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "armor_detector_nn/core/ba_adjuster.hpp"
#include "armor_detector_nn/core/pose_refine/pose_refiner.hpp"

namespace fyt::auto_aim {

namespace {

// Match armor_detector::rotationMatrixToRPY() semantics:
// 1) convert rvec->R_camera_armor
// 2) transform by fixed R_gimbal_camera
// 3) tf2 getRPY(roll, pitch, yaw)
void rvecToEulerLikeArmorDetector(
    const cv::Mat& rvec, double& yaw, double& pitch, double& roll) {
  cv::Mat R_cv;
  cv::Rodrigues(rvec, R_cv);

  Eigen::Matrix3d R_camera_armor;
  cv::cv2eigen(R_cv, R_camera_armor);

  Eigen::Matrix3d R_gimbal_camera = Eigen::Matrix3d::Identity();
  R_gimbal_camera << 0, 0, 1, -1, 0, 0, 0, -1, 0;
  Eigen::Matrix3d R_gimbal_armor = R_gimbal_camera * R_camera_armor;

  Eigen::Quaterniond q(R_gimbal_armor);
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  tf2::Matrix3x3 m(tf_q);
  m.getRPY(roll, pitch, yaw);
}

double reprojectionErrorSum(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const cv::Mat& rvec,
    const cv::Mat& tvec,
    const cv::Mat& K,
    const cv::Mat& D) {
  std::vector<cv::Point2f> projected;
  cv::projectPoints(object_points, rvec, tvec, K, D, projected);
  double err = 0.0;
  for (size_t j = 0; j < image_points.size(); ++j) {
    err += cv::norm(image_points[j] - projected[j]);
  }
  return err;
}

int selectIppeSolutionLikeArmorDetector(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Mat>& rvecs,
    const std::vector<cv::Mat>& tvecs,
    const cv::Mat& K,
    const cv::Mat& D,
    const std::string& publish_number) {
  // Baseline fallback: best positive-depth solution by reprojection error.
  int best_pos_z = -1;
  int best_any = -1;
  double best_pos_z_err = std::numeric_limits<double>::max();
  double best_any_err = std::numeric_limits<double>::max();
  std::vector<double> errors(rvecs.size(), std::numeric_limits<double>::max());

  for (size_t i = 0; i < rvecs.size(); ++i) {
    errors[i] = reprojectionErrorSum(object_points, image_points, rvecs[i], tvecs[i], K, D);
    double z = tvecs[i].at<double>(2);
    if (z > 0.0 && errors[i] < best_pos_z_err) {
      best_pos_z_err = errors[i];
      best_pos_z = static_cast<int>(i);
    }
    if (errors[i] < best_any_err) {
      best_any_err = errors[i];
      best_any = static_cast<int>(i);
    }
  }
  int fallback = (best_pos_z >= 0) ? best_pos_z : best_any;
  if (rvecs.size() < 2 || image_points.size() < 4) {
    return fallback;
  }

  constexpr double kProjectErrRatioThres = 3.0;
  // Keep armor_detector's current threshold semantics:
  // it compares radians against (10 * 180 / pi), i.e. effectively disabled.
  constexpr double kRollThresRad = 10.0 * 180.0 / M_PI;
  constexpr double kEps = 1e-9;

  double yaw1 = 0.0, pitch1 = 0.0, roll1 = 0.0;
  double yaw2 = 0.0, pitch2 = 0.0, roll2 = 0.0;
  rvecToEulerLikeArmorDetector(rvecs[0], yaw1, pitch1, roll1);
  rvecToEulerLikeArmorDetector(rvecs[1], yaw2, pitch2, roll2);

  const double err1 = errors[0];
  const double err2 = errors[1];
  if ((err2 / std::max(err1, kEps) > kProjectErrRatioThres) ||
      (std::abs(roll1) > kRollThresRad) ||
      (std::abs(roll2) > kRollThresRad)) {
    return fallback;
  }

  // Canonical order: [0]=left_bottom,[1]=left_top,[2]=right_top,[3]=right_bottom.
  const cv::Point2f left_axis = image_points[1] - image_points[0];
  const cv::Point2f right_axis = image_points[2] - image_points[3];
  double l_angle = std::atan2(left_axis.y, left_axis.x) * 180.0 / M_PI;
  double r_angle = std::atan2(right_axis.y, right_axis.x) * 180.0 / M_PI;
  double angle = (l_angle + r_angle) * 0.5 + 90.0;
  if (publish_number == "outpost") {
    angle = -angle;
  }

  // Match armor_detector::sortPnPResult sign disambiguation behavior.
  if ((angle > 0.0 && yaw1 > 0.0 && yaw2 < 0.0) ||
      (angle < 0.0 && yaw1 < 0.0 && yaw2 > 0.0)) {
    if (tvecs[1].at<double>(2) > 0.0) return 1;
    return fallback;
  }
  if (tvecs[0].at<double>(2) > 0.0) return 0;
  return fallback;
}

}  // namespace

ArmorPoseEstimatorAdapter::ArmorPoseEstimatorAdapter(const PoseConfig& config)
  : config_(config)
{
}

ArmorPoseEstimatorAdapter::~ArmorPoseEstimatorAdapter() = default;

void ArmorPoseEstimatorAdapter::setBundleAdjuster(
    std::unique_ptr<IBundleAdjuster> adjuster) {
  ba_adjuster_ = std::move(adjuster);
}

void ArmorPoseEstimatorAdapter::setRefiner(
    std::shared_ptr<IPoseRefiner> refiner) {
  refiner_ = std::move(refiner);
}

PoseEstimate ArmorPoseEstimatorAdapter::estimate(
    const ArmorDetection& detection,
    const sensor_msgs::msg::CameraInfo& camera_info,
    const Eigen::Matrix3d& R_imu_camera)
{
  if (detection.publish_type == "invalid") {
    return PoseEstimate{};
  }

  auto object_pts = getObjectPoints(detection.publish_type,
      config_.small_armor_width, config_.small_armor_height,
      config_.large_armor_width, config_.large_armor_height);

  std::vector<cv::Point2f> image_pts(detection.keypoints.begin(),
                                      detection.keypoints.end());

  cv::Mat K = (cv::Mat_<double>(3, 3)
    << camera_info.k[0], camera_info.k[1], camera_info.k[2],
       camera_info.k[3], camera_info.k[4], camera_info.k[5],
       camera_info.k[6], camera_info.k[7], camera_info.k[8]);

  cv::Mat D;
  if (!camera_info.d.empty()) {
    D = cv::Mat(camera_info.d, true).reshape(1, 1);
  } else {
    D = cv::Mat::zeros(1, 5, CV_64F);
  }

  auto result = solvePnP(image_pts, object_pts, K, D, detection.publish_number);

  if (!result.valid) {
    return result;
  }

  // Propagate tracker/time context before refinement so phase-4 sliding BA
  // can build per-track windows with real timestamps.
  result.track_id = detection.track_id;
  result.observation_stamp = detection.stamp;
  result.publish_number = detection.publish_number;
  result.R_imu_camera = R_imu_camera;

  // Phase 1+: Run refiner (single_yaw or sliding_window) if configured
  if (refiner_ && config_.refiner.mode != "none") {
    std::array<cv::Point2f, 4> img_pts_arr;
    std::copy_n(detection.keypoints.begin(), 4, img_pts_arr.begin());
    auto refined = refiner_->refine(result, img_pts_arr,
        {object_pts[0], object_pts[1], object_pts[2], object_pts[3]}, K, D);
    if (refined.valid &&
        refined.mode >= EstimateMode::SINGLE_BA_VALID) {
      refined.track_id = detection.track_id;
      refined.observation_stamp = detection.stamp;
      refined.publish_number = detection.publish_number;
      refined.R_imu_camera = R_imu_camera;
      return refined;
    }
  }

  // Fallthrough: return PnP result
  result.mode = EstimateMode::PNP_VALID;
  result.quality_score = 0.5;
  result.track_id = detection.track_id;
  result.observation_stamp = detection.stamp;
  result.publish_number = detection.publish_number;
  result.R_imu_camera = R_imu_camera;
  return result;
}

std::vector<PoseEstimate> ArmorPoseEstimatorAdapter::estimateBatch(
    const std::vector<ArmorDetection>& detections,
    const sensor_msgs::msg::CameraInfo& camera_info,
    const Eigen::Matrix3d& R_imu_camera)
{
  std::vector<PoseEstimate> results;
  results.reserve(detections.size());
  for (const auto& d : detections) {
    results.push_back(estimate(d, camera_info, R_imu_camera));
  }
  return results;
}

std::vector<cv::Point3f> ArmorPoseEstimatorAdapter::getObjectPoints(
    const std::string& publish_type,
    double small_w, double small_h,
    double large_w, double large_h)
{
  double w, h;
  if (publish_type == "large") {
    w = large_w;
    h = large_h;
  } else {
    w = small_w;
    h = small_h;
  }

  double half_w = w / 2.0;
  double half_h = h / 2.0;

  // Canonical order: left_bottom, left_top, right_top, right_bottom
  // X forward, Y left, Z up (ROS camera frame convention)
  return {
    cv::Point3f(0.0,  half_w, -half_h),  // left_bottom
    cv::Point3f(0.0,  half_w,  half_h),  // left_top
    cv::Point3f(0.0, -half_w,  half_h),  // right_top
    cv::Point3f(0.0, -half_w, -half_h),  // right_bottom
  };
}

double ArmorPoseEstimatorAdapter::distanceToImageCenter(
    const cv::Point2f& center,
    const cv::Point2f& image_center)
{
  return cv::norm(center - image_center);
}

PoseEstimate ArmorPoseEstimatorAdapter::solvePnP(
    const std::vector<cv::Point2f>& image_points,
    const std::vector<cv::Point3f>& object_points,
    const cv::Mat& camera_matrix,
    const cv::Mat& dist_coeffs,
    const std::string& publish_number)
{
  PoseEstimate result;

  if (image_points.size() != object_points.size() || image_points.size() < 4) {
    return result;
  }

  try {
    if (config_.pnp_method == "ippe") {
      std::vector<cv::Mat> rvecs, tvecs;
      cv::solvePnPGeneric(object_points, image_points,
                          camera_matrix, dist_coeffs,
                          rvecs, tvecs,
                          false,
                          cv::SOLVEPNP_IPPE);

      if (rvecs.empty()) return result;

      int best = selectIppeSolutionLikeArmorDetector(
        object_points, image_points, rvecs, tvecs,
        camera_matrix, dist_coeffs, publish_number);

      if (best >= 0) {
        result.rvec = rvecs[best];
        result.tvec = tvecs[best];
        result.reprojection_error = reprojectionErrorSum(
          object_points, image_points, result.rvec, result.tvec,
          camera_matrix, dist_coeffs);
        result.valid = true;
      }
    } else {
      cv::Mat rvec, tvec;
      if (cv::solvePnP(object_points, image_points,
                       camera_matrix, dist_coeffs,
                       rvec, tvec, false, cv::SOLVEPNP_ITERATIVE)) {
        result.rvec = rvec;
        result.tvec = tvec;
        result.valid = true;

        // Compute reprojection error
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points, rvec, tvec,
                          camera_matrix, dist_coeffs, projected);
        double err = 0.0;
        for (size_t j = 0; j < image_points.size(); ++j) {
          err += cv::norm(image_points[j] - projected[j]);
        }
        result.reprojection_error = err;
      }
    }
  } catch (const cv::Exception& e) {
    FYT_ERROR("armor_detector_nn", "PnP solve failed: %s", e.what());
    return result;
  }

  if (result.valid) {
    result.translation = Eigen::Vector3d(
      result.tvec.at<double>(0),
      result.tvec.at<double>(1),
      result.tvec.at<double>(2));

    cv::Mat R;
    cv::Rodrigues(result.rvec, R);
    Eigen::Matrix3d eigen_R;
    cv::cv2eigen(R, eigen_R);
    result.rotation = Eigen::Quaterniond(eigen_R);

    // Extract yaw/pitch/roll from rvec
    rvecToEulerLikeArmorDetector(
      result.rvec, result.yaw, result.pitch, result.roll);

    // Per-point average reprojection error
    result.reproj_error_raw = result.reprojection_error / 4.0;
    result.reproj_error_refined = result.reproj_error_raw;
    result.mode = EstimateMode::PNP_VALID;

    // Legacy BA refinement path
    if (config_.use_ba && ba_adjuster_) {
      result = ba_adjuster_->refine(result, image_points, object_points,
                                     camera_matrix, dist_coeffs);
    }
  }

  return result;
}

}  // namespace fyt::auto_aim
