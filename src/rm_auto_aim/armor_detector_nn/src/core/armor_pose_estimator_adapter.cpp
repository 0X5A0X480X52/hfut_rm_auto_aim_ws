#include "armor_detector_nn/core/armor_pose_estimator_adapter.hpp"

#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <rm_utils/logger/log.hpp>

#include "armor_detector_nn/core/ba_adjuster.hpp"
#include "armor_detector_nn/core/pose_refine/pose_refiner.hpp"

namespace fyt::auto_aim {

namespace {

// Extract yaw/pitch/roll from rvec (Rodrigues vector).
// Uses the R = Rz(yaw)*Ry(pitch)*Rx(roll) decomposition.
void rvecToEuler(const cv::Mat& rvec, double& yaw, double& pitch, double& roll) {
  cv::Mat R;
  cv::Rodrigues(rvec, R);

  double sp = -R.at<double>(2, 0);
  sp = std::clamp(sp, -1.0, 1.0);
  pitch = std::asin(sp);

  double cp = std::cos(pitch);
  if (std::abs(cp) < 1e-8) {
    roll = 0.0;
    yaw = std::atan2(-R.at<double>(0, 1), R.at<double>(1, 1));
  } else {
    yaw  = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
  }
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
    const sensor_msgs::msg::CameraInfo& camera_info)
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

  auto result = solvePnP(image_pts, object_pts, K, D);

  if (!result.valid) {
    return result;
  }

  // Phase 1+: Run refiner (single_yaw or sliding_window) if configured
  if (refiner_ && config_.refiner.mode != "none") {
    std::array<cv::Point2f, 4> img_pts_arr;
    std::copy_n(detection.keypoints.begin(), 4, img_pts_arr.begin());
    auto refined = refiner_->refine(result, img_pts_arr,
        {object_pts[0], object_pts[1], object_pts[2], object_pts[3]}, K, D);
    if (refined.valid &&
        refined.mode >= EstimateMode::SINGLE_BA_VALID) {
      refined.track_id = detection.track_id;
      return refined;
    }
  }

  // Fallthrough: return PnP result
  result.mode = EstimateMode::PNP_VALID;
  result.quality_score = 0.5;
  result.track_id = detection.track_id;
  return result;
}

std::vector<PoseEstimate> ArmorPoseEstimatorAdapter::estimateBatch(
    const std::vector<ArmorDetection>& detections,
    const sensor_msgs::msg::CameraInfo& camera_info)
{
  std::vector<PoseEstimate> results;
  results.reserve(detections.size());
  for (const auto& d : detections) {
    results.push_back(estimate(d, camera_info));
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
    const cv::Mat& dist_coeffs)
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

      int best = -1;
      double best_err = std::numeric_limits<double>::max();
      std::vector<double> errors(rvecs.size());

      for (size_t i = 0; i < rvecs.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(object_points, rvecs[i], tvecs[i],
                          camera_matrix, dist_coeffs, projected);
        double err = 0.0;
        for (size_t j = 0; j < image_points.size(); ++j) {
          err += cv::norm(image_points[j] - projected[j]);
        }
        errors[i] = err;

        double z = tvecs[i].at<double>(2);
        if (z > 0 && err < best_err) {
          best_err = err;
          best = static_cast<int>(i);
        }
      }

      // Fallback: if no solution has z > 0, pick global best reprojection error
      if (best < 0) {
        for (size_t i = 0; i < rvecs.size(); ++i) {
          if (errors[i] < best_err) {
            best_err = errors[i];
            best = static_cast<int>(i);
          }
        }
      }

      if (best >= 0) {
        result.rvec = rvecs[best];
        result.tvec = tvecs[best];
        result.reprojection_error = best_err;
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
    rvecToEuler(result.rvec, result.yaw, result.pitch, result.roll);

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
