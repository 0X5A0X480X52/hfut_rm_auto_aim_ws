#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

namespace armor_pnp_refiner {

enum class RefineMode {
  PNP_FALLBACK = 0,
  G2O_SINGLE_YAW,
  G2O_SINGLE_XYZ_YAW,
  G2O_WINDOW_XYZ_YAW,
  G2O_WINDOW_POSE3_EXPERIMENTAL
};

enum class RefineStatus {
  GOOD = 0,
  DEGRADED,
  REJECTED
};

enum class ArmorSizeType {
  SMALL = 0,
  LARGE,
  OUTPOST
};

struct PnpRefineInput {
  // Raw PnP result in camera frame.
  Eigen::Vector3d t_camera_armor{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_camera_armor{Eigen::Quaterniond::Identity()};

  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double roll_rad{0.0};

  // OpenCV solvePnP raw outputs.
  cv::Mat rvec;
  cv::Mat tvec;

  // 2D-3D correspondences. image_points[i] must match object_points[i].
  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point3f> object_points;

  // Per-keypoint pixel sigma. If empty, use config.pixel_sigma.
  std::vector<double> keypoint_sigma_px;

  // Camera intrinsics.
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;

  // Detection metadata for short-term association.
  double stamp_sec{0.0};
  cv::Rect2f bbox{};
  cv::Point2f center{};
  double detection_confidence{1.0};
  std::string armor_number;
  ArmorSizeType armor_type{ArmorSizeType::SMALL};

  // Optional external track id. armor_detector_nn provides this.
  std::optional<int> external_track_id{std::nullopt};

  // IMU-camera extrinsics for yaw convention compatibility.
  Eigen::Matrix3d R_imu_camera{Eigen::Matrix3d::Identity()};
  bool use_fixed_pitch_roll{true};
  double fixed_pitch_rad{0.0};
  double fixed_roll_rad{0.0};
};

struct PnpCovarianceDiagnostics {
  bool covariance_valid{false};
  Eigen::Matrix4d covariance_xyz_yaw{Eigen::Matrix4d::Identity()};
  double condition_number{0.0};
  double chi2_per_dof{0.0};
  double reproj_rms{0.0};
  double residual_scale{1.0};
  int num_observations{0};
  int num_inliers{0};
};

struct PnpRefineOutput {
  bool valid{false};
  bool refined{false};

  RefineMode mode{RefineMode::PNP_FALLBACK};
  RefineStatus status{RefineStatus::REJECTED};

  Eigen::Vector3d t_camera_armor{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_camera_armor{Eigen::Quaterniond::Identity()};
  cv::Mat rvec;
  cv::Mat tvec;

  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double roll_rad{0.0};

  Eigen::Matrix4d covariance_xyz_yaw{Eigen::Matrix4d::Identity()};
  bool covariance_valid{false};
  double confidence{0.0};

  double reproj_error_raw_px{0.0};
  double reproj_error_refined_px{0.0};
  double yaw_delta_rad{0.0};
  double pose_delta_m{0.0};
  double chi2_per_dof{0.0};
  double condition_number{0.0};
  double cost_before{0.0};
  double cost_after{0.0};
  double solve_time_ms{0.0};

  int refine_track_id{-1};
  int window_size{0};
  int num_points{0};
  int num_inliers{0};

  std::string reason;
};

struct ObservationCovarianceMeta {
  bool valid{false};
  bool cov_valid{false};
  Eigen::Matrix4d cov_xyz_yaw{Eigen::Matrix4d::Identity()};
  double confidence{0.0};
  double reproj_rms{0.0};
  double condition_number{0.0};
  int num_observations{0};
  int num_inliers{0};
};

}  // namespace armor_pnp_refiner
