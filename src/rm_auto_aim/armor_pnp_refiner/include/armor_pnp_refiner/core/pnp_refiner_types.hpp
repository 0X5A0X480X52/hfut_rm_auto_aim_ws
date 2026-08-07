#pragma once

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace armor_pnp_refiner {

enum class RefineMode { PNP_FALLBACK = 0, G2O_SINGLE_XYZ_YAW };

enum class RefineStatus { GOOD = 0, DEGRADED, REJECTED };

struct PnpRefineInput {
  // Raw PnP result in camera frame.
  Eigen::Vector3d t_camera_armor{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_camera_armor{Eigen::Quaterniond::Identity()};

  double yaw_rad{0.0};
  double pitch_rad{0.0};
  double roll_rad{0.0};

  // 2D-3D correspondences. image_points[i] must match object_points[i].
  std::vector<cv::Point2f> image_points;
  std::vector<cv::Point3f> object_points;

  // Camera intrinsics.
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;

  bool use_fixed_pitch_roll{true};
  double fixed_pitch_rad{0.0};
  double fixed_roll_rad{0.0};
};

struct PnpRefineOutput {
  bool valid{false};
  bool refined{false};

  RefineMode mode{RefineMode::PNP_FALLBACK};
  RefineStatus status{RefineStatus::REJECTED};

  Eigen::Vector3d t_camera_armor{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_camera_armor{Eigen::Quaterniond::Identity()};

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
  int num_points{0};
  int num_inliers{0};

  std::string reason;
};

}  // namespace armor_pnp_refiner
