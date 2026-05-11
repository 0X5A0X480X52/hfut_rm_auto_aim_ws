// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_CORE_OBSERVATION_HPP_
#define MAX_ENTROPY_TRACKER_CORE_OBSERVATION_HPP_

#include <Eigen/Dense>

#include <array>
#include <optional>
#include <string>

namespace fyt::auto_aim {

/// BA/PnP covariance metadata carried alongside a 3D observation.
/// All fields are optional; valid == false when the upstream detector
/// did not provide refiner metadata.
struct ObservationCovarianceMeta {
  bool valid = false;
  bool cov_valid = false;

  // Cov([x, y, z, yaw]) in the same frame and yaw convention as ObservationData.
  Eigen::Matrix4d cov_xyz_yaw = Eigen::Matrix4d::Identity();

  double confidence = 0.0;
  double reproj_rms = 0.0;
  double condition_number = 0.0;
  int num_observations = 0;
  int num_inliers = 0;
  int pose_estimate_mode = 0;

  // Diagnostics only. True when frame/yaw convention has been checked.
  bool frame_aligned = false;
};

/// 2D image-domain metadata carried alongside a 3D observation.
/// All fields are optional; valid == false when the source detector
/// did not provide image geometry.
struct ImageObservation2D {
  bool valid = false;
  int detection_id = -1;

  double bbox_x = 0.0;
  double bbox_y = 0.0;
  double bbox_w = 0.0;
  double bbox_h = 0.0;

  std::array<Eigen::Vector2d, 4> corners{};
  double image_center_x = 0.0;
  double image_center_y = 0.0;

  double detection_confidence = 0.0;
  std::string number;
  std::string type;
};

/// Observation from a single armor plate
struct ObservationData {
  // Required
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;

  // Optional metadata
  std::optional<int> panel_id;
  std::optional<std::string> layer;
  double confidence = 1.0;
  std::optional<double> timestamp;

  // 来源相机 frame_id
  std::string source_frame;

  // PnP 协方差传播结果（可选，仅在 r_source=PNP_COVARIANCE 时有效）
  std::optional<Eigen::Matrix3d> pos_covariance;  // 3x3 位置协方差 (x,y,z)
  std::optional<double> yaw_variance;              // yaw 方差

  // 2D evidence (Phase 1: append-only, optional)
  std::optional<ImageObservation2D> image;
  std::optional<int> track2d_id;

  // BA/PnP covariance metadata (Phase 1: append-only, optional)
  std::optional<ObservationCovarianceMeta> ba_pnp;

  Eigen::Vector3d position() const { return {x, y, z}; }

  Eigen::Vector4d as_4d() const { return {x, y, z, yaw}; }

  static ObservationData from_4d(const Eigen::Vector4d &v) {
    ObservationData obs;
    obs.x = v(0);
    obs.y = v(1);
    obs.z = v(2);
    obs.yaw = v(3);
    return obs;
  }
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_CORE_OBSERVATION_HPP_
