// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_OUTPOST_AMBIGUOUS_KF_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_OUTPOST_AMBIGUOUS_KF_HPP_

#include <Eigen/Dense>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim {

/// Lightweight dedicated KF for AMBIGUOUS mode:
/// - Position: 3D constant-velocity linear KF, state [x,vx,y,vy,z,vz]
/// - Yaw: 1D constant-velocity linear KF with angle unwrapping, state [yaw,yaw_rate]
class OutpostAmbiguousKF {
 public:
  explicit OutpostAmbiguousKF(const UnifiedConfig & config, double dt = 0.05);

  void initialize(const ObservationData & obs);
  void predict(double dt);
  void update(const ObservationData & obs,
              double position_confidence = 1.0,
              double yaw_confidence = 1.0);

  bool initialized() const { return initialized_; }

  Eigen::Vector3d armor_position() const;
  Eigen::Vector3d armor_velocity() const;
  double armor_yaw() const;
  double armor_yaw_rate() const;

 private:
  void rebuild_motion_model(double dt);
  static double clamp_conf(double c);
  double unwrap_yaw(double yaw_meas);

  UnifiedConfig config_;
  bool initialized_ = false;
  bool has_unwrap_ref_ = false;
  double yaw_unwrap_ref_ = 0.0;

  double dt_ = 0.05;
  double q_pos_acc_ = 4.0;
  double q_yaw_acc_ = 16.0;
  double r_pos_base_ = 0.01;
  double r_yaw_base_ = 0.03;

  // Position KF (6x6)
  Eigen::VectorXd x_pos_ = Eigen::VectorXd::Zero(6);
  Eigen::MatrixXd P_pos_ = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd F_pos_ = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd Q_pos_ = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd H_pos_ = Eigen::MatrixXd::Zero(3, 6);

  // Yaw KF (2x2)
  Eigen::Vector2d x_yaw_ = Eigen::Vector2d::Zero();
  Eigen::Matrix2d P_yaw_ = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d F_yaw_ = Eigen::Matrix2d::Identity();
  Eigen::Matrix2d Q_yaw_ = Eigen::Matrix2d::Zero();
  Eigen::RowVector2d H_yaw_ = Eigen::RowVector2d::Zero();
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_OUTPOST_AMBIGUOUS_KF_HPP_
