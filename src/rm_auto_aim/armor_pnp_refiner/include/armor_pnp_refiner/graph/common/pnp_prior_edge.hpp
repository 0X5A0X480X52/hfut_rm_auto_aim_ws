#pragma once

#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "armor_pnp_refiner/geometry/camera_model.hpp"
#include "armor_pnp_refiner/geometry/armor_geometry.hpp"
#include "armor_pnp_refiner/graph/common/reprojection_edge.hpp"

namespace armor_pnp_refiner {

// ── Yaw-only reprojection edge ────────────────────────────
// Connects: VertexYaw (vertex 0)
// Fixed: t_pnp, pitch, roll, object_point, K, D, R_imu_camera
class EdgeYawReprojection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexYaw> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeYawReprojection() = default;

  void setObjectPoint(const Eigen::Vector3d& p) { object_point_ = p; }
  void setCameraParams(const cv::Mat& K, const cv::Mat& D) { K_ = K; D_ = D; }
  void setFixedPose(const Eigen::Vector3d& t, double pitch, double roll) {
    t_ = t; pitch_ = pitch; roll_ = roll;
  }
  void setImuCameraRotation(const Eigen::Matrix3d& R_imu_camera) {
    R_imu_camera_ = R_imu_camera;
  }

  void computeError() override {
    const auto* v_yaw = static_cast<const VertexYaw*>(_vertices[0]);
    double yaw = v_yaw->estimate();

    Eigen::Matrix3d R_ca = geometry::buildCameraArmorRotationWithImu(
        yaw, pitch_, roll_, R_imu_camera_);

    Eigen::Vector3d p_cam = R_ca * object_point_ + t_;
    Eigen::Vector2d proj = geometry::projectPoint(p_cam, K_, D_);
    _error = measurement() - proj;
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

private:
  Eigen::Vector3d object_point_{0, 0, 0};
  Eigen::Vector3d t_{0, 0, 0};
  double pitch_{0.0}, roll_{0.0};
  Eigen::Matrix3d R_imu_camera_{Eigen::Matrix3d::Identity()};
  cv::Mat K_, D_;
};

// ── XYZ-Yaw reprojection edge ─────────────────────────────
// Connects: VertexXyzYaw (vertex 0)
// Fixed: pitch, roll, object_point, K, D
class EdgeXyzYawReprojection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexXyzYaw> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeXyzYawReprojection() = default;

  void setObjectPoint(const Eigen::Vector3d& p) { object_point_ = p; }
  void setCameraParams(const cv::Mat& K, const cv::Mat& D) { K_ = K; D_ = D; }
  void setFixedOrientation(double pitch, double roll) { pitch_ = pitch; roll_ = roll; }

  void computeError() override {
    const auto* v = static_cast<const VertexXyzYaw*>(_vertices[0]);
    const Eigen::Vector4d& est = v->estimate();
    Eigen::Vector3d t(est[0], est[1], est[2]);
    double yaw = est[3];

    Eigen::Matrix3d R_ca = geometry::buildCameraArmorRotation(yaw, pitch_, roll_);
    Eigen::Vector3d p_cam = R_ca * object_point_ + t;
    Eigen::Vector2d proj = geometry::projectPoint(p_cam, K_, D_);
    _error = measurement() - proj;
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

private:
  Eigen::Vector3d object_point_{0, 0, 0};
  double pitch_{0.0}, roll_{0.0};
  cv::Mat K_, D_;
};

}  // namespace armor_pnp_refiner
