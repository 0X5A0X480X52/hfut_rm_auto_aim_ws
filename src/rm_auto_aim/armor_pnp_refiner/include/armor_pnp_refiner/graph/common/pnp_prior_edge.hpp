#pragma once

#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_unary_edge.h>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "armor_pnp_refiner/geometry/armor_geometry.hpp"
#include "armor_pnp_refiner/geometry/camera_model.hpp"
#include "armor_pnp_refiner/graph/common/reprojection_edge.hpp"

namespace armor_pnp_refiner {

// Connects: VertexXyzYaw (vertex 0)
// Fixed: pitch, roll, object_point, K, D
class EdgeXyzYawReprojection : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, VertexXyzYaw> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeXyzYawReprojection() = default;

  void setObjectPoint(const Eigen::Vector3d &p) { object_point_ = p; }
  void setCameraParams(const cv::Mat &K, const cv::Mat &D) {
    K_ = K;
    D_ = D;
  }
  void setFixedOrientation(double pitch, double roll) {
    pitch_ = pitch;
    roll_ = roll;
  }

  void computeError() override {
    const auto *v = static_cast<const VertexXyzYaw *>(_vertices[0]);
    const Eigen::Vector4d &est = v->estimate();
    Eigen::Vector3d t(est[0], est[1], est[2]);
    double yaw = est[3];

    Eigen::Matrix3d R_ca = geometry::buildCameraArmorRotation(yaw, pitch_, roll_);
    Eigen::Vector3d p_cam = R_ca * object_point_ + t;
    Eigen::Vector2d proj = geometry::projectPoint(p_cam, K_, D_);
    _error = measurement() - proj;
  }

  bool read(std::istream &) override { return false; }
  bool write(std::ostream &) const override { return false; }

private:
  Eigen::Vector3d object_point_{0, 0, 0};
  double pitch_{0.0}, roll_{0.0};
  cv::Mat K_, D_;
};

}  // namespace armor_pnp_refiner
