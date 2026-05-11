#pragma once

#include <g2o/core/base_vertex.h>
#include "armor_pnp_refiner/geometry/armor_geometry.hpp"

namespace armor_pnp_refiner {

// 1-DOF yaw vertex for single-yaw optimization (Phase0/Phase1).
class VertexYaw : public g2o::BaseVertex<1, double> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  void setToOriginImpl() override { _estimate = 0.0; }

  void oplusImpl(const double* update) override {
    _estimate = geometry::normalizeAngle(_estimate + update[0]);
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

// 4-DOF vertex [tx, ty, tz, yaw] for xyz-yaw optimization (Phase1/Phase2).
class VertexXyzYaw : public g2o::BaseVertex<4, Eigen::Vector4d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  void setToOriginImpl() override { _estimate.setZero(); }

  void oplusImpl(const double* update) override {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
    _estimate[3] = geometry::normalizeAngle(_estimate[3] + update[3]);
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

}  // namespace armor_pnp_refiner
