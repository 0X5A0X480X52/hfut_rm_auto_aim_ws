#pragma once

#include <g2o/core/base_multi_edge.h>
#include <Eigen/Dense>

#include "armor_pnp_refiner/geometry/armor_geometry.hpp"
#include "armor_pnp_refiner/graph/common/reprojection_edge.hpp"

namespace armor_pnp_refiner {

// ── Second-order finite difference smooth edge ────────────
// Connects: VertexXyzYaw(k-2), VertexXyzYaw(k-1), VertexXyzYaw(k)
// Error: x_k - 2*x_{k-1} + x_{k-2}  (4D)
// Allows constant velocity, suppresses high-frequency jitter.
class EdgeSecondOrderSmooth : public g2o::BaseMultiEdge<4, Eigen::Vector4d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeSecondOrderSmooth() {
    resize(3);  // 3 vertices: k-2, k-1, k
  }

  void computeError() override {
    const auto* v0 = static_cast<const VertexXyzYaw*>(_vertices[0]);  // k-2
    const auto* v1 = static_cast<const VertexXyzYaw*>(_vertices[1]);  // k-1
    const auto* v2 = static_cast<const VertexXyzYaw*>(_vertices[2]);  // k

    const Eigen::Vector4d& x0 = v0->estimate();
    const Eigen::Vector4d& x1 = v1->estimate();
    const Eigen::Vector4d& x2 = v2->estimate();

    _error[0] = x2[0] - 2.0 * x1[0] + x0[0];
    _error[1] = x2[1] - 2.0 * x1[1] + x0[1];
    _error[2] = x2[2] - 2.0 * x1[2] + x0[2];
    _error[3] = geometry::normalizeAngle(x2[3] - 2.0 * x1[3] + x0[3]);
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

// ── Third-order finite difference smooth edge (optional) ──
// Connects: VertexXyzYaw(k-3), VertexXyzYaw(k-2), VertexXyzYaw(k-1), VertexXyzYaw(k)
// Error: x_k - 3*x_{k-1} + 3*x_{k-2} - x_{k-3}  (4D)
// Weak regularization for high-speed varying rotation scenarios.
class EdgeThirdOrderSmooth : public g2o::BaseMultiEdge<4, Eigen::Vector4d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeThirdOrderSmooth() {
    resize(4);  // 4 vertices: k-3, k-2, k-1, k
  }

  void computeError() override {
    const auto* v0 = static_cast<const VertexXyzYaw*>(_vertices[0]);  // k-3
    const auto* v1 = static_cast<const VertexXyzYaw*>(_vertices[1]);  // k-2
    const auto* v2 = static_cast<const VertexXyzYaw*>(_vertices[2]);  // k-1
    const auto* v3 = static_cast<const VertexXyzYaw*>(_vertices[3]);  // k

    const Eigen::Vector4d& x0 = v0->estimate();
    const Eigen::Vector4d& x1 = v1->estimate();
    const Eigen::Vector4d& x2 = v2->estimate();
    const Eigen::Vector4d& x3 = v3->estimate();

    _error[0] = x3[0] - 3.0 * x2[0] + 3.0 * x1[0] - x0[0];
    _error[1] = x3[1] - 3.0 * x2[1] + 3.0 * x1[1] - x0[1];
    _error[2] = x3[2] - 3.0 * x2[2] + 3.0 * x1[2] - x0[2];
    _error[3] = geometry::normalizeAngle(x3[3] - 3.0 * x2[3] + 3.0 * x1[3] - x0[3]);
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

}  // namespace armor_pnp_refiner
