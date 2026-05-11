#pragma once

#include <g2o/core/base_unary_edge.h>
#include <Eigen/Dense>

#include "armor_pnp_refiner/geometry/armor_geometry.hpp"
#include "armor_pnp_refiner/graph/common/reprojection_edge.hpp"

namespace armor_pnp_refiner {

// ── Translation prior edge ────────────────────────────────
// Connects: VertexXyzYaw (vertex 0)
// Error: t_current - t_pnp  (3D)
class EdgeTranslationPrior : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexXyzYaw> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeTranslationPrior() = default;

  void setPnpTranslation(const Eigen::Vector3d& t_pnp) { t_pnp_ = t_pnp; }

  void computeError() override {
    const auto* v = static_cast<const VertexXyzYaw*>(_vertices[0]);
    const Eigen::Vector4d& est = v->estimate();
    _error[0] = est[0] - t_pnp_[0];
    _error[1] = est[1] - t_pnp_[1];
    _error[2] = est[2] - t_pnp_[2];
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

private:
  Eigen::Vector3d t_pnp_{0, 0, 0};
};

// ── Yaw prior edge ────────────────────────────────────────
// Connects: VertexYaw or VertexXyzYaw (vertex 0)
// Error: wrap(yaw_current - yaw_pnp)  (1D)
class EdgeYawPrior : public g2o::BaseUnaryEdge<1, double, VertexYaw> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeYawPrior() = default;

  void setPnpYaw(double yaw_pnp) { yaw_pnp_ = yaw_pnp; }

  void computeError() override {
    const auto* v = static_cast<const VertexYaw*>(_vertices[0]);
    _error[0] = geometry::normalizeAngle(v->estimate() - yaw_pnp_);
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

private:
  double yaw_pnp_{0.0};
};

// Yaw prior for VertexXyzYaw.
class EdgeXyzYawPrior : public g2o::BaseUnaryEdge<1, double, VertexXyzYaw> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeXyzYawPrior() = default;

  void setPnpYaw(double yaw_pnp) { yaw_pnp_ = yaw_pnp; }

  void computeError() override {
    const auto* v = static_cast<const VertexXyzYaw*>(_vertices[0]);
    _error[0] = geometry::normalizeAngle(v->estimate()[3] - yaw_pnp_);
  }

  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }

private:
  double yaw_pnp_{0.0};
};

}  // namespace armor_pnp_refiner
