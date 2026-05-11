#pragma once

#include <Eigen/Dense>

namespace armor_pnp_refiner {

// Transform covariance from camera frame to another frame via Jacobian.
// Cov_target = J * Cov_camera * J^T
inline Eigen::Matrix4d transformCovariance(
    const Eigen::Matrix4d& cov_camera,
    const Eigen::Matrix4d& J_tf)
{
  return J_tf * cov_camera * J_tf.transpose();
}

}  // namespace armor_pnp_refiner
