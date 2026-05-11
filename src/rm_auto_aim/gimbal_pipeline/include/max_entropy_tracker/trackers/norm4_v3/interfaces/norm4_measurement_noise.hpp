// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_MEASUREMENT_NOISE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_MEASUREMENT_NOISE_HPP_

#include <Eigen/Dense>

#include "max_entropy_tracker/core/config.hpp"

namespace fyt::auto_aim::norm4_v3 {

enum class UpdateKind { Single, Dual };

class IMeasurementNoiseModel {
 public:
  virtual ~IMeasurementNoiseModel() = default;

  virtual Eigen::MatrixXd build_R(UpdateKind kind) const = 0;

  virtual double sigma_pos_xy() const = 0;
  virtual double sigma_pos_z() const = 0;
  virtual double sigma_yaw() const = 0;
  virtual double dual_scale() const = 0;
};

/// Exact reproduction of V1 fixed-Cartesian noise semantics.
class FixedCartesianNoiseModel : public IMeasurementNoiseModel {
 public:
  explicit FixedCartesianNoiseModel(const Norm4V3UkfConfig &ukf_cfg)
      : sp_(ukf_cfg.sigma_pos_xy),
        sz_(ukf_cfg.sigma_pos_z),
        sy_(ukf_cfg.sigma_yaw),
        dual_scale_(ukf_cfg.dual_raw_R_scale) {}

  Eigen::MatrixXd build_R(UpdateKind kind) const override {
    if (kind == UpdateKind::Single) {
      Eigen::Vector4d diag(sp_ * sp_, sp_ * sp_, sz_ * sz_, sy_ * sy_);
      return diag.asDiagonal();
    } else {
      Eigen::Matrix<double, 8, 1> diag;
      diag << sp_ * sp_, sp_ * sp_, sz_ * sz_, sy_ * sy_,
              sp_ * sp_, sp_ * sp_, sz_ * sz_, sy_ * sy_;
      diag *= dual_scale_;
      return diag.asDiagonal();
    }
  }

  double sigma_pos_xy() const override { return sp_; }
  double sigma_pos_z() const override { return sz_; }
  double sigma_yaw() const override { return sy_; }
  double dual_scale() const override { return dual_scale_; }

 private:
  double sp_, sz_, sy_, dual_scale_;
};

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_MEASUREMENT_NOISE_HPP_
