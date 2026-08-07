// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_MEASUREMENT_NOISE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_MEASUREMENT_NOISE_HPP_

#include <Eigen/Dense>

#include <string>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim::norm4_baseline {

class IMeasurementNoiseModel {
 public:
  virtual ~IMeasurementNoiseModel() = default;

  virtual Eigen::Matrix4d build_single_R(
      const ObservationData &obs) const = 0;

  virtual Eigen::Matrix<double, 8, 8> build_dual_R(
      const ObservationData &obs0,
      const ObservationData &obs1) const = 0;

  virtual std::string name() const = 0;

  virtual double sigma_pos_xy() const = 0;
  virtual double sigma_pos_z() const = 0;
  virtual double sigma_yaw() const = 0;
  virtual double dual_scale() const = 0;
};

/// Baseline fixed-Cartesian noise semantics.
class FixedCartesianNoiseModel : public IMeasurementNoiseModel {
 public:
  explicit FixedCartesianNoiseModel(const Norm4BaselineUkfConfig &ukf_cfg)
      : sp_(ukf_cfg.sigma_pos_xy),
        sz_(ukf_cfg.sigma_pos_z),
        sy_(ukf_cfg.sigma_yaw),
        dual_scale_(ukf_cfg.dual_raw_R_scale) {}

  Eigen::Matrix4d build_single_R(
      const ObservationData & /*obs*/) const override {
    Eigen::Vector4d diag(sp_ * sp_, sp_ * sp_, sz_ * sz_, sy_ * sy_);
    return diag.asDiagonal();
  }

  Eigen::Matrix<double, 8, 8> build_dual_R(
      const ObservationData & /*obs0*/,
      const ObservationData & /*obs1*/) const override {
    Eigen::Matrix<double, 8, 1> diag;
    diag << sp_ * sp_, sp_ * sp_, sz_ * sz_, sy_ * sy_,
            sp_ * sp_, sp_ * sp_, sz_ * sz_, sy_ * sy_;
    diag *= dual_scale_;
    return diag.asDiagonal();
  }

  std::string name() const override { return "fixed"; }

  double sigma_pos_xy() const override { return sp_; }
  double sigma_pos_z() const override { return sz_; }
  double sigma_yaw() const override { return sy_; }
  double dual_scale() const override { return dual_scale_; }

 private:
  double sp_, sz_, sy_, dual_scale_;
};

}  // namespace fyt::auto_aim::norm4_baseline

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_MEASUREMENT_NOISE_HPP_
