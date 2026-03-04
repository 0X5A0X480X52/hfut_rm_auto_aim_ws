// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_ROTATION_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_ROTATION_HPP_

#include <memory>
#include <stdexcept>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/process_models/base.hpp"

namespace fyt::auto_aim {

struct RotationConfig {
  double cv_process_noise_rate = 0.5;
  double ca_process_noise_acc = 1.0;
};

// ======================== CV Rotation ========================
// State: [DELTA, DELTA_RATE]

class CVRotation : public ProcessModelComponent {
 public:
  explicit CVRotation(RotationConfig cfg = {}) : cfg_(cfg) {}

  ComponentStateSpec get_state_spec() const override {
    return ComponentStateSpec::from_names({"DELTA", "DELTA_RATE"});
  }

  Eigen::VectorXd predict(const Eigen::VectorXd &x, double dt,
                          const Eigen::VectorXd * = nullptr) const override {
    Eigen::VectorXd xn = x;
    xn(0) = x(0) + x(1) * dt;
    return xn;
  }

  Eigen::MatrixXd build_Q(double dt) const override {
    double q = cfg_.cv_process_noise_rate;
    Eigen::Matrix2d Q;
    Q << dt * dt * dt / 3.0, dt * dt / 2.0, dt * dt / 2.0, dt;
    Q *= q * q;
    return Q;
  }

  Eigen::MatrixXd get_initial_covariance() const override {
    Eigen::Matrix2d P;
    P << 0.3, 0.0, 0.0, 0.5;
    return P;
  }

 private:
  RotationConfig cfg_;
};

// ======================== CA Rotation ========================
// State: [DELTA, DELTA_RATE, DELTA_ACC]

class CARotation : public ProcessModelComponent {
 public:
  explicit CARotation(RotationConfig cfg = {}) : cfg_(cfg) {}

  ComponentStateSpec get_state_spec() const override {
    return ComponentStateSpec::from_names({"DELTA", "DELTA_RATE", "DELTA_ACC"});
  }

  Eigen::VectorXd predict(const Eigen::VectorXd &x, double dt,
                          const Eigen::VectorXd * = nullptr) const override {
    Eigen::VectorXd xn = x;
    xn(0) = x(0) + x(1) * dt + 0.5 * x(2) * dt * dt;
    xn(1) = x(1) + x(2) * dt;
    return xn;
  }

  Eigen::MatrixXd build_Q(double dt) const override {
    double q = cfg_.ca_process_noise_acc;
    double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
    Eigen::Matrix3d Q;
    Q << dt5 / 20, dt4 / 8, dt3 / 6, dt4 / 8, dt3 / 3, dt2 / 2, dt3 / 6,
        dt2 / 2, dt;
    Q *= q * q;
    return Q;
  }

  Eigen::MatrixXd get_initial_covariance() const override {
    Eigen::Vector3d diag;
    diag << 0.3, 0.5, 1.0;
    return diag.asDiagonal();
  }

 private:
  RotationConfig cfg_;
};

inline std::unique_ptr<ProcessModelComponent> create_rotation_model(
    RotationModel type, RotationConfig cfg = {}) {
  switch (type) {
    case RotationModel::CV:
      return std::make_unique<CVRotation>(cfg);
    case RotationModel::CA:
      return std::make_unique<CARotation>(cfg);
    default:
      throw std::invalid_argument("Unknown rotation model type");
  }
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_ROTATION_HPP_
