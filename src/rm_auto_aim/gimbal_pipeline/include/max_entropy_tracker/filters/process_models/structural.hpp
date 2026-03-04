// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_STRUCTURAL_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_STRUCTURAL_HPP_

#include "max_entropy_tracker/filters/process_models/base.hpp"

namespace fyt::auto_aim {

struct StructuralConfig {
  double process_noise_r = 0.02;
  double process_noise_dz = 0.005;
  double initial_r1 = 0.15;
  double initial_r2 = 0.20;
  double initial_dza = 0.0;
};

/// Combined structural model: [R1, R2, DZA] – random walk
class StructuralModel : public ProcessModelComponent {
 public:
  explicit StructuralModel(StructuralConfig cfg = {}) : cfg_(cfg) {}

  ComponentStateSpec get_state_spec() const override {
    return ComponentStateSpec::from_names({"R1", "R2", "DZA"});
  }

  Eigen::VectorXd predict(const Eigen::VectorXd &x, double /*dt*/,
                          const Eigen::VectorXd * = nullptr) const override {
    return x;  // random walk: state stays, noise in Q
  }

  Eigen::MatrixXd build_Q(double dt) const override {
    double qr = cfg_.process_noise_r;
    double qd = cfg_.process_noise_dz;
    Eigen::Vector3d diag;
    diag << qr * qr * dt, qr * qr * dt, qd * qd * dt * 0.1;
    return diag.asDiagonal();
  }

  Eigen::VectorXd get_initial_state() const override {
    Eigen::Vector3d s;
    s << cfg_.initial_r1, cfg_.initial_r2, cfg_.initial_dza;
    return s;
  }

  Eigen::MatrixXd get_initial_covariance() const override {
    Eigen::Vector3d diag;
    diag << 0.01, 0.01, 0.05;
    return diag.asDiagonal();
  }

 private:
  StructuralConfig cfg_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_STRUCTURAL_HPP_
