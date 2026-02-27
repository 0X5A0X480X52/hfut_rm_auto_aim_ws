// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_TRANSLATION_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_TRANSLATION_HPP_

#include <cmath>
#include <stdexcept>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/process_models/base.hpp"

namespace fyt::auto_aim {

struct TranslationConfig {
  double cv_process_noise_vel = 0.5;
  double ca_process_noise_acc = 1.0;
  double singer_alpha = 0.5;
  double singer_sigma = 2.0;
};

// ======================== CV Translation ========================
// State: [X,VX, Y,VY, Z,VZ]  (6D for 3D, 4D for 2D)

class CVTranslation : public ProcessModelComponent {
 public:
  explicit CVTranslation(TranslationConfig cfg = {}, int n_dims = 3)
      : cfg_(cfg), n_dims_(n_dims) {
    if (n_dims == 3)
      names_ = {"X", "VX", "Y", "VY", "Z", "VZ"};
    else if (n_dims == 2)
      names_ = {"X", "VX", "Y", "VY"};
    else
      throw std::invalid_argument("n_dims must be 2 or 3");
  }

  ComponentStateSpec get_state_spec() const override {
    return ComponentStateSpec::from_names(names_);
  }

  Eigen::VectorXd predict(const Eigen::VectorXd &x, double dt,
                          const Eigen::VectorXd * = nullptr) const override {
    Eigen::VectorXd xn = x;
    for (int i = 0; i < n_dims_; ++i) {
      int p = i * 2, v = i * 2 + 1;
      xn(p) = x(p) + x(v) * dt;
    }
    return xn;
  }

  Eigen::MatrixXd build_Q(double dt) const override {
    double q = cfg_.cv_process_noise_vel;
    Eigen::Matrix2d Qb;
    Qb << dt * dt * dt / 3.0, dt * dt / 2.0, dt * dt / 2.0, dt;
    Qb *= q * q;

    int dim = state_dim();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim, dim);
    for (int i = 0; i < n_dims_; ++i) {
      int idx = i * 2;
      Q.block<2, 2>(idx, idx) = Qb;
    }
    return Q;
  }

  Eigen::MatrixXd get_initial_covariance() const override {
    return Eigen::MatrixXd::Identity(state_dim(), state_dim()) * 0.1;
  }

 private:
  TranslationConfig cfg_;
  int n_dims_;
  std::vector<std::string> names_;
};

// ======================== CA Translation ========================
// State: [X,VX,AX, Y,VY,AY, Z,VZ,AZ]  (9D for 3D)

class CATranslation : public ProcessModelComponent {
 public:
  explicit CATranslation(TranslationConfig cfg = {}, int n_dims = 3)
      : cfg_(cfg), n_dims_(n_dims) {
    if (n_dims == 3)
      names_ = {"X", "VX", "AX", "Y", "VY", "AY", "Z", "VZ", "AZ"};
    else if (n_dims == 2)
      names_ = {"X", "VX", "AX", "Y", "VY", "AY"};
    else
      throw std::invalid_argument("n_dims must be 2 or 3");
  }

  ComponentStateSpec get_state_spec() const override {
    return ComponentStateSpec::from_names(names_);
  }

  Eigen::VectorXd predict(const Eigen::VectorXd &x, double dt,
                          const Eigen::VectorXd * = nullptr) const override {
    Eigen::VectorXd xn = x;
    for (int i = 0; i < n_dims_; ++i) {
      int p = i * 3, v = i * 3 + 1, a = i * 3 + 2;
      xn(p) = x(p) + x(v) * dt + 0.5 * x(a) * dt * dt;
      xn(v) = x(v) + x(a) * dt;
      // acceleration stays
    }
    return xn;
  }

  Eigen::MatrixXd build_Q(double dt) const override {
    double q = cfg_.ca_process_noise_acc;
    Eigen::Matrix3d Qb;
    double dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
    Qb << dt5 / 20, dt4 / 8, dt3 / 6, dt4 / 8, dt3 / 3, dt2 / 2, dt3 / 6,
        dt2 / 2, dt;
    Qb *= q * q;

    int dim = state_dim();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim, dim);
    for (int i = 0; i < n_dims_; ++i) {
      int idx = i * 3;
      Q.block<3, 3>(idx, idx) = Qb;
    }
    return Q;
  }

  Eigen::MatrixXd get_initial_covariance() const override {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(state_dim(), state_dim()) * 0.1;
    for (int i = 0; i < n_dims_; ++i) P(i * 3 + 2, i * 3 + 2) = 1.0;
    return P;
  }

 private:
  TranslationConfig cfg_;
  int n_dims_;
  std::vector<std::string> names_;
};

// ======================== Singer Translation ========================
// State same as CA: [X,VX,AX, Y,VY,AY, Z,VZ,AZ]

class SingerTranslation : public ProcessModelComponent {
 public:
  explicit SingerTranslation(TranslationConfig cfg = {}, int n_dims = 3)
      : cfg_(cfg), n_dims_(n_dims) {
    if (n_dims == 3)
      names_ = {"X", "VX", "AX", "Y", "VY", "AY", "Z", "VZ", "AZ"};
    else if (n_dims == 2)
      names_ = {"X", "VX", "AX", "Y", "VY", "AY"};
    else
      throw std::invalid_argument("n_dims must be 2 or 3");
  }

  ComponentStateSpec get_state_spec() const override {
    return ComponentStateSpec::from_names(names_);
  }

  Eigen::VectorXd predict(const Eigen::VectorXd &x, double dt,
                          const Eigen::VectorXd * = nullptr) const override {
    double alpha = cfg_.singer_alpha;
    double rho = std::exp(-alpha * dt);
    Eigen::VectorXd xn = x;
    for (int i = 0; i < n_dims_; ++i) {
      int p = i * 3, v = i * 3 + 1, a = i * 3 + 2;
      double ak = x(a), vk = x(v), pk = x(p);
      if (std::abs(alpha) > 1e-6) {
        xn(p) = pk + vk * dt + ak * (dt - (1.0 - rho) / alpha) / alpha;
        xn(v) = vk + ak * (1.0 - rho) / alpha;
      } else {
        xn(p) = pk + vk * dt + 0.5 * ak * dt * dt;
        xn(v) = vk + ak * dt;
      }
      xn(a) = rho * ak;
    }
    return xn;
  }

  Eigen::MatrixXd build_Q(double dt) const override {
    double alpha = cfg_.singer_alpha;
    double sigma = cfg_.singer_sigma;

    if (std::abs(alpha) < 1e-6) {
      // Degenerate to CA
      CATranslation ca(TranslationConfig{0.5, sigma}, n_dims_);
      return ca.build_Q(dt);
    }

    double a = alpha, T = dt;
    double rho = std::exp(-a * T);
    double rho2 = std::exp(-2.0 * a * T);

    double q11 = (1.0 - rho2 + 2.0 * a * T +
                  2.0 * a * a * a * T * T * T / 3.0 -
                  2.0 * a * a * T * T - 4.0 * a * T * rho) /
                 (2.0 * std::pow(a, 5));
    double q12 =
        (rho2 + 1.0 - 2.0 * rho + 2.0 * a * T * rho - 2.0 * a * T +
         a * a * T * T) /
        (2.0 * std::pow(a, 4));
    double q13 = (1.0 - rho2 - 2.0 * a * T * rho) / (2.0 * a * a * a);
    double q22 =
        (4.0 * rho - 3.0 - rho2 + 2.0 * a * T) / (2.0 * a * a * a);
    double q23 = (rho2 + 1.0 - 2.0 * rho) / (2.0 * a * a);
    double q33 = (1.0 - rho2) / (2.0 * a);

    Eigen::Matrix3d Qb;
    Qb << q11, q12, q13, q12, q22, q23, q13, q23, q33;
    Qb *= 2.0 * a * sigma * sigma;

    int dim = state_dim();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim, dim);
    for (int i = 0; i < n_dims_; ++i) {
      int idx = i * 3;
      Q.block<3, 3>(idx, idx) = Qb;
    }
    return Q;
  }

  Eigen::MatrixXd get_initial_covariance() const override {
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(state_dim(), state_dim()) * 0.1;
    double sigma = cfg_.singer_sigma;
    for (int i = 0; i < n_dims_; ++i) P(i * 3 + 2, i * 3 + 2) = sigma * sigma;
    return P;
  }

 private:
  TranslationConfig cfg_;
  int n_dims_;
  std::vector<std::string> names_;
};

// ======================== Factory ========================

inline std::unique_ptr<ProcessModelComponent> create_translation_model(
    TranslationModel type, TranslationConfig cfg = {}, int n_dims = 3) {
  switch (type) {
    case TranslationModel::CV:
      return std::make_unique<CVTranslation>(cfg, n_dims);
    case TranslationModel::CA:
      return std::make_unique<CATranslation>(cfg, n_dims);
    case TranslationModel::SINGER:
      return std::make_unique<SingerTranslation>(cfg, n_dims);
    default:
      throw std::invalid_argument("Unknown translation model type");
  }
}

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_PROCESS_MODELS_TRANSLATION_HPP_
