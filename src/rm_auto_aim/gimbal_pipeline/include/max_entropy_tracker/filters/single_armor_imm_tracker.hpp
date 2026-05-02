// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_SINGLE_ARMOR_IMM_TRACKER_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_SINGLE_ARMOR_IMM_TRACKER_HPP_

#include <Eigen/Dense>
#include <array>

namespace fyt::auto_aim::kalman {

/// Configuration for the 4-model single-armor IMM tracker.
struct SingleArmorIMMConfig {
  double dt = 0.05;

  // ── Observability: which models are enabled ──
  bool enable_cv   = true;
  bool enable_ca   = true;
  bool enable_cs   = false;
  bool enable_ctrv = false;

  // ── CV process noise (velocity white noise) ──
  double q_cv = 0.5;

  // ── CA process noise (acceleration white noise) ──
  double q_ca = 1.0;

  // ── CS (Singer) parameters ──
  double cs_alpha = 0.5;     // maneuver frequency
  double cs_a_max = 10.0;    // max acceleration

  // ── CTRV process noise ──
  double q_ctrv_v   = 0.3;   // speed process noise
  double q_ctrv_omega = 0.5; // yaw-rate process noise

  // ── IMM Markov transition probabilities ──
  // Row i, Col j  =  P(from model j → to model i)
  // Default: equal probability to all models
  double p_stay  = 0.82;     // stay in current model
  double p_switch = 0.06;    // switch to any other model

  // ── Z-axis 1D CV-KF ──
  double q_z_vel = 0.8;      // z-velocity process noise

  // ── Yaw 1D CV-KF ──
  double q_yaw_rate = 0.6;   // yaw-rate process noise

  // ── Observation noise base ──
  double r_pos_base = 0.01;
  double r_yaw_base = 0.03;
};

/// Output state of the single-armor IMM tracker.
struct SingleArmorIMMState {
  Eigen::Vector3d pos{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel{Eigen::Vector3d::Zero()};
  double yaw = 0.0;
  double yaw_rate = 0.0;
  bool initialized = false;
};

// ── Internal per-model structures ──
namespace detail {

/// CV model: 4D state [x, vx, y, vy]
struct CvModel4D {
  Eigen::Vector4d x;
  Eigen::Matrix4d P, F, Q;
  Eigen::Matrix<double, 2, 4> H;
  void predict(double dt, double q_cv);
  void update(const Eigen::Vector2d &z, double r_scale, double r_base);
  // Adapters to 6D
  Eigen::Matrix<double, 6, 1> to6D() const;
  void from6D(const Eigen::Matrix<double, 6, 1> &x6);
  Eigen::Matrix<double, 6, 6> Pto6D() const;
  void Pfrom6D(const Eigen::Matrix<double, 6, 6> &P6);
};

/// CA model: 6D state [x, vx, ax, y, vy, ay]
struct CaModel6D {
  Eigen::Matrix<double, 6, 1> x;
  Eigen::Matrix<double, 6, 6> P, F, Q;
  Eigen::Matrix<double, 2, 6> H;
  void predict(double dt, double q_ca);
  void update(const Eigen::Vector2d &z, double r_scale, double r_base);
  // Identity adapters
  const Eigen::Matrix<double, 6, 1> &to6D() const { return x; }
  void from6D(const Eigen::Matrix<double, 6, 1> &x6) { x = x6; }
  const Eigen::Matrix<double, 6, 6> &Pto6D() const { return P; }
  void Pfrom6D(const Eigen::Matrix<double, 6, 6> &P6) { P = P6; }
};

/// CS (Singer) model: 6D state with time-correlated acceleration
struct CsModel6D {
  double alpha = 0.5;  // maneuver frequency
  double a_max = 10.0;
  Eigen::Matrix<double, 6, 1> x;
  Eigen::Matrix<double, 6, 6> P, F, Q;
  Eigen::Matrix<double, 2, 6> H;
  void rebuild(double dt, double alpha_, double a_max_, double dt_orig);
  void predict(double dt, double alpha_, double a_max_);
  void update(const Eigen::Vector2d &z, double r_scale, double r_base);
  const Eigen::Matrix<double, 6, 1> &to6D() const { return x; }
  void from6D(const Eigen::Matrix<double, 6, 1> &x6) { x = x6; }
  const Eigen::Matrix<double, 6, 6> &Pto6D() const { return P; }
  void Pfrom6D(const Eigen::Matrix<double, 6, 6> &P6) { P = P6; }
};

/// CTRV model: 5D polar state [x, y, v, theta, omega]
struct CtrvModel5D {
  // State: [x, y, v, theta, omega]^T
  Eigen::Matrix<double, 5, 1> x;
  Eigen::Matrix<double, 5, 5> P, F_jac, Q;
  // Observation extracts [x, y] from polar
  Eigen::Matrix<double, 2, 5> H;
  void predict(double dt, double q_v, double q_omega);
  void update(const Eigen::Vector2d &z, double r_scale, double r_base);
  // Adapters to 6D: vx=v*cos(theta), vy=v*sin(theta), ax=ay=0(从P推导)
  Eigen::Matrix<double, 6, 1> to6D() const;
  void from6D(const Eigen::Matrix<double, 6, 1> &x6);
  Eigen::Matrix<double, 6, 6> Pto6D() const;
  void Pfrom6D(const Eigen::Matrix<double, 6, 6> &P6);
};

}  // namespace detail

/// Self-contained 4-model IMM tracker for a single armor plate.
///
/// XY plane: IMM(CV + CA + CS + CTRV), unified 6D state [x,vx,ax,y,vy,ay]
/// Z axis:   1D CV-KF (z, vz)
/// Yaw:      1D CV-KF (yaw, yaw_rate) with angle unwrapping
class SingleArmorIMMTracker {
 public:
  explicit SingleArmorIMMTracker(const SingleArmorIMMConfig &cfg);

  void initialize(const Eigen::Vector3d &pos, double yaw);
  void predict(double dt);
  void update(const Eigen::Vector3d &pos_meas, double yaw_meas,
              double pos_conf = 1.0, double yaw_conf = 1.0);

  const SingleArmorIMMState &state() const { return state_; }
  bool initialized() const { return state_.initialized; }

  int model_count() const { return active_models_; }
  std::array<double, 4> model_probabilities() const { return mu_; }

 private:
  static constexpr int kMaxModels = 4;
  enum ModelIdx { kCV = 0, kCA = 1, kCS = 2, kCTRV = 3 };

  void imm_mixing();
  void imm_predict(double dt);
  void imm_update(const Eigen::Vector2d &z_xy, double r_scale);
  void compute_combined_xy();

  static double clamp_conf(double c) { return std::clamp(c, 0.05, 1.0); }
  double unwrap_yaw(double yaw_meas);

  SingleArmorIMMConfig cfg_;
  SingleArmorIMMState state_;

  // Which models are active
  int active_models_ = 0;
  std::array<bool, kMaxModels> enabled_{};

  // Model probabilities (mu)
  std::array<double, kMaxModels> mu_{};
  std::array<double, kMaxModels> mu_prior_{};

  // IMM transition matrix
  Eigen::Matrix4d trans_prob_;

  // Per-model internal state/covariance (stored as 6D unified)
  std::array<Eigen::Matrix<double, 6, 1>, kMaxModels> x_model_;
  std::array<Eigen::Matrix<double, 6, 6>, kMaxModels> P_model_;

  // Per-model observation matrices (2x6)
  std::array<Eigen::Matrix<double, 2, 6>, kMaxModels> H_model_;

  // Model instances
  detail::CvModel4D  cv_;
  detail::CaModel6D  ca_;
  detail::CsModel6D  cs_;
  detail::CtrvModel5D ctrv_;

  // ── Z-axis 1D CV-KF ──
  Eigen::Vector2d x_z_;
  Eigen::Matrix2d P_z_, F_z_, Q_z_;
  Eigen::RowVector2d H_z_;

  // ── Yaw 1D CV-KF ──
  Eigen::Vector2d x_yaw_;
  Eigen::Matrix2d P_yaw_, F_yaw_, Q_yaw_;
  Eigen::RowVector2d H_yaw_;
  bool has_unwrap_ref_ = false;
  double yaw_unwrap_ref_ = 0.0;
};

}  // namespace fyt::auto_aim::kalman

#endif  // MAX_ENTROPY_TRACKER_FILTERS_SINGLE_ARMOR_IMM_TRACKER_HPP_
