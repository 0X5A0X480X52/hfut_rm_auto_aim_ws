// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/filters/single_armor_imm_tracker.hpp"

#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::kalman {

namespace {

double sq(double x) { return x * x; }

// Log-likelihood of a 2D innovation under zero-mean Gaussian with covariance S.
double log_likelihood_2d(const Eigen::Vector2d &y, const Eigen::Matrix2d &S) {
  double det = S.determinant();
  if (det <= 1e-30) return -1e9;
  return -0.5 * (std::log(std::max(det, 1e-30)) +
                 static_cast<double>(y.transpose() * S.inverse() * y));
}

}  // namespace

// ===================================================================
//  CvModel4D
// ===================================================================
// 4D state: [x, vx, y, vy]
// Observation: 2D [x, y]

void detail::CvModel4D::predict(double dt, double q_cv) {
  dt = std::clamp(dt, 1e-3, 0.5);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;

  // F: x += vx*dt, y += vy*dt
  F.setIdentity();
  F(0, 1) = dt;
  F(2, 3) = dt;

  // Q: q_cv * [dt4/4, dt3/2; dt3/2, dt2] per axis
  const Eigen::Matrix2d q_blk = (Eigen::Matrix2d()
      << dt4 / 4.0, dt3 / 2.0, dt3 / 2.0, dt2).finished();
  Q.setZero();
  Q.block<2, 2>(0, 0) = q_cv * q_blk;
  Q.block<2, 2>(2, 2) = q_cv * q_blk;

  // H: extract x, y
  H.setZero();
  H(0, 0) = 1.0;
  H(1, 2) = 1.0;

  x = F * x;
  P = F * P * F.transpose() + Q;
}

void detail::CvModel4D::update(const Eigen::Vector2d &z,
                                double r_scale, double r_base) {
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sq(r_base) * r_scale;
  const Eigen::Vector2d y = z - H * x;
  const Eigen::Matrix2d S = H * P * H.transpose() + R;
  const Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
  x = x + K * y;
  P = (Eigen::Matrix4d::Identity() - K * H) * P;
}

Eigen::Matrix<double, 6, 1> detail::CvModel4D::to6D() const {
  Eigen::Matrix<double, 6, 1> x6;
  x6 << x(0), x(1), 0.0, x(2), x(3), 0.0;
  return x6;
}

void detail::CvModel4D::from6D(const Eigen::Matrix<double, 6, 1> &x6) {
  x << x6(0), x6(1), x6(3), x6(4);
}

Eigen::Matrix<double, 6, 6> detail::CvModel4D::Pto6D() const {
  Eigen::Matrix<double, 6, 6> P6 = Eigen::Matrix<double, 6, 6>::Zero();
  P6.block<2, 2>(0, 0) = P.block<2, 2>(0, 0);  // x, vx
  P6.block<2, 2>(3, 3) = P.block<2, 2>(2, 2);  // y, vy
  return P6;
}

void detail::CvModel4D::Pfrom6D(const Eigen::Matrix<double, 6, 6> &P6) {
  P.setZero();
  P.block<2, 2>(0, 0) = P6.block<2, 2>(0, 0);
  P.block<2, 2>(2, 2) = P6.block<2, 2>(3, 3);
}

// ===================================================================
//  CaModel6D
// ===================================================================
// 6D state: [x, vx, ax, y, vy, ay]

void detail::CaModel6D::predict(double dt, double q_ca) {
  dt = std::clamp(dt, 1e-3, 0.5);
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const double dt5 = dt4 * dt;
  const double dt6 = dt5 * dt;

  // F
  F.setIdentity();
  F(0, 1) = dt;   F(0, 2) = 0.5 * dt2;
  F(1, 2) = dt;
  F(3, 4) = dt;   F(3, 5) = 0.5 * dt2;
  F(4, 5) = dt;

  // Q per [x,vx,ax] block
  Eigen::Matrix3d q_blk;
  q_blk << dt6 / 36.0, dt5 / 12.0, dt4 / 6.0,
           dt5 / 12.0, dt4 / 4.0,  dt3 / 2.0,
           dt4 / 6.0,  dt3 / 2.0,  dt2;
  Q.setZero();
  Q.block<3, 3>(0, 0) = q_ca * q_blk;
  Q.block<3, 3>(3, 3) = q_ca * q_blk;

  // H: extract x, y
  H.setZero();
  H(0, 0) = 1.0;
  H(1, 3) = 1.0;

  x = F * x;
  P = F * P * F.transpose() + Q;
}

void detail::CaModel6D::update(const Eigen::Vector2d &z,
                                double r_scale, double r_base) {
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sq(r_base) * r_scale;
  const Eigen::Vector2d y = z - H * x;
  const Eigen::Matrix2d S = H * P * H.transpose() + R;
  const Eigen::Matrix<double, 6, 2> K = P * H.transpose() * S.inverse();
  x = x + K * y;
  P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
}

// ===================================================================
//  CsModel6D (Singer)
// ===================================================================
// 6D state: [x, vx, ax, y, vy, ay]
// Singer model: acceleration is 1st-order Markov with decay alpha
//   \dot{ax} = -alpha * ax + w
//   \dot{ay} = -alpha * ay + w
// Q derived from Singer variance = (a_max)^2 / 3  * (1 + 4 * p_max - p_0)

void detail::CsModel6D::rebuild(double dt, double alpha_, double a_max_,
                                 double dt_orig) {
  dt = std::clamp(dt, 1e-3, 0.5);
  (void)dt_orig;
  alpha = std::max(0.01, alpha_);
  a_max = std::max(0.1, a_max_);
  const double a = alpha;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;

  const double eat  = std::exp(-a * dt);
  const double eat2 = std::exp(-2.0 * a * dt);

  // Singer F for one axis [x, vx, ax]:
  //   x:  x += vx*dt + ax/a^2 * (a*dt + eat - 1)
  //   vx: vx += ax/a * (1 - eat)
  //   ax: ax *= eat
  const double f01 = dt;
  const double f02 = (a * dt + eat - 1.0) / (a * a);
  const double f12 = (1.0 - eat) / a;
  const double f22 = eat;

  F.setIdentity();
  F(0, 1) = f01;   F(0, 2) = f02;
  F(1, 2) = f12;
  F(2, 2) = f22;
  F(3, 4) = f01;   F(3, 5) = f02;
  F(4, 5) = f12;
  F(5, 5) = f22;

  // Singer process noise (per axis)
  // sigma^2 = (a_max)^2 / 3 * (1 + 4*p_max - p_0), approximate p_max≈0.25, p_0≈0.5
  const double p_max = 0.25;
  const double p_0   = 0.50;
  const double sigma2 = sq(a_max) / 3.0 * (1.0 + 4.0 * p_max - p_0);

  // 3x3 Q block (Fitzgerald approximation / standard Singer Q)
  Eigen::Matrix3d q_blk;
  const double a2 = a * a;
  const double a3 = a2 * a;
  const double a4 = a3 * a;
  const double q00 = sigma2 / a4 * (1.0 - eat2 + 2.0*a*dt + 2.0*a3*dt3/3.0
                                     - 2.0*a2*dt2 - 4.0*a*dt*eat);
  const double q01 = sigma2 / a3 * (eat2 + 1.0 - 2.0*eat + 2.0*a*dt*eat
                                     - 2.0*a*dt + a2*dt2);
  const double q02 = sigma2 / a2 * (1.0 - eat2 - 2.0*a*dt*eat);
  const double q11 = sigma2 / a2 * (4.0*eat - 3.0 - eat2 + 2.0*a*dt);
  const double q12 = sigma2 / a  * (eat2 + 1.0 - 2.0*eat);
  const double q22 = sigma2 * (1.0 - eat2);

  q_blk << q00, q01, q02,
           q01, q11, q12,
           q02, q12, q22;

  Q.setZero();
  Q.block<3, 3>(0, 0) = q_blk;
  Q.block<3, 3>(3, 3) = q_blk;

  // H: extract x, y
  H.setZero();
  H(0, 0) = 1.0;
  H(1, 3) = 1.0;
}

void detail::CsModel6D::predict(double dt, double alpha_, double a_max_) {
  rebuild(dt, alpha_, a_max_, dt);
  x = F * x;
  P = F * P * F.transpose() + Q;
}

void detail::CsModel6D::update(const Eigen::Vector2d &z,
                                double r_scale, double r_base) {
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sq(r_base) * r_scale;
  const Eigen::Vector2d y = z - H * x;
  const Eigen::Matrix2d S = H * P * H.transpose() + R;
  const Eigen::Matrix<double, 6, 2> K = P * H.transpose() * S.inverse();
  x = x + K * y;
  P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * P;
}

// ===================================================================
//  CtrvModel5D
// ===================================================================
// 5D state: [x, y, v, theta, omega]
// Nonlinear motion: uses EKF with Jacobian

void detail::CtrvModel5D::predict(double dt, double q_v, double q_omega) {
  dt = std::clamp(dt, 1e-3, 0.5);

  const double xp   = x(0);
  const double yp   = x(1);
  const double v    = std::max(x(2), 1e-6);
  const double th   = x(3);
  const double om   = x(4);

  // Nonlinear state transition
  if (std::abs(om) > 1e-6) {
    x(0) = xp + v / om * (std::sin(th + om * dt) - std::sin(th));
    x(1) = yp + v / om * (std::cos(th) - std::cos(th + om * dt));
  } else {
    x(0) = xp + v * std::cos(th) * dt;
    x(1) = yp + v * std::sin(th) * dt;
  }
  x(3) = normalize_angle(th + om * dt);

  // Jacobian F (5x5)
  F_jac.setIdentity();
  if (std::abs(om) > 1e-6) {
    const double st  = std::sin(th);
    const double stw = std::sin(th + om * dt);
    const double ct  = std::cos(th);
    const double ctw = std::cos(th + om * dt);

    F_jac(0, 2) = (stw - st) / om;
    F_jac(0, 3) = v / om * (ctw - ct);
    F_jac(0, 4) = v / (om * om) * (-stw + st + om * dt * ctw);

    F_jac(1, 2) = (ct - ctw) / om;
    F_jac(1, 3) = v / om * (stw - st);
    F_jac(1, 4) = v / (om * om) * (ctw - ct + om * dt * stw);
  } else {
    F_jac(0, 2) = std::cos(th) * dt;
    F_jac(0, 3) = -v * std::sin(th) * dt;
    F_jac(1, 2) = std::sin(th) * dt;
    F_jac(1, 3) =  v * std::cos(th) * dt;
  }
  F_jac(3, 4) = dt;

  // Process noise Q
  //   q0 from q_v, q1 from q_omega
  Eigen::Matrix<double, 5, 2> G = Eigen::Matrix<double, 5, 2>::Zero();
  G(2, 0) = 1.0;  // v noise
  G(4, 1) = 1.0;  // omega noise
  const Eigen::Matrix2d Qn = (Eigen::Matrix2d()
      << q_v * dt,               0.0,
         0.0,           q_omega * dt).finished();
  Q = G * Qn * G.transpose();

  // Observation: linear (extract x, y)
  H.setZero();
  H(0, 0) = 1.0;
  H(1, 1) = 1.0;

  P = F_jac * P * F_jac.transpose() + Q;
}

void detail::CtrvModel5D::update(const Eigen::Vector2d &z,
                                  double r_scale, double r_base) {
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sq(r_base) * r_scale;
  const Eigen::Vector2d z_pred = H * x;
  const Eigen::Vector2d y = z - z_pred;
  const Eigen::Matrix2d S = H * P * H.transpose() + R;
  const Eigen::Matrix<double, 5, 2> K = P * H.transpose() * S.inverse();
  x = x + K * y;
  P = (Eigen::Matrix<double, 5, 5>::Identity() - K * H) * P;
  // Keep angle in [-pi, pi]
  x(3) = normalize_angle(x(3));
}

Eigen::Matrix<double, 6, 1> detail::CtrvModel5D::to6D() const {
  const double v  = std::max(x(2), 1e-8);
  const double th = x(3);
  const double om = x(4);
  const double vx = v * std::cos(th);
  const double vy = v * std::sin(th);
  // ax, ay from centripetal decomposition and tangential:
  //   ax ≈ -v * omega * sin(theta)  (radial part ignored in simple mapping)
  const double ax = -v * om * std::sin(th);
  const double ay =  v * om * std::cos(th);

  Eigen::Matrix<double, 6, 1> x6;
  x6 << x(0), vx, ax, x(1), vy, ay;
  return x6;
}

void detail::CtrvModel5D::from6D(const Eigen::Matrix<double, 6, 1> &x6) {
  const double vx = x6(1);
  const double vy = x6(4);
  const double ax = x6(2);
  const double ay = x6(5);
  x(0) = x6(0);           // x
  x(1) = x6(3);           // y
  x(2) = std::hypot(vx, vy);  // v
  x(3) = std::atan2(vy, vx);   // theta
  if (x(2) > 1e-6) {
    x(4) = (vx * ay - vy * ax) / (x(2) * x(2));
  } else {
    x(4) = 0.0;
  }
  // Clamp omega
  x(4) = std::clamp(x(4), -20.0, 20.0);
}

Eigen::Matrix<double, 6, 6> detail::CtrvModel5D::Pto6D() const {
  // Linearize mapping: J = d[x6]/d[x5]
  const double v  = std::max(x(2), 1e-8);
  const double th = x(3);
  const double om = x(4);
  const double ct = std::cos(th);
  const double st = std::sin(th);

  Eigen::Matrix<double, 6, 5> J = Eigen::Matrix<double, 6, 5>::Zero();
  J(0, 0) = 1.0;                          // x6.x  = x5.x
  J(1, 1) =  ct;  J(1, 3) = -v * st;      // x6.vx = v*ct  → d/dv=ct, d/dth=-v*st
  J(2, 1) = -om * st;  J(2, 3) = -v * om * ct;  J(2, 4) = -v * st;  // x6.ax ≈ -v*om*st
  J(3, 2) = 1.0;                          // x6.y  = x5.y
  J(4, 1) =  st;  J(4, 3) =  v * ct;      // x6.vy = v*st
  J(5, 1) =  om * ct;  J(5, 3) = -v * om * st;  J(5, 4) =  v * ct;  // x6.ay ≈ v*om*ct

  return J * P * J.transpose();
}

void detail::CtrvModel5D::Pfrom6D(const Eigen::Matrix<double, 6, 6> &P6) {
  // Inverse mapping: J_inv ≈ pseudo-inverse or simplified diagonal mapping.
  // Use simple block extraction + conservative init.
  (void)P6;
  P.setIdentity();
  P(0, 0) = 0.05;  // x
  P(1, 1) = 0.05;  // y
  P(2, 2) = 1.0;   // v
  P(3, 3) = 0.10;  // theta
  P(4, 4) = 1.0;   // omega
}

// ===================================================================
//  SingleArmorIMMTracker
// ===================================================================

SingleArmorIMMTracker::SingleArmorIMMTracker(const SingleArmorIMMConfig &cfg)
    : cfg_(cfg) {
  const double dt = std::clamp(cfg_.dt, 1e-3, 0.5);

  // Determine active models
  if (cfg_.enable_cv)   { enabled_[kCV]   = true; ++active_models_; }
  if (cfg_.enable_ca)   { enabled_[kCA]   = true; ++active_models_; }
  if (cfg_.enable_cs)   { enabled_[kCS]   = true; ++active_models_; }
  if (cfg_.enable_ctrv) { enabled_[kCTRV] = true; ++active_models_; }

  if (active_models_ < 1) {
    enabled_[kCV] = true;
    active_models_ = 1;
  }

  // Build transition matrix
  trans_prob_ = Eigen::Matrix4d::Zero();
  const double p_stay  = std::clamp(cfg_.p_stay, 0.5, 0.98);
  const int N = active_models_;
  const double p_switch = (1.0 - p_stay) / std::max(1, N - 1);
  int ii = 0, jj = 0;
  for (int i = 0; i < kMaxModels; ++i) {
    if (!enabled_[i]) continue;
    jj = 0;
    for (int j = 0; j < kMaxModels; ++j) {
      if (!enabled_[j]) continue;
      trans_prob_(i, j) = (ii == jj) ? p_stay : p_switch;
      ++jj;
    }
    ++ii;
  }

  // Per-model observation matrix: extract [x, y] from 6D
  for (int i = 0; i < kMaxModels; ++i) {
    H_model_[i].setZero();
    H_model_[i](0, 0) = 1.0;
    H_model_[i](1, 3) = 1.0;
  }

  // Init model sub-filters
  cv_.P.setIdentity(); cv_.P(0,0)=0.05; cv_.P(2,2)=0.05; cv_.P(1,1)=1.0; cv_.P(3,3)=1.0;
  ca_.P.setIdentity(); ca_.P(0,0)=0.05; ca_.P(3,3)=0.05; ca_.P(1,1)=1.0; ca_.P(4,4)=1.0; ca_.P(2,2)=1.0; ca_.P(5,5)=1.0;
  cs_.P.setIdentity(); cs_.P(0,0)=0.05; cs_.P(3,3)=0.05; cs_.P(1,1)=1.0; cs_.P(4,4)=1.0; cs_.P(2,2)=1.0; cs_.P(5,5)=1.0;
  ctrv_.P.setIdentity(); ctrv_.P(0,0)=0.05; ctrv_.P(1,1)=0.05; ctrv_.P(2,2)=1.0; ctrv_.P(3,3)=0.10; ctrv_.P(4,4)=1.0;

  // Z-axis 1D KF
  H_z_ << 1.0, 0.0;
  P_z_.setIdentity(); P_z_(0, 0) = 0.05; P_z_(1, 1) = 1.0;

  // Yaw KF
  H_yaw_ << 1.0, 0.0;
  F_yaw_.setIdentity(); F_yaw_(0, 1) = dt;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const Eigen::Matrix2d q_blk = (Eigen::Matrix2d()
      << dt4 / 4.0, dt3 / 2.0, dt3 / 2.0, dt2).finished();
  Q_yaw_ = cfg_.q_yaw_rate * q_blk;
  P_yaw_.setIdentity(); P_yaw_(0, 0) = 0.10; P_yaw_(1, 1) = 1.0;

  // Init CS parameters
  cs_.alpha = std::max(0.01, cfg_.cs_alpha);
  cs_.a_max = std::max(1.0, cfg_.cs_a_max);
}

double SingleArmorIMMTracker::unwrap_yaw(double yaw_meas) {
  if (!has_unwrap_ref_) {
    yaw_unwrap_ref_ = yaw_meas;
    has_unwrap_ref_ = true;
    return yaw_unwrap_ref_;
  }
  yaw_unwrap_ref_ += normalize_angle(yaw_meas - normalize_angle(yaw_unwrap_ref_));
  return yaw_unwrap_ref_;
}

void SingleArmorIMMTracker::initialize(const Eigen::Vector3d &pos, double yaw) {
  has_unwrap_ref_ = false;
  const double yaw_u = unwrap_yaw(yaw);

  // Init per-model unified 6D state
  Eigen::Matrix<double, 6, 1> x6;
  x6 << pos.x(), 0.0, 0.0, pos.y(), 0.0, 0.0;

  Eigen::Matrix<double, 6, 6> P6 = Eigen::Matrix<double, 6, 6>::Identity();
  P6(0, 0) = 0.05; P6(3, 3) = 0.05;
  P6(1, 1) = 1.0;  P6(4, 4) = 1.0;
  P6(2, 2) = 1.0;  P6(5, 5) = 1.0;

  double total_enabled = 0.0;
  for (int i = 0; i < kMaxModels; ++i) {
    if (!enabled_[i]) continue;
    x_model_[i] = x6;
    P_model_[i] = P6;
    mu_[i] = 1.0 / static_cast<double>(active_models_);
    mu_prior_[i] = mu_[i];
    total_enabled += 1.0;
  }

  // Init per-model internal state
  cv_.from6D(x6);
  cv_.P = Eigen::Matrix4d::Identity(); cv_.P(0,0)=0.05; cv_.P(2,2)=0.05; cv_.P(1,1)=1.0; cv_.P(3,3)=1.0;

  ca_.from6D(x6);
  ca_.P.setIdentity(); ca_.P(0,0)=0.05; ca_.P(3,3)=0.05; ca_.P(1,1)=1.0; ca_.P(4,4)=1.0; ca_.P(2,2)=1.0; ca_.P(5,5)=1.0;

  cs_.from6D(x6);
  cs_.P.setIdentity(); cs_.P(0,0)=0.05; cs_.P(3,3)=0.05; cs_.P(1,1)=1.0; cs_.P(4,4)=1.0; cs_.P(2,2)=1.0; cs_.P(5,5)=1.0;

  ctrv_.from6D(x6);
  ctrv_.P.setIdentity(); ctrv_.P(0,0)=0.05; ctrv_.P(1,1)=0.05; ctrv_.P(2,2)=1.0; ctrv_.P(3,3)=0.10; ctrv_.P(4,4)=1.0;

  // Z init
  x_z_ << pos.z(), 0.0;
  P_z_.setIdentity(); P_z_(0, 0) = 0.05; P_z_(1, 1) = 1.0;

  // Yaw init
  x_yaw_ << yaw_u, 0.0;
  P_yaw_.setIdentity(); P_yaw_(0, 0) = 0.10; P_yaw_(1, 1) = 1.0;

  // Combined state
  state_.initialized = true;
  state_.pos = pos;
  state_.vel.setZero();
  state_.yaw = normalize_angle(yaw_u);
  state_.yaw_rate = 0.0;
}

// ── IMM Mixing ──
void SingleArmorIMMTracker::imm_mixing() {
  // Normalization constants: cbar_j = sum_i p_ij * mu_i
  std::array<double, kMaxModels> cbar{};
  for (int j = 0; j < kMaxModels; ++j) {
    if (!enabled_[j]) continue;
    cbar[j] = 0.0;
    for (int i = 0; i < kMaxModels; ++i) {
      if (!enabled_[i]) continue;
      cbar[j] += trans_prob_(j, i) * mu_[i];
    }
    if (cbar[j] < 1e-12) cbar[j] = 1e-12;
  }

  // Mixing weights: mu_{i|j} = p_ji * mu_i / cbar_j
  // Mixed state: X_{0j} = sum_i X_i * mu_{i|j}
  // Mixed cov:   P_{0j} = sum_i (P_i + (X_i - X_{0j}) * (X_i - X_{0j}).T) * mu_{i|j}

  std::array<Eigen::Matrix<double, 6, 1>, kMaxModels> x_mixed;
  std::array<Eigen::Matrix<double, 6, 6>, kMaxModels> P_mixed;

  for (int j = 0; j < kMaxModels; ++j) {
    if (!enabled_[j]) continue;
    x_mixed[j].setZero();
    for (int i = 0; i < kMaxModels; ++i) {
      if (!enabled_[i]) continue;
      const double w = trans_prob_(j, i) * mu_[i] / cbar[j];
      x_mixed[j] += w * x_model_[i];
    }
  }

  for (int j = 0; j < kMaxModels; ++j) {
    if (!enabled_[j]) continue;
    P_mixed[j].setZero();
    for (int i = 0; i < kMaxModels; ++i) {
      if (!enabled_[i]) continue;
      const double w = trans_prob_(j, i) * mu_[i] / cbar[j];
      const Eigen::Matrix<double, 6, 1> dx = x_model_[i] - x_mixed[j];
      P_mixed[j] += w * (P_model_[i] + dx * dx.transpose());
    }
  }

  for (int i = 0; i < kMaxModels; ++i) {
    if (!enabled_[i]) continue;
    x_model_[i] = x_mixed[i];
    P_model_[i] = P_mixed[i];
  }
}

// ── IMM Predict ──
void SingleArmorIMMTracker::imm_predict(double dt) {
  dt = std::clamp(dt, 1e-3, 0.5);

  if (enabled_[kCV]) {
    cv_.from6D(x_model_[kCV]);
    cv_.Pfrom6D(P_model_[kCV]);
    cv_.predict(dt, cfg_.q_cv);
    x_model_[kCV] = cv_.to6D();
    P_model_[kCV] = cv_.Pto6D();
  }

  if (enabled_[kCA]) {
    ca_.from6D(x_model_[kCA]);
    ca_.Pfrom6D(P_model_[kCA]);
    ca_.predict(dt, cfg_.q_ca);
    x_model_[kCA] = ca_.to6D();
    P_model_[kCA] = ca_.Pto6D();
  }

  if (enabled_[kCS]) {
    cs_.from6D(x_model_[kCS]);
    cs_.Pfrom6D(P_model_[kCS]);
    cs_.predict(dt, cfg_.cs_alpha, cfg_.cs_a_max);
    x_model_[kCS] = cs_.to6D();
    P_model_[kCS] = cs_.Pto6D();
  }

  if (enabled_[kCTRV]) {
    ctrv_.from6D(x_model_[kCTRV]);
    ctrv_.Pfrom6D(P_model_[kCTRV]);
    ctrv_.predict(dt, cfg_.q_ctrv_v, cfg_.q_ctrv_omega);
    x_model_[kCTRV] = ctrv_.to6D();
    P_model_[kCTRV] = ctrv_.Pto6D();
  }
}

// ── IMM Update ──
void SingleArmorIMMTracker::imm_update(const Eigen::Vector2d &z_xy,
                                        double r_scale) {
  const double r_pos = cfg_.r_pos_base;

  // Per-model update in its native space, then convert result to 6D
  if (enabled_[kCV]) {
    cv_.from6D(x_model_[kCV]);
    cv_.Pfrom6D(P_model_[kCV]);
    cv_.update(z_xy, r_scale, r_pos);
    x_model_[kCV] = cv_.to6D();
    P_model_[kCV] = cv_.Pto6D();
  }

  if (enabled_[kCA]) {
    ca_.from6D(x_model_[kCA]);
    ca_.Pfrom6D(P_model_[kCA]);
    ca_.update(z_xy, r_scale, r_pos);
    x_model_[kCA] = ca_.to6D();
    P_model_[kCA] = ca_.Pto6D();
  }

  if (enabled_[kCS]) {
    cs_.from6D(x_model_[kCS]);
    cs_.Pfrom6D(P_model_[kCS]);
    cs_.update(z_xy, r_scale, r_pos);
    x_model_[kCS] = cs_.to6D();
    P_model_[kCS] = cs_.Pto6D();
  }

  if (enabled_[kCTRV]) {
    ctrv_.from6D(x_model_[kCTRV]);
    ctrv_.Pfrom6D(P_model_[kCTRV]);
    ctrv_.update(z_xy, r_scale, r_pos);
    x_model_[kCTRV] = ctrv_.to6D();
    P_model_[kCTRV] = ctrv_.Pto6D();
  }

  // Compute per-model likelihoods
  std::array<double, kMaxModels> lambda{};
  const Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * sq(r_pos) * r_scale;
  for (int i = 0; i < kMaxModels; ++i) {
    if (!enabled_[i]) continue;
    const Eigen::Vector2d y = z_xy - H_model_[i] * x_model_[i];
    const Eigen::Matrix2d S = H_model_[i] * P_model_[i] * H_model_[i].transpose() + R;
    lambda[i] = std::exp(log_likelihood_2d(y, S));
    if (std::isnan(lambda[i]) || std::isinf(lambda[i])) lambda[i] = 1e-9;
  }

  // Update model probabilities: mu_j = lambda_j * cbar_j / sum(lambda_k * cbar_k)
  std::array<double, kMaxModels> cbar{};
  double sum_c_lambda = 0.0;
  for (int j = 0; j < kMaxModels; ++j) {
    if (!enabled_[j]) continue;
    cbar[j] = 0.0;
    for (int i = 0; i < kMaxModels; ++i) {
      if (!enabled_[i]) continue;
      cbar[j] += trans_prob_(j, i) * mu_[i];
    }
    mu_prior_[j] = cbar[j];
    sum_c_lambda += lambda[j] * cbar[j];
  }

  if (sum_c_lambda > 1e-30) {
    for (int i = 0; i < kMaxModels; ++i) {
      if (!enabled_[i]) continue;
      mu_[i] = lambda[i] * mu_prior_[i] / sum_c_lambda;
    }
  } else {
    // Fallback: equal weight
    for (int i = 0; i < kMaxModels; ++i) {
      if (!enabled_[i]) continue;
      mu_[i] = 1.0 / static_cast<double>(active_models_);
    }
  }
}

// ── Combined XY output ──
void SingleArmorIMMTracker::compute_combined_xy() {
  state_.pos.x() = 0.0;
  state_.pos.y() = 0.0;
  state_.vel.x() = 0.0;
  state_.vel.y() = 0.0;
  for (int i = 0; i < kMaxModels; ++i) {
    if (!enabled_[i]) continue;
    state_.pos.x() += mu_[i] * x_model_[i](0);
    state_.vel.x() += mu_[i] * x_model_[i](1);
    state_.pos.y() += mu_[i] * x_model_[i](3);
    state_.vel.y() += mu_[i] * x_model_[i](4);
  }
}

// ── Public API ──
void SingleArmorIMMTracker::predict(double dt) {
  if (!state_.initialized) return;
  dt = std::clamp(dt, 1e-3, 0.5);

  // XY IMM
  imm_mixing();
  imm_predict(dt);
  compute_combined_xy();

  // Z predict
  F_z_.setIdentity();
  F_z_(0, 1) = dt;
  const double dt2 = dt * dt;
  const double dt3 = dt2 * dt;
  const double dt4 = dt3 * dt;
  const Eigen::Matrix2d qz_blk = (Eigen::Matrix2d()
      << dt4 / 4.0, dt3 / 2.0, dt3 / 2.0, dt2).finished();
  Q_z_ = cfg_.q_z_vel * qz_blk;
  x_z_ = F_z_ * x_z_;
  P_z_ = F_z_ * P_z_ * F_z_.transpose() + Q_z_;
  state_.pos.z() = x_z_(0);
  state_.vel.z() = x_z_(1);

  // Yaw predict
  F_yaw_.setIdentity();
  F_yaw_(0, 1) = dt;
  Q_yaw_ = cfg_.q_yaw_rate * qz_blk;
  x_yaw_ = F_yaw_ * x_yaw_;
  P_yaw_ = F_yaw_ * P_yaw_ * F_yaw_.transpose() + Q_yaw_;
  state_.yaw = normalize_angle(x_yaw_(0));
  state_.yaw_rate = x_yaw_(1);
}

void SingleArmorIMMTracker::update(const Eigen::Vector3d &pos_meas,
                                    double yaw_meas,
                                    double pos_conf, double yaw_conf) {
  if (!state_.initialized) {
    initialize(pos_meas, yaw_meas);
    return;
  }

  const double pos_c = clamp_conf(pos_conf);
  const double xy_r_scale = 1.0 / pos_c;

  // XY IMM update
  imm_update(Eigen::Vector2d(pos_meas.x(), pos_meas.y()), xy_r_scale);
  compute_combined_xy();

  // Z update
  const double r_z = sq(cfg_.r_pos_base) / pos_c;
  const double y_z = pos_meas.z() - H_z_ * x_z_;
  const double S_z = (H_z_ * P_z_ * H_z_.transpose())(0, 0) + r_z;
  const Eigen::Vector2d K_z = P_z_ * H_z_.transpose() / S_z;
  x_z_ = x_z_ + K_z * y_z;
  P_z_ = (Eigen::Matrix2d::Identity() - K_z * H_z_) * P_z_;
  state_.pos.z() = x_z_(0);
  state_.vel.z() = x_z_(1);

  // Yaw update
  const double yaw_c = clamp_conf(yaw_conf);
  const double r_yaw = sq(cfg_.r_yaw_base) / yaw_c;
  const double z_yaw = unwrap_yaw(yaw_meas);
  const double y_yaw = z_yaw - H_yaw_ * x_yaw_;
  const double S_yaw = (H_yaw_ * P_yaw_ * H_yaw_.transpose())(0, 0) + r_yaw;
  const Eigen::Vector2d K_yaw = P_yaw_ * H_yaw_.transpose() / S_yaw;
  x_yaw_ = x_yaw_ + K_yaw * y_yaw;
  P_yaw_ = (Eigen::Matrix2d::Identity() - K_yaw * H_yaw_) * P_yaw_;
  state_.yaw = normalize_angle(x_yaw_(0));
  state_.yaw_rate = x_yaw_(1);
}

}  // namespace fyt::auto_aim::kalman
