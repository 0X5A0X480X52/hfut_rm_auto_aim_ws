// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_baseline/backends/norm4_single_armor_imm_bundle.hpp"

namespace fyt::auto_aim::norm4_baseline {

namespace {

StateLayout build_imm_layout() {
  StateLayout layout;
  layout.register_state("X", 0);
  layout.register_state("VX", 1);
  layout.register_state("AX", 2);
  layout.register_state("Y", 3);
  layout.register_state("VY", 4);
  layout.register_state("AY", 5);
  layout.register_state("Z", 6);
  layout.register_state("VZ", 7);
  layout.register_state("DELTA", 8);
  layout.register_state("DELTA_RATE", 9);
  layout.register_state("R1", 10);
  layout.register_state("R2", 11);
  layout.register_state("DZA", 12);
  layout.freeze();
  return layout;
}

}  // namespace

SingleArmorIMMBundle::SingleArmorIMMBundle(const ImmBundleConfig &cfg)
    : cfg_(cfg), state_idx_(build_imm_layout()) {
  imm_cfg_ = std::make_unique<kalman::SingleArmorIMMConfig>();
  imm_cfg_->dt = 0.05;
  imm_cfg_->enable_cv = cfg.enable_cv;
  imm_cfg_->enable_ca = cfg.enable_ca;
  imm_cfg_->enable_cs = cfg.enable_cs;
  imm_cfg_->enable_ctrv = cfg.enable_ctrv;
  imm_cfg_->q_cv = cfg.q_cv;
  imm_cfg_->q_ca = cfg.q_ca;
  imm_cfg_->cs_alpha = cfg.cs_alpha;
  imm_cfg_->cs_a_max = cfg.cs_a_max;
  imm_cfg_->p_stay = cfg.p_stay;
  imm_cfg_->p_switch = cfg.p_switch;
  imm_cfg_->q_z_vel = cfg.q_z_vel;
  imm_cfg_->q_yaw_rate = cfg.q_yaw_rate;
  imm_cfg_->r_pos_base = cfg.r_pos_base;
  imm_cfg_->r_yaw_base = cfg.r_yaw_base;
  imm_ = std::make_unique<kalman::SingleArmorIMMTracker>(*imm_cfg_);
}

Eigen::VectorXd SingleArmorIMMBundle::predict(const Eigen::VectorXd &x,
                                               double dt) {
  auto idx = state_idx_;

  // ── XY plane: simple CV predict (conservative, matches IMM internal model) ──
  Eigen::Matrix<double, 6, 1> xy;
  xy << x(idx.X()), x(idx.VX()), x(idx.AX()),
        x(idx.Y()), x(idx.VY()), x(idx.AY());

  // CV transition: x+dx, vx, ax -> vx*dt, 0; same for y
  Eigen::Matrix<double, 6, 6> F_xy = Eigen::Matrix<double, 6, 6>::Identity();
  F_xy(0, 1) = dt;   // x += vx * dt
  F_xy(0, 2) = 0.0;  // ax not integrated in CV
  F_xy(2, 2) = 0.0;  // ax decays
  F_xy(3, 4) = dt;   // y += vy * dt
  F_xy(3, 5) = 0.0;
  F_xy(5, 5) = 0.0;

  Eigen::Matrix<double, 6, 1> xy_pred = F_xy * xy;

  // ── Z axis: CV predict ──
  double z = x(idx.Z());
  double vz = x(idx.VZ());
  double z_pred = z + vz * dt;
  double vz_pred = vz;

  // ── Yaw: CV predict ──
  double delta = x(idx.DELTA());
  double delta_rate = x(idx.DELTA_RATE());
  double delta_pred = normalize_angle(delta + delta_rate * dt);
  double delta_rate_pred = delta_rate;

  // ── Structural: random walk ──
  double r1 = x(idx.R1());
  double r2 = x(idx.R2());
  double dza = x(idx.DZA());

  Eigen::VectorXd x_pred(kStateDim);
  x_pred(idx.X()) = xy_pred(0);
  x_pred(idx.VX()) = xy_pred(1);
  x_pred(idx.AX()) = xy_pred(2);
  x_pred(idx.Y()) = xy_pred(3);
  x_pred(idx.VY()) = xy_pred(4);
  x_pred(idx.AY()) = xy_pred(5);
  x_pred(idx.Z()) = z_pred;
  x_pred(idx.VZ()) = vz_pred;
  x_pred(idx.DELTA()) = delta_pred;
  x_pred(idx.DELTA_RATE()) = delta_rate_pred;
  x_pred(idx.R1()) = r1;
  x_pred(idx.R2()) = r2;
  x_pred(idx.DZA()) = dza;
  return x_pred;
}

Eigen::MatrixXd SingleArmorIMMBundle::build_Q(double dt) const {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(kStateDim, kStateDim);
  auto idx = state_idx_;

  // XY discrete noise (CV model: velocity noise drives position, acc noise drives acc)
  double q_cv = cfg_.q_cv;
  double q_ca = cfg_.q_ca;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;

  auto add_xy_noise = [&](int px, int pvx, int pax) {
    Q(px, px) += q_cv * dt3 / 3.0 + q_ca * dt4 / 4.0;
    Q(px, pvx) += q_cv * dt2 / 2.0 + q_ca * dt3 / 3.0;
    Q(px, pax) += q_ca * dt3 / 3.0;
    Q(pvx, px) += q_cv * dt2 / 2.0 + q_ca * dt3 / 3.0;
    Q(pvx, pvx) += q_cv * dt + q_ca * dt2;
    Q(pvx, pax) += q_ca * dt2;
    Q(pax, px) += q_ca * dt3 / 3.0;
    Q(pax, pvx) += q_ca * dt2;
    Q(pax, pax) += q_ca * dt;
  };

  add_xy_noise(idx.X(), idx.VX(), idx.AX());
  add_xy_noise(idx.Y(), idx.VY(), idx.AY());

  // Z noise
  double q_zv = cfg_.q_z_vel;
  Q(idx.Z(), idx.Z()) += q_zv * dt3 / 3.0;
  Q(idx.Z(), idx.VZ()) += q_zv * dt2 / 2.0;
  Q(idx.VZ(), idx.Z()) += q_zv * dt2 / 2.0;
  Q(idx.VZ(), idx.VZ()) += q_zv * dt;

  // Yaw noise
  double q_yr = cfg_.q_yaw_rate;
  Q(idx.DELTA(), idx.DELTA()) += q_yr * dt3 / 3.0;
  Q(idx.DELTA(), idx.DELTA_RATE()) += q_yr * dt2 / 2.0;
  Q(idx.DELTA_RATE(), idx.DELTA()) += q_yr * dt2 / 2.0;
  Q(idx.DELTA_RATE(), idx.DELTA_RATE()) += q_yr * dt;

  // Structural random walk
  Q(idx.R1(), idx.R1()) = cfg_.q_r * dt;
  Q(idx.R2(), idx.R2()) = cfg_.q_r * dt;
  Q(idx.DZA(), idx.DZA()) = cfg_.q_dza * dt;

  return Q;
}

Eigen::VectorXd SingleArmorIMMBundle::initial_state(
    const ObservationData &obs, int panel_id, double r1, double r2,
    double dza) const {
  const int p = ((panel_id % 4) + 4) % 4;
  double panel_angle = p * (M_PI / 2.0);
  double use_r = (p % 2 == 0) ? r1 : r2;

  double center_x = obs.x - use_r * std::cos(obs.yaw);
  double center_y = obs.y - use_r * std::sin(obs.yaw);
  double center_yaw = normalize_angle(obs.yaw - panel_angle);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(kStateDim);
  auto idx = state_idx_;
  x0(idx.X()) = center_x;
  x0(idx.VX()) = 0.0;
  x0(idx.AX()) = 0.0;
  x0(idx.Y()) = center_y;
  x0(idx.VY()) = 0.0;
  x0(idx.AY()) = 0.0;
  x0(idx.Z()) = obs.z;
  x0(idx.VZ()) = 0.0;
  x0(idx.DELTA()) = center_yaw;
  x0(idx.DELTA_RATE()) = 0.0;
  x0(idx.R1()) = r1;
  x0(idx.R2()) = r2;
  x0(idx.DZA()) = dza;
  return x0;
}

Eigen::MatrixXd SingleArmorIMMBundle::initial_covariance() const {
  Eigen::MatrixXd P = Eigen::MatrixXd::Identity(kStateDim, kStateDim) * 100.0;
  // Reduce initial uncertainty on structural params
  auto idx = state_idx_;
  P(idx.R1(), idx.R1()) = 0.01;
  P(idx.R2(), idx.R2()) = 0.01;
  P(idx.DZA(), idx.DZA()) = 0.001;
  return P;
}

}  // namespace fyt::auto_aim::norm4_baseline
