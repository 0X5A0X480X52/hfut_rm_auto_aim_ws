#include "gimbal_controller/fire_advice/probability_engine.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace gimbal_controller::fire_advice
{

namespace
{

constexpr double kGravity = 9.81;
constexpr double kMinNorm = 1e-9;
Eigen::Vector3d rotateAroundWorldZ(const Eigen::Vector3d & v, double yaw_delta)
{
  const double c = std::cos(yaw_delta);
  const double s = std::sin(yaw_delta);
  return Eigen::Vector3d(c * v.x() - s * v.y(), s * v.x() + c * v.y(), v.z());
}

void buildArmorBasisFromNormal(
  const Eigen::Vector3d & normal,
  Eigen::Vector3d & right,
  Eigen::Vector3d & up)
{
  Eigen::Vector3d n = normal.norm() > kMinNorm ? normal.normalized() : Eigen::Vector3d::UnitX();
  Eigen::Vector3d ref = std::abs(n.z()) < 0.9 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitY();
  right = n.cross(ref).normalized();
  up = right.cross(n).normalized();
}

Eigen::Vector3d directionFromYawPitch(double yaw, double pitch)
{
  const double cp = std::cos(pitch);
  return Eigen::Vector3d(cp * std::cos(yaw), cp * std::sin(yaw), std::sin(pitch));
}

Eigen::Matrix3d worldFromBarrel(double yaw, double pitch)
{
  const Eigen::Vector3d x = directionFromYawPitch(yaw, pitch).normalized();
  Eigen::Vector3d z_world = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d y = z_world.cross(x);
  if (y.norm() <= kMinNorm) {
    y = Eigen::Vector3d::UnitY();
  } else {
    y.normalize();
  }
  Eigen::Vector3d z = x.cross(y).normalized();
  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;
  return R;
}

}  // namespace

void ProbabilityEngine::setConfig(
  const ProbabilityConfig & cfg,
  const SigmaPointConfig & sigma_cfg,
  const FireGateConfig & gate_cfg)
{
  cfg_ = cfg;
  sigma_cfg_ = sigma_cfg;
  gate_cfg_ = gate_cfg;
  resetGateState();
}

double ProbabilityEngine::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

double ProbabilityEngine::normalCdf(double x)
{
  return 0.5 * (1.0 + std::erf(x / std::sqrt(2.0)));
}

double ProbabilityEngine::probabilityInside1d(double mean, double sigma, double half_size)
{
  const double s = std::max(sigma, 1e-6);
  const double upper = (half_size - mean) / s;
  const double lower = (-half_size - mean) / s;
  return clamp(normalCdf(upper) - normalCdf(lower), 0.0, 1.0);
}

double ProbabilityEngine::hitProbabilityIndependent(
  double e_u,
  double e_v,
  double sigma_u,
  double sigma_v,
  double width,
  double height)
{
  const double pu = probabilityInside1d(e_u, sigma_u, width * 0.5);
  const double pv = probabilityInside1d(e_v, sigma_v, height * 0.5);
  return clamp(pu * pv, 0.0, 1.0);
}

bool ProbabilityEngine::extractTrackerCovariance(
  const rm_interfaces::msg::TrackedRobot & robot,
  Eigen::Matrix3d & cov_xyz,
  std::string & source) const
{
  cov_xyz = Eigen::Matrix3d::Zero();
  cov_xyz(0, 0) = cfg_.fallback_sigma_x * cfg_.fallback_sigma_x;
  cov_xyz(1, 1) = cfg_.fallback_sigma_y * cfg_.fallback_sigma_y;
  cov_xyz(2, 2) = cfg_.fallback_sigma_z * cfg_.fallback_sigma_z;
  source = "fallback";

  if (!cfg_.use_tracker_covariance) {
    return true;
  }

  const int dim = static_cast<int>(robot.covariance_dim);
  if (dim < 3 || robot.state_covariance.size() < static_cast<size_t>(dim * dim)) {
    if (cfg_.strict_covariance) {
      source = "invalid";
      return false;
    }
    return true;
  }

  const auto & cov = robot.state_covariance;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      cov_xyz(r, c) = cov[r * dim + c];
    }
  }
  source = "tracker";
  return true;
}

std::vector<Eigen::Vector2d> ProbabilityEngine::buildSigmaPoints(
  const SigmaPointConfig & cfg,
  std::vector<double> & wm,
  std::vector<double> & wc)
{
  std::vector<Eigen::Vector2d> points;
  wm.clear();
  wc.clear();

  if (!cfg.enable) {
    points.push_back(Eigen::Vector2d::Zero());
    wm.push_back(1.0);
    wc.push_back(1.0);
    return points;
  }

  const double s1 = std::max(cfg.sigma_v0, 1e-6);
  const double s2 = std::max(cfg.sigma_delay, 1e-6);
  const double rho = std::clamp(cfg.rho, -0.99, 0.99);
  Eigen::Matrix2d cov;
  cov << s1 * s1, rho * s1 * s2,
    rho * s1 * s2, s2 * s2;

  if (!cfg.use_unscented) {
    points = {
      {0.0, 0.0},
      {s1, 0.0},
      {-s1, 0.0},
      {0.0, s2},
      {0.0, -s2}
    };
    wm = {0.4, 0.15, 0.15, 0.15, 0.15};
    wc = wm;
    return points;
  }

  const int n = 2;
  const double alpha = std::max(cfg.alpha, 1e-3);
  const double beta = cfg.beta;
  const double lambda = alpha * alpha * (n + cfg.kappa) - n;
  const double scaling = n + lambda;
  if (scaling <= 1e-9) {
    points.push_back(Eigen::Vector2d::Zero());
    wm.push_back(1.0);
    wc.push_back(1.0);
    return points;
  }

  Eigen::LLT<Eigen::Matrix2d> llt(scaling * cov);
  if (llt.info() != Eigen::Success) {
    points.push_back(Eigen::Vector2d::Zero());
    wm.push_back(1.0);
    wc.push_back(1.0);
    return points;
  }
  const Eigen::Matrix2d L = llt.matrixL();

  points.reserve(2 * n + 1);
  points.push_back(Eigen::Vector2d::Zero());
  for (int i = 0; i < n; ++i) {
    points.push_back(L.col(i));
    points.push_back(-L.col(i));
  }

  wm.resize(2 * n + 1, 0.5 / scaling);
  wc.resize(2 * n + 1, 0.5 / scaling);
  wm[0] = lambda / scaling;
  wc[0] = lambda / scaling + (1.0 - alpha * alpha + beta);
  return points;
}

void ProbabilityEngine::resetGateState()
{
  score_ = 0.0;
  fire_state_ = false;
  burst_probability_ = 0.0;
  log_evidence_ = 0.0;
  evidence_sum_ = 0.0;
  evidence_strength_ = 0.0;
  evidence_window_.clear();
  burst_state_ = BurstGateState::kIdle;
  burst_state_time_s_ = 0.0;
}

double ProbabilityEngine::burstProbability(
  const std::vector<double> & p_hits,
  int burst_count,
  int min_hit_count)
{
  if (p_hits.empty()) {
    return 0.0;
  }

  const int n = std::max(0, std::min(static_cast<int>(p_hits.size()), burst_count));
  if (n <= 0) {
    return 0.0;
  }

  const int m = std::max(1, min_hit_count);
  if (m > n) {
    return 0.0;
  }

  std::vector<double> dp(n + 1, 0.0);
  dp[0] = 1.0;
  for (int i = 0; i < n; ++i) {
    const double p = clamp(p_hits[i], 0.0, 1.0);
    for (int k = i; k >= 0; --k) {
      dp[k + 1] += dp[k] * p;
      dp[k] *= (1.0 - p);
    }
  }

  double sum = 0.0;
  for (int k = m; k <= n; ++k) {
    sum += dp[k];
  }
  return clamp(sum, 0.0, 1.0);
}

void ProbabilityEngine::updateGateLegacy(double p_window, double dt_s)
{
  const double dt = std::max(dt_s, 1e-4);
  if (gate_cfg_.integrator_mode) {
    const double base = gate_cfg_.integrator_base_probability;
    const double rise = gate_cfg_.integrator_rise;
    const double fall = gate_cfg_.integrator_fall;
    score_ = clamp(
      score_ + dt * (rise * std::max(0.0, p_window - base) - fall * std::max(0.0, base - p_window)),
      0.0,
      1.0);
  } else {
    score_ = gate_cfg_.alpha * score_ + (1.0 - gate_cfg_.alpha) * p_window;
  }

  if (!fire_state_ && score_ > gate_cfg_.fire_on_th) {
    fire_state_ = true;
  } else if (fire_state_ && score_ < gate_cfg_.fire_off_th) {
    fire_state_ = false;
  }

  burst_probability_ = 0.0;
  log_evidence_ = 0.0;
  evidence_sum_ = 0.0;
  evidence_strength_ = 0.0;
  burst_state_ = BurstGateState::kIdle;
  burst_state_time_s_ = 0.0;
}

void ProbabilityEngine::updateGateBurstEvidence(const std::vector<double> & p_hits, double dt_s)
{
  const double dt = std::max(dt_s, 1e-4);
  const int burst_count = std::max(1, gate_cfg_.burst_bullet_count);
  const int min_hit = std::max(1, gate_cfg_.min_hit_count);

  const double clip = std::max(gate_cfg_.log_evidence_clip, 1e-6);
  if (p_hits.empty()) {
    // "Unshootable" samples are treated as neutral evidence so they do not bury later positive windows.
    burst_probability_ = 0.0;
    log_evidence_ = 0.0;
  } else {
    burst_probability_ = burstProbability(p_hits, burst_count, min_hit);

    const double eps = std::max(gate_cfg_.evidence_epsilon, 1e-6);
    const double p0 = clamp(gate_cfg_.reference_probability_p0, eps, 1.0 - eps);
    const double pb = clamp(burst_probability_, eps, 1.0 - eps);
    const double raw_log_evidence =
      std::log((pb * (1.0 - p0)) / (p0 * (1.0 - pb)));
    const double neg_clip = std::max(clip * std::max(gate_cfg_.negative_clip_scale, 0.0), 1e-6);
    log_evidence_ = raw_log_evidence >= 0.0 ?
      clamp(raw_log_evidence, 0.0, clip) :
      clamp(raw_log_evidence, -neg_clip, 0.0) * std::max(gate_cfg_.negative_evidence_scale, 0.0);
    if (std::abs(log_evidence_) < std::max(gate_cfg_.evidence_deadband, 0.0)) {
      log_evidence_ = 0.0;
    }
  }

  const double window_s = std::max(gate_cfg_.evidence_window_ms, 0.0) * 1e-3;
  const int max_size = std::max(1, static_cast<int>(std::round(window_s / dt)));
  evidence_window_.push_back(log_evidence_);
  evidence_sum_ += log_evidence_;
  while (evidence_window_.size() > static_cast<size_t>(max_size)) {
    evidence_sum_ -= evidence_window_.front();
    evidence_window_.pop_front();
  }

  const double ml = static_cast<double>(max_size) * clip;
  if (ml > 1e-9) {
    evidence_strength_ = clamp((evidence_sum_ + ml) / (2.0 * ml), 0.0, 1.0);
  } else {
    evidence_strength_ = 0.5;
  }

  const double T = clamp(gate_cfg_.temperature, 0.0, 1.0);
  const double theta_on = clamp(
    gate_cfg_.theta_on_cold - (gate_cfg_.theta_on_cold - gate_cfg_.theta_on_hot) * T,
    0.0,
    1.0);
  const double theta_hold = clamp(
    gate_cfg_.theta_hold_cold - (gate_cfg_.theta_hold_cold - gate_cfg_.theta_hold_hot) * T,
    0.0,
    1.0);
  const double min_fire_s = std::max(gate_cfg_.min_fire_ms, 0.0) * 1e-3;

  switch (burst_state_) {
    case BurstGateState::kIdle:
      burst_state_time_s_ = 0.0;
      if (evidence_strength_ >= theta_on) {
        burst_state_ = BurstGateState::kFireCommit;
        burst_state_time_s_ = 0.0;
        fire_state_ = true;
      } else {
        fire_state_ = false;
      }
      break;
    case BurstGateState::kFireCommit:
      fire_state_ = true;
      burst_state_time_s_ += dt;
      if (burst_state_time_s_ >= min_fire_s) {
        if (evidence_strength_ < theta_hold) {
          burst_state_ = BurstGateState::kIdle;
          burst_state_time_s_ = 0.0;
          fire_state_ = false;
        }
      }
      break;
  }

  score_ = evidence_strength_;
}

void ProbabilityEngine::updateGate(double p_window, const std::vector<double> & p_hits, double dt_s)
{
  if (gate_cfg_.strategy == FireGateConfig::Strategy::kBurstEvidence) {
    updateGateBurstEvidence(p_hits, dt_s);
    return;
  }

  updateGateLegacy(p_window, dt_s);
}

ProbabilityDebugResult ProbabilityEngine::evaluate(
  const rm_interfaces::msg::TrackedRobot & robot,
  double distance,
  double yaw_error,
  double pitch_error,
  double flight_time_s,
  double bullet_speed,
  double muzzle_yaw,
  double muzzle_pitch,
  double muzzle_yaw_rate,
  double muzzle_pitch_rate,
  double muzzle_yaw_accel,
  double muzzle_pitch_accel,
  const Eigen::Vector3d & armor_center,
  const Eigen::Vector3d & armor_normal,
  const Eigen::Vector3d & target_center_velocity,
  double armor_yaw_rate,
  double dt_s,
  const std::string & target_id)
{
  ProbabilityDebugResult out;
  (void)distance;

  if (active_target_id_ != target_id) {
    active_target_id_ = target_id;
    resetGateState();
  }

  Eigen::Matrix3d tracker_cov_xyz = Eigen::Matrix3d::Zero();
  std::string covariance_source;
  if (!extractTrackerCovariance(robot, tracker_cov_xyz, covariance_source)) {
    return out;
  }

  const double tf = std::max(flight_time_s, 0.0);
  const double v0 = std::max(bullet_speed, 1e-3);
  const Eigen::Vector3d v_center = target_center_velocity;

  Eigen::Vector3d n0 = armor_normal.norm() > 1e-6 ? armor_normal.normalized() : Eigen::Vector3d::UnitX();
  Eigen::Vector3d right0 = Eigen::Vector3d::UnitY();
  Eigen::Vector3d up0 = Eigen::Vector3d::UnitZ();
  buildArmorBasisFromNormal(n0, right0, up0);

  out.armor_center = armor_center;
  out.armor_right = right0;
  out.armor_up = up0;
  out.armor_width_m = std::max(cfg_.armor_width_m, 1e-6);
  out.armor_height_m = std::max(cfg_.armor_height_m, 1e-6);

  const double step_s = std::max(cfg_.future_step_ms, 1.0) * 1e-3;
  const int steps = std::max(1, static_cast<int>(std::round(std::max(cfg_.future_window_ms, 0.0) * 1e-3 / step_s)));
  std::vector<double> wm;
  std::vector<double> wc;
  const auto sigma_points = buildSigmaPoints(sigma_cfg_, wm, wc);

  double best_p = -1.0;
  std::vector<double> p_hits;
  p_hits.reserve(static_cast<size_t>(steps) + 1);
  for (int i = 0; i <= steps; ++i) {
    const double tau = i * step_s;
    const Eigen::Vector3d c_nominal = armor_center + v_center * tau;
    const Eigen::Vector3d n_nominal = rotateAroundWorldZ(n0, armor_yaw_rate * tau).normalized();
    Eigen::Vector3d right_nominal = Eigen::Vector3d::UnitY();
    Eigen::Vector3d up_nominal = Eigen::Vector3d::UnitZ();
    buildArmorBasisFromNormal(n_nominal, right_nominal, up_nominal);

    Eigen::Matrix2d sigma_sp_uv = Eigen::Matrix2d::Zero();
    Eigen::Vector2d mean_uv = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> uv_points;
    uv_points.reserve(sigma_points.size());

    for (const auto & pt : sigma_points) {
      const double dv0 = pt.x();
      const double dtd = pt.y();
      const double vj = std::max(v0 + dv0, 1e-3);
      const double tau_j = std::max(tau + dtd, 0.0);
      const double yaw_s = muzzle_yaw + muzzle_yaw_rate * tau_j + 0.5 * muzzle_yaw_accel * tau_j * tau_j;
      const double pitch_s = muzzle_pitch + muzzle_pitch_rate * tau_j +
        0.5 * muzzle_pitch_accel * tau_j * tau_j;
      const Eigen::Matrix3d R_O_Bs = worldFromBarrel(yaw_s, pitch_s);

      const Eigen::Vector3d c_tau = armor_center + v_center * tau_j;
      const double dist_j = std::max(c_tau.norm(), 1e-3);
      const double tfj = dist_j / vj;
      const double yaw_j = yaw_error;
      const double pitch_j = pitch_error;

      const Eigen::Vector3d p_b_Bs(
        vj * tfj * std::cos(pitch_j) * std::cos(yaw_j),
        vj * tfj * std::cos(pitch_j) * std::sin(yaw_j),
        vj * tfj * std::sin(pitch_j) - 0.5 * kGravity * tfj * tfj);
      const Eigen::Vector3d bullet = R_O_Bs * p_b_Bs;

      const Eigen::Vector3d c_impact_ref = armor_center + v_center * (tau_j + tfj - tf);
      const Eigen::Vector3d n_impact = rotateAroundWorldZ(n0, armor_yaw_rate * (tau_j + tfj - tf)).normalized();
      Eigen::Vector3d r_impact = Eigen::Vector3d::UnitY();
      Eigen::Vector3d u_impact = Eigen::Vector3d::UnitZ();
      buildArmorBasisFromNormal(n_impact, r_impact, u_impact);

      const Eigen::Vector3d delta = bullet - c_impact_ref;
      Eigen::Vector2d uv(r_impact.dot(delta), u_impact.dot(delta));
      uv_points.push_back(uv);
    }

    for (size_t k = 0; k < uv_points.size(); ++k) {
      mean_uv += wm[k] * uv_points[k];
    }
    for (size_t k = 0; k < uv_points.size(); ++k) {
      const Eigen::Vector2d duv = uv_points[k] - mean_uv;
      sigma_sp_uv += wc[k] * (duv * duv.transpose());
    }
    sigma_sp_uv = 0.5 * (sigma_sp_uv + sigma_sp_uv.transpose());

    const double e_u = mean_uv.x();
    const double e_v = mean_uv.y();

    const double growth_t = tf + tau;
    const double sx_b = std::abs(cfg_.sigma_x0 + cfg_.growth_x * growth_t);
    const double sy_b = std::abs(cfg_.sigma_y0 + cfg_.growth_y * growth_t);
    const double sz_b = std::abs(cfg_.sigma_z0 + cfg_.growth_z * growth_t);
    Eigen::Matrix3d bullet_cov_xyz = Eigen::Matrix3d::Zero();
    bullet_cov_xyz(0, 0) = sx_b * sx_b;
    bullet_cov_xyz(1, 1) = sy_b * sy_b;
    bullet_cov_xyz(2, 2) = sz_b * sz_b;

    Eigen::Matrix<double, 2, 3> J;
    J.row(0) = right_nominal.transpose();
    J.row(1) = up_nominal.transpose();
    const Eigen::Matrix2d sigma_b_uv = J * bullet_cov_xyz * J.transpose();
    const Eigen::Matrix2d sigma_tracker_uv = J * tracker_cov_xyz * J.transpose();
    const Eigen::Matrix2d sigma_total_uv = sigma_b_uv + sigma_tracker_uv + sigma_sp_uv;
    const double sigma_u = std::sqrt(std::max(sigma_total_uv(0, 0), 1e-12));
    const double sigma_v = std::sqrt(std::max(sigma_total_uv(1, 1), 1e-12));

    double p_hit = hitProbabilityIndependent(
      e_u, e_v, sigma_u, sigma_v, out.armor_width_m, out.armor_height_m);

    const double yaw_nom = muzzle_yaw + muzzle_yaw_rate * tau + 0.5 * muzzle_yaw_accel * tau * tau;
    const double pitch_nom = muzzle_pitch + muzzle_pitch_rate * tau + 0.5 * muzzle_pitch_accel * tau * tau;
    const Eigen::Matrix3d R_O_Bs_nom = worldFromBarrel(yaw_nom, pitch_nom);
    const double dist_nom = std::max(c_nominal.norm(), 1e-3);
    const double tf_nom = dist_nom / v0;
    const Eigen::Vector3d v_b_Bs_nom(
      v0 * std::cos(pitch_error) * std::cos(yaw_error),
      v0 * std::cos(pitch_error) * std::sin(yaw_error),
      v0 * std::sin(pitch_error) - kGravity * tf_nom);
    const Eigen::Vector3d v_b_world_nom = R_O_Bs_nom * v_b_Bs_nom;
    const double impact_dt = tau + tf_nom - tf;
    const Eigen::Vector3d n_impact_nom = rotateAroundWorldZ(n0, armor_yaw_rate * impact_dt).normalized();
    const double dot_vn = v_b_world_nom.dot(n_impact_nom);
    const double v_norm = std::max(v_b_world_nom.norm(), 1e-6);
    const double n_norm = std::max(n_impact_nom.norm(), 1e-6);
    const double cos_vn = clamp(dot_vn / (v_norm * n_norm), -1.0, 1.0);
    const double max_comp_angle_deg = clamp(cfg_.max_complement_angle_deg, 0.0, 180.0);
    const double cos_comp_th = std::cos(max_comp_angle_deg * M_PI / 180.0);
    const double front_cos_threshold = std::min(-cos_comp_th, -std::max(cfg_.front_face_epsilon, 0.0));
    const bool front_ok = cos_vn <= front_cos_threshold;
    const double normal_velocity = std::max(0.0, -dot_vn);

    bool normal_gate_pass = true;
    if (cfg_.enable_normal_velocity_gate) {
      if (cfg_.require_front_face && !front_ok) {
        normal_gate_pass = false;
      }
      if (normal_velocity < std::max(cfg_.normal_v_activate_min, 0.0)) {
        normal_gate_pass = false;
      }
    }
    double normal_weight = 1.0;
    if (normal_gate_pass && cfg_.enable_normal_velocity_weight) {
      const double v_ref = std::max(cfg_.normal_v_ref, 1e-3);
      const double w_min = clamp(cfg_.normal_w_min, 0.0, 1.0);
      normal_weight = clamp(normal_velocity / v_ref, w_min, 1.0);
      p_hit *= normal_weight;
    }
    if (!normal_gate_pass) {
      p_hit = 0.0;
    }

    TauDebugSample s;
    s.tau_s = tau;
    s.p_hit = p_hit;
    s.e_u = e_u;
    s.e_v = e_v;
    s.sigma_u = sigma_u;
    s.sigma_v = sigma_v;
    s.front_ok = front_ok;
    s.normal_gate_pass = normal_gate_pass;
    s.normal_velocity = normal_velocity;
    s.normal_weight = normal_weight;
    const Eigen::Vector3d p3 = c_nominal + right_nominal * e_u + up_nominal * e_v;
    s.impact_x = p3.x();
    s.impact_y = p3.y();
    s.impact_z = p3.z();
    out.tau_samples.push_back(s);
    if (normal_gate_pass || !gate_cfg_.neutralize_unshootable_samples) {
      p_hits.push_back(p_hit);
    }

    if (p_hit > best_p) {
      best_p = p_hit;
      out.armor_center = c_nominal;
      out.armor_right = right_nominal;
      out.armor_up = up_nominal;
      out.best_tau_s = tau;
      out.best_p_hit = p_hit;
      out.best_e_u = e_u;
      out.best_e_v = e_v;
      out.best_sigma_u = sigma_u;
      out.best_sigma_v = sigma_v;
    }
  }

  if (out.tau_samples.empty()) {
    return out;
  }

  if (cfg_.softmax_fusion) {
    const double beta = cfg_.softmax_beta;
    double denom = 0.0;
    double numer = 0.0;
    for (const auto & s : out.tau_samples) {
      const double w = std::exp(beta * s.p_hit);
      denom += w;
      numer += w * s.p_hit;
    }
    out.p_window = denom > 1e-9 ? numer / denom : out.best_p_hit;
  } else {
    out.p_window = out.best_p_hit;
  }

  updateGate(out.p_window, p_hits, dt_s);
  out.fire_score = score_;
  out.fire_state = fire_state_;
  out.burst_probability = burst_probability_;
  out.log_evidence = log_evidence_;
  out.evidence_sum = evidence_sum_;
  out.evidence_strength = evidence_strength_;
  out.gate_strategy = static_cast<int>(gate_cfg_.strategy);
  out.gate_state = static_cast<int>(burst_state_);
  out.valid = true;
  return out;
}

}  // namespace gimbal_controller::fire_advice
