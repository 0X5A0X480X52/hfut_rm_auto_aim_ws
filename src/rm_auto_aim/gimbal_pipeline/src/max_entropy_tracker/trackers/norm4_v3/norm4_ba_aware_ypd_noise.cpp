// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v3/interfaces/norm4_ba_aware_ypd_noise.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "max_entropy_tracker/utils/constraints.hpp"

namespace fyt::auto_aim::norm4_v3 {

namespace {
constexpr double kMinSigmaSq = 1e-12;

double clamp_sigma(double val, double lo, double hi) {
  return std::clamp(val, lo, hi);
}

// yaw noise from corner pixel positions and bbox size
double compute_sigma_yaw(double sigma_corner_px, double sigma_yaw_scale,
                         double max_bbox_dim, double min_yaw, double max_yaw) {
  if (max_bbox_dim < 1.0) return max_yaw;
  double sy = sigma_yaw_scale * sigma_corner_px / max_bbox_dim;
  return clamp_sigma(sy, min_yaw, max_yaw);
}

// Diagonal SPD check + symmetry enforcement
Eigen::Matrix4d diag_spd_check(const Eigen::Matrix4d &R,
                               const Eigen::Vector4d &floor_diag) {
  Eigen::Matrix4d out = 0.5 * (R + R.transpose());
  for (int i = 0; i < 4; ++i) {
    if (out(i, i) < floor_diag(i) || !std::isfinite(out(i, i))) {
      out(i, i) = floor_diag(i);
    }
  }
  const Eigen::MatrixXd spd = fyt::auto_aim::ensure_positive_definite(out, 1e-9);
  return spd.topLeftCorner<4, 4>();
}
}  // namespace

BaAwareYpdNoiseModel::BaAwareYpdNoiseModel(
    const MeasurementNoiseConfig &cfg, const Norm4V3UkfConfig &ukf_cfg)
    : cfg_(cfg), ukf_config_(ukf_cfg) {}

Eigen::Matrix4d BaAwareYpdNoiseModel::build_R_fixed() const {
  const auto &f = cfg_.r_fixed;
  Eigen::Vector4d d(f.sigma_x * f.sigma_x,
                    f.sigma_y * f.sigma_y,
                    f.sigma_z * f.sigma_z,
                    f.sigma_yaw * f.sigma_yaw);
  return d.asDiagonal();
}

Eigen::Matrix4d BaAwareYpdNoiseModel::build_R_floor() const {
  const auto &f = cfg_.r_floor;
  Eigen::Vector4d d(f.sigma_x * f.sigma_x,
                    f.sigma_y * f.sigma_y,
                    f.sigma_z * f.sigma_z,
                    f.sigma_yaw * f.sigma_yaw);
  return d.asDiagonal();
}

Eigen::Matrix4d BaAwareYpdNoiseModel::build_R_ypd(
    const ObservationData &obs) const {
  const auto &yp = cfg_.ypd_prior;
  const auto &cam = cfg_.camera;

  // ── Fallback when no image metadata ──
  if (!obs.image.has_value() || !obs.image->valid) {
    return build_R_fixed();
  }

  const auto &img = *obs.image;

  // ── Distance estimates from bbox size ──
  double armor_w = cfg_.armor_geometry.small_width;
  double armor_h = cfg_.armor_geometry.small_height;
  if (img.type == "large") {
    armor_w = cfg_.armor_geometry.large_width;
    armor_h = cfg_.armor_geometry.large_height;
  } else if (img.type == "outpost") {
    armor_w = cfg_.armor_geometry.outpost_width;
    armor_h = cfg_.armor_geometry.outpost_height;
  }

  double bbox_w = std::max(img.bbox_w, 2.0);
  double bbox_h = std::max(img.bbox_h, 2.0);

  double dist_w = cam.fx * armor_w / bbox_w;
  double dist_h = cam.fy * armor_h / bbox_h;
  double dist_est = (dist_w + dist_h) * 0.5;

  // ── Angular noise from pixel center uncertainty ──
  double du = img.image_center_x - cam.cx;
  double dv = img.image_center_y - cam.cy;
  double sigma_azi = cam.fx / (cam.fx * cam.fx + du * du) * yp.sigma_center_px;
  double sigma_ele = cam.fy / (cam.fy * cam.fy + dv * dv) * yp.sigma_center_px;
  sigma_azi = clamp_sigma(sigma_azi, yp.sigma_azi_min, yp.sigma_azi_max);
  sigma_ele = clamp_sigma(sigma_ele, yp.sigma_ele_min, yp.sigma_ele_max);

  // ── Distance noise from bbox size uncertainty ──
  double sigma_dist_w = dist_w / bbox_w * yp.sigma_size_px;
  double sigma_dist_h = dist_h / bbox_h * yp.sigma_size_px;
  double inv_sd2 = 1.0 / std::max(sigma_dist_w * sigma_dist_w, kMinSigmaSq) +
                   1.0 / std::max(sigma_dist_h * sigma_dist_h, kMinSigmaSq);
  double sigma_dist = 1.0 / std::sqrt(inv_sd2);
  sigma_dist = clamp_sigma(sigma_dist, yp.sigma_dist_min, yp.sigma_dist_max);

  // ── Yaw noise from corner positions ──
  double max_bbox_dim = std::max(bbox_w, bbox_h);
  double sigma_yaw = compute_sigma_yaw(yp.sigma_corner_px, yp.sigma_yaw_scale,
                                       max_bbox_dim, yp.sigma_yaw_min,
                                       yp.sigma_yaw_max);

  // ── R_YPD_local in (azi, ele, dist, yaw) space ──
  Eigen::Vector4d diag_local(sigma_azi * sigma_azi,
                             sigma_ele * sigma_ele,
                             sigma_dist * sigma_dist,
                             sigma_yaw * sigma_yaw);

  // ── Jacobian d(x,y,z,yaw) / d(azi,ele,dist,yaw) ──
  // Phase 1: diagonal small-angle approximation.
  // x ≈ dist * azi,  y ≈ dist * ele,  z ≈ dist,  yaw = yaw.
  // ∂x/∂azi ≈ dist,  ∂y/∂ele ≈ dist,  ∂z/∂dist = 1.
  // Future: full spherical-to-Cartesian Jacobian for off-center targets.
  Eigen::Matrix4d J = Eigen::Matrix4d::Identity();
  J(0, 0) = dist_est;   // ∂x/∂azi
  J(1, 1) = dist_est;   // ∂y/∂ele

  Eigen::Matrix4d R_ypd = J * diag_local.asDiagonal() * J.transpose();

  // ── Quality scale from detection confidence ──
  const auto &qs = cfg_.quality_scale;
  if (qs.enable) {
    double conf = std::max(img.detection_confidence, qs.confidence_floor);
    double qs_factor = std::clamp(1.0 / conf, qs.min_scale, qs.max_scale);
    R_ypd *= qs_factor;
  }
  R_ypd *= yp.global_scale;

  // Ensure SPD
  Eigen::Vector4d floor_diag(kMinSigmaSq, kMinSigmaSq, kMinSigmaSq, kMinSigmaSq);
  return diag_spd_check(R_ypd, floor_diag);
}

Eigen::Matrix4d BaAwareYpdNoiseModel::build_R_ba(
    const ObservationData &obs, double &weight) const {
  weight = 0.0;
  const auto &ba = cfg_.ba_covariance;

  if (!ba.enable) return Eigen::Matrix4d::Identity();

  // ── Existence checks ──
  if (!obs.ba_pnp.has_value()) return Eigen::Matrix4d::Identity();
  const auto &meta = *obs.ba_pnp;
  if (!meta.valid) return Eigen::Matrix4d::Identity();

  // ── Required quality gates ──
  if (ba.require_cov_valid && !meta.cov_valid) return Eigen::Matrix4d::Identity();
  if (ba.require_frame_aligned && !meta.frame_aligned) return Eigen::Matrix4d::Identity();
  if (!std::isfinite(meta.confidence) || !std::isfinite(meta.reproj_rms) ||
      !std::isfinite(meta.condition_number)) {
    return Eigen::Matrix4d::Identity();
  }
  if (meta.confidence < ba.min_confidence) return Eigen::Matrix4d::Identity();
  if (meta.reproj_rms > ba.max_reproj_rms_px) return Eigen::Matrix4d::Identity();
  if (meta.condition_number > ba.max_condition_number) return Eigen::Matrix4d::Identity();
  if (meta.num_observations < ba.min_observations) return Eigen::Matrix4d::Identity();
  if (meta.num_observations > 0) {
    double inlier_ratio = static_cast<double>(meta.num_inliers) /
                          static_cast<double>(meta.num_observations);
    if (inlier_ratio < ba.min_inlier_ratio) return Eigen::Matrix4d::Identity();
  }

  // ── Covariance matrix cleanup ──
  Eigen::Matrix4d R_ba_raw = meta.cov_xyz_yaw;

  // Finite check
  if (!R_ba_raw.allFinite()) return Eigen::Matrix4d::Identity();

  // Symmetrize
  R_ba_raw = 0.5 * (R_ba_raw + R_ba_raw.transpose());

  // Eigenvalue clamp
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eig(R_ba_raw);
  if (eig.info() != Eigen::Success) return Eigen::Matrix4d::Identity();
  Eigen::Vector4d evals = eig.eigenvalues();
  const auto &ec = ba.eigen_clamp;
  for (int i = 0; i < 4; ++i) {
    evals(i) = std::clamp(evals(i), ec.min, ec.max);
  }
  Eigen::Matrix4d R_clamped =
      eig.eigenvectors() * evals.asDiagonal() * eig.eigenvectors().transpose();

  // Diagonal clamp
  const auto &dc = ba.diag_clamp;
  R_clamped(0, 0) = std::clamp(R_clamped(0, 0), dc.x_min, dc.x_max);
  R_clamped(1, 1) = std::clamp(R_clamped(1, 1), dc.y_min, dc.y_max);
  R_clamped(2, 2) = std::clamp(R_clamped(2, 2), dc.z_min, dc.z_max);
  R_clamped(3, 3) = std::clamp(R_clamped(3, 3), dc.yaw_min, dc.yaw_max);

  // ── Weight computation ──
  double max_weight = std::clamp(ba.max_weight, 0.0, 1.0);
  double weight_power = std::max(0.0, ba.weight_power);
  weight = max_weight * std::pow(std::clamp(meta.confidence, 0.0, 1.0), weight_power);
  if (!std::isfinite(weight)) weight = 0.0;
  weight = std::clamp(weight, 0.0, 1.0);
  if (weight <= 0.0) return R_clamped;

  // ── Scale inflation ──
  const double ba_scale = std::isfinite(ba.scale) ? std::max(ba.scale, 1e-6) : 1.0;
  return R_clamped * ba_scale;
}

Eigen::Matrix4d BaAwareYpdNoiseModel::blend_final(
    const Eigen::Matrix4d &R_ypd, const Eigen::Matrix4d &R_ba,
    double w_ba) const {
  double lambda = std::clamp(cfg_.dynamic_blend.lambda, 0.0, 1.0);
  w_ba = std::clamp(w_ba, 0.0, 1.0);

  Eigen::Matrix4d R_dynamic =
      (1.0 - w_ba) * R_ypd + w_ba * R_ba;

  Eigen::Matrix4d R_fixed = build_R_fixed();
  Eigen::Matrix4d R_floor = build_R_floor();

  Eigen::Matrix4d R_final =
      R_floor + (1.0 - lambda) * R_fixed + lambda * R_dynamic;

  // Final SPD enforcement
  Eigen::Vector4d floor_diag(kMinSigmaSq, kMinSigmaSq, kMinSigmaSq, kMinSigmaSq);
  return diag_spd_check(R_final, floor_diag);
}

Eigen::Matrix4d BaAwareYpdNoiseModel::build_single_R(
    const ObservationData &obs) const {
  last_snapshot_ = ObservationNoiseDebugSnapshot{};

  // 1. Build R_fixed
  Eigen::Matrix4d R_fixed = build_R_fixed();
  last_snapshot_.diag_fixed = R_fixed.diagonal();

  // 2. Build R_floor
  Eigen::Matrix4d R_floor = build_R_floor();
  last_snapshot_.diag_floor = R_floor.diagonal();

  // 3. Build R_YPD
  Eigen::Matrix4d R_ypd = build_R_ypd(obs);
  last_snapshot_.diag_ypd = R_ypd.diagonal();
  last_snapshot_.image_valid = obs.image.has_value() && obs.image->valid;

  // 4. Build/validate R_BA
  double w_ba = 0.0;
  Eigen::Matrix4d R_ba = build_R_ba(obs, w_ba);
  last_snapshot_.ba_valid = obs.ba_pnp.has_value() && obs.ba_pnp->valid;
  last_snapshot_.ba_cov_valid =
      obs.ba_pnp.has_value() && obs.ba_pnp->cov_valid;
  last_snapshot_.ba_used = (w_ba > 0.0);
  last_snapshot_.ba_weight = w_ba;
  if (obs.ba_pnp.has_value()) {
    last_snapshot_.ba_confidence = obs.ba_pnp->confidence;
    last_snapshot_.ba_reproj_rms = obs.ba_pnp->reproj_rms;
    last_snapshot_.ba_condition_number = obs.ba_pnp->condition_number;
  }
  last_snapshot_.diag_ba = R_ba.diagonal();

  // 5. Blend
  last_snapshot_.lambda = cfg_.dynamic_blend.lambda;
  Eigen::Matrix4d R_final = blend_final(R_ypd, R_ba, w_ba);
  last_snapshot_.diag_final = R_final.diagonal();

  return R_final;
}

Eigen::Matrix<double, 8, 8> BaAwareYpdNoiseModel::build_dual_R(
    const ObservationData &obs0, const ObservationData &obs1) const {
  Eigen::Matrix4d R0 = build_single_R(obs0);
  Eigen::Matrix4d R1 = build_single_R(obs1);

  Eigen::Matrix<double, 8, 8> R_dual = Eigen::Matrix<double, 8, 8>::Zero();
  R_dual.block<4, 4>(0, 0) = R0;
  R_dual.block<4, 4>(4, 4) = R1;
  R_dual *= ukf_config_.dual_raw_R_scale;
  return R_dual;
}

}  // namespace fyt::auto_aim::norm4_v3
