#include "armor_pnp_refiner/quality/quality_gate.hpp"

#include <cmath>

namespace armor_pnp_refiner {

QualityGate::QualityGate(const PnpRefinerConfig &config) : config_(config) {}

RefineStatus QualityGate::evaluate(const PnpRefineOutput &refined) {
  if (!checkFinite(refined)) {
    return RefineStatus::REJECTED;
  }
  if (!checkPositiveDepth(refined)) {
    return RefineStatus::REJECTED;
  }

  bool all_good = true;
  bool marginal = false;

  if (!checkPoseDelta(refined)) {
    marginal = true;
    all_good = false;
  }
  if (!checkYawDelta(refined)) {
    marginal = true;
    all_good = false;
  }
  if (!checkReprojectionError(refined)) {
    marginal = true;
    all_good = false;
  }
  if (!checkConditionNumber(refined)) {
    marginal = true;
    all_good = false;
  }
  if (!checkChi2(refined)) {
    marginal = true;
    all_good = false;
  }

  return decideStatus(all_good, marginal, refined.confidence);
}

bool QualityGate::checkFinite(const PnpRefineOutput &out) const {
  if (!config_.require_finite) return true;
  return std::isfinite(out.t_camera_armor.x()) && std::isfinite(out.t_camera_armor.y()) &&
         std::isfinite(out.t_camera_armor.z()) && std::isfinite(out.yaw_rad);
}

bool QualityGate::checkPositiveDepth(const PnpRefineOutput &out) const {
  if (!config_.require_positive_depth) return true;
  // In camera frame, positive Z means in front of camera.
  return out.t_camera_armor.z() > 0.0;
}

bool QualityGate::checkReprojectionError(const PnpRefineOutput &out) const {
  return out.reproj_error_refined_px <= config_.max_reproj_error_px;
}

bool QualityGate::checkPoseDelta(const PnpRefineOutput &out) const {
  return out.pose_delta_m <= config_.max_pose_delta_m;
}

bool QualityGate::checkYawDelta(const PnpRefineOutput &out) const {
  return out.yaw_delta_rad <= config_.max_yaw_delta_rad;
}

bool QualityGate::checkConditionNumber(const PnpRefineOutput &out) const {
  if (out.condition_number <= 0) return true;  // not computed
  return out.condition_number <= config_.max_condition_number;
}

bool QualityGate::checkChi2(const PnpRefineOutput &out) const {
  if (out.chi2_per_dof <= 0) return true;  // not computed
  return out.chi2_per_dof <= config_.max_chi2_per_dof;
}

RefineStatus QualityGate::decideStatus(bool all_passed, bool marginal, double confidence) {
  if (!all_passed && confidence < config_.reject_confidence) {
    return RefineStatus::REJECTED;
  }
  if (marginal || confidence < config_.good_confidence) {
    return RefineStatus::DEGRADED;
  }
  return RefineStatus::GOOD;
}

}  // namespace armor_pnp_refiner
