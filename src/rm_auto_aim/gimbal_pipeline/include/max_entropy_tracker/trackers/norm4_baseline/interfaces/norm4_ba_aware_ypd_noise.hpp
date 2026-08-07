// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_BA_AWARE_YPD_NOISE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_BA_AWARE_YPD_NOISE_HPP_

#include <Eigen/Dense>

#include <string>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/interfaces/norm4_measurement_noise.hpp"

namespace fyt::auto_aim::norm4_baseline {

struct ObservationNoiseDebugSnapshot {
  bool image_valid = false;
  bool ba_valid = false;
  bool ba_cov_valid = false;
  bool ba_used = false;
  double lambda = 0.0;
  double ba_weight = 0.0;
  double ba_confidence = 0.0;
  double ba_reproj_rms = 0.0;
  double ba_condition_number = 0.0;
  Eigen::Vector4d diag_fixed = Eigen::Vector4d::Zero();
  Eigen::Vector4d diag_floor = Eigen::Vector4d::Zero();
  Eigen::Vector4d diag_ypd = Eigen::Vector4d::Zero();
  Eigen::Vector4d diag_ba = Eigen::Vector4d::Zero();
  Eigen::Vector4d diag_final = Eigen::Vector4d::Zero();
};

/// Phase 1 dynamic observation noise model:
///   R_final = R_floor + (1-λ)R_fixed + λ[(1-w)R_YPD + w·s_BA·R_BA]
class BaAwareYpdNoiseModel : public IMeasurementNoiseModel {
 public:
  explicit BaAwareYpdNoiseModel(const MeasurementNoiseConfig &cfg,
                                const Norm4BaselineUkfConfig &ukf_cfg);

  Eigen::Matrix4d build_single_R(
      const ObservationData &obs) const override;

  Eigen::Matrix<double, 8, 8> build_dual_R(
      const ObservationData &obs0,
      const ObservationData &obs1) const override;

  std::string name() const override { return "ypd_ba"; }

  double sigma_pos_xy() const override { return ukf_config_.sigma_pos_xy; }
  double sigma_pos_z() const override { return ukf_config_.sigma_pos_z; }
  double sigma_yaw() const override { return ukf_config_.sigma_yaw; }
  double dual_scale() const override { return ukf_config_.dual_raw_R_scale; }

  const ObservationNoiseDebugSnapshot &last_snapshot() const {
    return last_snapshot_;
  }

 private:
  Eigen::Matrix4d build_R_fixed() const;
  Eigen::Matrix4d build_R_floor() const;
  Eigen::Matrix4d build_R_ypd(const ObservationData &obs) const;
  Eigen::Matrix4d build_R_ba(const ObservationData &obs, double &weight) const;
  Eigen::Matrix4d blend_final(const Eigen::Matrix4d &R_ypd,
                              const Eigen::Matrix4d &R_ba,
                              double w_ba) const;

  Eigen::Matrix4d ensure_spd(const Eigen::Matrix4d &R,
                             const std::string &fallback_label) const;

  MeasurementNoiseConfig cfg_;
  Norm4BaselineUkfConfig ukf_config_;
  mutable ObservationNoiseDebugSnapshot last_snapshot_;
};

}  // namespace fyt::auto_aim::norm4_baseline

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_BA_AWARE_YPD_NOISE_HPP_
