// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_SINGLE_ARMOR_IMM_BUNDLE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_SINGLE_ARMOR_IMM_BUNDLE_HPP_

#include <memory>

#include "max_entropy_tracker/filters/process_models/composite.hpp"
#include "max_entropy_tracker/filters/single_armor_imm_tracker.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/models/norm4_motion_model_bundle.hpp"
#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::norm4_v3 {

/// Configuration for IMM bundle z/yaw model selection.
struct ImmBundleConfig {
  // XY IMM model enable flags
  bool enable_cv = true;
  bool enable_ca = true;
  bool enable_cs = false;
  bool enable_ctrv = false;

  // Process noise
  double q_cv = 0.5;
  double q_ca = 1.0;
  double q_z_vel = 0.8;
  double q_yaw_rate = 0.6;

  // CS (Singer) parameters
  double cs_alpha = 0.5;
  double cs_a_max = 10.0;

  // IMM Markov transition
  double p_stay = 0.82;
  double p_switch = 0.06;

  // z/yaw model type (fixed to CV for M2)
  std::string z_model = "cv";
  std::string yaw_model = "cv";

  // Structural process noise
  double q_r = 0.02;
  double q_dza = 0.005;

  // Observation noise (base, used for IMM internal update)
  double r_pos_base = 0.01;
  double r_yaw_base = 0.03;
};

/// Motion model bundle that wraps SingleArmorIMMTracker.
///
/// State layout (13D):
///   [X, VX, AX, Y, VY, AY, Z, VZ, DELTA, DELTA_RATE, R1, R2, DZA]
class SingleArmorIMMBundle : public IMotionModelBundle {
 public:
  explicit SingleArmorIMMBundle(const ImmBundleConfig &cfg);

  int state_dim() const override { return kStateDim; }
  const DynamicStateIndex &state_idx() const override { return state_idx_; }

  Eigen::VectorXd predict(const Eigen::VectorXd &x, double dt) override;
  Eigen::MatrixXd build_Q(double dt) const override;

  Eigen::VectorXd initial_state(const ObservationData &obs, int panel_id,
                                 double r1, double r2,
                                 double dza) const override;
  Eigen::MatrixXd initial_covariance() const override;

  const kalman::SingleArmorIMMTracker &imm_tracker() const { return *imm_; }
  kalman::SingleArmorIMMTracker &imm_tracker() { return *imm_; }

 private:
  static constexpr int kStateDim = 13;

  ImmBundleConfig cfg_;
  std::unique_ptr<kalman::SingleArmorIMMTracker> imm_;
  std::unique_ptr<kalman::SingleArmorIMMConfig> imm_cfg_;
  DynamicStateIndex state_idx_;
};

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_SINGLE_ARMOR_IMM_BUNDLE_HPP_
