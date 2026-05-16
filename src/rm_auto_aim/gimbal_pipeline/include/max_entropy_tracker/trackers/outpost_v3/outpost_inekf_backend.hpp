// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_INEKF_BACKEND_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_INEKF_BACKEND_HPP_

#include <Eigen/Dense>

#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/filters/process_models/base.hpp"
#include "max_entropy_tracker/filters/process_models/composite.hpp"
#include "max_entropy_tracker/filters/spin_filter_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/hypothesis/norm4_hypothesis_types.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/interfaces/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/outpost_v3/outpost_hypothesis_types.hpp"

namespace fyt::auto_aim::outpost_v3 {

/// Fixed state indices for the Outpost 12D InEKF.
///
/// State layout:
///   0:X, 1:Y, 2:Z, 3:VX, 4:VY, 5:VZ,
///   6:AX, 7:AY, 8:AZ, 9:YAW, 10:YAW_RATE, 11:YAW_ACC
struct OutpostStateIndex {
  static constexpr int X = 0;
  static constexpr int Y = 1;
  static constexpr int Z = 2;
  static constexpr int VX = 3;
  static constexpr int VY = 4;
  static constexpr int VZ = 5;
  static constexpr int AX = 6;
  static constexpr int AY = 7;
  static constexpr int AZ = 8;
  static constexpr int YAW = 9;
  static constexpr int YAW_RATE = 10;
  static constexpr int YAW_ACC = 11;

  static constexpr int kDim = 12;
  static constexpr int kObsDim = 3;  // 3D position observation

  static StateLayout build_layout();
  static DynamicStateIndex make_idx();
};

/// Standard InEKF backend for outpost with fixed structure parameters.
///
/// Error type: LEFT_INVARIANT
///   R = Exp(δψ) · R̂         (left perturbation on SO(2))
///   p = p̂ + δρ               (additive in world frame)
///   v = v̂ + δv, a = â + δa   (additive)
///   ω = ω̂ + δω, α = α̂ + δα   (additive)
///
/// Innovation is formed in body frame:
///   ν = Rz(-yaw) · (z_obs − z_pred)
///
/// State (12D):
///   x = [p(3), v(3), a(3), yaw, yaw_rate, yaw_acc]ᵀ
///
/// Structure parameters (radius, z_offsets, panel_angles)
/// are known constants — NOT in the state.
class OutpostInEKFBackend : public norm4_v3::IStructuredBackend,
                            public SpinFilterInterface {
 public:
  OutpostInEKFBackend(const OutpostV3Config &cfg, double dt = 0.05);

  // ── IStructuredBackend ──
  void reset(const ObservationData &obs, int panel_id, double r1, double r2,
             double dza) override;
  void predict(double dt) override;
  bool initialized() const override { return initialized_; }

  norm4_v3::PredictContext buildPredictContext() const override;

  norm4_v3::MeasurementEval evaluateSingle(
      const norm4_v3::PredictContext &ctx, const ObservationData &obs,
      int panel_id) const override;

  norm4_v3::MeasurementEval evaluateDual(
      const norm4_v3::PredictContext &ctx, const ObservationData &obs0,
      const ObservationData &obs1, int panel_id_0,
      int panel_id_1) const override;

  norm4_v3::UkfTrial tryUpdateSingle(
      const norm4_v3::PredictContext &ctx, const ObservationData &obs,
      int panel_id) const override;

  norm4_v3::UkfTrial tryUpdateDual(
      const norm4_v3::PredictContext &ctx, const ObservationData &obs0,
      const ObservationData &obs1, int panel_id_0,
      int panel_id_1) const override;

  void commit(const norm4_v3::UkfTrial &trial) override;

  norm4_v3::BackendSnapshot snapshot() const override;

  SpinFilterInterface &spin_filter() override { return *this; }
  const SpinFilterInterface &spin_filter() const override { return *this; }

  // ── SpinFilterInterface ──
  const Eigen::VectorXd &x() const override { return x_; }
  Eigen::VectorXd &x() override { return x_; }
  const Eigen::MatrixXd &P() const override { return P_; }
  Eigen::MatrixXd &P() override { return P_; }
  const DynamicStateIndex &state_idx() const override { return state_idx_; }

  Eigen::Vector3d get_center_position() const override;
  std::pair<double, double> get_radii() const override;
  double get_dza() const override { return 0.0; }
  double get_yaw() const override;
  double get_delta() const override;
  int get_k() const override { return k_; }

  const Eigen::VectorXd &last_innov_xyz() const override {
    return last_innov_xyz_;
  }
  double last_innov_yaw() const override { return last_innov_yaw_; }
  double last_nis() const override { return last_nis_; }
  int last_update_type() const override { return last_update_type_; }

 private:
  // ── Lie group operations (SO(2)) ──
  static double left_jacobian_SO2(double dpsi);
  static double right_jacobian_SO2(double dpsi);

  // ── Observation model ──
  Eigen::Vector3d obs_model(const Eigen::VectorXd &x,
                            int panel_id) const;

  // ── World-frame Jacobian (3 × 12) ──
  Eigen::Matrix<double, 3, OutpostStateIndex::kDim> obs_jacobian_world(
      const Eigen::VectorXd &x, int panel_id) const;

  // ── Body-frame innovation ──
  Eigen::Vector3d compute_body_frame_innovation(
      const Eigen::Vector3d &z_obs, const Eigen::Vector3d &z_pred,
      double center_yaw) const;

  // ── Body-frame H Jacobian ──
  Eigen::Matrix<double, 3, OutpostStateIndex::kDim> compute_body_frame_H(
      const Eigen::Matrix<double, 3, OutpostStateIndex::kDim> &H_world,
      double center_yaw, int panel_id) const;

  // ── Body-frame R ──
  Eigen::Matrix3d rotate_R_to_body_frame(const Eigen::Matrix3d &R_world,
                                          double center_yaw) const;

  // ── Build R from config ──
  Eigen::Matrix3d build_observation_R() const;

  // ── Process noise Q ──
  Eigen::Matrix<double, OutpostStateIndex::kDim, OutpostStateIndex::kDim>
  build_process_Q(double dt) const;

  // ── Retraction ──
  Eigen::VectorXd retract_left_invariant(
      const Eigen::VectorXd &x_prior, const Eigen::VectorXd &dx) const;

  // ── Posterior checks ──
  bool check_posterior_sanity(const Eigen::VectorXd &x_prior,
                               const Eigen::VectorXd &x_post,
                               const Eigen::MatrixXd &P_post) const;

  double compute_reconstruction_error(const Eigen::VectorXd &x_post,
                                       const ObservationData &obs,
                                       int panel_id) const;

  // ── Initialization ──
  Eigen::VectorXd initialize_state(const ObservationData &obs,
                                    int panel_id) const;

  OutpostV3Config cfg_;
  double dt_;
  OutpostPanelGeometry geom_;

  // Static layout for the fixed 12D state
  static StateLayout s_layout_;
  static bool s_layout_initialized_;
  DynamicStateIndex state_idx_;

  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  bool initialized_ = false;

  int k_ = 0;
  int last_k_ = 0;
  int current_panel_id_ = -1;

  Eigen::VectorXd last_innov_xyz_;
  double last_innov_yaw_ = 0.0;
  double last_nis_ = -1.0;
  int last_update_type_ = 0;
};

}  // namespace fyt::auto_aim::outpost_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_INEKF_BACKEND_HPP_
