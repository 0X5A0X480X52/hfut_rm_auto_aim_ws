// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_INEKF_BACKEND_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_INEKF_BACKEND_HPP_

#include <memory>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/filters/spin_filter_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/interfaces/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/interfaces/norm4_measurement_noise.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/models/norm4_motion_model_bundle.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/models/norm4_structure_provider.hpp"

namespace fyt::auto_aim::norm4_baseline {

/// Error type for invariant EKF.
///
/// LEFT_INVARIANT:  X = Exp(δξ) · X̂
///   R = Exp(δψ·e_z) · R̂      (left perturbation on SO(2))
///   p = p̂ + δρ                (additive in world frame)
///   v = v̂ + δv                (additive in world frame)
///   β = β̂ + δβ                (additive)
///   θ = θ̂ + δθ                (additive, slow structure)
///
/// Innovation is formed in body frame:  ν_pos = R̂⁻¹·(z_pos − h_pos(X̂))
/// This decouples the position residual from the global yaw estimate.
enum class InvariantErrorType { LEFT_INVARIANT };

/// Full Left-Invariant Error-State EKF backend.
///
/// Fast states (position, velocity, yaw, yaw_rate) are updated via
/// analytical Jacobian on the Lie algebra + Joseph-form covariance.
/// Slow structure parameters (r1, r2, dza) are gated through
/// IStructureProvider with reduced Kalman gain.
///
/// Error state (11D baseline, grows with motion model):
///   δx = [δρ₃, δv₃, δψ, δβ, δr₁, δr₂, δdza]ᵀ  + optional AX/AY/AZ/DELTA_ACC
///
/// Innovation coordinate: body frame (left-invariant residual)
///   ν_body = [R̂⁻¹·(p_obs − p_pred);  wrap(yaw_obs − yaw_pred)]
class InvariantPoseBackend : public IStructuredBackend,
                               public SpinFilterInterface {
 public:
  InvariantPoseBackend(std::unique_ptr<IMotionModelBundle> motion,
                       std::unique_ptr<IMeasurementNoiseModel> noise,
                       std::unique_ptr<IStructureProvider> structure,
                       const Norm4BaselineUkfConfig &ukf_config,
                       const UnifiedConfig &config, double dt = 0.05);

  // ── IStructuredBackend ──
  void reset(const ObservationData &obs, int panel_id, double r1, double r2,
             double dza) override;
  void predict(double dt) override;
  bool initialized() const override { return initialized_; }

  PredictContext buildPredictContext() const override;

  MeasurementEval evaluateSingle(const PredictContext &ctx,
                                  const ObservationData &obs,
                                  int panel_id) const override;

  MeasurementEval evaluateDual(const PredictContext &ctx,
                                const ObservationData &obs0,
                                const ObservationData &obs1,
                                int panel_id_0, int panel_id_1) const override;

  UkfTrial tryUpdateSingle(const PredictContext &ctx,
                            const ObservationData &obs,
                            int panel_id) const override;

  UkfTrial tryUpdateDual(const PredictContext &ctx,
                          const ObservationData &obs0,
                          const ObservationData &obs1,
                          int panel_id_0, int panel_id_1) const override;

  void commit(const UkfTrial &trial) override;

  BackendSnapshot snapshot() const override;

  SpinFilterInterface &spin_filter() override { return *this; }
  const SpinFilterInterface &spin_filter() const override { return *this; }

  // ── SpinFilterInterface ──
  const Eigen::VectorXd &x() const override { return x_; }
  Eigen::VectorXd &x() override { return x_; }
  const Eigen::MatrixXd &P() const override { return P_; }
  Eigen::MatrixXd &P() override { return P_; }
  const DynamicStateIndex &state_idx() const override {
    return motion_->state_idx();
  }
  Eigen::Vector3d get_center_position() const override;
  std::pair<double, double> get_radii() const override;
  double get_dza() const override;
  double get_yaw() const override;
  double get_delta() const override;
  int get_k() const override { return k_; }
  const Eigen::VectorXd &last_innov_xyz() const override {
    return last_innov_xyz_;
  }
  double last_innov_yaw() const override { return last_innov_yaw_; }
  double last_nis() const override { return last_nis_; }
  int last_update_type() const override { return last_update_type_; }

  /// Error type in use.
  static constexpr InvariantErrorType kErrorType = InvariantErrorType::LEFT_INVARIANT;

 private:
  // ── Lie group operations (SO(2)) ──
  /// SO(2) exponential: ψ → R_z(ψ). For SO(2), this is just the angle itself.
  static double exp_SO2(double dpsi) { return dpsi; }
  /// SO(2) logarithm: R_z(ψ) → ψ. Identity at ψ=0.
  static double log_SO2(double /* R_angle */) { return 0.0; }  // unused at retraction-time
  /// SO(2) adjoint: Ad_Rz(ψ) on a 2D vector v.
  static Eigen::Matrix2d adjoint_SO2(double psi);
  /// Left Jacobian of SO(2): J_l(δψ) = sin(δψ)/δψ. For small δψ, ≈ 1.
  static double left_jacobian_SO2(double dpsi);
  /// Right Jacobian of SO(2): J_r(δψ) = sin(δψ)/δψ. For small δψ, ≈ 1.
  static double right_jacobian_SO2(double dpsi);

  // ── Observation model (world-frame prediction) ──
  Eigen::Vector4d obs_model_single(const Eigen::VectorXd &x, int k,
                                    int panel_id) const;

  // ── Analytical Jacobian for single-obs (4 × n), world-frame ──
  Eigen::MatrixXd obs_jacobian_single_world(const Eigen::VectorXd &x, int k,
                                             int panel_id) const;

  // ── Body-frame innovation:  ν = [R̂⁻¹·(z_pos − h_pos);  wrap(yaw_obs, yaw_pred)] ──
  Eigen::Vector4d compute_left_invariant_innovation(
      const Eigen::Vector4d &z_obs, const Eigen::Vector4d &z_pred,
      double center_yaw_pred) const;

  // ── Body-frame H Jacobian: H_body = Ad_Rz⁻¹ · H_world (with structural simplification) ──
  Eigen::MatrixXd compute_body_frame_H(const Eigen::MatrixXd &H_world,
                                        double center_yaw_pred, int panel_id,
                                        const Eigen::VectorXd &x) const;

  // ── Body-frame R matrix: R_body = Ad_Rz⁻¹ · R_world · Ad_Rz⁻ᵀ ──
  Eigen::Matrix4d rotate_R_to_body_frame(const Eigen::Matrix4d &R_world,
                                          double center_yaw_pred) const;

  // ── Posterior checks ──
  bool check_posterior_sanity(const Eigen::VectorXd &x_prior,
                               const Eigen::VectorXd &x_post,
                               const Eigen::MatrixXd &P_post) const;

  double compute_reconstruction_error(const Eigen::VectorXd &x_post, int k,
                                       const ObservationData &obs,
                                       int panel_id) const;

  // ── State retraction (left-invariant):  X̂⁺ = Exp(δξ̂) ∘ X̂ ──
  // For SO(2): yaw⁺ = normalize_angle(yaw + δψ)
  // For ℝⁿ:  x⁺ = x + δx
  Eigen::VectorXd retract_left_invariant(const Eigen::VectorXd &x_prior,
                                          const Eigen::VectorXd &dx) const;

  Eigen::VectorXd initialize_invariant_state(const ObservationData &obs,
                                              int panel_id, double r1,
                                              double r2, double dza) const;
  Eigen::MatrixXd build_invariant_Q(double dt) const;

  void apply_state_constraints();

  UnifiedConfig config_;
  Norm4BaselineUkfConfig ukf_config_;
  double dt_;
  std::unique_ptr<IMotionModelBundle> motion_;
  std::unique_ptr<IMeasurementNoiseModel> noise_;
  std::unique_ptr<IStructureProvider> structure_;

  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  bool initialized_ = false;

  int k_ = 0;
  int last_k_ = 0;
  int current_panel_id_ = -1;
  int phase_index_ = -1;

  Eigen::VectorXd last_innov_xyz_;
  double last_innov_yaw_ = 0.0;
  double last_nis_ = -1.0;
  int last_update_type_ = 0;
};

}  // namespace fyt::auto_aim::norm4_baseline

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_INEKF_BACKEND_HPP_
