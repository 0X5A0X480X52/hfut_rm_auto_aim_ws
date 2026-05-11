// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_INEKF_BACKEND_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_INEKF_BACKEND_HPP_

#include <memory>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/filters/spin_filter_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/interfaces/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/interfaces/norm4_measurement_noise.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/models/norm4_motion_model_bundle.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/models/norm4_structure_provider.hpp"

namespace fyt::auto_aim::norm4_v3 {

/// Full InEKF / invariant error-state EKF backend.
///
/// Fast states (position, velocity, yaw, yaw_rate) are updated via analytical
/// Jacobian + Joseph-form covariance.  Slow structure parameters (r1, r2, dza)
/// are gated through IStructureProvider.
///
/// Error state (11D baseline, grows with motion model):
///   δx = [δρ(3), δv(3), δψ, δβ, δr1, δr2, δdza]  + optional AX/AY/AZ
class InvariantPoseBackend : public IStructuredBackend,
                               public SpinFilterInterface {
 public:
  InvariantPoseBackend(std::unique_ptr<IMotionModelBundle> motion,
                       std::unique_ptr<IMeasurementNoiseModel> noise,
                       std::unique_ptr<IStructureProvider> structure,
                       const Norm4V3UkfConfig &ukf_config,
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

 private:
  // ── Observation model ──
  Eigen::Vector4d obs_model_single(const Eigen::VectorXd &x, int k,
                                    int panel_id) const;

  // ── Analytical Jacobian for single-obs (4 × n) ──
  Eigen::MatrixXd obs_jacobian_single(const Eigen::VectorXd &x, int k,
                                       int panel_id) const;

  // ── Posterior checks ──
  bool check_posterior_sanity(const Eigen::VectorXd &x_prior,
                               const Eigen::VectorXd &x_post,
                               const Eigen::MatrixXd &P_post) const;

  double compute_reconstruction_error(const Eigen::VectorXd &x_post, int k,
                                       const ObservationData &obs,
                                       int panel_id) const;
  Eigen::VectorXd retract_se25_state(const Eigen::VectorXd &x_prior,
                                      const Eigen::VectorXd &dx) const;

  Eigen::VectorXd initialize_invariant_state(const ObservationData &obs,
                                              int panel_id, double r1,
                                              double r2, double dza) const;
  Eigen::MatrixXd build_invariant_Q(double dt) const;

  void apply_state_constraints();

  UnifiedConfig config_;
  Norm4V3UkfConfig ukf_config_;
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

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_INEKF_BACKEND_HPP_
