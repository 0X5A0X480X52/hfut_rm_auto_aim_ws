// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_UKF_BACKEND_V1_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_UKF_BACKEND_V1_HPP_

#include <memory>
#include <optional>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/filters/process_models/composite.hpp"
#include "max_entropy_tracker/filters/spin_filter_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/norm4_hypothesis_types.hpp"
#include "max_entropy_tracker/utils/sigma_points.hpp"

namespace fyt::auto_aim::norm4_v3 {

class Norm4UkfBackendV1 : public SpinFilterInterface {
 public:
  explicit Norm4UkfBackendV1(const UnifiedConfig &config, double dt = 0.05);

  void reset(const ObservationData &obs, int panel_id, double r1, double r2,
             double dza);
  void predict(double dt);
  bool initialized() const { return initialized_; }

  PredictContext buildPredictContext() const;

  MeasurementEval evaluateSingle(const PredictContext &ctx,
                                  const ObservationData &obs,
                                  int panel_id) const;

  MeasurementEval evaluateDual(const PredictContext &ctx,
                                const ObservationData &obs0,
                                const ObservationData &obs1,
                                int panel_id_0, int panel_id_1) const;

  UkfTrial tryUpdateSingle(const PredictContext &ctx,
                            const ObservationData &obs,
                            int panel_id) const;

  UkfTrial tryUpdateDual(const PredictContext &ctx,
                          const ObservationData &obs0,
                          const ObservationData &obs1,
                          int panel_id_0, int panel_id_1) const;

  void commit(const UkfTrial &trial);

  // ── SpinFilterInterface ──
  const Eigen::VectorXd &x() const override { return x_; }
  Eigen::VectorXd &x() override { return x_; }
  const Eigen::MatrixXd &P() const override { return P_; }
  Eigen::MatrixXd &P() override { return P_; }
  const DynamicStateIndex &state_idx() const override { return state_idx_; }

  Eigen::Vector3d get_center_position() const override;
  std::pair<double, double> get_radii() const override;
  double get_dza() const override;
  double get_yaw() const override;
  double get_delta() const override;
  int get_k() const override { return k_; }

  const Eigen::VectorXd &last_innov_xyz() const override { return last_innov_xyz_; }
  double last_innov_yaw() const override { return last_innov_yaw_; }
  double last_nis() const override { return last_nis_; }
  int last_update_type() const override { return last_update_type_; }

 private:
  static std::string r_type_for_panel(int panel_id);
  static std::string armor_layer_for_panel(int panel_id);

  Eigen::Vector4d obs_model_single(const Eigen::VectorXd &x, int k,
                                    int panel_id) const;

  void generate_sigma_points(const Eigen::VectorXd &x,
                              const Eigen::MatrixXd &P,
                              Eigen::MatrixXd &out_sigma_pts,
                              Eigen::VectorXd &out_Wm,
                              Eigen::VectorXd &out_Wc) const;

  Eigen::Vector4d compute_z_pred_and_S(
      const Eigen::MatrixXd &sigma_pts,
      const Eigen::VectorXd &Wm,
      const Eigen::VectorXd &Wc,
      double panel_angle,
      MeasurementEval *out_eval) const;

  bool check_posterior_sanity(const Eigen::VectorXd &x_prior,
                               const Eigen::VectorXd &x_post,
                               const Eigen::MatrixXd &P_post) const;

  double compute_reconstruction_error(
      const Eigen::VectorXd &x_post, int k,
      const ObservationData &obs, int panel_id) const;

  void apply_state_constraints();

  UnifiedConfig config_;
  double dt_ = 0.05;
  std::shared_ptr<CompositeProcessModel> process_model_;
  DynamicStateIndex state_idx_;

  Eigen::VectorXd x_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd Q_;
  bool initialized_ = false;

  int k_ = 0;
  int last_k_ = 0;
  int current_panel_id_ = -1;

  Eigen::VectorXd last_innov_xyz_;
  double last_innov_yaw_ = 0.0;
  double last_nis_ = -1.0;
  int last_update_type_ = 0;

  mutable std::unique_ptr<SigmaPointGenerator> sigma_gen_;
};

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_UKF_BACKEND_V1_HPP_
