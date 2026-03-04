// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_DUAL_RADIUS_SPIN_UKF_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_DUAL_RADIUS_SPIN_UKF_HPP_

#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "max_entropy_tracker/filters/base_ukf.hpp"
#include "max_entropy_tracker/filters/process_models/composite.hpp"

namespace fyt::auto_aim {

/**
 * Dual-radius spin UKF for 4-panel robots.
 *
 * yaw decomposition: yaw = k*π + delta,  k ∈ {0,1}, delta ∈ [-π/2, π/2]
 * State vector dimension depends on process model (default Singer+CV → 14D).
 */
class DualRadiusSpinUKF : public BaseUKF {
 public:
  static constexpr int N_PANELS = 4;
  static constexpr double PANEL_ANGLE_STEP = M_PI / 2.0;

  explicit DualRadiusSpinUKF(
      const UnifiedConfig &config, double dt = 0.05,
      std::shared_ptr<CompositeProcessModel> process_model = nullptr);

  /* ---------- BaseUKF interface ---------- */
  int state_dim() const override;
  int obs_dim() const override { return 4; }

  void initialize(const std::vector<ObservationData> &observations,
                  double r1 = 0.15, double r2 = 0.20, double dza = 0.0,
                  int panel_id = -1) override;

  void predict(std::optional<double> dt = std::nullopt) override;

  bool update(const std::vector<ObservationData> &observations,
              const std::vector<std::string> &r_types = {},
              const std::vector<std::string> &armor_layers = {},
              double height_confidence = 1.0,
              double position_confidence = 1.0,
              double panel_angle = 0.0) override;

  /* ---------- State queries ---------- */
  double get_yaw() const;
  double get_delta() const;
  int get_k() const { return k_; }
  Eigen::Vector3d get_center_position() const;
  std::pair<double, double> get_radii() const;
  double get_dza() const;
  bool is_dza_converged(double var_threshold = 0.01,
                        double min_value = 0.005) const;

  const CompositeProcessModel &process_model() const { return *motion_model_; }
  const DynamicStateIndex &state_idx() const { return state_idx_; }

 private:
  /* ---------- internal ---------- */
  std::shared_ptr<CompositeProcessModel> create_process_model(
      const UnifiedConfig &cfg, double dt) const;

  Eigen::VectorXd observation_model(const Eigen::VectorXd &x,
                                    const std::string &r_type,
                                    const std::string &armor_layer,
                                    double panel_angle) const;

  Eigen::VectorXd observation_model_geometry(const Eigen::VectorXd &x) const;

  bool update_single(const ObservationData &obs, const std::string &r_type,
                     const std::string &armor_layer,
                     double height_confidence,
                     double position_confidence, double panel_angle);

  bool update_dual(const ObservationData &obs1, const ObservationData &obs2,
                   const std::string &r_type_1, const std::string &r_type_2,
                   const std::string &layer_1, const std::string &layer_2,
                   double height_confidence);

  std::string infer_armor_layer(double z_obs) const;

  void handle_mode_switch();
  void apply_constraints();

  bool check_innovation_gate(const Eigen::VectorXd &innov,
                             const Eigen::MatrixXd &z_pred_points,
                             const Eigen::VectorXd &z_pred,
                             const Eigen::MatrixXd &R,
                             const Eigen::VectorXd &Wc) const;

  std::shared_ptr<CompositeProcessModel> motion_model_;
  DynamicStateIndex state_idx_;

  int k_ = 0;
  std::optional<int> last_k_;
  int mode_switches_ = 0;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_DUAL_RADIUS_SPIN_UKF_HPP_
