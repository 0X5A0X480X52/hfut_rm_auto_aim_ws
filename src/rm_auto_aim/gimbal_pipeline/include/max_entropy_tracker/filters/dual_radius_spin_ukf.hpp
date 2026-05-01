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
#include "max_entropy_tracker/filters/spin_filter_interface.hpp"

namespace fyt::auto_aim {

/**
 * Dual-radius spin UKF for 4-panel robots.
 *
 * yaw decomposition: yaw = k*π + delta,  k ∈ {0,1}, delta ∈ [-π/2, π/2]
 * State vector dimension depends on process model (default Singer+CV → 14D).
 */
class DualRadiusSpinUKF : public BaseUKF, public SpinFilterInterface {
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
    double get_yaw() const override;
    double get_delta() const override;
    int get_k() const override { return k_; }
    Eigen::Vector3d get_center_position() const override;
    std::pair<double, double> get_radii() const override;
    double get_dza() const override;
  bool is_dza_converged(double var_threshold = 0.01,
                        double min_value = 0.005) const;

  const CompositeProcessModel &process_model() const { return *motion_model_; }
    const DynamicStateIndex &state_idx() const override { return state_idx_; }

    const Eigen::VectorXd &x() const override { return BaseUKF::x(); }
    Eigen::VectorXd &x() override { return BaseUKF::x(); }
    const Eigen::MatrixXd &P() const override { return BaseUKF::P(); }
    Eigen::MatrixXd &P() override { return BaseUKF::P(); }

  /* ---------- Maneuver detection getters ---------- */
  /// 3-D position innovation from the last update(); size-0 if no update yet.
    const Eigen::VectorXd &last_innov_xyz() const override { return last_innov_xyz_; }
  /// yaw innovation (single-obs only; 0 for dual-obs or no-update).
    double last_innov_yaw() const override { return last_innov_yaw_; }
  /// z-dimension innovation from the last single-obs update (innov(2)).
  double last_z_innovation() const { return last_innov_xyz_.size() >= 3 ? last_innov_xyz_(2) : 0.0; }
  /// Normalized Innovation Squared (NIS); -1 = no update since last predict().
    double last_nis() const override { return last_nis_; }
  /// Update type: 0=none, 1=single-observation, 2=dual-observation.
    int last_update_type() const override { return last_update_type_; }

  /* ---------- Panel mismatch correction ---------- */
  /**
   * Apply an in-place panel correction when a mismatch is detected.
   *
   * Actions taken:
   *   1. Swap x_(R1) ↔ x_(R2) and the corresponding rows/cols of P
   *      (because even panels use r1, odd panels use r2, and parity flipped).
   *   2. Recompute k_ and x_(DELTA) from the new center_yaw.
   *   3. Inflate the covariances of DELTA, R1, R2, DZA so the filter
   *      can quickly re-converge after the correction.
   *
   * @param new_center_yaw  Recomputed center_yaw = armor_yaw - new_panel_id * π/2
   */
  void apply_panel_correction(double new_center_yaw);

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

  // 方向一：根据 delta_angle 计算几何参数更新权重 (0=完全冻结, 1=完全开放)
  double weight_from_delta_angle(double delta) const;

  std::shared_ptr<CompositeProcessModel> motion_model_;
  DynamicStateIndex state_idx_;

  int k_ = 0;
  std::optional<int> last_k_;
  int mode_switches_ = 0;

  // Maneuver detection cache (reset in predict(), filled in update_single/dual)
  Eigen::VectorXd last_innov_xyz_;   ///< 3-D position innovation [x, y, z]
  double last_innov_yaw_   = 0.0;   ///< yaw innovation (single-obs only)
  double last_nis_         = -1.0;  ///< NIS; -1 = no update this cycle
  int    last_update_type_ = 0;     ///< 0=none, 1=single, 2=dual
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_DUAL_RADIUS_SPIN_UKF_HPP_
