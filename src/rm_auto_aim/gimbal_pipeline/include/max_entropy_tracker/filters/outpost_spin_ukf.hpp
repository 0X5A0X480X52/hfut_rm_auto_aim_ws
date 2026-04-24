// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_FILTERS_OUTPOST_SPIN_UKF_HPP_
#define MAX_ENTROPY_TRACKER_FILTERS_OUTPOST_SPIN_UKF_HPP_

#include <array>
#include <memory>
#include <optional>
#include <vector>

#include "max_entropy_tracker/filters/base_ukf.hpp"
#include "max_entropy_tracker/filters/process_models/composite.hpp"
#include "max_entropy_tracker/filters/spin_filter_interface.hpp"

namespace fyt::auto_aim {

/// Dedicated spin UKF for outpost 3-panel geometry.
class OutpostSpinUKF : public BaseUKF, public SpinFilterInterface {
 public:
  static constexpr int N_PANELS = 3;

  explicit OutpostSpinUKF(
      const UnifiedConfig &config, double dt = 0.05,
      std::shared_ptr<CompositeProcessModel> process_model = nullptr);

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

  bool update_with_panel(const ObservationData &obs, int panel_id,
                         double position_confidence = 1.0);

  // SpinFilterInterface
  const Eigen::VectorXd &x() const override { return BaseUKF::x(); }
  Eigen::VectorXd &x() override { return BaseUKF::x(); }
  const Eigen::MatrixXd &P() const override { return BaseUKF::P(); }
  Eigen::MatrixXd &P() override { return BaseUKF::P(); }
  const DynamicStateIndex &state_idx() const override { return state_idx_; }

  Eigen::Vector3d get_center_position() const override;
  std::pair<double, double> get_radii() const override;
  double get_dza() const override { return 0.0; }
  double get_yaw() const override;
  double get_delta() const override;
  int get_k() const override { return 0; }

  const Eigen::VectorXd &last_innov_xyz() const override { return last_innov_xyz_; }
  double last_innov_yaw() const override { return last_innov_yaw_; }
  double last_nis() const override { return last_nis_; }
  int last_update_type() const override { return last_update_type_; }

 private:
  std::shared_ptr<CompositeProcessModel> create_process_model(
      const UnifiedConfig &cfg) const;

  Eigen::VectorXd observation_model(const Eigen::VectorXd &x,
                                    int panel_id) const;

  int sanitize_panel_id(int panel_id) const;
  void apply_angle_constraints();

  std::shared_ptr<CompositeProcessModel> motion_model_;
  DynamicStateIndex state_idx_;

  double radius_ = 0.26;
    std::array<double, N_PANELS> z_offsets_{0.06, 0.0, -0.06};
  std::array<double, N_PANELS> panel_angles_{0.0, 2.0 * M_PI / 3.0,
                                                                                         -2.0 * M_PI / 3.0};
  int selected_panel_id_ = 0;

  Eigen::VectorXd last_innov_xyz_;
  double last_innov_yaw_ = 0.0;
  double last_nis_ = -1.0;
  int last_update_type_ = 0;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_FILTERS_OUTPOST_SPIN_UKF_HPP_
