// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_ARMOR_TRACKER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_ARMOR_TRACKER_HPP_

#include <array>
#include <deque>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

#include "max_entropy_tracker/filters/outpost_spin_ukf.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"

namespace fyt::auto_aim {

class OutpostArmorTracker : public BaseTracker {
 public:
  explicit OutpostArmorTracker(const UnifiedConfig &config, double dt = 0.05,
                               bool enable_oscillation = false);

  void initialize(const std::vector<ObservationData> &obs, double r1 = 0.15,
                  double r2 = 0.20, double dza = 0.0) override;
  void predict(std::optional<double> target_time = std::nullopt) override;
  bool update(const std::vector<ObservationData> &obs) override;

  Eigen::Vector3d get_center_position() const override;
  double get_yaw() const override;
  std::pair<double, double> get_radii() const override;
  SpinFilterInterface &spin_filter() override { return outpost_ukf_; }
  const SpinFilterInterface &spin_filter() const override { return outpost_ukf_; }
  ManeuverResult assess_maneuver() const override;

  Eigen::Vector3d get_publish_velocity() const override { return publish_velocity_; }
  bool is_ambiguous_single_mode() const override;
  int effective_num_armors() const override;
  int selected_panel_id() const;
  double normalized_entropy() const;
  double max_panel_probability() const;
  double confidence_scale() const override;

  std::vector<geometry_msgs::msg::Pose> build_armors_offset_for_message() const override;

 private:
  enum class TrackMode {
    STRUCTURED_3_ARMORS = 0,
    AMBIGUOUS_SINGLE_ARMOR = 1,
  };

  struct PanelHypothesis {
    int panel_id = 0;
    double cost = 0.0;
    double probability = 0.0;
    double center_yaw = 0.0;
    double center_z = 0.0;
  };

  const ObservationData *select_observation(
      const std::vector<ObservationData> &obs) const;

  std::array<PanelHypothesis, 3> evaluate_hypotheses(
      const ObservationData &obs,
      double predicted_center_z,
      double history_center_z) const;

  void compute_probabilities(std::array<PanelHypothesis, 3> &hyps) const;

  void update_mode_from_entropy(int best_panel, double entropy_norm,
                                double max_prob);

  double history_center_z_median() const;
  void push_center_z_history(double center_z);

  bool update_internal_state(const ObservationData &obs,
                             const PanelHypothesis &best,
                             double dt);
  void apply_motion_constraints_from_config();
  void sync_internal_state_from_filter();

  void update_publish_state();

  const UnifiedConfig config_;
  double radius_ = 0.26;
  std::array<double, 3> z_offsets_{-0.06, 0.0, 0.06};
  std::array<double, 3> panel_angles_{0.0, 2.0 * M_PI / 3.0,
                                      4.0 * M_PI / 3.0};
  OutpostSpinUKF outpost_ukf_;
  ManeuverDetector maneuver_detector_;

  // Internal center-centric state
  Eigen::Vector3d center_position_est_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d center_velocity_est_{Eigen::Vector3d::Zero()};
  double center_yaw_est_ = 0.0;
  double yaw_rate_est_ = 0.0;

  // Published state (center in structured mode, single armor in ambiguous mode)
  Eigen::Vector3d publish_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d publish_velocity_{Eigen::Vector3d::Zero()};

  TrackMode mode_ = TrackMode::AMBIGUOUS_SINGLE_ARMOR;
  int selected_panel_id_ = 0;
  int last_best_panel_id_ = -1;
  int stable_counter_ = 0;
  double entropy_norm_ = 1.0;
  double max_prob_ = 0.0;

  std::deque<double> center_z_history_;
  std::optional<double> last_internal_update_time_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_ARMOR_TRACKER_HPP_
