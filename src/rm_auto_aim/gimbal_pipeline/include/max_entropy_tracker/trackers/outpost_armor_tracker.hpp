// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_ARMOR_TRACKER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_ARMOR_TRACKER_HPP_

#include <array>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>

#include "max_entropy_tracker/binder/factory/binder_factory.hpp"
#include "max_entropy_tracker/binder/model/robot_binding_profile.hpp"
#include "max_entropy_tracker/filters/outpost_spin_ukf.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"

namespace fyt::auto_aim {
class OutpostArmorTracker : public BaseTracker {
 public:
  enum class HeightSemantic {
    UNKNOWN = -1,
    HIGH = 0,
    MIDDLE = 1,
    LOW = 2,
  };

  struct DebugSnapshot {
    bool valid = false;
    int track_mode = 1;  // 0=STRUCTURED_3_ARMORS, 1=AMBIGUOUS_SINGLE_ARMOR
    int estimated_id = -1;      // -1 in ambiguous single-armor mode
    int runtime_panel_id = -1;  // internal panel index used by filter
    int bound_height_label = -1;  // 0=HIGH,1=MIDDLE,2=LOW
    int obs_inferred_id = -1;   // inferred from joint yaw+z hypothesis cost
    int obs_inferred_id_z = -1;  // inferred from obs z-jump and height gaps
    int candidate_panel_id = -1;
    double candidate_prob = std::numeric_limits<double>::quiet_NaN();
    double candidate_margin = std::numeric_limits<double>::quiet_NaN();
    double selected_xy_residual = std::numeric_limits<double>::quiet_NaN();
    double entropy_norm = 1.0;
    double max_prob = 0.0;
    std::array<double, 3> hyp_costs{0.0, 0.0, 0.0};
    std::array<double, 3> hyp_probs{0.0, 0.0, 0.0};
    double center_yaw_est = 0.0;
    bool has_observation = false;
    double obs_x = std::numeric_limits<double>::quiet_NaN();
    double obs_y = std::numeric_limits<double>::quiet_NaN();
    double obs_z = std::numeric_limits<double>::quiet_NaN();
    double obs_yaw = std::numeric_limits<double>::quiet_NaN();
    double obs_z_jump = std::numeric_limits<double>::quiet_NaN();
    double obs_dz_from_audit_center = std::numeric_limits<double>::quiet_NaN();
    std::array<double, 3> obs_z_audit_costs{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};

    // Binding-engine diagnostics
    double binding_confidence = std::numeric_limits<double>::quiet_NaN();
    int switch_event = 0;      // 0=no switch, 1=switch confirmed
    int switch_reason = 0;     // 0=none,1=confirmed,2=reject_prob,3=reject_margin,4=transition_abort
    int transition_state = 0;  // 0=LOCKED, 1=TRANSITION_CANDIDATE
    int z_audit_conflict_count = 0;
    double z_audit_confidence = std::numeric_limits<double>::quiet_NaN();
    double publish_x = std::numeric_limits<double>::quiet_NaN();
    double publish_y = std::numeric_limits<double>::quiet_NaN();
    double publish_z = std::numeric_limits<double>::quiet_NaN();
    double period_confidence = std::numeric_limits<double>::quiet_NaN();
    int period_update_applied = 0;
    int period_phase_index = -1;
    int spin_direction = 0;  // +1=CCW, -1=CW, 0=unknown
    double dz_small_est = std::numeric_limits<double>::quiet_NaN();
    double dz_large_est = std::numeric_limits<double>::quiet_NaN();
  };

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

  Eigen::Vector3d get_publish_velocity() const override { return center_velocity_est_; }
  bool is_ambiguous_single_mode() const override;
  int effective_num_armors() const override;
  int selected_panel_id() const;
  double normalized_entropy() const;
  double max_panel_probability() const;
  const DebugSnapshot &debug_snapshot() const;
  double confidence_scale() const override;

  std::vector<geometry_msgs::msg::Pose> build_armors_offset_for_message() const override;

 private:
  enum class TrackMode {
    STRUCTURED_3_ARMORS = 0,
    AMBIGUOUS_SINGLE_ARMOR = 1,
  };

  enum class BindingTransitionState {
    LOCKED = 0,
    TRANSITION_CANDIDATE = 1,
  };

  struct PanelHypothesis {
    int panel_id = 0;
    double cost = 0.0;
    double probability = 0.0;
    double center_yaw = 0.0;
    double center_z = 0.0;
    double yaw_err = 0.0;
    double z_state_err = 0.0;
    double z_hist_err = 0.0;
    double xy_residual = 0.0;
    double switch_penalty = 0.0;
  };

  struct ZJumpAuditResult {
    int panel_id = -1;
    double z_jump = std::numeric_limits<double>::quiet_NaN();
    double dz_from_center = std::numeric_limits<double>::quiet_NaN();
    std::array<double, 3> costs{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()};
  };

  const ObservationData *select_observation(
      const std::vector<ObservationData> &obs) const;

  std::array<PanelHypothesis, 3> evaluate_hypotheses(
      const ObservationData &obs,
      double predicted_center_z,
      double history_center_z) const;

  int infer_panel_id_from_hypotheses(
      const std::array<PanelHypothesis, 3> &hyps) const;

    ZJumpAuditResult infer_panel_id_from_z_jump_audit(
      const ObservationData &obs);

  void update_periodic_evidence(double z_jump, bool allow_model_update);
  void apply_periodic_jump_prior(std::array<PanelHypothesis, 3> &hyps,
                                 double z_jump) const;
  std::array<double, 3> periodic_template_for_spin() const;
  double compute_period_confidence_for_phase(
      int phase, const std::array<double, 3> &templ,
      int sample_count) const;
  int hypothesis_index_for_panel(const std::array<PanelHypothesis, 3> &hyps,
                                 int panel_id) const;
  double compute_same_panel_score(const PanelHypothesis &hyp,
                                  double predicted_center_z) const;
  int semantic_from_panel(int panel_id) const;
  void update_binding_state_machine(int candidate_panel, double candidate_prob,
                                    double candidate_margin,
                                    double same_panel_score,
                                    double switch_score);
  double binding_confidence_from_scores(double candidate_prob,
                                        double candidate_margin,
                                        double same_panel_score,
                                        double switch_score) const;

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
  BinderConfig build_binder_config_from_outpost() const;

  const UnifiedConfig config_;
  double radius_ = 0.26;
  std::array<double, 3> z_offsets_{0.06, 0.0, -0.06};
  std::array<double, 3> panel_angles_{0.0, 2.0 * M_PI / 3.0,
                                      -2.0 * M_PI / 3.0};
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
  DebugSnapshot debug_snapshot_{};

  std::deque<double> center_z_history_;
  std::optional<double> last_internal_update_time_;

  // Independent z-only audit state (observation stream based).
  bool z_audit_initialized_ = false;
  double z_audit_center_est_ = std::numeric_limits<double>::quiet_NaN();
  double z_audit_prev_obs_z_ = std::numeric_limits<double>::quiet_NaN();
  int z_audit_prev_panel_id_ = -1;

  // Binding engine state (periodic dz evidence + transition confirmation).
  int bound_panel_id_ = -1;
  BindingTransitionState binding_transition_state_ =
      BindingTransitionState::LOCKED;
  int transition_candidate_panel_ = -1;
  int transition_confirm_count_ = 0;
  int switch_event_ = 0;
  int switch_reason_ = 0;
  int z_audit_conflict_count_ = 0;
  double z_audit_confidence_ = std::numeric_limits<double>::quiet_NaN();
  bool binding_conflict_for_update_ = false;
  double binding_confidence_ = std::numeric_limits<double>::quiet_NaN();
  int bound_height_label_ = static_cast<int>(HeightSemantic::UNKNOWN);

  int candidate_panel_id_ = -1;
  double candidate_prob_ = std::numeric_limits<double>::quiet_NaN();
  double candidate_margin_ = std::numeric_limits<double>::quiet_NaN();
  double selected_xy_residual_ = std::numeric_limits<double>::quiet_NaN();

  std::deque<double> dz_jump_history_;
  double dz_small_est_ = std::numeric_limits<double>::quiet_NaN();
  double dz_large_est_ = std::numeric_limits<double>::quiet_NaN();
  int period_update_applied_ = 0;
  int period_phase_index_ = -1;
  double period_confidence_ = std::numeric_limits<double>::quiet_NaN();
  int spin_direction_ = 0;

  binder::RobotBindingProfile binding_profile_;
  std::unique_ptr<binder::BinderPipeline> binder_pipeline_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_ARMOR_TRACKER_HPP_
