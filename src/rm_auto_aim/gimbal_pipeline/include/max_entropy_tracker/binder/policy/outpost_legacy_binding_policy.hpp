// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_POLICY_OUTPOST_LEGACY_BINDING_POLICY_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_POLICY_OUTPOST_LEGACY_BINDING_POLICY_HPP_

#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <optional>

#include <Eigen/Dense>

#include "max_entropy_tracker/binder/model/binder_enums.hpp"
#include "max_entropy_tracker/binder/model/binder_types.hpp"
#include "max_entropy_tracker/binder/model/binding_hypothesis.hpp"
#include "max_entropy_tracker/binder/model/robot_binding_profile.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim::binder {

struct OutpostLegacyBindingInput {
  const ObservationData * obs = nullptr;
  int obs_count = 1;

  Eigen::Vector3d predicted_center_pos = Eigen::Vector3d::Zero();
  double predicted_center_yaw = 0.0;
  double yaw_rate_est = 0.0;

  bool ambiguous_mode = true;
  int lost_frames = 0;
  std::optional<double> last_timestamp;
  std::optional<double> last_obs_z;
};

struct OutpostLegacyBindingOutput {
  BinderOutput binder_output;

  int candidate_id = -1;
  double candidate_prob = 0.0;
  double candidate_margin = 0.0;
  double max_prob = 0.0;
  double entropy_norm = 1.0;
  double selected_xy_residual = std::numeric_limits<double>::quiet_NaN();

  std::array<BindingHypothesis, 3> hypotheses{};

  int z_audit_panel_id = -1;
  double z_audit_confidence = 0.0;
  double z_jump = std::numeric_limits<double>::quiet_NaN();
  double dz_from_center = std::numeric_limits<double>::quiet_NaN();
  std::array<double, 3> z_audit_costs{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};

  double period_confidence = 0.0;
  int period_phase = -1;
  int spin_direction = 0;
  double dz_small_est = std::numeric_limits<double>::quiet_NaN();
  double dz_large_est = std::numeric_limits<double>::quiet_NaN();

  int transition_state = 0;
  int transition_candidate_id = -1;
};

class OutpostLegacyBindingPolicy {
 public:
  explicit OutpostLegacyBindingPolicy(const UnifiedConfig & config,
                                      const RobotBindingProfile & profile);

  void reset(int init_panel_id, std::optional<double> obs_z = std::nullopt);

  OutpostLegacyBindingOutput step(const OutpostLegacyBindingInput & input);

 private:
  enum class TransitionState { LOCKED = 0, TRANSITION_CANDIDATE = 1 };

  struct ZJumpAuditResult {
    int panel_id = -1;
    double z_jump = std::numeric_limits<double>::quiet_NaN();
    double dz_from_center = std::numeric_limits<double>::quiet_NaN();
    std::array<double, 3> costs{{
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::quiet_NaN()}};
  };

  ZJumpAuditResult infer_panel_id_from_z_jump_audit(const ObservationData & obs);
  std::array<BindingHypothesis, 3> evaluate_hypotheses(
      const ObservationData & obs,
      const Eigen::Vector3d & predicted_center_pos,
      double predicted_center_yaw,
      double history_center_z) const;
  void compute_probabilities(std::array<BindingHypothesis, 3> & hyps) const;

  void update_periodic_evidence(double z_jump, bool allow_model_update,
                                double yaw_rate_est);
  void apply_periodic_jump_prior(std::array<BindingHypothesis, 3> & hyps,
                                 double z_jump) const;
  std::array<double, 3> periodic_template_for_spin() const;
  double compute_period_confidence_for_phase(int phase,
                                             const std::array<double, 3> & templ,
                                             int sample_count) const;

  int hypothesis_index_for_panel(
      const std::array<BindingHypothesis, 3> & hyps, int panel_id) const;
  double compute_same_panel_score(const BindingHypothesis & hyp,
                                  double predicted_center_z) const;
  void update_binding_state_machine(int candidate_panel, double candidate_prob,
                                    double candidate_margin,
                                    double same_panel_score,
                                    double switch_score);
  double binding_confidence_from_scores(double candidate_prob,
                                        double candidate_margin,
                                        double same_panel_score,
                                        double switch_score) const;

  double history_center_z_median() const;
  void push_center_z_history(double center_z);
  HeightLabel height_label_from_panel(int panel_id) const;

  UnifiedConfig config_;
  RobotBindingProfile profile_;
  double radius_ = 0.26;
  std::array<double, 3> z_offsets_{{0.06, 0.0, -0.06}};
  std::array<double, 3> panel_angles_{{0.0, 2.0 * M_PI / 3.0,
                                       -2.0 * M_PI / 3.0}};

  int selected_panel_id_ = 0;
  int bound_panel_id_ = -1;
  HeightLabel bound_height_label_ = HeightLabel::UNKNOWN;
  TransitionState transition_state_ = TransitionState::LOCKED;
  int transition_candidate_panel_ = -1;
  int transition_confirm_count_ = 0;
  int switch_event_ = 0;
  int switch_reason_ = 0;
  bool binding_conflict_for_update_ = false;
  double binding_confidence_ = 0.5;
  double entropy_norm_ = 1.0;

  bool z_audit_initialized_ = false;
  double z_audit_center_est_ = std::numeric_limits<double>::quiet_NaN();
  double z_audit_prev_obs_z_ = std::numeric_limits<double>::quiet_NaN();
  int z_audit_prev_panel_id_ = -1;
  int z_audit_conflict_count_ = 0;
  double z_audit_confidence_ = 0.0;

  std::deque<double> dz_jump_history_;
  double dz_small_est_ = std::numeric_limits<double>::quiet_NaN();
  double dz_large_est_ = std::numeric_limits<double>::quiet_NaN();
  double period_confidence_ = 0.0;
  int period_phase_index_ = -1;
  int spin_direction_ = 0;

  std::deque<double> center_z_history_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_POLICY_OUTPOST_LEGACY_BINDING_POLICY_HPP_
