// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_POLICY_OUTPOST_LEGACY_BINDING_POLICY_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_POLICY_OUTPOST_LEGACY_BINDING_POLICY_HPP_

#include <array>
#include <limits>
#include <optional>

#include <Eigen/Dense>

#include "max_entropy_tracker/binder/core/center_z_history.hpp"
#include "max_entropy_tracker/binder/core/outpost_binding_fsm.hpp"
#include "max_entropy_tracker/binder/decoder/outpost_periodic_dz_evidence.hpp"
#include "max_entropy_tracker/binder/decoder/outpost_z_audit.hpp"
#include "max_entropy_tracker/binder/model/binder_types.hpp"
#include "max_entropy_tracker/binder/model/binding_hypothesis.hpp"
#include "max_entropy_tracker/binder/model/outpost_binding_types.hpp"
#include "max_entropy_tracker/binder/model/robot_binding_profile.hpp"
#include "max_entropy_tracker/binder/scorer/outpost_binding_confidence.hpp"
#include "max_entropy_tracker/binder/scorer/outpost_hypothesis_evaluator.hpp"
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
  OutpostLegacyBindingOutput build_output(
      const OutpostBindingFSMOutput & fsm_out,
      const OutpostZAuditResult & audit,
      const std::array<BindingHypothesis, 3> & hyps,
      int candidate_panel,
      double candidate_prob,
      double candidate_margin,
      int selected_idx,
      double selected_panel_score,
      double switch_score,
      double binding_confidence,
      bool binding_conflict_for_update) const;

  void apply_directional_topology_prior(
      std::array<BindingHypothesis, 3> & hyps,
      int bound_panel_id,
      int spin_direction) const;

  UnifiedConfig config_;
  RobotBindingProfile profile_;
  std::array<double, 3> z_offsets_{{0.06, 0.0, -0.06}};

  OutpostZAudit z_audit_;
  OutpostPeriodicDzEvidence periodic_;
  OutpostHypothesisEvaluator hypothesis_evaluator_;
  OutpostBindingConfidenceScorer confidence_scorer_;
  OutpostBindingFSM binding_fsm_;
  CenterZHistory center_z_history_;

  double entropy_norm_ = 1.0;
  double binding_confidence_ = 0.5;
  int z_audit_conflict_count_ = 0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_POLICY_OUTPOST_LEGACY_BINDING_POLICY_HPP_
