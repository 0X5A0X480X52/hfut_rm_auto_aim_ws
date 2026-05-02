// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_SCORER_OUTPOST_HYPOTHESIS_EVALUATOR_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_SCORER_OUTPOST_HYPOTHESIS_EVALUATOR_HPP_

#include <array>
#include <cmath>

#include <Eigen/Dense>

#include "max_entropy_tracker/binder/model/binding_hypothesis.hpp"
#include "max_entropy_tracker/binder/model/outpost_binding_types.hpp"
#include "max_entropy_tracker/binder/model/robot_binding_profile.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim::binder {

class OutpostHypothesisEvaluator {
 public:
  OutpostHypothesisEvaluator(const UnifiedConfig & config,
                             const RobotBindingProfile & profile);

  std::array<BindingHypothesis, 3> evaluate(
      const ObservationData & obs,
      const Eigen::Vector3d & predicted_center_pos,
      double predicted_center_yaw,
      double history_center_z,
      int bound_panel_id) const;

  void apply_z_audit_prior(std::array<BindingHypothesis, 3> & hyps,
                           const OutpostZAuditResult & audit,
                           double audit_confidence,
                           bool ambiguous_mode,
                           double previous_entropy) const;

  void compute_probabilities(std::array<BindingHypothesis, 3> & hyps) const;
  int best_index(const std::array<BindingHypothesis, 3> & hyps) const;
  int second_index(const std::array<BindingHypothesis, 3> & hyps,
                   int best_idx) const;
  int hypothesis_index_for_panel(
      const std::array<BindingHypothesis, 3> & hyps, int panel_id) const;
  double same_panel_score(const BindingHypothesis & hyp,
                          double predicted_center_z) const;
  double entropy_norm(const std::array<BindingHypothesis, 3> & hyps) const;

 private:
  UnifiedConfig config_;
  RobotBindingProfile profile_;
  double radius_ = 0.26;
  std::array<double, 3> z_offsets_{{0.06, 0.0, -0.06}};
  std::array<double, 3> panel_angles_{{0.0, 2.0 * M_PI / 3.0,
                                       -2.0 * M_PI / 3.0}};
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_SCORER_OUTPOST_HYPOTHESIS_EVALUATOR_HPP_
