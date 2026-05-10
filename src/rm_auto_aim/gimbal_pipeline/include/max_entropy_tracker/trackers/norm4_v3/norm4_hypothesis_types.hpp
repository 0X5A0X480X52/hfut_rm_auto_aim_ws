// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_HYPOTHESIS_TYPES_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_HYPOTHESIS_TYPES_HPP_

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace fyt::auto_aim::norm4_v3 {

enum class HypothesisKind { Single, Dual };

struct PanelAssignment {
  int obs_index = -1;
  int panel_id = -1;
};

struct Hypothesis {
  HypothesisKind kind = HypothesisKind::Single;
  std::array<PanelAssignment, 2> assignments{};
  int assignment_count = 0;
  double prior_log_weight = 0.0;
  std::string debug_name;
};

struct MeasurementEval {
  bool valid = false;
  bool gate_pass = false;

  double nis = 0.0;
  double mahalanobis = 0.0;
  double log_likelihood = 0.0;
  double score = 0.0;

  double chi2_pos = 0.0;
  double chi2_yaw = 0.0;

  Eigen::VectorXd innovation;
  Eigen::MatrixXd S;
  Eigen::VectorXd z_pred;
  Eigen::VectorXd z_obs;

  std::string reject_reason;
};

struct UkfTrial {
  bool success = false;

  Hypothesis hypothesis;
  MeasurementEval eval;

  Eigen::VectorXd x_post;
  Eigen::MatrixXd P_post;
  int k_post = 0;
  int last_k_post = 0;

  double reconstruction_pos_error = 0.0;
  double reconstruction_yaw_error = 0.0;
  bool posterior_sanity_pass = false;
  std::string reject_reason;
};

struct PredictContext {
  Eigen::VectorXd x_prior;
  Eigen::MatrixXd P_prior;
  int k_prior = 0;
  int last_k_prior = 0;
  double timestamp = 0.0;
};

struct TopKEntry {
  Hypothesis hypothesis;
  MeasurementEval eval;
  double normalized_weight = 0.0;
};

struct HypothesisDebugFrame {
  bool valid = false;
  int obs_count = 0;
  bool committed = false;
  bool degraded = false;

  std::vector<TopKEntry> topk;
  double top1_confidence = 0.0;
  double top1_top2_margin = 0.0;
  std::string decision_reason;
};

struct PanelProfile {
  int panel_id = 0;
  bool upper = false;
  bool use_r2 = false;
  double phase_offset = 0.0;
  double z_sign = -1.0;
};

inline PanelProfile get_panel_profile(int panel_id) {
  const int p = ((panel_id % 4) + 4) % 4;
  PanelProfile pp;
  pp.panel_id = p;
  pp.upper = (p % 2 == 1);
  pp.use_r2 = (p % 2 == 1);
  pp.phase_offset = p * (M_PI / 2.0);
  pp.z_sign = (p % 2 == 0) ? -1.0 : 1.0;
  return pp;
}

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_HYPOTHESIS_TYPES_HPP_
