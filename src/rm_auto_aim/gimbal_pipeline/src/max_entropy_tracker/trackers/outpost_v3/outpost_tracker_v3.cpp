// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/outpost_v3/outpost_tracker_v3.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace fyt::auto_aim {

OutpostTrackerV3::OutpostTrackerV3(const outpost_v3::OutpostV3Config &cfg,
                                   double dt)
    : BaseTracker(dt),
      cfg_(cfg),
      backend_(std::make_unique<outpost_v3::OutpostInEKFBackend>(cfg_, dt)) {}

void OutpostTrackerV3::initialize(const std::vector<ObservationData> &obs,
                                  double /*r1*/, double /*r2*/,
                                  double /*dza*/) {
  if (obs.empty())
    throw std::invalid_argument("At least one observation required");

  // Start with panel 0 as initial guess — the hypothesis enumeration
  // in update() will naturally correct this if wrong.
  int init_panel = obs[0].panel_id.value_or(0);
  // r1/r2/dza ignored — outpost InEKF uses known structure constants
  backend_->reset(obs[0], init_panel, 0.15, 0.20, 0.0);
  current_panel_id_ = init_panel;
  mode_ = outpost_v3::OutpostV3Mode::AMBIGUOUS;
  consecutive_degraded_ = 0;
  consecutive_stable_ = 0;

  transition_to(TrackerState::INITIALIZING);
  mark_initialized();
  update_time(obs[0].timestamp.value_or(0.0));
}

void OutpostTrackerV3::predict(std::optional<double> target_time) {
  if (!is_initialized()) return;
  double dt = compute_dt(target_time);
  if (dt <= 0.0) return;
  backend_->predict(dt);
  if (target_time.has_value()) update_time(target_time.value());
}

bool OutpostTrackerV3::update(const std::vector<ObservationData> &obs) {
  if (!is_initialized() || obs.empty()) {
    handle_observation_loss(3, 10);
    return false;
  }

  double obs_ts = obs[0].timestamp.value_or(current_time_.value_or(0.0));

  // Predict to observation time
  if (current_time_.has_value() && obs_ts > current_time_.value()) {
    backend_->predict(obs_ts - current_time_.value());
  }

  // Build prior snapshot — all hypotheses evaluated from same prior
  auto ctx = backend_->buildPredictContext();

  // Generate hypotheses: 3 single-hypotheses (one per panel)
  auto hypotheses = hypothesis_generator_.generate(obs);
  hypothesis_generator_.attach_prior(
      &hypotheses, current_panel_id_,
      cfg_.prior.panel_switch_penalty);

  // Evaluate all hypotheses
  std::vector<norm4_v3::MeasurementEval> evals;
  evals.reserve(hypotheses.size());

  for (const auto &hyp : hypotheses) {
    auto eval = backend_->evaluateSingle(
        ctx, obs[hyp.obs_index], hyp.panel_id);
    eval.score = eval.log_likelihood + hyp.prior_log_weight;
    evals.push_back(eval);
  }

  // Select TopK, compute confidence via softmax
  const auto &sel_cfg = cfg_.hypothesis_selector;
  int topk_count = std::max(1, sel_cfg.topk);
  std::vector<norm4_v3::TopKEntry> topk;
  double top1_confidence = 0.0, top1_top2_margin = 0.0;
  select_topk(evals, hypotheses, topk_count, &topk,
              &top1_confidence, &top1_top2_margin);

  bool committed = false;
  std::string decision_reason;

  // Find first gate-passing hypothesis in ranked order
  int best_idx = -1;
  for (size_t i = 0; i < topk.size(); ++i) {
    if (topk[i].eval.gate_pass && topk[i].eval.valid) {
      best_idx = static_cast<int>(i);
      break;
    }
  }

  bool allow_commit = (mode_ == outpost_v3::OutpostV3Mode::STRUCTURED);

  if (!allow_commit) {
    decision_reason = "ambiguous_mode_predict_only";
  } else if (best_idx >= 0) {
    // Commit gate: confidence / margin check
    bool commit_gate_pass = true;
    std::ostringstream gate_oss;

    if (top1_confidence < sel_cfg.min_top1_confidence) {
      commit_gate_pass = false;
      gate_oss << "conf=" << top1_confidence
               << "<" << sel_cfg.min_top1_confidence;
    }
    if (top1_top2_margin < sel_cfg.min_top1_top2_margin) {
      commit_gate_pass = false;
      if (!gate_oss.str().empty()) gate_oss << ",";
      gate_oss << "margin=" << top1_top2_margin
               << "<" << sel_cfg.min_top1_top2_margin;
    }

    if (commit_gate_pass) {
      int panel_id =
          topk[best_idx].hypothesis.assignments[0].panel_id;
      int obs_idx =
          topk[best_idx].hypothesis.assignments[0].obs_index;

      auto trial = backend_->tryUpdateSingle(
          ctx, obs[obs_idx], panel_id);

      // Trial gate: success + sanity + reconstruction
      bool trial_ok = trial.success && trial.posterior_sanity_pass;
      if (trial_ok &&
          trial.reconstruction_pos_error >
              sel_cfg.max_reconstruction_pos_error) {
        trial_ok = false;
        std::ostringstream oss;
        oss << "reconstruction=" << trial.reconstruction_pos_error
            << ">" << sel_cfg.max_reconstruction_pos_error;
        trial.reject_reason = oss.str();
      }

      if (trial_ok) {
        backend_->commit(trial);
        committed = true;
        current_panel_id_ = panel_id;

        std::ostringstream oss;
        oss << "committed_P" << panel_id
            << "_nis=" << trial.eval.nis
            << "_conf=" << top1_confidence
            << "_margin=" << top1_top2_margin
            << "_recon=" << trial.reconstruction_pos_error;
        decision_reason = oss.str();
      } else {
        std::ostringstream oss;
        oss << "trial_rejected:" << trial.reject_reason;
        decision_reason = oss.str();
      }
    } else {
      decision_reason = "commit_gate_fail:" + gate_oss.str();
    }
  } else if (allow_commit) {
    decision_reason = "all_gate_fail";
  }

  // Update state tracking
  update_time(obs_ts);
  increment_frame();

  if (committed) {
    handle_observation_received(3);
  } else {
    handle_observation_loss(3, 10);
  }

  // Update mode routing
  update_mode_routing(top1_confidence, top1_top2_margin, committed);

  // Populate debug
  populate_debug_snapshot(committed, topk, top1_confidence,
                          top1_top2_margin, decision_reason);

  return true;
}

Eigen::Vector3d OutpostTrackerV3::get_center_position() const {
  return backend_->get_center_position();
}

double OutpostTrackerV3::get_yaw() const { return backend_->get_yaw(); }

std::pair<double, double> OutpostTrackerV3::get_radii() const {
  return backend_->get_radii();
}

SpinFilterInterface &OutpostTrackerV3::spin_filter() {
  return backend_->spin_filter();
}

const SpinFilterInterface &OutpostTrackerV3::spin_filter() const {
  return backend_->spin_filter();
}

ManeuverResult OutpostTrackerV3::assess_maneuver() const {
  ManeuverResult r;
  r.nis = backend_->last_nis();
  r.innov_norm = backend_->last_innov_xyz().norm();
  r.update_type = backend_->last_update_type();
  return r;
}

Eigen::Vector3d OutpostTrackerV3::get_publish_velocity() const {
  const auto &b_x = backend_->x();
  using Idx = outpost_v3::OutpostStateIndex;
  return Eigen::Vector3d(b_x(Idx::VX), b_x(Idx::VY), b_x(Idx::VZ));
}

bool OutpostTrackerV3::is_ambiguous_single_mode() const {
  return mode_ == outpost_v3::OutpostV3Mode::AMBIGUOUS;
}

int OutpostTrackerV3::effective_num_armors() const {
  return outpost_v3::kNumPanels;
}

double OutpostTrackerV3::confidence_scale() const {
  return (mode_ == outpost_v3::OutpostV3Mode::AMBIGUOUS) ? 0.7 : 1.0;
}

std::vector<geometry_msgs::msg::Pose>
OutpostTrackerV3::build_armors_offset_for_message() const {
  return {};
}

// ── TopK selection with softmax ──

void OutpostTrackerV3::select_topk(
    std::vector<norm4_v3::MeasurementEval> &evals,
    const std::vector<outpost_v3::OutpostHypothesis> &hyps,
    int topk_count, std::vector<norm4_v3::TopKEntry> *topk_out,
    double *confidence_out, double *margin_out) const {
  const int n = static_cast<int>(evals.size());
  if (n == 0) {
    *confidence_out = 0.0;
    *margin_out = 0.0;
    return;
  }

  std::vector<int> indices(n);
  std::iota(indices.begin(), indices.end(), 0);

  // Sort: gate_pass first, then by score descending
  std::sort(indices.begin(), indices.end(),
            [&evals](int a, int b) {
              if (evals[a].gate_pass != evals[b].gate_pass)
                return evals[a].gate_pass;
              return evals[a].score > evals[b].score;
            });

  const int k = std::min(topk_count, n);
  topk_out->clear();
  topk_out->reserve(k);

  // Log-sum-exp softmax over topk
  double max_score = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < k; ++i) {
    max_score = std::max(max_score, evals[indices[i]].score);
  }

  double logsumexp = 0.0;
  std::vector<double> weights(k);
  for (int i = 0; i < k; ++i) {
    weights[i] = std::exp(evals[indices[i]].score - max_score);
    logsumexp += weights[i];
  }

  for (int i = 0; i < k; ++i) {
    weights[i] /= logsumexp;
    norm4_v3::TopKEntry entry;
    // Map OutpostHypothesis to norm4_v3::Hypothesis
    const auto &hyp = hyps[indices[i]];
    entry.hypothesis.kind = norm4_v3::HypothesisKind::Single;
    entry.hypothesis.assignments[0] = {hyp.obs_index, hyp.panel_id};
    entry.hypothesis.assignment_count = 1;
    entry.hypothesis.prior_log_weight = hyp.prior_log_weight;
    entry.hypothesis.debug_name = hyp.debug_name;
    entry.eval = evals[indices[i]];
    entry.normalized_weight = weights[i];
    topk_out->push_back(entry);
  }

  *confidence_out =
      topk_out->empty() ? 0.0 : topk_out->front().normalized_weight;
  *margin_out =
      (k >= 2)
          ? (topk_out->at(0).eval.score - topk_out->at(1).eval.score)
          : std::numeric_limits<double>::infinity();
}

// ── Mode routing ──

void OutpostTrackerV3::update_mode_routing(double confidence, double margin,
                                           bool committed) {
  const auto &mr = cfg_.mode_routing;

  if (mode_ == outpost_v3::OutpostV3Mode::AMBIGUOUS) {
    if (confidence > mr.P_enter_structured &&
        margin > mr.M_enter_structured) {
      consecutive_stable_++;
      if (consecutive_stable_ >= mr.stable_frames) {
        mode_ = outpost_v3::OutpostV3Mode::STRUCTURED;
        consecutive_stable_ = 0;
        consecutive_degraded_ = 0;
      }
    } else {
      consecutive_stable_ = 0;
    }
  } else {
    // STRUCTURED
    if (committed) {
      consecutive_degraded_ = 0;
    } else {
      consecutive_degraded_++;
    }

    if (confidence < mr.P_exit_structured ||
        margin < mr.M_exit_structured) {
      consecutive_degraded_++;
    }

    if (consecutive_degraded_ >= mr.degraded_frames) {
      mode_ = outpost_v3::OutpostV3Mode::AMBIGUOUS;
      consecutive_degraded_ = 0;
      consecutive_stable_ = 0;
    }
  }
}

// ── Debug snapshot ──

void OutpostTrackerV3::populate_debug_snapshot(
    bool committed, const std::vector<norm4_v3::TopKEntry> &topk,
    double top1_confidence, double top1_top2_margin,
    const std::string &decision_reason) {
  debug_snapshot_.valid = true;
  debug_snapshot_.mode_state =
      (mode_ == outpost_v3::OutpostV3Mode::AMBIGUOUS) ? 1 : 0;
  debug_snapshot_.current_panel_id = current_panel_id_;
  debug_snapshot_.committed = committed;
  debug_snapshot_.top1_confidence = top1_confidence;
  debug_snapshot_.top1_top2_margin = top1_top2_margin;
  debug_snapshot_.decision_reason = decision_reason;
  debug_snapshot_.consecutive_degraded = consecutive_degraded_;
  debug_snapshot_.consecutive_stable = consecutive_stable_;

  if (!topk.empty()) {
    const auto &top = topk.front();
    debug_snapshot_.candidate_panel_id =
        top.hypothesis.assignments[0].panel_id;
    debug_snapshot_.candidate_prob = top.normalized_weight;
    debug_snapshot_.entropy_norm = 1.0 - top.normalized_weight;
    debug_snapshot_.max_prob = top.normalized_weight;
    debug_snapshot_.top1_nis = top.eval.nis;

    if (topk.size() >= 2) {
      const auto &second = topk[1];
      debug_snapshot_.candidate_margin =
          top.eval.score - second.eval.score;
    }
  }
}

}  // namespace fyt::auto_aim
