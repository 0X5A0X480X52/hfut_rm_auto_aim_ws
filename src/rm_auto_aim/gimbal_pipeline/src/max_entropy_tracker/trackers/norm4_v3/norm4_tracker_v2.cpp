// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v3/norm4_tracker_v2.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace fyt::auto_aim {

Norm4ArmorTrackerV2::Norm4ArmorTrackerV2(const UnifiedConfig &config, double dt,
                                         bool /*enable_oscillation*/)
    : BaseTracker(dt),
      config_(config),
      backend_(std::make_unique<norm4_v3::Norm4UkfBackendV1>(config, dt)),
      maneuver_detector_(config.maneuver) {}

void Norm4ArmorTrackerV2::initialize(const std::vector<ObservationData> &obs,
                                     double r1, double r2, double dza) {
  if (obs.empty())
    throw std::invalid_argument("At least one observation required");

  default_r1_ = r1;
  default_r2_ = r2;
  default_dza_ = dza;

  int init_panel = obs[0].panel_id.value_or(0);
  backend_->reset(obs[0], init_panel, default_r1_, default_r2_, default_dza_);
  current_panel_id_ = init_panel;

  transition_to(TrackerState::INITIALIZING);
  mark_initialized();
  update_time(obs[0].timestamp.value_or(0.0));
}

void Norm4ArmorTrackerV2::predict(std::optional<double> target_time) {
  if (!is_initialized()) return;
  double dt = compute_dt(target_time);
  if (dt <= 0.0) return;
  backend_->predict(dt);
  if (target_time.has_value()) update_time(target_time.value());
}

bool Norm4ArmorTrackerV2::update(const std::vector<ObservationData> &obs) {
  if (!is_initialized() || obs.empty()) {
    handle_observation_loss(config_.tracker.tracking_thres,
                            config_.tracker.lost_thres);
    return false;
  }

  double obs_ts =
      obs[0].timestamp.value_or(current_time_.value_or(0.0));

  // Predict to observation time.
  if (current_time_.has_value() && obs_ts > current_time_.value()) {
    backend_->predict(obs_ts - current_time_.value());
  }

  // Build prior snapshot — all hypotheses evaluated from this single prior.
  auto ctx = backend_->buildPredictContext();

  // Generate hypotheses: 1 obs → 4 single, 2+ obs → 8 dual.
  auto hypotheses = hypothesis_generator_.generate(obs);
  hypothesis_generator_.attach_prior(&hypotheses);

  // Evaluate all hypotheses from the same prior.
  std::vector<norm4_v3::MeasurementEval> evals;
  evals.reserve(hypotheses.size());

  for (const auto &hyp : hypotheses) {
    norm4_v3::MeasurementEval eval;
    if (hyp.kind == norm4_v3::HypothesisKind::Single) {
      int panel = hyp.assignments[0].panel_id;
      eval = backend_->evaluateSingle(ctx, obs[hyp.assignments[0].obs_index],
                                       panel);
    } else {
      int p0 = hyp.assignments[0].panel_id;
      int p1 = hyp.assignments[1].panel_id;
      eval = backend_->evaluateDual(ctx, obs[hyp.assignments[0].obs_index],
                                     obs[hyp.assignments[1].obs_index], p0, p1);
    }
    // Combine prior log weight into score.
    eval.score = eval.log_likelihood + hyp.prior_log_weight;
    evals.push_back(eval);
  }

  // Select TopK, compute confidence.
  std::vector<norm4_v3::TopKEntry> topk;
  double top1_confidence = 0.0, top1_top2_margin = 0.0;
  select_topk(evals, hypotheses, /*topk_count=*/4, &topk,
              &top1_confidence, &top1_top2_margin);

  bool committed = false;
  std::string decision_reason;

  // Find the first gate-passing hypothesis in ranked order.
  int best_idx = -1;
  for (size_t i = 0; i < topk.size(); ++i) {
    if (topk[i].eval.gate_pass && topk[i].eval.valid) {
      best_idx = static_cast<int>(i);
      break;
    }
  }

  if (best_idx >= 0) {
    const auto &best_hyp = topk[best_idx].hypothesis;
    norm4_v3::UkfTrial trial;

    if (best_hyp.kind == norm4_v3::HypothesisKind::Single) {
      trial = backend_->tryUpdateSingle(
          ctx, obs[best_hyp.assignments[0].obs_index],
          best_hyp.assignments[0].panel_id);
    } else {
      trial = backend_->tryUpdateDual(
          ctx, obs[best_hyp.assignments[0].obs_index],
          obs[best_hyp.assignments[1].obs_index],
          best_hyp.assignments[0].panel_id,
          best_hyp.assignments[1].panel_id);
    }

    if (trial.success && trial.posterior_sanity_pass) {
      backend_->commit(trial);
      committed = true;

      if (best_hyp.kind == norm4_v3::HypothesisKind::Single) {
        current_panel_id_ = best_hyp.assignments[0].panel_id;
      }

      std::ostringstream oss;
      oss << "committed_" << best_hyp.debug_name
          << "_nis=" << trial.eval.nis
          << "_conf=" << top1_confidence
          << "_margin=" << top1_top2_margin;
      decision_reason = oss.str();
    } else {
      std::ostringstream oss;
      oss << "trial_rejected:" << trial.reject_reason;
      decision_reason = oss.str();
    }
  } else {
    decision_reason = "all_gate_fail";
  }

  // Update state tracking.
  update_time(obs_ts);
  increment_frame();

  if (committed) {
    handle_observation_received(config_.tracker.tracking_thres);
  } else {
    handle_observation_loss(config_.tracker.tracking_thres,
                            config_.tracker.lost_thres);
  }

  // Populate debug frame.
  last_hypothesis_debug_.valid = true;
  last_hypothesis_debug_.obs_count = static_cast<int>(obs.size());
  last_hypothesis_debug_.committed = committed;
  last_hypothesis_debug_.degraded = !committed;
  last_hypothesis_debug_.topk = topk;
  last_hypothesis_debug_.top1_confidence = top1_confidence;
  last_hypothesis_debug_.top1_top2_margin = top1_top2_margin;
  last_hypothesis_debug_.decision_reason = decision_reason;

  return committed;
}

Eigen::Vector3d Norm4ArmorTrackerV2::get_center_position() const {
  return backend_->get_center_position();
}

double Norm4ArmorTrackerV2::get_yaw() const { return backend_->get_yaw(); }

std::pair<double, double> Norm4ArmorTrackerV2::get_radii() const {
  return backend_->get_radii();
}

SpinFilterInterface &Norm4ArmorTrackerV2::spin_filter() { return *backend_; }

const SpinFilterInterface &Norm4ArmorTrackerV2::spin_filter() const {
  return *backend_;
}

ManeuverResult Norm4ArmorTrackerV2::assess_maneuver() const {
  return maneuver_detector_.detect(backend_->last_nis(),
                                   backend_->last_innov_xyz().norm(),
                                   backend_->last_update_type());
}

Eigen::Vector3d Norm4ArmorTrackerV2::get_publish_velocity() const {
  const auto idx = backend_->state_idx();
  const auto &x = backend_->x();
  Eigen::Vector3d vel(x(idx.VX()), x(idx.VY()), x(idx.VZ()));
  return vel;
}

bool Norm4ArmorTrackerV2::is_ambiguous_single_mode() const { return false; }

int Norm4ArmorTrackerV2::effective_num_armors() const { return 4; }

double Norm4ArmorTrackerV2::confidence_scale() const { return 1.0; }

std::vector<geometry_msgs::msg::Pose>
Norm4ArmorTrackerV2::build_armors_offset_for_message() const {
  return {};
}

void Norm4ArmorTrackerV2::select_topk(
    std::vector<norm4_v3::MeasurementEval> &evals,
    const std::vector<norm4_v3::Hypothesis> &hyps,
    int topk_count,
    std::vector<norm4_v3::TopKEntry> *topk_out,
    double *confidence_out, double *margin_out) const {
  const int n = static_cast<int>(evals.size());
  if (n == 0) {
    *confidence_out = 0.0;
    *margin_out = 0.0;
    return;
  }

  // Create indexed list.
  std::vector<int> indices(n);
  std::iota(indices.begin(), indices.end(), 0);

  // Sort: gate_pass first, then by score descending.
  std::sort(indices.begin(), indices.end(),
            [&evals](int a, int b) {
              if (evals[a].gate_pass != evals[b].gate_pass)
                return evals[a].gate_pass;
              return evals[a].score > evals[b].score;
            });

  // Take TopK.
  const int k = std::min(topk_count, n);
  topk_out->clear();
  topk_out->reserve(k);

  // Compute log-sum-exp over topk for softmax.
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
    entry.hypothesis = hyps[indices[i]];
    entry.eval = evals[indices[i]];
    entry.normalized_weight = weights[i];
    topk_out->push_back(entry);
  }

  *confidence_out = topk_out->empty() ? 0.0 : topk_out->front().normalized_weight;
  *margin_out = (k >= 2)
                    ? (topk_out->at(0).eval.score - topk_out->at(1).eval.score)
                    : std::numeric_limits<double>::infinity();
}

}  // namespace fyt::auto_aim
