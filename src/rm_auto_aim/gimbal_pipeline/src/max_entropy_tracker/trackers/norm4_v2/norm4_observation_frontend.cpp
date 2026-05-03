// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/trackers/norm4_v2/norm4_observation_frontend.hpp"

#include <algorithm>
#include <cmath>

#include "max_entropy_tracker/utils/angle_utils.hpp"

namespace fyt::auto_aim::norm4_v2 {

namespace {

constexpr double kLog4 = 1.3862943611198906;
constexpr double kDualZDiffStrong = 0.015;

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }
int clamp_panel(int panel_id) {
  int v = panel_id % 4;
  if (v < 0) v += 4;
  return v;
}

}  // namespace

ObservationFrontend::ObservationFrontend(const UnifiedConfig &cfg) : cfg_(cfg) {
  panel_associator_.configure_periodic_binding(
      cfg_.tracker.periodic_binding_enable, cfg_.tracker.periodic_binding_weight,
      cfg_.tracker.periodic_binding_spin_rate_gate);
  const double step = std::abs(cfg_.tracker.panel_angle_step) > 1e-6
                          ? std::abs(cfg_.tracker.panel_angle_step)
                          : (M_PI / 2.0);
  panel_angles_ = {0.0, step, 2.0 * step, -step};
}

void ObservationFrontend::reset_history() { panel_associator_.reset_history(); }

const ObservationData *ObservationFrontend::select_primary_observation(
    const std::vector<ObservationData> &obs,
    const Norm4RuntimeContext & /*ctx*/) const {
  if (obs.empty()) return nullptr;
  const ObservationData *best = &obs.front();
  for (const auto &o : obs) {
    if (o.timestamp.has_value()) {
      if (!best->timestamp.has_value() ||
          o.timestamp.value() > best->timestamp.value()) {
        best = &o;
        continue;
      }
    }
    if ((!o.timestamp.has_value() && !best->timestamp.has_value()) &&
        (o.confidence > best->confidence)) {
      best = &o;
    }
  }
  return best;
}

binder::HeightLabel ObservationFrontend::default_height_label(int panel_id) const {
  const int p = ((panel_id % 4) + 4) % 4;
  return (p % 2 == 0) ? binder::HeightLabel::LOWER : binder::HeightLabel::UPPER;
}

std::string ObservationFrontend::layer_from_label(binder::HeightLabel label) {
  if (label == binder::HeightLabel::UPPER) return "upper";
  if (label == binder::HeightLabel::LOWER) return "lower";
  return "";
}

double ObservationFrontend::radius_for_panel(int panel_id,
                                             const Norm4RuntimeContext &ctx) const {
  const int p = ((panel_id % 4) + 4) % 4;
  return (p % 2 == 0) ? ctx.r1 : ctx.r2;
}

int ObservationFrontend::infer_panel_for_observation(
    const ObservationData &obs, const Norm4RuntimeContext &ctx,
    PanelAssociator::AssociationDiagnostics *diag) {
  auto [panel_id, _cw, _err] = panel_associator_.associate_panel(
      obs.yaw, ctx.reference_center_yaw, obs.z, ctx.center_pos.z(), obs.x, obs.y,
      ctx.center_pos.x(), ctx.center_pos.y(), ctx.r1, ctx.r2, ctx.yaw_rate,
      std::abs(ctx.dza), diag);
  return clamp_panel(panel_id);
}

DualObservationAssignment ObservationFrontend::assign_dual_observations(
    const ObservationData &obs1, const ObservationData &obs2,
    const Norm4RuntimeContext &ctx) const {
  DualObservationAssignment best;
  if (!ctx.reference_center_yaw.has_value()) return best;

  const double z_diff = obs1.z - obs2.z;
  const bool z_strong = std::abs(z_diff) >= kDualZDiffStrong;
  const int high_index = z_diff >= 0.0 ? 0 : 1;

  auto single_cost = [&](const ObservationData &obs, int panel_id) {
    const int p = clamp_panel(panel_id);
    const double panel_angle = panel_angles_[p];
    const double expected_yaw = normalize_angle(ctx.center_yaw + panel_angle);
    const double yaw_err = std::abs(normalize_angle(obs.yaw - expected_yaw));
    const double radius = radius_for_panel(p, ctx);
    const double pred_x = ctx.center_pos.x() + radius * std::cos(expected_yaw);
    const double pred_y = ctx.center_pos.y() + radius * std::sin(expected_yaw);
    const double xy_err = std::hypot(obs.x - pred_x, obs.y - pred_y);

    double z_state_err = 0.0;
    if (ctx.dza_converged || std::abs(ctx.dza) > 1e-4) {
      const double z_offset =
          default_height_label(p) == binder::HeightLabel::UPPER
              ? std::abs(ctx.dza)
              : -std::abs(ctx.dza);
      z_state_err = std::abs(obs.z - (ctx.center_pos.z() + z_offset));
    }
    return yaw_err + PanelAssociator::DEFAULT_POS_WEIGHT * xy_err +
           2.0 * z_state_err;
  };

  for (int p1 = 0; p1 < 4; ++p1) {
    for (int p2 = 0; p2 < 4; ++p2) {
      if (p1 == p2) continue;
      const int diff = std::abs(p1 - p2);
      const bool adjacent = (diff == 1 || diff == 3);
      if (!adjacent) continue;

      double cost = single_cost(obs1, p1) + single_cost(obs2, p2);

      const auto label1 = default_height_label(p1);
      const auto label2 = default_height_label(p2);
      if (z_strong) {
        const bool obs1_should_upper = (high_index == 0);
        const bool obs2_should_upper = (high_index == 1);
        if (obs1_should_upper && label1 != binder::HeightLabel::UPPER) cost += 4.0;
        if (!obs1_should_upper && label1 != binder::HeightLabel::LOWER) cost += 4.0;
        if (obs2_should_upper && label2 != binder::HeightLabel::UPPER) cost += 4.0;
        if (!obs2_should_upper && label2 != binder::HeightLabel::LOWER) cost += 4.0;
      }

      if (!best.valid || cost < best.cost) {
        best.valid = true;
        best.panel_id_1 = p1;
        best.panel_id_2 = p2;
        best.label_1 = z_strong
                           ? (high_index == 0 ? binder::HeightLabel::UPPER
                                               : binder::HeightLabel::LOWER)
                           : label1;
        best.label_2 = z_strong
                           ? (high_index == 1 ? binder::HeightLabel::UPPER
                                               : binder::HeightLabel::LOWER)
                           : label2;
        best.layer_1 = layer_from_label(best.label_1);
        best.layer_2 = layer_from_label(best.label_2);
        best.height_confidence =
            z_strong ? clamp01(std::abs(z_diff) / 0.05) : 0.35;
        best.cost = cost;
      }
    }
  }
  return best;
}

void ObservationFrontend::apply_forced_assignment(
    BindingCandidate *candidate, const ObservationData &obs,
    const Norm4RuntimeContext &ctx, int panel_id,
    binder::HeightLabel label) const {
  if (candidate == nullptr || panel_id < 0) return;
  const int p = clamp_panel(panel_id);
  candidate->candidate_panel_id = p;
  candidate->candidate_height_label =
      label == binder::HeightLabel::UNKNOWN ? default_height_label(p) : label;

  const double radius = radius_for_panel(p, ctx);
  const double pred_armor_yaw = normalize_angle(ctx.center_yaw + panel_angles_[p]);
  const double pred_x = ctx.center_pos.x() + radius * std::cos(pred_armor_yaw);
  const double pred_y = ctx.center_pos.y() + radius * std::sin(pred_armor_yaw);
  candidate->selected_xy_residual = std::hypot(obs.x - pred_x, obs.y - pred_y);
  candidate->selected_yaw_err = std::abs(normalize_angle(obs.yaw - pred_armor_yaw));
  candidate->candidate_prob = std::max(candidate->candidate_prob, 0.85);
  candidate->max_prob = candidate->candidate_prob;
  candidate->candidate_margin = std::max(candidate->candidate_margin, 0.50);
  candidate->entropy_norm = std::min(candidate->entropy_norm, 0.35);
  candidate->height_confidence = std::max(candidate->height_confidence, 0.80);
}

BindingCandidate ObservationFrontend::build_binding_candidate(
    const ObservationData &obs, const Norm4RuntimeContext &ctx) {
  BindingCandidate c;
  c.candidate_panel_id = infer_panel_for_observation(obs, ctx, &c.assoc_diag);
  c.selected_yaw_err = c.assoc_diag.selected_yaw_err;
  c.cost_margin = c.assoc_diag.cost_margin;
  c.candidate_height_label = default_height_label(c.candidate_panel_id);
  c.height_confidence = std::isfinite(c.cost_margin)
                            ? clamp01(c.cost_margin / std::max(
                                                        1e-3,
                                                        cfg_.tracker.jump_binding_cost_margin_min))
                            : 0.0;

  const double radius = radius_for_panel(c.candidate_panel_id, ctx);
  const double pred_armor_yaw =
      normalize_angle(ctx.center_yaw + panel_angles_[c.candidate_panel_id]);
  const double pred_x = ctx.center_pos.x() + radius * std::cos(pred_armor_yaw);
  const double pred_y = ctx.center_pos.y() + radius * std::sin(pred_armor_yaw);
  c.selected_xy_residual = std::hypot(obs.x - pred_x, obs.y - pred_y);

  const double temp = std::max(1e-3, 1.5 * cfg_.tracker.jump_binding_cost_margin_min);
  const double margin = std::isfinite(c.cost_margin) ? std::max(0.0, c.cost_margin) : 0.0;
  const double scaled = margin / temp;
  const double exp_other = std::exp(-scaled);
  const double p_best = 1.0 / (1.0 + 3.0 * exp_other);
  const double p_other = (1.0 - p_best) / 3.0;
  c.candidate_prob = clamp01(p_best);
  c.max_prob = c.candidate_prob;
  c.candidate_margin = std::max(0.0, p_best - p_other);

  double entropy = 0.0;
  const double pb = std::max(1e-12, p_best);
  const double po = std::max(1e-12, p_other);
  entropy -= pb * std::log(pb);
  entropy -= 3.0 * po * std::log(po);
  c.entropy_norm = clamp01(entropy / kLog4);

  if (ctx.last_obs_z.has_value()) {
    c.z_jump = obs.z - ctx.last_obs_z.value();
    c.has_z_jump = std::isfinite(c.z_jump);
  }

  return c;
}

}  // namespace fyt::auto_aim::norm4_v2
