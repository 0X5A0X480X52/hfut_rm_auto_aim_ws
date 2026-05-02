// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/decoder/four_panel_jump_decoder.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::binder {

FourPanelJumpDecoder::FourPanelJumpDecoder(
    const FourPanelJumpDecoderConfig & config)
    : config_(config) {}

bool FourPanelJumpDecoder::evaluate_gates(
    int candidate_panel, double z_jump, bool has_z_jump,
    double selected_yaw_err, double cost_margin, int n_panels,
    const DecoderContext & ctx) const {
  const int raw_diff = std::abs(candidate_panel - ctx.last_panel_id);
  const bool adjacent_panel = (raw_diff == 1 || raw_diff == n_panels - 1);

  const bool jump_mag_ok =
      has_z_jump && (std::abs(z_jump) >= std::max(0.0, config_.z_jump_min));

  bool dz_match_ok = true;
  if (jump_mag_ok && std::isfinite(ctx.dz_jump_est)) {
    dz_match_ok =
        std::abs(std::abs(z_jump) - ctx.dz_jump_est) <=
        std::max(0.0, config_.dz_match_tolerance);
  }

  const double yaw_err =
      std::isfinite(selected_yaw_err) ? selected_yaw_err : 1e9;
  const bool yaw_ok = yaw_err <= std::max(1e-3, config_.yaw_err_gate);

  const double margin = std::isfinite(cost_margin) ? cost_margin : 0.0;
  const bool margin_ok = margin >= std::max(0.0, config_.cost_margin_min);

  return adjacent_panel && jump_mag_ok && dz_match_ok && yaw_ok && margin_ok;
}

JumpDecision FourPanelJumpDecoder::decode(const BinderFrameInput & input,
                                          DecoderContext & ctx) {
  JumpDecision jd;
  if (!input.profile) return jd;

  const int n = input.profile->panel_count;
  const bool gate = evaluate_gates(input.candidate_id, input.z_jump,
                                   input.has_z_jump, input.selected_yaw_err,
                                   input.cost_margin, n, ctx);

  if (!gate) return jd;

  jd.detected = true;
  jd.jump_kind = JumpKind::DZ;
  jd.from_id = ctx.last_panel_id;
  jd.to_id = input.candidate_id;
  jd.confidence = 0.8;
  jd.reason_code = 1;

  // Update EMA estimate (migrated from update_jump_statistics)
  if (input.has_z_jump) {
    const double abs_jump = std::abs(input.z_jump);
    if (abs_jump >= std::max(0.0, config_.z_jump_min)) {
      const double alpha = std::clamp(config_.dz_ema_alpha, 0.01, 1.0);
      if (!std::isfinite(ctx.dz_jump_est)) {
        ctx.dz_jump_est = abs_jump;
      } else {
        ctx.dz_jump_est =
            (1.0 - alpha) * ctx.dz_jump_est + alpha * abs_jump;
      }
    }
  }

  return jd;
}

}  // namespace fyt::auto_aim::binder
