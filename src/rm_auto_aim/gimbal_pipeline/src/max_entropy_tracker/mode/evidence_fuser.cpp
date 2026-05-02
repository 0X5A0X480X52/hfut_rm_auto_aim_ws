// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/mode/evidence_fuser.hpp"

#include <algorithm>
#include <iostream>

#include "max_entropy_tracker/binder/model/binder_enums.hpp"

namespace fyt::auto_aim::mode {

namespace {

double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }

double normalized_margin(double margin) { return clamp01(margin / 0.25); }

void normalize_weights(double w_dual, double w_margin, double w_health,
                       double w_entropy, bool dual_available, double *nd,
                       double *nm, double *nh, double *ne) {
  const double sd = dual_available ? std::max(0.0, w_dual) : 0.0;
  const double sm = std::max(0.0, w_margin);
  const double sh = std::max(0.0, w_health);
  const double se = std::max(0.0, w_entropy);
  const double sum = sd + sm + sh + se;
  if (sum <= 1e-9) {
    *nd = 0.25;
    *nm = 0.25;
    *nh = 0.25;
    *ne = 0.25;
    return;
  }
  *nd = sd / sum;
  *nm = sm / sum;
  *nh = sh / sum;
  *ne = se / sum;
}

}  // namespace

EvidenceFuser::EvidenceFuser(const EvidenceFuserConfig & cfg) : cfg_(cfg) {}

ModeEvidence EvidenceFuser::fuse(
    double timestamp, int obs_count, int candidate_id, bool has_2dz_signature,
    double entropy_norm, double max_prob, double candidate_margin,
    const binder::BinderDebugSnapshot & binder_dbg) {
  ModeEvidence ev;
  ev.timestamp = timestamp;
  ev.obs_count = obs_count;
  ev.has_dual_obs = (obs_count >= 2);
  ev.candidate_id = candidate_id;
  ev.has_2dz_signature = has_2dz_signature;
  ev.jump_detected = binder_dbg.jump_detected;
  ev.jump_confidence = clamp01(binder_dbg.jump_confidence);
  ev.candidate_margin = std::max(0.0, candidate_margin);
  ev.binder_health_score = clamp01(binder_dbg.health_score);
  ev.binder_bad_frames = std::max(0, binder_dbg.consecutive_bad_frames);
  ev.binder_force_rebind = binder_dbg.force_rebind_flag;
  ev.entropy_norm = clamp01(entropy_norm);
  ev.max_prob = clamp01(max_prob);

  // jump_detected is an instantaneous event; only consume rising-edge pulses.
  bool jump_rising_edge = ev.jump_detected && !last_jump_detected_;
  if (ev.jump_detected && last_jump_event_time_ >= 0.0 &&
      (timestamp - last_jump_event_time_) <
          std::max(0.0, cfg_.jump_event_refractory_s)) {
    jump_rising_edge = false;
  }
  if (jump_rising_edge) {
    last_jump_event_time_ = timestamp;
  }
  last_jump_detected_ = ev.jump_detected;
  ev.jump_event_detected = jump_rising_edge;

  const double jump_event_strength =
      jump_rising_edge ? clamp01(ev.jump_confidence) : 0.0;
  const double dual_strength = ev.has_dual_obs ? 1.0 : 0.0;
  const double margin_strength = normalized_margin(ev.candidate_margin);
  const double health_strength = ev.binder_health_score;
  const double certainty_strength =
      std::max(clamp01(1.0 - ev.entropy_norm), ev.max_prob);

  double nw_dual = 0.0;
  double nw_margin = 0.0;
  double nw_health = 0.0;
  double nw_entropy = 0.0;
  normalize_weights(cfg_.w_dual, cfg_.w_margin, cfg_.w_health, cfg_.w_entropy,
                    ev.has_dual_obs, &nw_dual, &nw_margin, &nw_health,
                    &nw_entropy);

  const double structured_strength =
      nw_dual * dual_strength + nw_margin * margin_strength +
      nw_health * health_strength + nw_entropy * certainty_strength;

  double event_strength = jump_event_strength;
  if (has_2dz_signature) {
    event_strength =
        std::max(event_strength, 0.70 + std::max(0.0, cfg_.signature_2dz_bonus));
  }
  ev.continuous_enter_score = clamp01(structured_strength);
  ev.event_enter_score = clamp01(event_strength);
  const double enter = std::max(ev.continuous_enter_score, ev.event_enter_score);
  ev.enter_score = clamp01(enter);

  // Exit evidence should come from degraded structured confidence only.
  // Do not invert jump as "anti-evidence", and do not hard-force to 1 here.
  // Forced rebind is handled in ModeFSM when current mode is STRUCTURED.
  const double exit = 1.0 - structured_strength;
  ev.exit_score = clamp01(exit);

  std::cout << "[EvidenceFuser] Fused evidence: timestamp=" << timestamp
            << ", obs_count=" << obs_count
            << ", has_2dz_signature=" << has_2dz_signature
            << ", candidate_id=" << candidate_id
            << ", entropy_norm=" << entropy_norm
            << ", max_prob=" << max_prob
            << ", candidate_margin=" << candidate_margin
            << ", jump_detected=" << ev.jump_detected
            << ", jump_rising_edge=" << jump_rising_edge
            << ", jump_confidence=" << ev.jump_confidence
            << ", binder_health_score=" << ev.binder_health_score
            << ", binder_bad_frames=" << ev.binder_bad_frames
            << ", binder_force_rebind=" << ev.binder_force_rebind
            << ", structured_strength=" << structured_strength
            << ", continuous_enter_score=" << ev.continuous_enter_score
            << ", event_enter_score=" << ev.event_enter_score
            << ", enter_score=" << ev.enter_score
            << ", exit_score=" << ev.exit_score
            << std::endl;

  return ev;
}

}  // namespace fyt::auto_aim::mode
