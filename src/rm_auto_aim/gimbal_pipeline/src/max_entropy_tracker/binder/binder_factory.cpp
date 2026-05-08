// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/factory/binder_factory.hpp"

#include <algorithm>

#include "max_entropy_tracker/binder/decoder/four_panel_jump_decoder.hpp"
#include "max_entropy_tracker/binder/decoder/generic_cyclic_jump_decoder.hpp"
#include "max_entropy_tracker/binder/decoder/outpost_trilevel_jump_decoder.hpp"
#include "max_entropy_tracker/binder/id_binder/dual_obs_direct_binder.hpp"
#include "max_entropy_tracker/binder/id_binder/hybrid_id_binder.hpp"
#include "max_entropy_tracker/binder/id_binder/single_obs_sequence_binder.hpp"
#include "max_entropy_tracker/binder/scorer/null_hypothesis_scorer.hpp"
#include "max_entropy_tracker/binder/scorer/residual_hypothesis_scorer.hpp"
#include "max_entropy_tracker/binder/scorer/soft_fusion_scorer.hpp"

namespace fyt::auto_aim::binder {

std::unique_ptr<BinderPipeline> BinderFactory::create(
    const RobotBindingProfile & profile,
    const BinderConfig & config) {
  // ── Assemble decoder ──
  std::unique_ptr<JumpEventDecoder> decoder;
  if (profile.robot_type == 3) {  // OUTPOST_3
    OutpostTriLevelJumpDecoderConfig dcfg;
    dcfg.min_candidate_prob = config.min_candidate_prob;
    dcfg.min_candidate_margin = config.min_candidate_margin;
    dcfg.switch_strong_score = config.switch_strong_score;
    dcfg.periodic_enable = config.periodic_enable;
    dcfg.periodic_window = config.periodic_window;
    dcfg.periodic_weight = config.periodic_weight;
    dcfg.periodic_min_spin_rate = config.periodic_min_spin_rate;
    dcfg.periodic_update_min_jump = config.periodic_update_min_jump;
    dcfg.periodic_signature_threshold = config.periodic_signature_threshold;
    dcfg.dz_ema_alpha = config.dz_ema_alpha;
    dcfg.reacquire_gap_dt_gate = config.reacquire_gap_dt_gate;
    dcfg.reacquire_lost_frames_gate = config.reacquire_lost_frames_gate;
    dcfg.z_cluster_ema_alpha = config.z_cluster_ema_alpha;
    dcfg.z_cluster_assign_gate = config.z_cluster_assign_gate;
    dcfg.z_audit_enable = config.z_audit_rebind_enable;
    dcfg.z_audit_min_confidence = config.z_audit_rebind_min_confidence;
    dcfg.z_audit_min_jump = config.z_audit_rebind_min_jump;
    dcfg.z_audit_confirm_frames = config.z_audit_rebind_confirm_frames;
    // Copy z_offsets from profile if available
    if (profile.z_offsets.size() >= 3) {
      dcfg.z_offsets[0] = profile.z_offsets[0];
      dcfg.z_offsets[1] = profile.z_offsets[1];
      dcfg.z_offsets[2] = profile.z_offsets[2];
    }
    decoder = std::make_unique<OutpostTriLevelJumpDecoder>(dcfg);
  } else if (profile.panel_count >= 4) {
    FourPanelJumpDecoderConfig dcfg;
    dcfg.z_jump_min = config.z_jump_min;
    dcfg.dz_match_tolerance = config.dz_match_tolerance;
    dcfg.yaw_err_gate = config.yaw_err_gate;
    dcfg.cost_margin_min = config.cost_margin_min;
    dcfg.dz_ema_alpha = config.dz_ema_alpha;
    decoder = std::make_unique<FourPanelJumpDecoder>(dcfg);
  } else {
    decoder = std::make_unique<GenericCyclicJumpDecoder>();
  }

  // ── Assemble ID binder ──
  std::unique_ptr<IDBinder> id_binder;
  {
    SingleObsSequenceBinderConfig scfg;
    scfg.history_window = config.single_obs_history_window;
    scfg.dz_gate = config.dz_gate;
    scfg.pending_confirm_window =
        std::max(1, config.pending_window_frames > 0
                        ? config.pending_window_frames
                        : config.confirm_frames + 1);
    auto single = std::make_unique<SingleObsSequenceBinder>(scfg);
    auto dual = std::make_unique<DualObsDirectBinder>();
    id_binder = std::make_unique<HybridIDBinder>(
        std::move(single), std::move(dual), true);
  }

  // ── Assemble scorer ──
  std::unique_ptr<HypothesisScorer> scorer;
  if (config.scorer_enable) {
    ResidualHypothesisScorerConfig scfg;
    scfg.same_panel_yaw_gate = config.same_panel_yaw_gate;
    scfg.same_panel_z_gate = config.same_panel_z_gate;
    scfg.same_panel_xy_gate = config.same_panel_xy_gate;
    scfg.consecutive_bad_threshold = config.force_rebind_bad_frames;
    scorer = std::make_unique<ResidualHypothesisScorer>(scfg);
  } else {
    scorer = std::make_unique<NullHypothesisScorer>();
  }

  // Phase 6: wrap with soft fusion scorer when enabled.
  if (config.enable_soft_fusion) {
    SoftFusionConfig sfc;
    sfc.w_seq = config.soft_fusion_w_seq;
    sfc.w_geo = config.soft_fusion_w_geo;
    sfc.w_dyn = config.soft_fusion_w_dyn;
    sfc.w_continuity = config.soft_fusion_w_continuity;
    sfc.w_topology = config.soft_fusion_w_topology;
    scorer = std::make_unique<SoftFusionScorer>(std::move(scorer), sfc);
  }

  // ── Assemble pipeline ──
  BinderPipelineConfig pcfg;
  pcfg.fsm.confirm_frames = config.confirm_frames;
  pcfg.fsm.lock_new_hold_frames = config.lock_new_hold_frames;
  pcfg.fsm.force_rebind_bad_frames = config.force_rebind_bad_frames;
  pcfg.fsm.pending_window_frames = config.pending_window_frames;
  pcfg.fsm.post_jump_min_confidence = config.post_jump_min_confidence;

  return std::make_unique<BinderPipeline>(
      std::move(decoder), std::move(id_binder), std::move(scorer), pcfg);
}

}  // namespace fyt::auto_aim::binder
