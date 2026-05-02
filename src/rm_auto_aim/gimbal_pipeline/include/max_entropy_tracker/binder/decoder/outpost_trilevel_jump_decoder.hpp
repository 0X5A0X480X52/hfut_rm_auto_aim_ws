// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_DECODER_OUTPOST_TRILEVEL_JUMP_DECODER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_DECODER_OUTPOST_TRILEVEL_JUMP_DECODER_HPP_

#include <array>
#include <limits>

#include "max_entropy_tracker/binder/decoder/jump_event_decoder.hpp"

namespace fyt::auto_aim::binder {

struct OutpostTriLevelJumpDecoderConfig {
  double min_candidate_prob = 0.40;
  double min_candidate_margin = 0.12;
  double switch_strong_score = 0.60;

  bool periodic_enable = true;
  int periodic_window = 12;
  double periodic_weight = 0.60;
  double periodic_min_spin_rate = 0.8;
  double periodic_update_min_jump = 0.015;
  double dz_ema_alpha = 0.20;
  double periodic_signature_threshold = 0.60;

  bool z_audit_enable = true;
  double z_audit_min_confidence = 0.60;
  double z_audit_min_jump = 0.015;
  int z_audit_confirm_frames = 3;

  double reacquire_gap_dt_gate = 0.12;
  int reacquire_lost_frames_gate = 1;
  double z_cluster_ema_alpha = 0.25;
  double z_cluster_assign_gate = 0.10;
  int z_cluster_window_slots = 8;

  std::array<double, 3> z_offsets{0.06, 0.0, -0.06};
  std::array<int, 3> cyclic_order{0, 2, 1};
};

class OutpostTriLevelJumpDecoder : public JumpEventDecoder {
 public:
  explicit OutpostTriLevelJumpDecoder(
      const OutpostTriLevelJumpDecoderConfig & config);

  JumpDecision decode(const BinderFrameInput & input,
                      DecoderContext & ctx) override;
  const char * name() const override { return "OutpostTriLevelJumpDecoder"; }

  void update_periodic_evidence(double z_jump, double yaw_rate_est,
                                bool allow_model_update, DecoderContext & ctx);
  void apply_periodic_prior(std::vector<double> & costs, double z_jump,
                            int bound_id, const DecoderContext & ctx) const;

 private:
  JumpDecision decode_from_z_audit(const BinderFrameInput & input,
                                    DecoderContext & ctx);
  JumpDecision decode_from_reacquire(const BinderFrameInput & input,
                                     DecoderContext & ctx);
  JumpDecision decode_from_cost(const BinderFrameInput & input,
                                const DecoderContext & ctx);
  void update_z_cluster(int slot_id, double obs_z);
  int allocate_cluster_slot();
  void ensure_active_cluster(double obs_z);
  void rotate_cluster_after_jump(int to_alias_id, double obs_z);
  bool infer_dz_bands_from_raw_clusters(double & dz_small,
                                        double & dz_large) const;
  JumpKind classify_jump_kind_by_bands(double abs_jump,
                                       const DecoderContext & ctx) const;

  OutpostTriLevelJumpDecoderConfig config_;

  struct ZCluster {
    bool initialized = false;
    double mean = 0.0;
    double var = 0.02;
    double raw_mean = std::numeric_limits<double>::quiet_NaN();
    double raw_var = 0.02;
    int count = 0;
  };
  static constexpr int kMaxClusterSlots = 16;
  std::array<ZCluster, kMaxClusterSlots> z_clusters_{};
  int active_cluster_slot_ = -1;
  int next_cluster_slot_ = 0;
  std::array<int, 3> alias_to_slot_{{-1, -1, -1}};

  bool z_audit_init_ = false;
  double z_audit_center_ = std::numeric_limits<double>::quiet_NaN();
  double z_audit_prev_z_ = std::numeric_limits<double>::quiet_NaN();
  int z_audit_prev_panel_ = -1;
  int z_audit_conflict_count_ = 0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_DECODER_OUTPOST_TRILEVEL_JUMP_DECODER_HPP_
