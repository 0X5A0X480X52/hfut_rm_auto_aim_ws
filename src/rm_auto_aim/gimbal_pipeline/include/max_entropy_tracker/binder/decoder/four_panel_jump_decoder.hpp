// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_DECODER_FOUR_PANEL_JUMP_DECODER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_DECODER_FOUR_PANEL_JUMP_DECODER_HPP_

#include "max_entropy_tracker/binder/decoder/jump_event_decoder.hpp"

namespace fyt::auto_aim::binder {

struct FourPanelJumpDecoderConfig {
  double z_jump_min = 0.015;
  double dz_match_tolerance = 0.03;
  double yaw_err_gate = 0.35;
  double cost_margin_min = 0.08;
  double dz_ema_alpha = 0.20;
};

class FourPanelJumpDecoder : public JumpEventDecoder {
 public:
  explicit FourPanelJumpDecoder(const FourPanelJumpDecoderConfig & config);
  JumpDecision decode(const BinderFrameInput & input,
                      DecoderContext & ctx) override;
  const char * name() const override { return "FourPanelJumpDecoder"; }

 private:
  bool evaluate_gates(int candidate_panel, double z_jump, bool has_z_jump,
                      double selected_yaw_err, double cost_margin,
                      int n_panels, const DecoderContext & ctx) const;

  FourPanelJumpDecoderConfig config_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_DECODER_FOUR_PANEL_JUMP_DECODER_HPP_
