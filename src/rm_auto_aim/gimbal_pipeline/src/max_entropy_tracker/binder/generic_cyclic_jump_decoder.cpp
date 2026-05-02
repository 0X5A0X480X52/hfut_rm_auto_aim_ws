// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/decoder/generic_cyclic_jump_decoder.hpp"

#include <cmath>

namespace fyt::auto_aim::binder {

JumpDecision GenericCyclicJumpDecoder::decode(const BinderFrameInput & input,
                                              DecoderContext & ctx) {
  JumpDecision jd;
  if (!input.profile || input.candidate_id < 0) return jd;

  const int n = input.profile->panel_count;
  if (n < 2) return jd;

  if (input.candidate_id == ctx.last_panel_id) return jd;

  // Minimal adjacency check for any N-panel cyclic layout
  const int raw_diff = std::abs(input.candidate_id - ctx.last_panel_id);
  const bool adjacent = (raw_diff == 1 || raw_diff == n - 1);

  if (!adjacent) return jd;

  jd.detected = true;
  jd.jump_kind = JumpKind::DZ;
  jd.from_id = ctx.last_panel_id;
  jd.to_id = input.candidate_id;
  jd.confidence = 0.5;
  jd.reason_code = 1;
  return jd;
}

}  // namespace fyt::auto_aim::binder
