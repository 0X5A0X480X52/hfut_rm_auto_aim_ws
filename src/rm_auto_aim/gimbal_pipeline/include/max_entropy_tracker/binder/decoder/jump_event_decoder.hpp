// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_DECODER_JUMP_EVENT_DECODER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_DECODER_JUMP_EVENT_DECODER_HPP_

#include <deque>
#include <limits>
#include <optional>

#include "max_entropy_tracker/binder/model/binder_types.hpp"

namespace fyt::auto_aim::binder {

struct DecoderContext {
  std::optional<double> last_obs_z;
  int last_panel_id = -1;

  double dz_jump_est = std::numeric_limits<double>::quiet_NaN();

  std::deque<double> dz_history;
  double dz_small_est = std::numeric_limits<double>::quiet_NaN();
  double dz_large_est = std::numeric_limits<double>::quiet_NaN();
  int period_phase = -1;
  double period_confidence = 0.0;
  int spin_direction = 0;
  int pending_spin_direction = 0;
  int pending_spin_direction_count = 0;
};

class JumpEventDecoder {
 public:
  virtual ~JumpEventDecoder() = default;
  virtual JumpDecision decode(const BinderFrameInput & input,
                              DecoderContext & ctx) = 0;
  virtual const char * name() const = 0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_DECODER_JUMP_EVENT_DECODER_HPP_
