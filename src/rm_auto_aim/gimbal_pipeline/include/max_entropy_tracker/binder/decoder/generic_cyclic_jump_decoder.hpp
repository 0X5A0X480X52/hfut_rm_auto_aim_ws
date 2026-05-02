// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_DECODER_GENERIC_CYCLIC_JUMP_DECODER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_DECODER_GENERIC_CYCLIC_JUMP_DECODER_HPP_

#include "max_entropy_tracker/binder/decoder/jump_event_decoder.hpp"

namespace fyt::auto_aim::binder {

class GenericCyclicJumpDecoder : public JumpEventDecoder {
 public:
  JumpDecision decode(const BinderFrameInput & input,
                      DecoderContext & ctx) override;
  const char * name() const override { return "GenericCyclicJumpDecoder"; }
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_DECODER_GENERIC_CYCLIC_JUMP_DECODER_HPP_
