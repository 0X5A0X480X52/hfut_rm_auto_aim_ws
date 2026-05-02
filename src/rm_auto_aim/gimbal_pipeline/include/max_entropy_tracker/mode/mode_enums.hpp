// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_MODE_MODE_ENUMS_HPP_
#define MAX_ENTROPY_TRACKER_MODE_MODE_ENUMS_HPP_

namespace fyt::auto_aim::mode {

enum class TrackMode {
  AMBIGUOUS = 0,
  STRUCTURED = 1,
};

enum class TransitionReason {
  NONE = 0,
  STRONG_EVIDENCE_ENTER = 1,
  WEAK_EVIDENCE_EXIT = 2,
  FORCED_REBIND_EXIT = 3,
};

}  // namespace fyt::auto_aim::mode

#endif  // MAX_ENTROPY_TRACKER_MODE_MODE_ENUMS_HPP_
