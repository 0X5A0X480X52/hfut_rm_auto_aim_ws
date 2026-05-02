// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDER_ENUMS_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDER_ENUMS_HPP_

namespace fyt::auto_aim::binder {

enum class HeightLabel { UNKNOWN = -1, LOWER = 0, MIDDLE = 1, UPPER = 2 };

enum class BindingFSMState {
  LOCKED = 0,
  PENDING_SWITCH = 1,
  UNLOCKED = 2,
  LOCKED_NEW = 3
};

enum class BindingAction {
  HOLD = 0,
  PENDING = 1,
  SWITCH = 2,
  FORCE_REBIND = 3,
  RELOCK = 4
};

enum class JumpKind {
  NONE = 0,
  DZ = 1,
  DOUBLE_DZ = 2,
  AMBIGUOUS = 3,
  INVALID = 4
};

enum class TrackEventType {
  CONTINUITY = 0,
  REACQUIRE = 1,
  SWITCH_CANDIDATE = 2,
  AMBIGUOUS = 3
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_MODEL_BINDER_ENUMS_HPP_
