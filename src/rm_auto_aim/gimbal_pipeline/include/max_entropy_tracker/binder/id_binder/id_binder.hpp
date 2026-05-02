// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_ID_BINDER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_ID_BINDER_HPP_

#include <deque>

#include "max_entropy_tracker/binder/model/binder_types.hpp"

namespace fyt::auto_aim::binder {

struct BinderContext {
  int current_bound_id = -1;
  HeightLabel current_bound_label = HeightLabel::UNKNOWN;

  std::deque<double> z_history;
  std::deque<double> z_jump_history;
  std::deque<int> panel_id_history;
};

class IDBinder {
 public:
  virtual ~IDBinder() = default;
  virtual TargetDecision propose(const BinderFrameInput & input,
                                 const JumpDecision & jump,
                                 const BinderContext & ctx) = 0;
  virtual const char * name() const = 0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_ID_BINDER_ID_BINDER_HPP_
