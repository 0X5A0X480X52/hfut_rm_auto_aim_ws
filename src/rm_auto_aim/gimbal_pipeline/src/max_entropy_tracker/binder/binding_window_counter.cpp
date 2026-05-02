// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/binder/core/binding_window_counter.hpp"

#include <algorithm>

namespace fyt::auto_aim::binder {

BindingWindowCounter::BindingWindowCounter(int threshold)
    : threshold_(std::max(1, threshold)) {}

void BindingWindowCounter::reset() { count_ = 0; }

bool BindingWindowCounter::tick(bool condition) {
  if (condition) {
    ++count_;
  } else {
    count_ = 0;
  }
  return count_ >= threshold_;
}

}  // namespace fyt::auto_aim::binder
