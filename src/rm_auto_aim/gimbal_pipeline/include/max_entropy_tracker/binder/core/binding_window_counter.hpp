// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_CORE_BINDING_WINDOW_COUNTER_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_CORE_BINDING_WINDOW_COUNTER_HPP_

namespace fyt::auto_aim::binder {

class BindingWindowCounter {
 public:
  explicit BindingWindowCounter(int threshold);

  void reset();

  bool tick(bool condition);

  int count() const { return count_; }
  int threshold() const { return threshold_; }

 private:
  int threshold_;
  int count_ = 0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_CORE_BINDING_WINDOW_COUNTER_HPP_
