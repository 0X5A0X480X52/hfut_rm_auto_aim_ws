// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_CORE_CENTER_Z_HISTORY_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_CORE_CENTER_Z_HISTORY_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <vector>

namespace fyt::auto_aim::binder {

class CenterZHistory {
 public:
  explicit CenterZHistory(int window = 1) : window_(std::max(1, window)) {}

  void set_window(int window) { window_ = std::max(1, window); trim(); }
  void reset() { values_.clear(); }

  void push(double z) {
    if (!std::isfinite(z)) return;
    values_.push_back(z);
    trim();
  }

  double median() const {
    if (values_.empty()) return std::numeric_limits<double>::quiet_NaN();
    std::vector<double> values(values_.begin(), values_.end());
    const auto mid = values.begin() + static_cast<long>(values.size() / 2);
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
  }

  bool empty() const { return values_.empty(); }

 private:
  void trim() {
    while (static_cast<int>(values_.size()) > window_) {
      values_.pop_front();
    }
  }

  int window_ = 1;
  std::deque<double> values_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_CORE_CENTER_Z_HISTORY_HPP_
