// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_ASSOCIATION_OSCILLATION_DETECTOR_HPP_
#define MAX_ENTROPY_TRACKER_ASSOCIATION_OSCILLATION_DETECTOR_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>
#include <utility>

namespace fyt::auto_aim {

/// Monitors r1/r2 for sustained oscillation and recommends reset.
class OscillationDetector {
 public:
  OscillationDetector(int window_size = 50, double osc_threshold = 0.05,
                      int count_threshold = 5, int min_reset_interval = 100,
                      bool enabled = false)
      : window_size_(window_size),
        osc_threshold_(osc_threshold),
        count_threshold_(count_threshold),
        min_reset_interval_(min_reset_interval),
        enabled_(enabled) {}

  /// Returns true when a parameter reset is recommended.
  bool update(double r1, double r2) {
    ++current_frame_;
    if (!enabled_) return false;

    r1_hist_.push_back(r1);
    r2_hist_.push_back(r2);
    if (static_cast<int>(r1_hist_.size()) > window_size_) {
      r1_hist_.pop_front();
      r2_hist_.pop_front();
    }
    if (static_cast<int>(r1_hist_.size()) < window_size_) return false;

    if (detect_oscillation())
      ++osc_count_;
    else
      osc_count_ = 0;

    bool should_reset =
        (osc_count_ >= count_threshold_) &&
        (current_frame_ - last_reset_frame_ > min_reset_interval_);
    if (should_reset) {
      last_reset_frame_ = current_frame_;
      osc_count_ = 0;
    }
    return should_reset;
  }

  /// Median values for reset.
  std::pair<double, double> get_reset_values() const {
    return {median(r1_hist_), median(r2_hist_)};
  }

  void reset() {
    r1_hist_.clear();
    r2_hist_.clear();
    osc_count_ = 0;
    current_frame_ = 0;
  }

 private:
  bool detect_oscillation() const {
    double r1_std = calc_std(r1_hist_);
    double r2_std = calc_std(r2_hist_);
    double r1_change = 0, r2_change = 0;
    if (r1_hist_.size() >= 10) {
      int n = static_cast<int>(r1_hist_.size());
      r1_change = std::abs(r1_hist_[n - 1] - r1_hist_[n - 10]);
      r2_change = std::abs(r2_hist_[n - 1] - r2_hist_[n - 10]);
    }
    return (r1_std > osc_threshold_ && r1_change > 0.02) ||
           (r2_std > osc_threshold_ && r2_change > 0.02);
  }

  static double calc_std(const std::deque<double> &v) {
    if (v.empty()) return 0;
    double mean = std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    double sq = 0;
    for (double x : v) sq += (x - mean) * (x - mean);
    return std::sqrt(sq / v.size());
  }

  static double median(const std::deque<double> &v) {
    if (v.empty()) return 0.15;
    std::vector<double> tmp(v.begin(), v.end());
    std::sort(tmp.begin(), tmp.end());
    return tmp[tmp.size() / 2];
  }

  int window_size_;
  double osc_threshold_;
  int count_threshold_, min_reset_interval_;
  bool enabled_;

  std::deque<double> r1_hist_, r2_hist_;
  int osc_count_ = 0;
  int last_reset_frame_ = -100;
  int current_frame_ = 0;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_ASSOCIATION_OSCILLATION_DETECTOR_HPP_
