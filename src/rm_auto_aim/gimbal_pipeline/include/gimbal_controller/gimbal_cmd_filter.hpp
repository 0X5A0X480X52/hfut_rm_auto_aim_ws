// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0

#ifndef GIMBAL_CONTROLLER__GIMBAL_CMD_FILTER_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CMD_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>

#include <rm_interfaces/msg/gimbal_cmd.hpp>

#include "max_entropy_tracker/utils/one_euro_filter.hpp"

namespace gimbal_controller {

struct GimbalCmdFilterConfig {
  bool enable_clamping{true};
  double max_yaw_diff{15.0};
  double max_pitch_diff{10.0};

  bool enable_outlier_rejection{true};
  double outlier_threshold_yaw{8.0};
  double outlier_threshold_pitch{5.0};
  int max_outlier_count{3};

  bool enable_rate_limiter{true};
  double max_yaw_rate{5.0};
  double max_pitch_rate{3.0};

  bool enable_moving_average{false};
  int moving_average_window_size{3};

  bool enable_ema{false};
  double ema_alpha{0.7};

  bool enable_one_euro{false};
  double one_euro_freq{250.0};
  double one_euro_min_cutoff{1.0};
  double one_euro_beta{0.007};
  double one_euro_d_cutoff{1.0};
};

class GimbalCmdFilter {
 public:
  explicit GimbalCmdFilter(const GimbalCmdFilterConfig &cfg = {}) { setConfig(cfg); }

  void setConfig(const GimbalCmdFilterConfig &cfg) {
    cfg_ = cfg;

    yaw_euro_.set_freq(cfg_.one_euro_freq);
    yaw_euro_.set_min_cutoff(cfg_.one_euro_min_cutoff);
    yaw_euro_.set_beta(cfg_.one_euro_beta);
    yaw_euro_.set_d_cutoff(cfg_.one_euro_d_cutoff);

    pitch_euro_.set_freq(cfg_.one_euro_freq);
    pitch_euro_.set_min_cutoff(cfg_.one_euro_min_cutoff);
    pitch_euro_.set_beta(cfg_.one_euro_beta);
    pitch_euro_.set_d_cutoff(cfg_.one_euro_d_cutoff);

    resetMovingAverageState();
  }

  void reset() {
    has_prev_ = false;
    outlier_yaw_count_ = 0;
    outlier_pitch_count_ = 0;
    yaw_euro_.reset();
    pitch_euro_.reset();
    resetMovingAverageState();
    prev_cmd_ = rm_interfaces::msg::GimbalCmd{};
  }

  void filter(rm_interfaces::msg::GimbalCmd &cmd) {
    const double raw_yaw_diff = cmd.yaw_diff;
    const double raw_pitch_diff = cmd.pitch_diff;

    if (cfg_.enable_clamping) {
      cmd.yaw_diff = std::clamp(cmd.yaw_diff, -cfg_.max_yaw_diff, cfg_.max_yaw_diff);
      cmd.pitch_diff = std::clamp(cmd.pitch_diff, -cfg_.max_pitch_diff, cfg_.max_pitch_diff);
    }

    if (!has_prev_) {
      if (cfg_.enable_one_euro) {
        cmd.yaw_diff = yaw_euro_.filter(cmd.yaw_diff);
        cmd.pitch_diff = pitch_euro_.filter(cmd.pitch_diff);
      }
      syncAbsoluteAngles(cmd, raw_yaw_diff, raw_pitch_diff);
      savePrev(cmd);
      has_prev_ = true;
      return;
    }

    if (cfg_.enable_outlier_rejection) {
      const bool yaw_outlier =
        std::abs(cmd.yaw_diff - prev_cmd_.yaw_diff) > cfg_.outlier_threshold_yaw;
      const bool pitch_outlier =
        std::abs(cmd.pitch_diff - prev_cmd_.pitch_diff) > cfg_.outlier_threshold_pitch;

      if (yaw_outlier || pitch_outlier) {
        outlier_yaw_count_ += yaw_outlier ? 1 : 0;
        outlier_pitch_count_ += pitch_outlier ? 1 : 0;

        if (outlier_yaw_count_ <= cfg_.max_outlier_count &&
            outlier_pitch_count_ <= cfg_.max_outlier_count) {
          cmd.yaw_diff = prev_cmd_.yaw_diff;
          cmd.pitch_diff = prev_cmd_.pitch_diff;
          cmd.yaw = prev_cmd_.yaw;
          cmd.pitch = prev_cmd_.pitch;
          savePrev(cmd);
          return;
        }

        outlier_yaw_count_ = 0;
        outlier_pitch_count_ = 0;
      } else {
        outlier_yaw_count_ = 0;
        outlier_pitch_count_ = 0;
      }
    }

    if (cfg_.enable_rate_limiter) {
      double delta_yaw = cmd.yaw_diff - prev_cmd_.yaw_diff;
      double delta_pitch = cmd.pitch_diff - prev_cmd_.pitch_diff;

      delta_yaw = std::clamp(delta_yaw, -cfg_.max_yaw_rate, cfg_.max_yaw_rate);
      delta_pitch = std::clamp(delta_pitch, -cfg_.max_pitch_rate, cfg_.max_pitch_rate);

      cmd.yaw_diff = prev_cmd_.yaw_diff + delta_yaw;
      cmd.pitch_diff = prev_cmd_.pitch_diff + delta_pitch;
    }

    if (cfg_.enable_moving_average) {
      applyMovingAverage(cmd.yaw_diff, cmd.pitch_diff);
    }

    if (cfg_.enable_ema) {
      const double a = cfg_.ema_alpha;
      cmd.yaw_diff = a * cmd.yaw_diff + (1.0 - a) * prev_cmd_.yaw_diff;
      cmd.pitch_diff = a * cmd.pitch_diff + (1.0 - a) * prev_cmd_.pitch_diff;
    }

    if (cfg_.enable_one_euro) {
      cmd.yaw_diff = yaw_euro_.filter(cmd.yaw_diff);
      cmd.pitch_diff = pitch_euro_.filter(cmd.pitch_diff);
    }

    syncAbsoluteAngles(cmd, raw_yaw_diff, raw_pitch_diff);
    savePrev(cmd);
  }

 private:
  static int normalizeWindowSize(int w) { return std::max(1, w); }

  static double normalizeAngleDeg(double angle_deg) {
    while (angle_deg > 180.0) {
      angle_deg -= 360.0;
    }
    while (angle_deg < -180.0) {
      angle_deg += 360.0;
    }
    return angle_deg;
  }

  static void updateAxisMovingAverage(
    double &value, std::deque<double> &window, double &sum, std::size_t max_window_size) {
    window.push_back(value);
    sum += value;

    while (window.size() > max_window_size) {
      sum -= window.front();
      window.pop_front();
    }

    value = sum / static_cast<double>(window.size());
  }

  void applyMovingAverage(double &yaw_diff, double &pitch_diff) {
    const std::size_t win =
      static_cast<std::size_t>(normalizeWindowSize(cfg_.moving_average_window_size));
    updateAxisMovingAverage(yaw_diff, yaw_window_, yaw_window_sum_, win);
    updateAxisMovingAverage(pitch_diff, pitch_window_, pitch_window_sum_, win);
  }

  void resetMovingAverageState() {
    yaw_window_.clear();
    pitch_window_.clear();
    yaw_window_sum_ = 0.0;
    pitch_window_sum_ = 0.0;
  }

  void syncAbsoluteAngles(
    rm_interfaces::msg::GimbalCmd &cmd,
    double raw_yaw_diff,
    double raw_pitch_diff) const {
    cmd.yaw += cmd.yaw_diff - raw_yaw_diff;
    cmd.pitch += cmd.pitch_diff - raw_pitch_diff;
    cmd.yaw = normalizeAngleDeg(cmd.yaw);
  }

  void savePrev(const rm_interfaces::msg::GimbalCmd &cmd) { prev_cmd_ = cmd; }

  GimbalCmdFilterConfig cfg_{};

  rm_interfaces::msg::GimbalCmd prev_cmd_{};
  bool has_prev_{false};

  int outlier_yaw_count_{0};
  int outlier_pitch_count_{0};

  std::deque<double> yaw_window_{};
  std::deque<double> pitch_window_{};
  double yaw_window_sum_{0.0};
  double pitch_window_sum_{0.0};

  ::fyt::auto_aim::OneEuroFilter yaw_euro_{250.0, 1.0, 0.007, 1.0};
  ::fyt::auto_aim::OneEuroFilter pitch_euro_{250.0, 1.0, 0.007, 1.0};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__GIMBAL_CMD_FILTER_HPP_
