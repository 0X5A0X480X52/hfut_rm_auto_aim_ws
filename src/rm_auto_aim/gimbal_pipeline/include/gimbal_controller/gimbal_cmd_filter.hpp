// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// GimbalCmdFilter — 输出端保护滤波器
//
// 在 GimbalCmd 发布前对 yaw_diff / pitch_diff 执行可配置的多级保护：
//
//   0. 绝对限幅 (Clamping)        — 硬上限，防止极端值
//   1. 外点检测 (Outlier Reject)  — 帧间突变超阈值时 hold 上帧
//   2. 变化率限制 (Rate Limiter)  — 限制每帧变化量
//   3. EMA 平滑                   — 指数移动平均
//   4. 1-Euro 自适应滤波           — 速度自适应低通
//
// 各级独立可开关（通过 GimbalCmdFilterConfig 中 enable_* 字段控制）。
// 目标切换或从 idle 恢复时须调用 reset() 以清空历史状态。

#ifndef GIMBAL_CONTROLLER__GIMBAL_CMD_FILTER_HPP_
#define GIMBAL_CONTROLLER__GIMBAL_CMD_FILTER_HPP_

#include <algorithm>
#include <cmath>
#include <string>

#include <rm_interfaces/msg/gimbal_cmd.hpp>

#include "max_entropy_tracker/utils/one_euro_filter.hpp"

namespace gimbal_controller {

// ──────────────────────────────────────────────────────────────────
//  Config — 所有可调参数
// ──────────────────────────────────────────────────────────────────
struct GimbalCmdFilterConfig {
  // ── 0. Clamping 绝对限幅 ──
  bool   enable_clamping{true};
  double max_yaw_diff{15.0};    // 度
  double max_pitch_diff{10.0};  // 度

  // ── 1. 外点检测 ──
  bool   enable_outlier_rejection{true};
  double outlier_threshold_yaw{8.0};    // 帧间跳变判定阈值 (度)
  double outlier_threshold_pitch{5.0};  // 帧间跳变判定阈值 (度)
  int    max_outlier_count{3};          // 连续外点超此数后强制 accept 并重置

  // ── 2. Rate Limiter ──
  bool   enable_rate_limiter{true};
  double max_yaw_rate{5.0};    // 每帧最大变化量 (度/帧)
  double max_pitch_rate{3.0};  // 每帧最大变化量 (度/帧)

  // ── 3. EMA 平滑 ──
  bool   enable_ema{false};
  double ema_alpha{0.7};  // (0, 1]，越大越跟随

  // ── 4. 1-Euro 自适应滤波 ──
  bool   enable_one_euro{false};
  double one_euro_freq{250.0};       // 采样频率 Hz（应与 control_rate 一致）
  double one_euro_min_cutoff{1.0};   // 最小截止频率
  double one_euro_beta{0.007};       // 速度灵敏度系数
  double one_euro_d_cutoff{1.0};     // 导数截止频率
};

// ──────────────────────────────────────────────────────────────────
//  GimbalCmdFilter
// ──────────────────────────────────────────────────────────────────
class GimbalCmdFilter {
 public:
  explicit GimbalCmdFilter(const GimbalCmdFilterConfig &cfg = {}) {
    setConfig(cfg);
  }

  /// 更新配置（允许运行时修改参数）
  void setConfig(const GimbalCmdFilterConfig &cfg) {
    cfg_ = cfg;
    // 同步 1-Euro 滤波器参数
    yaw_euro_.set_freq(cfg_.one_euro_freq);
    yaw_euro_.set_min_cutoff(cfg_.one_euro_min_cutoff);
    yaw_euro_.set_beta(cfg_.one_euro_beta);
    yaw_euro_.set_d_cutoff(cfg_.one_euro_d_cutoff);

    pitch_euro_.set_freq(cfg_.one_euro_freq);
    pitch_euro_.set_min_cutoff(cfg_.one_euro_min_cutoff);
    pitch_euro_.set_beta(cfg_.one_euro_beta);
    pitch_euro_.set_d_cutoff(cfg_.one_euro_d_cutoff);
  }

  /// 重置所有内部状态（目标切换 / idle→tracking 时调用）
  void reset() {
    has_prev_       = false;
    outlier_yaw_count_   = 0;
    outlier_pitch_count_ = 0;
    yaw_euro_.reset();
    pitch_euro_.reset();
    prev_cmd_ = rm_interfaces::msg::GimbalCmd{};
  }

  /// 就地修改 cmd，依次执行启用的各级保护
  void filter(rm_interfaces::msg::GimbalCmd &cmd) {
    // ── 0. Clamping ──
    if (cfg_.enable_clamping) {
      cmd.yaw_diff   = std::clamp(cmd.yaw_diff,
                                  -cfg_.max_yaw_diff,   cfg_.max_yaw_diff);
      cmd.pitch_diff = std::clamp(cmd.pitch_diff,
                                  -cfg_.max_pitch_diff, cfg_.max_pitch_diff);
    }

    if (!has_prev_) {
      // 首帧：跳过需要历史的保护，直接记录
      savePrev(cmd);
      has_prev_ = true;
      // 仍对首帧执行 1-Euro 初始化（会自动设初值）
      if (cfg_.enable_one_euro) {
        cmd.yaw_diff   = yaw_euro_.filter(cmd.yaw_diff);
        cmd.pitch_diff = pitch_euro_.filter(cmd.pitch_diff);
        savePrev(cmd);
      }
      return;
    }

    // ── 1. 外点检测 ──
    if (cfg_.enable_outlier_rejection) {
      bool yaw_outlier   = std::abs(cmd.yaw_diff   - prev_cmd_.yaw_diff)
                           > cfg_.outlier_threshold_yaw;
      bool pitch_outlier = std::abs(cmd.pitch_diff - prev_cmd_.pitch_diff)
                           > cfg_.outlier_threshold_pitch;

      if (yaw_outlier || pitch_outlier) {
        outlier_yaw_count_   += yaw_outlier   ? 1 : 0;
        outlier_pitch_count_ += pitch_outlier ? 1 : 0;

        // 连续外点未超上限 → hold 上帧值
        if (outlier_yaw_count_   <= cfg_.max_outlier_count &&
            outlier_pitch_count_ <= cfg_.max_outlier_count) {
          cmd.yaw_diff   = prev_cmd_.yaw_diff;
          cmd.pitch_diff = prev_cmd_.pitch_diff;
          // cmd.yaw / pitch 也修正为一致（保持时序连贯）
          cmd.yaw   = prev_cmd_.yaw;
          cmd.pitch = prev_cmd_.pitch;
          savePrev(cmd);
          return;
        } else {
          // 连续外点超上限 → accept 当前值，重置计数（允许跳变）
          outlier_yaw_count_   = 0;
          outlier_pitch_count_ = 0;
        }
      } else {
        outlier_yaw_count_   = 0;
        outlier_pitch_count_ = 0;
      }
    }

    // ── 2. Rate Limiter ──
    if (cfg_.enable_rate_limiter) {
      double delta_yaw   = cmd.yaw_diff   - prev_cmd_.yaw_diff;
      double delta_pitch = cmd.pitch_diff - prev_cmd_.pitch_diff;

      delta_yaw   = std::clamp(delta_yaw,   -cfg_.max_yaw_rate,   cfg_.max_yaw_rate);
      delta_pitch = std::clamp(delta_pitch, -cfg_.max_pitch_rate, cfg_.max_pitch_rate);

      cmd.yaw_diff   = prev_cmd_.yaw_diff   + delta_yaw;
      cmd.pitch_diff = prev_cmd_.pitch_diff + delta_pitch;
    }

    // ── 3. EMA ──
    if (cfg_.enable_ema) {
      double a = cfg_.ema_alpha;
      cmd.yaw_diff   = a * cmd.yaw_diff   + (1.0 - a) * prev_cmd_.yaw_diff;
      cmd.pitch_diff = a * cmd.pitch_diff + (1.0 - a) * prev_cmd_.pitch_diff;
    }

    // ── 4. 1-Euro ──
    if (cfg_.enable_one_euro) {
      cmd.yaw_diff   = yaw_euro_.filter(cmd.yaw_diff);
      cmd.pitch_diff = pitch_euro_.filter(cmd.pitch_diff);
    }

    savePrev(cmd);
  }

 private:
  void savePrev(const rm_interfaces::msg::GimbalCmd &cmd) {
    prev_cmd_ = cmd;
  }

  GimbalCmdFilterConfig cfg_;

  rm_interfaces::msg::GimbalCmd prev_cmd_{};
  bool has_prev_{false};

  // 外点连续计数器
  int outlier_yaw_count_{0};
  int outlier_pitch_count_{0};

  // 1-Euro filters
  ::fyt::auto_aim::OneEuroFilter yaw_euro_{250.0, 1.0, 0.007, 1.0};
  ::fyt::auto_aim::OneEuroFilter pitch_euro_{250.0, 1.0, 0.007, 1.0};
};

}  // namespace gimbal_controller

#endif  // GIMBAL_CONTROLLER__GIMBAL_CMD_FILTER_HPP_
