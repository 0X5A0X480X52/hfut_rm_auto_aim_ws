// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0
//
// PredictionLogger — 实时记录观测值与 Tracker 后验状态到 CSV，
// 供离线分析脚本评估未来时间窗口内的预测误差。
//
// 两个输出文件：
//   observation_log_<timestamp>.csv   —— 每条装甲板观测（odom坐标系）
//   tracker_state_log_<timestamp>.csv —— 每次 tracker 更新后的后验状态

#pragma once

#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <limits>

namespace fyt::auto_aim {

// 单条装甲板观测（已转换到 odom 坐标系）
struct LogObservation {
  double x;
  double y;
  double z;
  double yaw;
  int    panel_id;     // -1 表示未知
  double confidence;
  bool   is_dual_obs;  // 本帧是否同时有 ≥2 个装甲板
};

// Tracker 后验状态快照
struct LogTrackerState {
  // 中心位姿
  double center_x;
  double center_y;
  double center_z;
  // 速度
  double vel_x;
  double vel_y;
  double vel_z;
  // 旋转状态
  double yaw;
  double yaw_velocity;
  double yaw_acceleration;
  // 结构参数（smoother 最终输出）
  double radius_1;
  double radius_2;
  double dza;
  // 元信息
  uint8_t track_state;         // 0=DETECTING 1=TRACKING 2=TEMP_LOST
  int     num_armors;
  int     visible_armor_count;
  bool    is_visible;
  double  confidence;

  // ── 机动检测指标 (Maneuver Detection Metrics) ──
  // UKF 创新向量（位置分量）
  double innov_x   = 0.0;
  double innov_y   = 0.0;
  double innov_z   = 0.0;
  double innov_yaw = 0.0;   // 单观测时有效；双观测几何更新无 yaw 分量，填 0
  // 归一化创新平方 (NIS = innov^T * Pzz^{-1} * innov)
  // -1.0 = 本周期未执行 UKF update（纯预测帧或初始化前）
  double nis        = -1.0;
  int    update_type = 0;   // 0=无更新, 1=单观测, 2=双观测
  // 状态协方差 P 对角线 —— 位置方差
  double p_var_x  = 0.0;
  double p_var_y  = 0.0;
  double p_var_z  = 0.0;
  // 状态协方差 P 对角线 —— 速度方差
  double p_var_vx = 0.0;
  double p_var_vy = 0.0;
  double p_var_vz = 0.0;
  // 状态协方差 P 对角线 —— 加速度方差（CV 模型时为 NaN）
  double p_var_ax = 0.0;
  double p_var_ay = 0.0;
  double p_var_az = 0.0;
  // 加速度状态估计（CA / Singer 模型时有效；CV 模型时为 NaN）
  double accel_x         = 0.0;
  double accel_y         = 0.0;
  double accel_z         = 0.0;
  double accel_magnitude = 0.0;

  // ── Outpost audit fields (only populated for robot_id="outpost") ──
  int outpost_mode = -1;     // 0=STRUCTURED_3_ARMORS, 1=AMBIGUOUS_SINGLE_ARMOR
  int estimated_id = -1;     // canonical estimated id; -1 in ambiguous single mode
  int runtime_panel_id = -1; // internal filter panel index
  int bound_height_label = -1;  // 0=HIGH,1=MIDDLE,2=LOW
  int obs_inferred_id = -1;  // inferred id from observation yaw and center yaw
  int obs_inferred_id_z = -1;  // inferred id from observation z-jump audit
  int candidate_panel_id = -1;
  double candidate_prob = std::numeric_limits<double>::quiet_NaN();
  double candidate_margin = std::numeric_limits<double>::quiet_NaN();
  double selected_xy_residual = std::numeric_limits<double>::quiet_NaN();

  double outpost_entropy = std::numeric_limits<double>::quiet_NaN();
  double outpost_max_prob = std::numeric_limits<double>::quiet_NaN();

  double hyp_cost_0 = std::numeric_limits<double>::quiet_NaN();
  double hyp_cost_1 = std::numeric_limits<double>::quiet_NaN();
  double hyp_cost_2 = std::numeric_limits<double>::quiet_NaN();
  double hyp_prob_0 = std::numeric_limits<double>::quiet_NaN();
  double hyp_prob_1 = std::numeric_limits<double>::quiet_NaN();
  double hyp_prob_2 = std::numeric_limits<double>::quiet_NaN();

  double center_yaw_est = std::numeric_limits<double>::quiet_NaN();
  int has_observation = 0;
  double obs_x = std::numeric_limits<double>::quiet_NaN();
  double obs_y = std::numeric_limits<double>::quiet_NaN();
  double obs_z = std::numeric_limits<double>::quiet_NaN();
  double obs_yaw = std::numeric_limits<double>::quiet_NaN();
  double obs_z_jump = std::numeric_limits<double>::quiet_NaN();
  double obs_dz_from_audit_center = std::numeric_limits<double>::quiet_NaN();
  double obs_z_audit_cost_0 = std::numeric_limits<double>::quiet_NaN();
  double obs_z_audit_cost_1 = std::numeric_limits<double>::quiet_NaN();
  double obs_z_audit_cost_2 = std::numeric_limits<double>::quiet_NaN();

  // ── Binding/period evidence diagnostics ──
  double binding_confidence = std::numeric_limits<double>::quiet_NaN();
  int switch_event = 0;      // 0=no switch, 1=switch confirmed
  int switch_reason = 0;     // 0=none,1=confirmed,2=reject_prob,3=reject_margin,4=transition_abort
  int transition_state = 0;  // 0=LOCKED, 1=TRANSITION_CANDIDATE
  double period_confidence = std::numeric_limits<double>::quiet_NaN();
  int period_update_applied = 0;
  int period_phase_index = -1;
  int spin_direction = 0;  // +1=CCW, -1=CW, 0=unknown
  double dz_small_est = std::numeric_limits<double>::quiet_NaN();
  double dz_large_est = std::numeric_limits<double>::quiet_NaN();
};

// -----------------------------------------------------------------------------
class PredictionLogger {
 public:
  // output_dir    : 日志目录（不存在则自动创建）
  // robot_id_filter: 只记录该 robot_id；为空字符串则记录全部
  // flush_every_n : 每 N 行 flush 一次（缓解实时性影响）
  PredictionLogger(const std::string &output_dir,
                   const std::string &robot_id_filter,
                   int flush_every_n = 50);

  ~PredictionLogger();

  // 禁用拷贝
  PredictionLogger(const PredictionLogger &) = delete;
  PredictionLogger &operator=(const PredictionLogger &) = delete;

  // 记录一批装甲板观测（由 armorsCallback 在 Step 2 之后调用）
  // timestamp_ns : ROS 消息时间戳（纳秒）
  // robot_id     : 机器人编号字符串
  // obs_list     : 已转换到 odom 坐标系的观测列表
  void logObservations(int64_t timestamp_ns,
                       const std::string &robot_id,
                       const std::vector<LogObservation> &obs_list);

  // 记录 tracker 后验状态（由 armorsCallback 在 buildTrackedRobotsMsg 之后调用）
  void logTrackerState(int64_t timestamp_ns,
                       const std::string &robot_id,
                       const LogTrackerState &state);

  // 立即 flush 所有缓冲（析构时自动调用）
  void flush();

  bool is_open() const { return obs_file_.is_open() && state_file_.is_open(); }

 private:
  bool shouldLog(const std::string &robot_id) const;

  std::ofstream obs_file_;
  std::ofstream state_file_;

  std::string robot_id_filter_;
  int flush_every_n_;
  int obs_write_count_   = 0;
  int state_write_count_ = 0;
};

}  // namespace fyt::auto_aim
