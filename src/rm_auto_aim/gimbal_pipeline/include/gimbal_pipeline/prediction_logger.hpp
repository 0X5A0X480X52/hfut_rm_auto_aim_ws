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
