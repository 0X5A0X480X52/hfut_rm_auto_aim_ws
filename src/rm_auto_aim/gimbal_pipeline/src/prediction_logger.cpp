// Copyright (C) FYT Vision Group. All rights reserved.
// Licensed under the Apache License, Version 2.0

#include "gimbal_pipeline/prediction_logger.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace fyt::auto_aim {

namespace {

// 生成 "YYYYMMDD_HHMMSS" 格式的时间戳字符串
std::string currentTimestamp() {
  auto now    = std::chrono::system_clock::now();
  std::time_t tt = std::chrono::system_clock::to_time_t(now);
  std::tm  tm_buf{};
  localtime_r(&tt, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
  return oss.str();
}

}  // namespace

// ---------------------------------------------------------------------------
PredictionLogger::PredictionLogger(const std::string &output_dir,
                                   const std::string &robot_id_filter,
                                   int flush_every_n)
    : robot_id_filter_(robot_id_filter), flush_every_n_(flush_every_n) {
  // 确保目录存在
  std::filesystem::create_directories(output_dir);

  std::string ts = currentTimestamp();

  // ----- 观测日志 -----
  std::string obs_path = output_dir + "/observation_log_" + ts + ".csv";
  obs_file_.open(obs_path, std::ios::out | std::ios::trunc);
  if (!obs_file_.is_open()) {
    throw std::runtime_error("[PredictionLogger] Cannot open: " + obs_path);
  }
  // 写表头
  obs_file_ << "timestamp_ns,robot_id,panel_id,"
               "obs_x,obs_y,obs_z,obs_yaw,"
               "confidence,is_dual_obs\n";

  // ----- 状态日志 -----
  std::string state_path = output_dir + "/tracker_state_log_" + ts + ".csv";
  state_file_.open(state_path, std::ios::out | std::ios::trunc);
  if (!state_file_.is_open()) {
    throw std::runtime_error("[PredictionLogger] Cannot open: " + state_path);
  }
  // 写表头
  state_file_ << "timestamp_ns,robot_id,track_state,"
                 "center_x,center_y,center_z,"
                 "vel_x,vel_y,vel_z,"
                 "yaw,yaw_velocity,yaw_acceleration,"
                 "radius_1,radius_2,dza,"
                 "num_armors,visible_armor_count,is_visible,confidence\n";
}

// ---------------------------------------------------------------------------
PredictionLogger::~PredictionLogger() {
  flush();
  if (obs_file_.is_open())   obs_file_.close();
  if (state_file_.is_open()) state_file_.close();
}

// ---------------------------------------------------------------------------
bool PredictionLogger::shouldLog(const std::string &robot_id) const {
  return robot_id_filter_.empty() || (robot_id == robot_id_filter_);
}

// ---------------------------------------------------------------------------
void PredictionLogger::logObservations(
    int64_t timestamp_ns, const std::string &robot_id,
    const std::vector<LogObservation> &obs_list) {
  if (!obs_file_.is_open() || !shouldLog(robot_id)) return;

  for (const auto &o : obs_list) {
    obs_file_ << timestamp_ns << ','
              << robot_id    << ','
              << o.panel_id  << ','
              << o.x         << ','
              << o.y         << ','
              << o.z         << ','
              << o.yaw       << ','
              << o.confidence << ','
              << (o.is_dual_obs ? 1 : 0) << '\n';
    ++obs_write_count_;
  }

  if (obs_write_count_ % flush_every_n_ < static_cast<int>(obs_list.size())) {
    obs_file_.flush();
  }
}

// ---------------------------------------------------------------------------
void PredictionLogger::logTrackerState(int64_t timestamp_ns,
                                       const std::string &robot_id,
                                       const LogTrackerState &s) {
  if (!state_file_.is_open() || !shouldLog(robot_id)) return;

  state_file_ << timestamp_ns         << ','
              << robot_id             << ','
              << static_cast<int>(s.track_state) << ','
              << s.center_x           << ','
              << s.center_y           << ','
              << s.center_z           << ','
              << s.vel_x              << ','
              << s.vel_y              << ','
              << s.vel_z              << ','
              << s.yaw                << ','
              << s.yaw_velocity       << ','
              << s.yaw_acceleration   << ','
              << s.radius_1           << ','
              << s.radius_2           << ','
              << s.dza                << ','
              << s.num_armors         << ','
              << s.visible_armor_count << ','
              << (s.is_visible ? 1 : 0) << ','
              << s.confidence         << '\n';
  ++state_write_count_;

  if (state_write_count_ % flush_every_n_ == 0) {
    state_file_.flush();
  }
}

// ---------------------------------------------------------------------------
void PredictionLogger::flush() {
  if (obs_file_.is_open())   obs_file_.flush();
  if (state_file_.is_open()) state_file_.flush();
}

}  // namespace fyt::auto_aim
