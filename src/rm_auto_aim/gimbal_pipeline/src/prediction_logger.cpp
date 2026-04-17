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
                 "num_armors,visible_armor_count,is_visible,confidence,"
                 "innov_x,innov_y,innov_z,innov_yaw,"
                 "nis,update_type,"
                 "p_var_x,p_var_y,p_var_z,"
                 "p_var_vx,p_var_vy,p_var_vz,"
                 "p_var_ax,p_var_ay,p_var_az,"
                 "accel_x,accel_y,accel_z,accel_magnitude,"
                 "outpost_mode,estimated_id,runtime_panel_id,obs_inferred_id,obs_inferred_id_z,"
                 "outpost_entropy,outpost_max_prob,"
                 "hyp_cost_0,hyp_cost_1,hyp_cost_2,"
                 "hyp_prob_0,hyp_prob_1,hyp_prob_2,"
                 "center_yaw_est,has_observation,"
                 "obs_x,obs_y,obs_z,obs_yaw,"
                 "obs_z_jump,obs_dz_from_audit_center,"
                 "obs_z_audit_cost_0,obs_z_audit_cost_1,obs_z_audit_cost_2,"
                 "binding_confidence,switch_event,transition_state,"
                 "period_confidence,period_phase_index,spin_direction,"
                 "dz_small_est,dz_large_est\n";
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
              << s.confidence         << ','
              << s.innov_x            << ','
              << s.innov_y            << ','
              << s.innov_z            << ','
              << s.innov_yaw          << ','
              << s.nis                << ','
              << s.update_type        << ','
              << s.p_var_x            << ','
              << s.p_var_y            << ','
              << s.p_var_z            << ','
              << s.p_var_vx           << ','
              << s.p_var_vy           << ','
              << s.p_var_vz           << ','
              << s.p_var_ax           << ','
              << s.p_var_ay           << ','
              << s.p_var_az           << ','
              << s.accel_x            << ','
              << s.accel_y            << ','
              << s.accel_z            << ','
              << s.accel_magnitude    << ','
              << s.outpost_mode       << ','
              << s.estimated_id       << ','
              << s.runtime_panel_id   << ','
              << s.obs_inferred_id    << ','
              << s.obs_inferred_id_z  << ','
              << s.outpost_entropy    << ','
              << s.outpost_max_prob   << ','
              << s.hyp_cost_0         << ','
              << s.hyp_cost_1         << ','
              << s.hyp_cost_2         << ','
              << s.hyp_prob_0         << ','
              << s.hyp_prob_1         << ','
              << s.hyp_prob_2         << ','
              << s.center_yaw_est     << ','
              << s.has_observation    << ','
              << s.obs_x              << ','
              << s.obs_y              << ','
              << s.obs_z              << ','
              << s.obs_yaw            << ','
              << s.obs_z_jump         << ','
              << s.obs_dz_from_audit_center << ','
              << s.obs_z_audit_cost_0 << ','
              << s.obs_z_audit_cost_1 << ','
              << s.obs_z_audit_cost_2 << ','
              << s.binding_confidence << ','
              << s.switch_event << ','
              << s.transition_state << ','
              << s.period_confidence << ','
              << s.period_phase_index << ','
              << s.spin_direction << ','
              << s.dz_small_est << ','
              << s.dz_large_est << '\n';
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
