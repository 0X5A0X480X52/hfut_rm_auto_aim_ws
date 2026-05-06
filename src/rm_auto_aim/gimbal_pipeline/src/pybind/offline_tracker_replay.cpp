#include "gimbal_pipeline/pybind/offline_tracker_replay.hpp"

#include <cmath>
#include <stdexcept>

namespace fyt::auto_aim::pybind {

namespace {

template <typename T>
T safe_value_or(const std::optional<T> &opt, T fallback) {
  return opt.has_value() ? opt.value() : fallback;
}

}  // namespace

OfflineTrackerReplay::OfflineTrackerReplay() {
  smoother_cfg_.enable = false;
  smoother_cfg_.enable_outlier_filter = false;
  rebuild_manager();
}

void OfflineTrackerReplay::reset(double dt,
                                 double default_r1,
                                 double default_r2,
                                 double default_dza,
                                 double timeout_seconds,
                                 bool enable_oscillation) {
  dt_ = dt;
  default_r1_ = default_r1;
  default_r2_ = default_r2;
  default_dza_ = default_dza;
  timeout_seconds_ = timeout_seconds;
  enable_oscillation_ = enable_oscillation;
  rebuild_manager();
  last_timestamp_sec_ = 0.0;
}

void OfflineTrackerReplay::clear() {
  if (manager_) {
    manager_->clear();
  }
  last_timestamp_sec_ = 0.0;
}

void OfflineTrackerReplay::set_fixed_robot_id(const std::string &robot_id) {
  fixed_robot_id_ = robot_id;
}

void OfflineTrackerReplay::apply_numeric_overrides(
    const std::unordered_map<std::string, double> &overrides) {
  for (const auto &[k, v] : overrides) {
    apply_override(config_, k, v);
  }
  rebuild_manager();
}

bool OfflineTrackerReplay::feed(double timestamp_sec,
                                std::vector<ObservationData> observations) {
  for (auto &obs : observations) {
    obs.timestamp = timestamp_sec;
  }

  std::unordered_map<std::string, std::vector<ObservationData>> obs_by_robot;
  obs_by_robot[fixed_robot_id_] = std::move(observations);

  manager_->process_frame(obs_by_robot, timestamp_sec, smoother_cfg_);
  last_timestamp_sec_ = timestamp_sec;

  auto *tracker = manager_->get(fixed_robot_id_);
  return tracker != nullptr && tracker->is_initialized();
}

bool OfflineTrackerReplay::feed_empty(double timestamp_sec) {
  std::unordered_map<std::string, std::vector<ObservationData>> obs_by_robot;
  manager_->process_frame(obs_by_robot, timestamp_sec, smoother_cfg_);
  last_timestamp_sec_ = timestamp_sec;

  auto *tracker = manager_->get(fixed_robot_id_);
  return tracker != nullptr && tracker->is_initialized();
}

ReplaySnapshot OfflineTrackerReplay::snapshot() const {
  ReplaySnapshot s;
  s.robot_id = fixed_robot_id_;
  s.timestamp_sec = last_timestamp_sec_;
  s.tracker_count = manager_ ? manager_->num_trackers() : 0;

  if (!manager_) {
    return s;
  }

  const auto *tracker = manager_->get(fixed_robot_id_);
  if (!tracker || !tracker->is_initialized()) {
    return s;
  }

  s.valid = true;
  s.track_state = static_cast<int>(tracker->state());
  s.frame_count = tracker->frame_count();
  s.lost_count = tracker->lost_count();

  const auto center = tracker->get_center_position();
  s.center_x = center.x();
  s.center_y = center.y();
  s.center_z = center.z();

  const auto vel = tracker->get_publish_velocity();
  s.vel_x = vel.x();
  s.vel_y = vel.y();
  s.vel_z = vel.z();

  s.yaw = tracker->get_yaw();
  auto [r1, r2] = tracker->get_radii();
  s.radius_1 = r1;
  s.radius_2 = r2;

  const auto &f = tracker->spin_filter();
  s.dza = f.get_dza();
  const auto &innov = f.last_innov_xyz();
  if (innov.size() >= 3) {
    s.innov_x = innov(0);
    s.innov_y = innov(1);
    s.innov_z = innov(2);
  }
  s.innov_yaw = f.last_innov_yaw();
  s.nis = f.last_nis();
  s.update_type = f.last_update_type();

  const auto &idx = f.state_idx();
  const auto &P = f.P();
  s.p_var_x = P(idx.X(), idx.X());
  s.p_var_y = P(idx.Y(), idx.Y());
  s.p_var_z = P(idx.Z(), idx.Z());

  s.obs_count = manager_->visible_observation_count(fixed_robot_id_);
  s.dual_obs = manager_->is_last_dual_observation(fixed_robot_id_);

  if (const auto *outpost = dynamic_cast<const OutpostTrackerV2 *>(tracker);
      outpost != nullptr) {
    const auto &d = outpost->debug_snapshot();
    s.mode = d.track_mode;
    s.candidate_panel_id = d.candidate_panel_id;
    s.candidate_prob = d.candidate_prob;
    s.candidate_margin = d.candidate_margin;
    s.entropy_norm = d.entropy_norm;
    s.max_prob = d.max_prob;
    s.switch_event = d.switch_event;
    s.switch_reason = d.switch_reason;
    s.binding_confidence = d.binding_confidence;
    s.z_audit_conflict_count = d.z_audit_conflict_count;
  } else if (const auto *norm4 = dynamic_cast<const Norm4ArmorTracker *>(tracker);
             norm4 != nullptr) {
    const auto &d = norm4->debug_snapshot();
    s.mode = d.track_mode;
    s.candidate_panel_id = d.candidate_panel_id;
    s.candidate_prob = d.candidate_prob;
    s.candidate_margin = d.candidate_margin;
    s.entropy_norm = d.entropy_norm;
    s.max_prob = d.max_prob;
    s.switch_event = d.switch_event;
    s.switch_reason = d.switch_reason;
    s.binding_confidence = d.binding_confidence;
    s.degraded_single_obs_mode = d.degraded_single_obs_mode;
    s.single_obs_streak = d.single_obs_streak;
  }

  return s;
}

std::unique_ptr<TrackerManager> OfflineTrackerReplay::create_manager() const {
  return std::make_unique<TrackerManager>(config_, dt_, default_r1_, default_r2_,
                                          default_dza_, timeout_seconds_,
                                          enable_oscillation_);
}

void OfflineTrackerReplay::rebuild_manager() {
  manager_ = create_manager();
}

void OfflineTrackerReplay::apply_override(UnifiedConfig &cfg,
                                          const std::string &key,
                                          double value) {
  if (key == "ukf.obs_noise_pos") cfg.ukf.obs_noise_pos = value;
  else if (key == "ukf.obs_noise_yaw") cfg.ukf.obs_noise_yaw = value;
  else if (key == "motion.cv_process_noise_vel") cfg.motion.cv_process_noise_vel = value;
  else if (key == "motion.ca_process_noise_acc") cfg.motion.ca_process_noise_acc = value;
  else if (key == "motion.process_noise_r") cfg.motion.process_noise_r = value;
  else if (key == "motion.process_noise_dz") cfg.motion.process_noise_dz = value;
  else if (key == "spin.spin_process_noise_delta_rate") cfg.spin.spin_process_noise_delta_rate = value;
  else if (key == "spin.spin_process_noise_delta_acc") cfg.spin.spin_process_noise_delta_acc = value;
  else if (key == "outpost.softmax_temperature") cfg.outpost.softmax_temperature = value;
  else if (key == "outpost.binding_min_candidate_prob") cfg.outpost.binding_min_candidate_prob = value;
  else if (key == "outpost.binding_min_candidate_margin") cfg.outpost.binding_min_candidate_margin = value;
  else if (key == "tracker.degraded_q_scale_r") cfg.tracker.degraded_q_scale_r = value;
  else if (key == "tracker.degraded_q_scale_dza") cfg.tracker.degraded_q_scale_dza = value;
  else {
    throw std::invalid_argument("Unsupported override key: " + key);
  }
}

}  // namespace fyt::auto_aim::pybind
