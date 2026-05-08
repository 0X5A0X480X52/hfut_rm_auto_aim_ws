// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/pose_tracker_backend/single/single_tracker_manager.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim {

SingleArmorProxyManager::SingleArmorProxyManager(
    const SingleProxyManagerConfig &cfg, const UnifiedConfig &unified)
    : cfg_(cfg), unified_cfg_(unified) {}

void SingleArmorProxyManager::predict_all(double dt) {
  for (auto &[id, entry] : proxies_) {
    if (entry.filter && entry.filter->initialized()) {
      entry.filter->predict(dt);
    }
  }
}

void SingleArmorProxyManager::update(
    const std::vector<ObservationData> &obs,
    const std::vector<Armor2DTrackEvidence> &track_evidences) {
  if (track_evidences.empty()) return;

  for (const auto &ev : track_evidences) {
    if (!ev.valid) continue;
    if (ev.observation_index < 0 ||
        ev.observation_index >= static_cast<int>(obs.size())) {
      continue;
    }

    const auto &observation = obs[ev.observation_index];
    auto it = proxies_.find(ev.track_id);

    if (it == proxies_.end()) {
      // Create new proxy for this track2d_id.
      ProxyEntry entry;
      entry.track2d_id = ev.track_id;
      entry.proxy_id = next_proxy_id_++;
      entry.filter = std::make_unique<AmbiguousSingleArmorFilterAdapter>(
          unified_cfg_);
      entry.filter->initialize(observation);
      entry.hits = 1;
      entry.missed = 0;
      entry.age = ev.age;
      entry.vel_history.push_back(Eigen::Vector3d::Zero());
      entry.yaw_rate_history.push_back(0.0);
      entry.panel_id = observation.panel_id.value_or(-1);
      proxies_[ev.track_id] = std::move(entry);
    } else {
      // Update existing proxy.
      auto &entry = it->second;
      if (entry.filter && entry.filter->initialized()) {
        entry.filter->update(observation);
      }

      entry.hits++;
      entry.missed = 0;
      entry.age = ev.age;

      // Update rolling kinematics windows.
      Eigen::Vector3d vel = entry.filter ? entry.filter->armor_velocity()
                                         : Eigen::Vector3d::Zero();
      entry.vel_history.push_back(vel);
      if (static_cast<int>(entry.vel_history.size()) >
          cfg_.kin_summary_window_size) {
        entry.vel_history.pop_front();
      }

      double yaw_rate =
          entry.filter ? entry.filter->armor_yaw_rate() : 0.0;
      entry.yaw_rate_history.push_back(yaw_rate);
      if (static_cast<int>(entry.yaw_rate_history.size()) >
          cfg_.kin_summary_window_size) {
        entry.yaw_rate_history.pop_front();
      }

      entry.last_vel = vel;
      entry.panel_id = observation.panel_id.value_or(entry.panel_id);
    }
  }

  // Increment missed for proxies that did not receive an update this frame.
  for (auto &[id, entry] : proxies_) {
    bool updated = false;
    for (const auto &ev : track_evidences) {
      if (ev.valid && ev.track_id == id) {
        updated = true;
        break;
      }
    }
    if (!updated) {
      entry.missed++;
    }
  }
}

std::vector<SingleArmorTrackEvidence> SingleArmorProxyManager::prune_lost() {
  std::vector<SingleArmorTrackEvidence> removed;
  for (auto it = proxies_.begin(); it != proxies_.end();) {
    if (it->second.missed > cfg_.max_lost_frames) {
      removed.push_back(SingleArmorTrackEvidence{});
      it = proxies_.erase(it);
    } else {
      ++it;
    }
  }
  return removed;
}

std::vector<SingleArmorTrackEvidence>
SingleArmorProxyManager::collect_evidence() const {
  std::vector<SingleArmorTrackEvidence> result;
  result.reserve(proxies_.size());

  for (const auto &[id, entry] : proxies_) {
    if (!entry.filter || !entry.filter->initialized()) continue;

    SingleArmorTrackEvidence ev;
    ev.valid = true;
    ev.track2d_id = entry.track2d_id;
    ev.proxy_id = entry.proxy_id;
    ev.armor_pos = entry.filter->armor_position();
    ev.armor_vel = entry.filter->armor_velocity();
    ev.armor_yaw = entry.filter->armor_yaw();
    ev.armor_yaw_rate = entry.filter->armor_yaw_rate();
    ev.panel_id = entry.panel_id;
    ev.age = entry.age;
    ev.hits = entry.hits;
    ev.missed = entry.missed;
    ev.initialized = entry.filter->initialized();

    // Compute kinematic summary.
    ev.kin_summary.velocity_var_window =
        compute_variance(entry.vel_history);
    ev.kin_summary.yaw_rate_continuity_score =
        1.0 - std::min(1.0, compute_variance(entry.yaw_rate_history) /
                                std::max(1e-6, cfg_.yaw_rate_jump_gate));

    if (entry.vel_history.size() >= 2) {
      double acc_norm = 0.0;
      int acc_count = 0;
      Eigen::Vector3d prev = entry.vel_history.front();
      for (size_t i = 1; i < entry.vel_history.size(); ++i) {
        acc_norm += (entry.vel_history[i] - prev).norm();
        prev = entry.vel_history[i];
        ++acc_count;
      }
      ev.kin_summary.acc_norm_window =
          acc_count > 0 ? acc_norm / acc_count : 0.0;
    }

    if (entry.vel_history.size() >= 2) {
      const auto &v0 = entry.vel_history.front();
      const auto &v1 = entry.vel_history.back();
      double dot = v0.dot(v1);
      double n0 = v0.norm();
      double n1 = v1.norm();
      if (n0 > 1e-9 && n1 > 1e-9) {
        ev.kin_summary.velocity_dir_cos =
            std::max(-1.0, std::min(1.0, dot / (n0 * n1)));
      }
    }

    ev.kin_summary.yaw_rate_converged =
        ev.kin_summary.yaw_rate_continuity_score > 0.7;

    result.push_back(ev);
  }
  return result;
}

void SingleArmorProxyManager::reset() {
  proxies_.clear();
  next_proxy_id_ = 0;
}

double SingleArmorProxyManager::compute_variance(
    const std::deque<Eigen::Vector3d> &vals) {
  if (vals.size() < 2) return 0.0;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto &v : vals) mean += v;
  mean /= static_cast<double>(vals.size());
  double var = 0.0;
  for (const auto &v : vals) var += (v - mean).squaredNorm();
  return var / static_cast<double>(vals.size());
}

double SingleArmorProxyManager::compute_variance(
    const std::deque<double> &vals) {
  if (vals.size() < 2) return 0.0;
  double mean = 0.0;
  for (double v : vals) mean += v;
  mean /= static_cast<double>(vals.size());
  double var = 0.0;
  for (double v : vals) var += (v - mean) * (v - mean);
  return var / static_cast<double>(vals.size());
}

}  // namespace fyt::auto_aim
