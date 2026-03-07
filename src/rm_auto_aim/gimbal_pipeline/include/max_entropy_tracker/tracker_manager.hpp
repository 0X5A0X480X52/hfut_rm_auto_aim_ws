// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKER_MANAGER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKER_MANAGER_HPP_

#include <chrono>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>
#include <iostream>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/adaptive_armor_tracker.hpp"

namespace fyt::auto_aim {

/// Manages per-robot AdaptiveArmorTracker instances.
class TrackerManager {
 public:
  struct TrackerEntry {
    std::unique_ptr<AdaptiveArmorTracker> tracker;
    double last_update_time = 0.0;
    int observation_count = 0;
  };

  explicit TrackerManager(const UnifiedConfig &config, double dt = 0.01,
                          double default_r1 = 0.15, double default_r2 = 0.20,
                          double default_dza = 0.0,
                          double timeout_seconds = 0.5,
                          bool enable_oscillation = false)
      : config_(config),
        dt_(dt),
        default_r1_(default_r1),
        default_r2_(default_r2),
        default_dza_(default_dza),
        timeout_(timeout_seconds),
        enable_osc_(enable_oscillation) {}

  /// Get existing tracker or create+initialize a new one.
  AdaptiveArmorTracker *get_or_create(
      const std::string &robot_id,
      const std::vector<ObservationData> &initial_obs,
      double current_time) {
    auto it = trackers_.find(robot_id);
    if (it != trackers_.end()) return it->second.tracker.get();
    if (initial_obs.empty()) return nullptr;

    auto t = std::make_unique<AdaptiveArmorTracker>(config_, dt_, enable_osc_);
    t->initialize(initial_obs, default_r1_, default_r2_, default_dza_);

    auto *ptr = t.get();
    TrackerEntry entry;
    entry.tracker = std::move(t);
    entry.last_update_time = current_time;
    entry.observation_count = static_cast<int>(initial_obs.size());
    trackers_[robot_id] = std::move(entry);
    return ptr;
  }

  /// Update a robot's tracker; auto-creates if needed.
  bool update(const std::string &robot_id,
              const std::vector<ObservationData> &obs,
              double current_time) {
    std::cout << "Updating tracker for robot_id=" << robot_id
              << " with obs_count=" << obs.size() << std::endl;
    if (obs.empty()) return false;
    double t = current_time;
    auto it = trackers_.find(robot_id);
    if (it == trackers_.end()) {
      std::cout << "No existing tracker for robot_id=" << robot_id
                << ", creating new one." << std::endl;
      return get_or_create(robot_id, obs, current_time) != nullptr;
    }

    bool ok = it->second.tracker->update(obs);
    if (ok) {
      it->second.last_update_time = t;
      it->second.observation_count += static_cast<int>(obs.size());
      std::cout << "Updated tracker for robot_id=" << robot_id
                << ", total_obs_count=" << it->second.observation_count
                << std::endl;
    }

    std::cout << "Tracker state for robot_id=" << robot_id
              << " is now " << (it->second.tracker->is_tracking() ? "TRACKING" : "OTHER")
              << std::endl;
    return ok;
  }

  /// Notify trackers that were NOT observed in the current frame.
  /// This drives the state machine: TRACKING → TEMP_LOST → LOST.
  void notify_missing(const std::set<std::string> &observed_ids,
                      double /*current_time*/) {
    for (auto &[id, entry] : trackers_) {
      if (observed_ids.count(id) == 0 && entry.tracker->is_initialized()) {
        // Call update with empty observations to trigger handle_observation_loss
        entry.tracker->update({});
        std::cout << "[TrackerManager] notify_missing: robot_id=" << id
                  << " state=" << tracker_state_to_string(entry.tracker->state())
                  << " lost_count=" << entry.tracker->lost_count() << std::endl;
      }
    }
    // Also clean up trackers that have transitioned to LOST state
    for (auto it = trackers_.begin(); it != trackers_.end();) {
      if (it->second.tracker->is_lost()) {
        std::cout << "[TrackerManager] Removing LOST tracker: robot_id=" << it->first << std::endl;
        it = trackers_.erase(it);
      } else {
        ++it;
      }
    }
  }

  /// Predict all active trackers to target_time.
  void predict_all(std::optional<double> target_time = std::nullopt) {
    for (auto &[id, entry] : trackers_) {
      if (entry.tracker->is_initialized())
        entry.tracker->predict(target_time);
    }
  }

  /// Remove trackers not updated within timeout.
  std::vector<std::string> remove_stale(double current_time) {
    double t = current_time;
    std::vector<std::string> removed;
    for (auto it = trackers_.begin(); it != trackers_.end();) {
      if (t - it->second.last_update_time > timeout_) {
        std::cout << "Removing stale tracker for robot_id=" << it->first
                  << ", last_update_time=" << it->second.last_update_time
                  << ", current_time=" << t << std::endl;
        std::cout << "delay=" << (t - it->second.last_update_time) << "s exceeds timeout=" << timeout_ << "s" << std::endl;
        removed.push_back(it->first);
        it = trackers_.erase(it);
      } else {
        ++it;
      }
    }
    return removed;
  }

  AdaptiveArmorTracker *get(const std::string &id) {
    auto it = trackers_.find(id);
    return (it != trackers_.end()) ? it->second.tracker.get() : nullptr;
  }

  const std::unordered_map<std::string, TrackerEntry> &trackers() const {
    return trackers_;
  }

  /// Returns IDs of trackers in TRACKING state only.
  std::vector<std::string> tracking_robot_ids() const {
    std::vector<std::string> ids;
    for (const auto &[id, e] : trackers_)
      if (e.tracker->is_tracking()) ids.push_back(id);
    return ids;
  }

  /// Returns IDs of trackers in TRACKING or TEMP_LOST state (active trackers).
  std::vector<std::string> active_robot_ids() const {
    std::vector<std::string> ids;
    for (const auto &[id, e] : trackers_)
      if (e.tracker->is_tracking() || e.tracker->is_temp_lost())
        ids.push_back(id);
    return ids;
  }

  int num_trackers() const { return static_cast<int>(trackers_.size()); }
  void clear() { trackers_.clear(); }

 private:
  static double now() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(
               steady_clock::now().time_since_epoch())
        .count();
  }

  UnifiedConfig config_;
  double dt_, default_r1_, default_r2_, default_dza_, timeout_;
  bool enable_osc_;
  std::unordered_map<std::string, TrackerEntry> trackers_;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKER_MANAGER_HPP_
