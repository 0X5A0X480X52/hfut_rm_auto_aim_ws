#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/norm_4armor_tracker.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/tracker/norm4_tracker_v2.hpp"
#include "max_entropy_tracker/trackers/outpost_tracker_v2.hpp"
#include "max_entropy_tracker/tracker_manager.hpp"
#include "max_entropy_tracker/utils/output_smoother.hpp"

namespace fyt::auto_aim::pybind {

struct ReplaySnapshot {
  bool valid = false;
  std::string robot_id;

  double timestamp_sec = 0.0;
  int tracker_count = 0;

  int track_state = -1;
  int frame_count = 0;
  int lost_count = 0;

  double center_x = 0.0;
  double center_y = 0.0;
  double center_z = 0.0;
  double vel_x = 0.0;
  double vel_y = 0.0;
  double vel_z = 0.0;
  double yaw = 0.0;
  double radius_1 = 0.0;
  double radius_2 = 0.0;
  double dza = 0.0;

  double innov_x = 0.0;
  double innov_y = 0.0;
  double innov_z = 0.0;
  double innov_yaw = 0.0;
  double nis = -1.0;
  int update_type = 0;

  double p_var_x = 0.0;
  double p_var_y = 0.0;
  double p_var_z = 0.0;

  int obs_count = 0;
  bool dual_obs = false;

  // Common semantic diagnostics
  int mode = -1;
  int candidate_panel_id = -1;
  double candidate_prob = 0.0;
  double candidate_margin = 0.0;
  double entropy_norm = 1.0;
  double max_prob = 0.0;
  int switch_event = 0;
  int switch_reason = 0;
  double binding_confidence = 0.0;

  // outpost-only
  int z_audit_conflict_count = 0;

  // norm4-only
  bool degraded_single_obs_mode = false;
  int single_obs_streak = 0;
};

class OfflineTrackerReplay {
 public:
  OfflineTrackerReplay();

  void reset(double dt = 0.05,
             double default_r1 = 0.15,
             double default_r2 = 0.20,
             double default_dza = 0.0,
             double timeout_seconds = 0.5,
             bool enable_oscillation = false);

  void clear();

  void set_fixed_robot_id(const std::string &robot_id);
  const std::string &fixed_robot_id() const { return fixed_robot_id_; }

  void apply_numeric_overrides(const std::unordered_map<std::string, double> &overrides);

  bool feed(double timestamp_sec, std::vector<ObservationData> observations);
  bool feed_empty(double timestamp_sec);

  ReplaySnapshot snapshot() const;

  const UnifiedConfig &config() const { return config_; }

 private:
  std::unique_ptr<TrackerManager> create_manager() const;
  void rebuild_manager();

  static void apply_override(UnifiedConfig &cfg,
                             const std::string &key,
                             double value);

  UnifiedConfig config_;
  SmootherConfig smoother_cfg_;
  std::unique_ptr<TrackerManager> manager_;

  std::string fixed_robot_id_ = "outpost";
  double last_timestamp_sec_ = 0.0;

  double dt_ = 0.05;
  double default_r1_ = 0.15;
  double default_r2_ = 0.20;
  double default_dza_ = 0.0;
  double timeout_seconds_ = 0.5;
  bool enable_oscillation_ = false;
};

}  // namespace fyt::auto_aim::pybind
