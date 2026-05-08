// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_RUNTIME_CONTEXT_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_RUNTIME_CONTEXT_HPP_

#include <memory>
#include <optional>

#include <Eigen/Dense>

#include "max_entropy_tracker/binder/model/binder_enums.hpp"
#include "max_entropy_tracker/evidence/evidence_frame.hpp"
#include "max_entropy_tracker/mode/mode_enums.hpp"

namespace fyt::auto_aim::norm4_v2 {

// Forward declaration for ping-pong risk.
enum class PingPongReason;
struct PingPongRisk;

struct Norm4RuntimeContext {
  mode::TrackMode mode = mode::TrackMode::AMBIGUOUS;

  int selected_panel_id = -1;
  int bound_panel_id = -1;
  binder::HeightLabel bound_height_label = binder::HeightLabel::UNKNOWN;
  double binding_confidence = 0.0;

  Eigen::Vector3d center_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d center_vel = Eigen::Vector3d::Zero();
  double center_yaw = 0.0;
  double yaw_rate = 0.0;

  Eigen::Vector3d publish_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d publish_vel = Eigen::Vector3d::Zero();

  double r1 = 0.15;
  double r2 = 0.20;
  double dza = 0.0;
  bool dza_converged = false;

  std::optional<double> reference_center_yaw;
  double entropy_norm = 1.0;
  double max_prob = 0.0;
  int spin_direction = 0;

  std::optional<double> last_timestamp;
  std::optional<double> last_obs_z;
  int lost_frames = 0;

  // Phase 4: ping-pong suppression state.
  double ping_pong_risk_score = 0.0;
  bool ping_pong_pending = false;
  bool ping_pong_should_hold = false;
  int ping_pong_reason = 0;
  int ping_pong_hold_counter = 0;
  int ping_pong_consistent_counter = 0;

  // Phase 5: unified evidence frame for debug / downstream consumption.
  evidence::ArmorEvidenceFrame evidence_frame{};
};

}  // namespace fyt::auto_aim::norm4_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_RUNTIME_CONTEXT_HPP_
