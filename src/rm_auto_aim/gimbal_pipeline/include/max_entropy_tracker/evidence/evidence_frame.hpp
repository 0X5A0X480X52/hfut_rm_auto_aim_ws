// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_EVIDENCE_EVIDENCE_FRAME_HPP_
#define MAX_ENTROPY_TRACKER_EVIDENCE_EVIDENCE_FRAME_HPP_

#include <vector>

#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/pose_tracker_backend/single/single_tracker_types.hpp"
#include "max_entropy_tracker/tracking2d/armor_2d_types.hpp"

namespace fyt::auto_aim::evidence {

/// Spatial / relational pre-computed evidence for one frame.
struct RelationEvidence {
  bool valid = false;
  double z_jump = 0.0;
  bool has_z_jump = false;
  double yaw_delta = 0.0;
  double spatial_consistency = 1.0;
  bool has_dual_obs = false;
  int dual_panel_id_1 = -1;
  int dual_panel_id_2 = -1;
};

/// Completeness stats for observability / debug.
struct CompletenessStats {
  bool has_3d_obs = false;
  bool has_2d_tracks = false;
  bool has_proxy = false;
  bool has_geometry = false;
  bool has_relation = false;

  double fraction() const {
    int n = 0, d = 5;
    if (has_3d_obs) ++n;
    if (has_2d_tracks) ++n;
    if (has_proxy) ++n;
    if (has_geometry) ++n;
    if (has_relation) ++n;
    return static_cast<double>(n) / d;
  }
};

/// Unified evidence frame aggregating all observation, 2D, proxy and relation
/// evidence for consumption by downstream stages (binder, mode FSM, backends).
struct ArmorEvidenceFrame {
  double timestamp = 0.0;

  // 3D observations (from detector / TF transform).
  std::vector<ObservationData> observations;
  int obs_count = 0;

  // 2D track evidence (from IoUArmor2DTracker).
  std::vector<Armor2DTrackEvidence> track2d_evidence;

  // Single-armor proxy evidence (from SingleArmorProxyManager).
  std::vector<SingleArmorTrackEvidence> proxy_evidence;

  // Pre-computed relation evidence.
  RelationEvidence relation;

  // Completeness statistics.
  CompletenessStats completeness;
};

}  // namespace fyt::auto_aim::evidence

#endif  // MAX_ENTROPY_TRACKER_EVIDENCE_EVIDENCE_FRAME_HPP_
