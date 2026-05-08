// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/evidence/evidence_builder.hpp"

#include <algorithm>
#include <cmath>

namespace fyt::auto_aim::evidence {

EvidenceBuilder::EvidenceBuilder(const EvidenceBuilderConfig &cfg,
                                 const UnifiedConfig &unified)
    : cfg_(cfg) {
  if (cfg_.enable_2d_tracker) {
    tracker_2d_ = std::make_unique<IoUArmor2DTracker>(cfg_.iou_2d);
  }
  if (cfg_.enable_proxy_manager) {
    proxy_manager_ =
        std::make_unique<SingleArmorProxyManager>(cfg_.proxy, unified);
  }
}

std::vector<Armor2DDetection> EvidenceBuilder::observations_to_detections(
    const std::vector<ObservationData> &obs, double timestamp) {
  std::vector<Armor2DDetection> detections;
  detections.reserve(obs.size());

  for (int i = 0; i < static_cast<int>(obs.size()); ++i) {
    const auto &o = obs[i];
    if (!(o.image.has_value() && o.image->valid)) continue;
    if (o.image->bbox_w <= 1e-6 || o.image->bbox_h <= 1e-6) continue;

    Armor2DDetection det;
    det.detection_id = i;
    det.timestamp = timestamp;
    det.observation_index = i;
    det.bbox_x = o.image->bbox_x;
    det.bbox_y = o.image->bbox_y;
    det.bbox_w = o.image->bbox_w;
    det.bbox_h = o.image->bbox_h;
    det.corners = o.image->corners;
    det.confidence = o.image->detection_confidence;
    det.number = o.image->number;
    det.type = o.image->type;
    detections.push_back(det);
  }
  return detections;
}

RelationEvidence EvidenceBuilder::build_relation(
    const std::vector<ObservationData> &obs,
    const ArmorEvidenceFrame &frame) {
  RelationEvidence rel;

  if (obs.empty()) return rel;

  rel.valid = true;
  rel.has_dual_obs = (obs.size() >= 2);

  // z_jump from observation history: use the proxy data if available.
  if (frame.track2d_evidence.size() >= 2) {
    const auto &a = frame.track2d_evidence[0];
    const auto &b = frame.track2d_evidence[1];
    if (a.valid && b.valid &&
        a.observation_index >= 0 &&
        b.observation_index >= 0 &&
        a.observation_index < static_cast<int>(obs.size()) &&
        b.observation_index < static_cast<int>(obs.size())) {
      rel.z_jump = obs[a.observation_index].z - obs[b.observation_index].z;
      rel.has_z_jump = std::abs(rel.z_jump) > 0.015;
    }
  }

  // Spatial consistency: check if dual observations are geometrically plausible.
  if (rel.has_dual_obs) {
    double dx = obs[0].x - obs[1].x;
    double dy = obs[0].y - obs[1].y;
    double dist = std::sqrt(dx * dx + dy * dy);
    // Two armors on a normal robot should be within ~0.4m.
    rel.spatial_consistency = std::max(0.0, 1.0 - dist / 0.5);
  }

  // Yaw delta between consecutive proxy evidence entries.
  if (frame.proxy_evidence.size() >= 2) {
    rel.yaw_delta = std::abs(
        frame.proxy_evidence[0].armor_yaw - frame.proxy_evidence[1].armor_yaw);
  }

  if (rel.has_dual_obs) {
    rel.dual_panel_id_1 = obs[0].panel_id.value_or(-1);
    rel.dual_panel_id_2 = obs[1].panel_id.value_or(-1);
  }

  return rel;
}

ArmorEvidenceFrame EvidenceBuilder::build(
    const std::vector<ObservationData> &obs, double timestamp) {
  ArmorEvidenceFrame frame;
  frame.timestamp = timestamp;
  frame.observations = obs;
  frame.obs_count = static_cast<int>(obs.size());
  frame.completeness.has_3d_obs = !obs.empty();

  // Stage 1: 2D tracker.
  if (tracker_2d_ && !obs.empty()) {
    auto detections = observations_to_detections(obs, timestamp);
    frame.track2d_evidence = tracker_2d_->update(detections, timestamp);
    frame.completeness.has_2d_tracks = !frame.track2d_evidence.empty();
    for (const auto &te : frame.track2d_evidence) {
      if (!te.valid || te.observation_index < 0 ||
          te.observation_index >= static_cast<int>(frame.observations.size())) {
        continue;
      }
      frame.observations[te.observation_index].track2d_id = te.track_id;
    }
  }

  // Stage 2: Single-armor proxy manager.
  if (proxy_manager_ && !obs.empty()) {
    double dt = 0.0;
    if (last_timestamp_.has_value()) {
      dt = std::max(0.0, timestamp - last_timestamp_.value());
      dt = std::min(dt, 0.2);
    }
    proxy_manager_->predict_all(dt);
    proxy_manager_->update(obs, frame.track2d_evidence);
    proxy_manager_->prune_lost();
    frame.proxy_evidence = proxy_manager_->collect_evidence();
    frame.completeness.has_proxy = !frame.proxy_evidence.empty();
  }

  // Stage 3: Relation evidence.
  frame.relation = build_relation(obs, frame);
  frame.completeness.has_relation = frame.relation.valid;
  frame.completeness.has_geometry =
      std::any_of(obs.begin(), obs.end(), [](const ObservationData &o) {
        return o.image.has_value() && o.image->valid;
      });
  last_timestamp_ = timestamp;

  return frame;
}

void EvidenceBuilder::reset() {
  if (tracker_2d_) tracker_2d_->reset();
  if (proxy_manager_) proxy_manager_->reset();
  last_timestamp_.reset();
}

}  // namespace fyt::auto_aim::evidence
