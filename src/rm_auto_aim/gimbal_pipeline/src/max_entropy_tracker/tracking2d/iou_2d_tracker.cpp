// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#include "max_entropy_tracker/tracking2d/iou_2d_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fyt::auto_aim {

IoUArmor2DTracker::IoUArmor2DTracker(const IoU2DTrackerConfig& config)
    : config_(config) {}

double IoUArmor2DTracker::computeIoU(
    double ax, double ay, double aw, double ah,
    double bx, double by, double bw, double bh) {
  double x1 = std::max(ax, bx);
  double y1 = std::max(ay, by);
  double x2 = std::min(ax + aw, bx + bw);
  double y2 = std::min(ay + ah, by + bh);

  double inter_w = std::max(0.0, x2 - x1);
  double inter_h = std::max(0.0, y2 - y1);
  double inter_area = inter_w * inter_h;

  double area_a = aw * ah;
  double area_b = bw * bh;
  double union_area = area_a + area_b - inter_area;

  if (union_area <= 0.0) return 0.0;
  return inter_area / union_area;
}

void IoUArmor2DTracker::predictBbox(InternalTrack& track, double timestamp) {
  if (track.age > 0) {
    double dt = timestamp - track.last_timestamp;
    dt = std::clamp(dt, 0.0, 0.2);
    track.bbox_x += track.velocity_x * dt;
    track.bbox_y += track.velocity_y * dt;
  }
}

std::vector<std::pair<int, int>> IoUArmor2DTracker::hungarianMatch(
    const std::vector<std::vector<double>>& cost_matrix) {
  const int N = static_cast<int>(cost_matrix.size());
  if (N == 0) return {};

  const double INF = 1e18;
  std::vector<double> u(N + 1, 0.0), v(N + 1, 0.0);
  std::vector<int> p(N + 1, 0), way(N + 1, 0);

  for (int i = 1; i <= N; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(N + 1, INF);
    std::vector<char> used(N + 1, false);
    do {
      used[j0] = true;
      int i0 = p[j0];
      double delta = INF;
      int j1 = 0;
      for (int j = 1; j <= N; ++j) {
        if (used[j]) continue;
        double cur = cost_matrix[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= N; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<std::pair<int, int>> matches;
  matches.reserve(N);
  for (int j = 1; j <= N; ++j) {
    if (p[j] > 0) {
      matches.emplace_back(p[j] - 1, j - 1);
    }
  }
  return matches;
}

std::vector<Armor2DTrackEvidence> IoUArmor2DTracker::update(
    const std::vector<Armor2DDetection>& detections,
    double timestamp) {
  const int N_tracks = static_cast<int>(tracks_.size());
  const int N_dets = static_cast<int>(detections.size());

  std::vector<Armor2DTrackEvidence> results;
  results.reserve(N_dets);
  for (const auto& det : detections) {
    Armor2DTrackEvidence ev;
    ev.valid = false;
    ev.observation_index = det.observation_index;
    results.push_back(ev);
  }

  // No detections: age all tracks, remove expired.
  if (N_dets == 0) {
    for (auto& t : tracks_) {
      t.missed++;
    }
    tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
        [this](const InternalTrack& t) {
          return t.missed > config_.max_missed;
        }),
      tracks_.end());
    return results;
  }

  // No tracks yet: create new ones for all detections.
  if (N_tracks == 0) {
    for (int dj = 0; dj < N_dets; ++dj) {
      const auto& det = detections[dj];
      InternalTrack ts;
      ts.id = next_id_++;
      ts.bbox_x = det.bbox_x;
      ts.bbox_y = det.bbox_y;
      ts.bbox_w = det.bbox_w;
      ts.bbox_h = det.bbox_h;
      ts.center_x = det.bbox_x + det.bbox_w * 0.5;
      ts.center_y = det.bbox_y + det.bbox_h * 0.5;
      ts.age = 0;
      ts.hits = 1;
      ts.missed = 0;
      ts.last_timestamp = timestamp;
      ts.confirmed = (ts.hits >= config_.min_hits);
      tracks_.push_back(std::move(ts));

      results[dj].valid = true;
      results[dj].track_id = tracks_.back().id;
      results[dj].observation_index = det.observation_index;
      results[dj].association_quality = 1.0;
      results[dj].confirmed = tracks_.back().confirmed;
      results[dj].age = 0;
      results[dj].hits = 1;
      results[dj].missed = 0;
      results[dj].center_x = tracks_.back().center_x;
      results[dj].center_y = tracks_.back().center_y;
    }
    return results;
  }

  // Predict track bounding boxes.
  std::vector<InternalTrack> predicted = tracks_;
  for (auto& t : predicted) {
    predictBbox(t, timestamp);
  }

  // Build cost matrix (square, dim = max(N_tracks, N_dets)).
  int dim = std::max(N_tracks, N_dets);
  std::vector<std::vector<double>> cost(dim, std::vector<double>(dim, 1e6));

  for (int ti = 0; ti < N_tracks; ++ti) {
    for (int dj = 0; dj < N_dets; ++dj) {
      const auto& pred = predicted[ti];
      const auto& det = detections[dj];

      double iou = computeIoU(pred.bbox_x, pred.bbox_y, pred.bbox_w, pred.bbox_h,
                              det.bbox_x, det.bbox_y, det.bbox_w, det.bbox_h);

      double det_cx = det.bbox_x + det.bbox_w * 0.5;
      double det_cy = det.bbox_y + det.bbox_h * 0.5;
      double dx = tracks_[ti].center_x - det_cx;
      double dy = tracks_[ti].center_y - det_cy;
      double center_dist = std::sqrt(dx * dx + dy * dy);

      if (iou >= config_.iou_threshold &&
          center_dist <= config_.max_center_dist_px) {
        cost[ti][dj] = 1.0 - iou;
      }
    }
  }

  auto matches = hungarianMatch(cost);

  std::vector<bool> det_matched(N_dets, false);
  std::vector<bool> track_matched(N_tracks, false);

  for (const auto& [ti, dj] : matches) {
    if (ti >= N_tracks || dj >= N_dets) continue;

    double iou = computeIoU(predicted[ti].bbox_x, predicted[ti].bbox_y,
                            predicted[ti].bbox_w, predicted[ti].bbox_h,
                            detections[dj].bbox_x, detections[dj].bbox_y,
                            detections[dj].bbox_w, detections[dj].bbox_h);

    double det_cx = detections[dj].bbox_x + detections[dj].bbox_w * 0.5;
    double det_cy = detections[dj].bbox_y + detections[dj].bbox_h * 0.5;
    double dx = tracks_[ti].center_x - det_cx;
    double dy = tracks_[ti].center_y - det_cy;
    double center_dist = std::sqrt(dx * dx + dy * dy);

    if (iou < config_.iou_threshold ||
        center_dist > config_.max_center_dist_px) {
      continue;
    }

    // Valid match: update velocity.
    if (tracks_[ti].age > 0) {
      double dt = timestamp - tracks_[ti].last_timestamp;
      if (dt > 0.001) {
        double vx = (det_cx - tracks_[ti].center_x) / dt;
        double vy = (det_cy - tracks_[ti].center_y) / dt;
        double alpha = config_.velocity_smoothing;
        tracks_[ti].velocity_x = tracks_[ti].velocity_x * (1.0 - alpha) + vx * alpha;
        tracks_[ti].velocity_y = tracks_[ti].velocity_y * (1.0 - alpha) + vy * alpha;
      }
    }

    tracks_[ti].bbox_x = detections[dj].bbox_x;
    tracks_[ti].bbox_y = detections[dj].bbox_y;
    tracks_[ti].bbox_w = detections[dj].bbox_w;
    tracks_[ti].bbox_h = detections[dj].bbox_h;
    tracks_[ti].center_x = det_cx;
    tracks_[ti].center_y = det_cy;
    tracks_[ti].age++;
    tracks_[ti].hits++;
    tracks_[ti].missed = 0;
    tracks_[ti].last_timestamp = timestamp;
    tracks_[ti].confirmed = (tracks_[ti].hits >= config_.min_hits);

    // association_quality: blend IOU and center-distance score.
    double center_score = std::max(0.0, 1.0 - center_dist / config_.max_center_dist_px);
    double assoc_quality = std::max(iou, center_score * 0.8);

    results[dj].valid = true;
    results[dj].track_id = tracks_[ti].id;
    results[dj].observation_index = detections[dj].observation_index;
    results[dj].association_quality = assoc_quality;
    results[dj].confirmed = tracks_[ti].confirmed;
    results[dj].age = tracks_[ti].age;
    results[dj].hits = tracks_[ti].hits;
    results[dj].missed = tracks_[ti].missed;
    results[dj].center_x = tracks_[ti].center_x;
    results[dj].center_y = tracks_[ti].center_y;
    results[dj].velocity_x = tracks_[ti].velocity_x;
    results[dj].velocity_y = tracks_[ti].velocity_y;

    det_matched[dj] = true;
    track_matched[ti] = true;
  }

  // Unmatched detections: create new tracks.
  for (int dj = 0; dj < N_dets; ++dj) {
    if (!det_matched[dj]) {
      const auto& det = detections[dj];
      InternalTrack ts;
      ts.id = next_id_++;
      ts.bbox_x = det.bbox_x;
      ts.bbox_y = det.bbox_y;
      ts.bbox_w = det.bbox_w;
      ts.bbox_h = det.bbox_h;
      ts.center_x = det.bbox_x + det.bbox_w * 0.5;
      ts.center_y = det.bbox_y + det.bbox_h * 0.5;
      ts.age = 0;
      ts.hits = 1;
      ts.missed = 0;
      ts.last_timestamp = timestamp;
      ts.confirmed = (ts.hits >= config_.min_hits);
      tracks_.push_back(std::move(ts));

      results[dj].valid = true;
      results[dj].track_id = tracks_.back().id;
      results[dj].observation_index = det.observation_index;
      results[dj].association_quality = 1.0;
      results[dj].confirmed = tracks_.back().confirmed;
      results[dj].age = 0;
      results[dj].hits = 1;
      results[dj].missed = 0;
      results[dj].center_x = tracks_.back().center_x;
      results[dj].center_y = tracks_.back().center_y;
    }
  }

  // Unmatched tracks: increment miss count.
  for (int ti = 0; ti < N_tracks; ++ti) {
    if (!track_matched[ti]) {
      tracks_[ti].missed++;
    }
  }

  // Remove expired tracks.
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
      [this](const InternalTrack& t) {
        return t.missed > config_.max_missed;
      }),
    tracks_.end());

  return results;
}

void IoUArmor2DTracker::reset() {
  tracks_.clear();
  next_id_ = 0;
}

}  // namespace fyt::auto_aim
