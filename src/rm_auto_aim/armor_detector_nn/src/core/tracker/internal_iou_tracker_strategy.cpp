#include "armor_detector_nn/core/tracker/internal_iou_tracker_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace fyt::auto_aim {

InternalIoUTrackerStrategy::InternalIoUTrackerStrategy(
    const TrackerConfig& config)
  : config_(config)
{
}

double InternalIoUTrackerStrategy::computeIoU(
    const cv::Rect2f& a, const cv::Rect2f& b)
{
  float x1 = std::max(a.x, b.x);
  float y1 = std::max(a.y, b.y);
  float x2 = std::min(a.x + a.width, b.x + b.width);
  float y2 = std::min(a.y + a.height, b.y + b.height);

  float inter_w = std::max(0.0f, x2 - x1);
  float inter_h = std::max(0.0f, y2 - y1);
  float inter_area = inter_w * inter_h;

  float area_a = a.area();
  float area_b = b.area();
  float union_area = area_a + area_b - inter_area;

  if (union_area <= 0.0f) return 0.0;
  return static_cast<double>(inter_area / union_area);
}

std::vector<cv::Point> InternalIoUTrackerStrategy::hungarianMatch(
    const cv::Mat& cost_matrix)
{
  int N = cost_matrix.rows;

  std::vector<cv::Point> matches;
  std::vector<bool> row_used(N, false);
  std::vector<bool> col_used(N, false);

  // Collect all admissible (cost, row, col) entries.
  std::vector<std::tuple<double, int, int>> candidates;
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < N; ++j) {
      double c = cost_matrix.at<double>(i, j);
      if (c < 1e5) {
        candidates.emplace_back(c, i, j);
      }
    }
  }
  std::sort(candidates.begin(), candidates.end());

  for (const auto& [c, i, j] : candidates) {
    if (!row_used[i] && !col_used[j]) {
      matches.emplace_back(i, j);
      row_used[i] = true;
      col_used[j] = true;
    }
  }

  return matches;
}

std::vector<TrackedDetection> InternalIoUTrackerStrategy::associate(
    const std::vector<ArmorDetection>& detections,
    const rclcpp::Time& stamp)
{
  int N_tracks = static_cast<int>(tracks_.size());
  int N_dets   = static_cast<int>(detections.size());

  std::vector<TrackedDetection> results;
  results.reserve(N_dets);
  for (const auto& det : detections) {
    results.push_back({det, -1, false});
  }

  // No tracks yet — create new ones for all detections.
  if (N_tracks == 0 || N_dets == 0) {
    for (auto& td : results) {
      int new_id = next_id_++;
      td.track_id = new_id;
      td.matched = true;
      td.det.track_id  = new_id;
      td.det.track_age = 0;
      td.det.track_hits = 1;

      TrackState ts;
      ts.id = new_id;
      ts.predicted_bbox = td.det.bbox;
      ts.center = cv::Point2f(
        td.det.bbox.x + td.det.bbox.width * 0.5f,
        td.det.bbox.y + td.det.bbox.height * 0.5f);
      ts.age = 0;
      ts.hits = 1;
      ts.missed = 0;
      ts.last_stamp = stamp;
      ts.publish_type   = td.det.publish_type;
      ts.publish_number = td.det.publish_number;
      ts.confirmed = (ts.hits >= config_.min_hits);
      tracks_.push_back(std::move(ts));
    }
    return results;
  }

  // Build cost matrix: dim × dim, fill with large default.
  int dim = std::max(N_tracks, N_dets);
  cv::Mat cost(dim, dim, CV_64F, cv::Scalar(1e6));

  // Predict track positions via linear extrapolation.
  std::vector<cv::Rect2f> predicted_bboxes(N_tracks);
  for (int i = 0; i < N_tracks; ++i) {
    cv::Rect2f pred = tracks_[i].predicted_bbox;
    if (tracks_[i].age > 0 && tracks_[i].velocity.x != 0.0f) {
      double dt = (stamp - tracks_[i].last_stamp).seconds();
      dt = std::clamp(dt, 0.0, 0.2);  // cap prediction horizon
      pred.x += static_cast<float>(tracks_[i].velocity.x * dt);
      pred.y += static_cast<float>(tracks_[i].velocity.y * dt);
    }
    predicted_bboxes[i] = pred;
  }

  for (int ti = 0; ti < N_tracks; ++ti) {
    for (int dj = 0; dj < N_dets; ++dj) {
      const auto& pred = predicted_bboxes[ti];
      const auto& det_bbox = detections[dj].bbox;

      double iou = computeIoU(pred, det_bbox);

      cv::Point2f det_center(det_bbox.x + det_bbox.width * 0.5f,
                             det_bbox.y + det_bbox.height * 0.5f);
      double center_dist = cv::norm(tracks_[ti].center - det_center);

      if (iou >= config_.iou_threshold &&
          center_dist <= config_.max_center_dist_px) {
        cost.at<double>(ti, dj) = 1.0 - iou;
      }
    }
  }

  auto matches = hungarianMatch(cost);

  // Process matches.
  std::vector<bool> det_matched(N_dets, false);
  std::vector<bool> track_matched(N_tracks, false);

  for (const auto& m : matches) {
    int ti = m.x;
    int dj = m.y;
    if (ti >= N_tracks || dj >= N_dets) continue;

    // Re-validate gate.
    double iou = computeIoU(predicted_bboxes[ti], detections[dj].bbox);
    cv::Point2f det_center(
      detections[dj].bbox.x + detections[dj].bbox.width * 0.5f,
      detections[dj].bbox.y + detections[dj].bbox.height * 0.5f);
    double center_dist = cv::norm(tracks_[ti].center - det_center);

    if (iou < config_.iou_threshold ||
        center_dist > config_.max_center_dist_px) {
      continue;
    }

    // Valid match.
    results[dj].track_id = tracks_[ti].id;
    results[dj].matched = true;
    results[dj].det.track_id  = tracks_[ti].id;
    results[dj].det.track_age = tracks_[ti].age;
    results[dj].det.track_hits = tracks_[ti].hits;

    // Update velocity estimate.
    cv::Point2f new_center(
      detections[dj].bbox.x + detections[dj].bbox.width * 0.5f,
      detections[dj].bbox.y + detections[dj].bbox.height * 0.5f);
    if (tracks_[ti].age > 0) {
      double dt = (stamp - tracks_[ti].last_stamp).seconds();
      if (dt > 0.001) {
        cv::Point2f vel = (new_center - tracks_[ti].center) * (1.0 / dt);
        tracks_[ti].velocity = tracks_[ti].velocity * 0.7f + vel * 0.3f;
      }
    }

    tracks_[ti].predicted_bbox = detections[dj].bbox;
    tracks_[ti].center = new_center;
    tracks_[ti].age++;
    tracks_[ti].hits++;
    tracks_[ti].missed = 0;
    tracks_[ti].last_stamp = stamp;
    tracks_[ti].confirmed = (tracks_[ti].hits >= config_.min_hits);

    det_matched[dj] = true;
    track_matched[ti] = true;
  }

  // Unmatched detections → create new tracks.
  for (int dj = 0; dj < N_dets; ++dj) {
    if (!det_matched[dj]) {
      int new_id = next_id_++;
      results[dj].track_id = new_id;
      results[dj].matched = true;
      results[dj].det.track_id  = new_id;
      results[dj].det.track_age = 0;
      results[dj].det.track_hits = 1;

      TrackState ts;
      ts.id = new_id;
      ts.predicted_bbox = detections[dj].bbox;
      ts.center = cv::Point2f(
        detections[dj].bbox.x + detections[dj].bbox.width * 0.5f,
        detections[dj].bbox.y + detections[dj].bbox.height * 0.5f);
      ts.age = 0;
      ts.hits = 1;
      ts.missed = 0;
      ts.last_stamp = stamp;
      ts.publish_type   = detections[dj].publish_type;
      ts.publish_number = detections[dj].publish_number;
      ts.confirmed = (ts.hits >= config_.min_hits);
      tracks_.push_back(std::move(ts));
    }
  }

  // Unmatched tracks → increment miss count.
  for (int ti = 0; ti < N_tracks; ++ti) {
    if (!track_matched[ti]) {
      tracks_[ti].missed++;
      tracks_[ti].hits = 0;
    }
  }

  // Remove expired tracks.
  tracks_.erase(
    std::remove_if(tracks_.begin(), tracks_.end(),
      [this](const TrackState& t) {
        return t.missed > config_.max_missed;
      }),
    tracks_.end()
  );

  return results;
}

std::vector<TrackState> InternalIoUTrackerStrategy::getActiveTracks() const {
  return tracks_;
}

void InternalIoUTrackerStrategy::reset() {
  tracks_.clear();
  next_id_ = 0;
}

TrackerStats InternalIoUTrackerStrategy::getStats() const {
  TrackerStats s;
  s.total_tracks = static_cast<int>(tracks_.size());
  s.confirmed_tracks = static_cast<int>(
    std::count_if(tracks_.begin(), tracks_.end(),
                  [](const TrackState& t) { return t.confirmed; }));
  // matched_this_frame / new_this_frame should be set externally
  // by the caller — these fields reflect the last associate() result.
  return s;
}

}  // namespace fyt::auto_aim
