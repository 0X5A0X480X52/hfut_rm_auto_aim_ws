#include "armor_pnp_refiner/association/short_term_associator.hpp"

#include <cmath>
#include <algorithm>

namespace armor_pnp_refiner {

ShortTermAssociator::ShortTermAssociator(const PnpRefinerConfig& config)
  : config_(config), gate_(config) {}

int ShortTermAssociator::associate(const PnpRefineInput& input)
{
  // If external track id is available and enabled, use it directly.
  if (input.external_track_id.has_value() && config_.use_external_track_id_if_available) {
    return input.external_track_id.value();
  }

  if (!config_.enable_internal_association) {
    return -1;  // No association possible.
  }

  double now = input.stamp_sec;

  // Prune expired tracks first.
  pruneExpired(now);

  // Find best matching track.
  int best_id = -1;
  double best_score = 0.0;

  for (const auto& track : tracks_) {
    if (!gate_.passTimeGap(track.last_stamp_sec, now)) continue;
    if (!gate_.passArmorNumber(track.armor_number, input.armor_number)) continue;
    if (!gate_.passArmorType(track.armor_type, input.armor_type)) continue;

    // Gate check: use IoU or center distance.
    bool spatial_ok = gate_.passIoU(track.last_bbox, input.bbox) ||
                      gate_.passCenterDistance(track.last_center, input.center);
    if (!spatial_ok) continue;

    double score = computeAssociationScore(track, input);
    if (score > best_score) {
      best_score = score;
      best_id = track.refine_track_id;
    }
  }

  // Update matched track or create new.
  if (best_id > 0) {
    for (auto& track : tracks_) {
      if (track.refine_track_id == best_id) {
        track.last_bbox = input.bbox;
        track.last_center = input.center;
        track.last_t = input.t_camera_armor;
        track.last_yaw = input.yaw_rad;
        track.last_stamp_sec = now;
        track.hits++;
        track.misses = 0;
        break;
      }
    }
  } else {
    // Create new internal track.
    InternalTrack new_track;
    new_track.refine_track_id = next_id_++;
    new_track.last_bbox = input.bbox;
    new_track.last_center = input.center;
    new_track.last_t = input.t_camera_armor;
    new_track.last_yaw = input.yaw_rad;
    new_track.last_stamp_sec = now;
    new_track.armor_type = input.armor_type;
    new_track.armor_number = input.armor_number;
    new_track.hits = 1;
    tracks_.push_back(new_track);
    best_id = new_track.refine_track_id;
  }

  // Update misses for unmatched tracks.
  for (auto& track : tracks_) {
    if (track.refine_track_id != best_id) {
      track.misses++;
    }
  }

  return best_id;
}

void ShortTermAssociator::reset()
{
  tracks_.clear();
  next_id_ = 1;
}

void ShortTermAssociator::pruneExpired(double now_sec)
{
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
          [this, now_sec](const InternalTrack& t) {
            return t.misses > config_.max_missed_frames ||
                   (now_sec - t.last_stamp_sec) * 1000.0 > config_.max_time_span_ms;
          }),
      tracks_.end());
}

double ShortTermAssociator::computeIoU(const cv::Rect2f& a, const cv::Rect2f& b) const
{
  float inter_x = std::max(0.0f, std::min(a.x + a.width, b.x + b.width) - std::max(a.x, b.x));
  float inter_y = std::max(0.0f, std::min(a.y + a.height, b.y + b.height) - std::max(a.y, b.y));
  float inter = inter_x * inter_y;
  float area_a = a.width * a.height;
  float area_b = b.width * b.height;
  return static_cast<double>(inter / (area_a + area_b - inter + 1e-6f));
}

double ShortTermAssociator::computeAssociationScore(
    const InternalTrack& track, const PnpRefineInput& obs) const
{
  double iou = computeIoU(track.last_bbox, obs.bbox);

  double dx = obs.center.x - track.last_center.x;
  double dy = obs.center.y - track.last_center.y;
  double center_dist = std::sqrt(dx * dx + dy * dy);
  double center_norm = std::max(0.0, 1.0 - center_dist / config_.center_distance_threshold_px);

  double w_iou = 0.6, w_center = 0.4;
  return w_iou * iou + w_center * center_norm;
}

}  // namespace armor_pnp_refiner
