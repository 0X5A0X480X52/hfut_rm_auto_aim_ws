#include "armor_detector_nn/core/corner_refine/roi_pca_corner_refiner.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

#include <opencv2/imgproc.hpp>

#include <rm_utils/logger/log.hpp>

namespace fyt::auto_aim {

namespace {

double elapsedMs(
    const std::chrono::steady_clock::time_point& start) {
  return std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - start).count();
}

}  // namespace

RoiPcaCornerRefiner::RoiPcaCornerRefiner(
    const CornerRefineConfig& config,
    int frame_cols,
    int frame_rows)
  : config_(config),
    frame_cols_(frame_cols),
    frame_rows_(frame_rows)
{
}

std::pair<cv::Rect2f, cv::Rect2f>
RoiPcaCornerRefiner::extractLightBarROIs(
    const std::array<cv::Point2f, 4>& corners)
{
  // Left bar: corners[0] (top-left) + corners[3] (bottom-left)
  cv::Point2f lc = (corners[0] + corners[3]) * 0.5f;
  float lh_half = cv::norm(corners[0] - corners[3]) * 0.5f * config_.roi_expand_ratio;
  float lw_half = lh_half * 0.4f;

  cv::Rect2f left_roi(
    lc.x - lw_half, lc.y - lh_half,
    lw_half * 2.0f, lh_half * 2.0f);

  // Right bar: corners[1] (top-right) + corners[2] (bottom-right)
  cv::Point2f rc = (corners[1] + corners[2]) * 0.5f;
  float rh_half = cv::norm(corners[1] - corners[2]) * 0.5f * config_.roi_expand_ratio;
  float rw_half = rh_half * 0.4f;

  cv::Rect2f right_roi(
    rc.x - rw_half, rc.y - rh_half,
    rw_half * 2.0f, rh_half * 2.0f);

  left_roi  &= cv::Rect2f(0, 0, static_cast<float>(frame_cols_),
                           static_cast<float>(frame_rows_));
  right_roi &= cv::Rect2f(0, 0, static_cast<float>(frame_cols_),
                           static_cast<float>(frame_rows_));

  return {left_roi, right_roi};
}

RoiPcaCornerRefiner::LightBarEndpoints
RoiPcaCornerRefiner::refineLightBar(
    const cv::Mat& frame,
    const cv::Rect2f& roi,
    fyt::EnemyColor color)
{
  LightBarEndpoints result;

  cv::Rect roi_int(
    static_cast<int>(roi.x), static_cast<int>(roi.y),
    static_cast<int>(roi.width), static_cast<int>(roi.height));
  roi_int &= cv::Rect(0, 0, frame.cols, frame.rows);
  if (roi_int.width < 3 || roi_int.height < 3) {
    result.reason = RefineFailReason::ROI_TOO_SMALL;
    return result;
  }

  cv::Mat roi_img = frame(roi_int);

  // Channel enhancement: enhance the target color.
  cv::Mat gray;
  {
    std::vector<cv::Mat> channels;
    cv::split(roi_img, channels);
    // BGR order: channels[0]=B, channels[1]=G, channels[2]=R
    if (color == fyt::EnemyColor::RED) {
      gray = channels[2] - channels[0];  // R - B
    } else {
      gray = channels[0] - channels[2];  // B - R
    }
  }

  // OTSU binarization
  cv::Mat binary;
  cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

  // Collect bright points
  std::vector<cv::Point2f> bright_points;
  cv::findNonZero(binary, bright_points);
  for (auto& p : bright_points) {
    p.x += static_cast<float>(roi.x);
    p.y += static_cast<float>(roi.y);
  }

  if (static_cast<int>(bright_points.size()) < config_.min_bright_points) {
    result.reason = RefineFailReason::TOO_FEW_BRIGHT_POINTS;
    return result;
  }

  // PCA
  cv::Mat data(static_cast<int>(bright_points.size()), 2, CV_32F);
  for (size_t i = 0; i < bright_points.size(); ++i) {
    data.at<float>(static_cast<int>(i), 0) = bright_points[i].x;
    data.at<float>(static_cast<int>(i), 1) = bright_points[i].y;
  }

  cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);

  float lambda1 = pca.eigenvalues.at<float>(0);
  float lambda2 = pca.eigenvalues.at<float>(1);
  float stability = lambda1 / (lambda1 + lambda2 + 1e-10f);
  if (stability < config_.pca_stability_threshold) {
    result.reason = RefineFailReason::PCA_UNSTABLE;
    return result;
  }

  // Project bright points onto principal axis.
  cv::Point2f mean(pca.mean.at<float>(0), pca.mean.at<float>(1));
  cv::Point2f dir(pca.eigenvectors.at<float>(0, 0),
                  pca.eigenvectors.at<float>(0, 1));

  float min_proj = std::numeric_limits<float>::max();
  float max_proj = std::numeric_limits<float>::lowest();
  for (const auto& p : bright_points) {
    float proj = (p.x - mean.x) * dir.x + (p.y - mean.y) * dir.y;
    min_proj = std::min(min_proj, proj);
    max_proj = std::max(max_proj, proj);
  }

  // Extend slightly beyond the PCA projection range.
  float extend = (max_proj - min_proj) * 0.1f;
  cv::Point2f top_pt    = mean + (min_proj - extend) * dir;
  cv::Point2f bottom_pt = mean + (max_proj + extend) * dir;

  // Search for the peak gradient along the principal direction.
  // This refines the endpoints to the actual brightness boundary.
  auto searchEdge = [&](cv::Point2f start, cv::Point2f direction,
                         float search_len, int steps) -> cv::Point2f {
    cv::Point2f best = start;
    float best_grad = 0.0f;

    for (int s = -steps; s <= steps; ++s) {
      cv::Point2f pt = start + (search_len * s / steps) * direction;

      int px = static_cast<int>(std::round(pt.x));
      int py = static_cast<int>(std::round(pt.y));
      if (px < 1 || px >= frame.cols - 1 ||
          py < 1 || py >= frame.rows - 1) continue;

      // Gradient along perpendicular direction.
      cv::Point2f perp(-direction.y, direction.x);
      float grad = 0.0f;
      for (int t = -2; t <= 2; ++t) {
        int sx = static_cast<int>(std::round(pt.x + t * perp.x));
        int sy = static_cast<int>(std::round(pt.y + t * perp.y));
        if (sx < 1 || sx >= frame.cols - 1 ||
            sy < 1 || sy >= frame.rows - 1) continue;
        float dx = static_cast<float>(frame.at<uchar>(sy, sx + 1)) -
                   static_cast<float>(frame.at<uchar>(sy, sx - 1));
        grad += std::abs(dx);
      }
      if (grad > best_grad) {
        best_grad = grad;
        best = pt;
      }
    }
    return best;
  };

  float search_range = (max_proj - min_proj) * 0.2f;
  result.top    = searchEdge(top_pt, dir, search_range, 10);
  result.bottom = searchEdge(bottom_pt, -dir, search_range, 10);
  result.ok = true;

  return result;
}

bool RoiPcaCornerRefiner::validateGeometry(
    const std::array<cv::Point2f, 4>& corners)
{
  // Area check.
  double area = cv::contourArea(
    std::vector<cv::Point2f>(corners.begin(), corners.end()));
  if (area < 20.0 || area > static_cast<double>(frame_cols_ * frame_rows_)) {
    return false;
  }

  // Convexity check.
  if (!cv::isContourConvex(
        std::vector<cv::Point2f>(corners.begin(), corners.end()))) {
    return false;
  }

  // Aspect ratio check.
  float w_top    = cv::norm(corners[0] - corners[1]);
  float w_bottom = cv::norm(corners[3] - corners[2]);
  float h_left   = cv::norm(corners[0] - corners[3]);
  float h_right  = cv::norm(corners[1] - corners[2]);

  float avg_w = (w_top + w_bottom) * 0.5f;
  float avg_h = (h_left + h_right) * 0.5f;
  float aspect = avg_w / std::max(1.0f, avg_h);

  if (aspect < config_.min_aspect_ratio ||
      aspect > config_.max_aspect_ratio) {
    return false;
  }

  return true;
}

double RoiPcaCornerRefiner::computeQuality(
    const std::array<cv::Point2f, 4>& refined,
    const std::array<cv::Point2f, 4>& original)
{
  // Higher quality when refined keypoints are close to original
  // (avoid large jumps), scaled by consistency of corner positions.
  double total_dist = 0.0;
  for (int i = 0; i < 4; ++i) {
    total_dist += cv::norm(refined[i] - original[i]);
  }
  double avg_dist = total_dist / 4.0;
  return std::max(0.0, 1.0 - avg_dist / 20.0);
}

RefineResult RoiPcaCornerRefiner::refine(
    const cv::Mat& frame,
    const ArmorDetection& detection)
{
  auto t_start = std::chrono::steady_clock::now();
  RefineResult result;
  result.refined_keypoints = detection.keypoints;

  // Step 1: extract left/right ROI.
  auto [left_roi, right_roi] = extractLightBarROIs(detection.keypoints);

  // Step 2: ROI size gate.
  float min_area = 20.0f;
  if (left_roi.area() < min_area || right_roi.area() < min_area) {
    result.reason = RefineFailReason::ROI_TOO_SMALL;
    return result;
  }

  // Step 3: timeout check.
  if (elapsedMs(t_start) > config_.time_budget_ms) {
    result.reason = RefineFailReason::TIMEOUT;
    return result;
  }

  // Step 4: PCA endpoint refinement for both light bars.
  auto left_ep  = refineLightBar(frame, left_roi, detection.color);
  auto right_ep = refineLightBar(frame, right_roi, detection.color);

  if (!left_ep.ok || !right_ep.ok) {
    result.reason = left_ep.ok ? right_ep.reason : left_ep.reason;
    return result;
  }

  // Step 5: assemble refined keypoints.
  // Order: top-left[0], top-right[1], bottom-right[2], bottom-left[3]
  std::array<cv::Point2f, 4> refined = {{
    left_ep.top,
    right_ep.top,
    right_ep.bottom,
    left_ep.bottom
  }};

  // Step 6: geometry validation.
  if (!validateGeometry(refined)) {
    result.reason = RefineFailReason::GEOMETRY_INVALID;
    return result;
  }

  // Step 7: success.
  result.ok = true;
  result.refined_keypoints = refined;
  result.refine_quality = computeQuality(refined, detection.keypoints);
  result.elapsed_ms = elapsedMs(t_start);
  result.reason = RefineFailReason::NONE;
  return result;
}

}  // namespace fyt::auto_aim
