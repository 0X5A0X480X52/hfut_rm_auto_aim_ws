// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_ASSOCIATION_PANEL_MISMATCH_DETECTOR_HPP_
#define MAX_ENTROPY_TRACKER_ASSOCIATION_PANEL_MISMATCH_DETECTOR_HPP_

#include <cmath>
#include <cstdio>
#include <deque>
#include <numeric>
#include <string>

namespace fyt::auto_aim {

/**
 * Detects panel_id mismatches by comparing the z-cost under two hypotheses:
 *   H0 (current):  armor is at the layer implied by current panel parity
 *   H1 (flipped):  armor is at the opposite layer (panel_id ^ 1)
 *
 * Algorithm (5 steps, executed each single-observation update):
 *   Step 1 — push z observation and compute dz² into rolling window.
 *   Step 2 — if dz²_mean < T1  →  z looks fine, reset suspect counter.
 *   Step 3 — compute J_current and J_flipped over the window.
 *   Step 4 — select the hypothesis with the smaller accumulated cost.
 *   Step 5 — if selected parity != current parity, increment consecutive_suspect_;
 *             when consecutive_suspect_ reaches confirm_count  →  patch correction;
 *             when consecutive_suspect_ reaches reinit_count   →  re-initialize.
 *
 * Note: detection is suppressed while dza has not converged, because with
 * dza ≈ 0 both hypotheses produce the same prediction and cannot be compared.
 */
class PanelMismatchDetector {
 public:
  /// Classification returned to the caller.
  enum class Action {
    NONE    = 0,  ///< No mismatch detected; keep state as-is
    PATCH   = 1,  ///< Moderate confidence: apply in-place state correction
    REINIT  = 2,  ///< High confidence: full re-initialization recommended
  };

  struct Result {
    Action action        = Action::NONE;
    int    new_panel_id  = -1;   ///< Suggested panel_id after correction
    double confidence    = 0.0;  ///< [0, 1]
  };

  /**
   * @param window_size    Rolling buffer length W (default 8 frames)
   * @param threshold_t1   dz² mean below which z is considered OK  (default 0.0009 = 3cm²)
   * @param confirm_count  Consecutive suspect frames to trigger PATCH (default 3)
   * @param reinit_count   Consecutive suspect frames to trigger REINIT (default 5)
   * @param enabled        Master enable flag
   */
  explicit PanelMismatchDetector(int window_size  = 8,
                                 double threshold_t1 = 0.1,
                                 int confirm_count   = 3,
                                 int reinit_count    = 5,
                                 bool enabled        = true)
      : W_(window_size),
        T1_(threshold_t1),
        N_confirm_(confirm_count),
        N_reinit_(reinit_count),
        enabled_(enabled) {}

  /**
   * Push a new frame and run the 5-step detection.
   *
   * @param panel_id      Current panel_id assigned by PanelAssociator
   * @param z_obs         Observed armor z (world frame)
   * @param z_mean        UKF state: robot center z  (x_(Z))
   * @param dza           UKF state: dza half-offset  (x_(DZA))
   * @param armor_layer   Layer string assigned this frame ("upper"/"lower"/"")
   * @param dza_converged Whether dza is considered converged by the UKF
   * @param z_innov       z-dimension innovation (z_obs - z_predicted),
   *                      taken from UKF innov(2) to avoid recomputation
   */
  Result update(int panel_id, double z_obs, double z_mean, double dza,
                const std::string &armor_layer, bool dza_converged,
                double z_innov) {
    if (!enabled_) return {};

    // Suppressed if dza not converged: J_current ≈ J_flipped → high false-positive risk
    if (!dza_converged || dza < 0.005) {
      buffer_.clear();
      consecutive_suspect_ = 0;
      return {};
    }

    // ── Step 1: compute predicted z and dz² ──
    // current layer determines the sign of dza offset
    bool current_is_upper = (armor_layer == "upper");
    double z_pred_current = z_mean + (current_is_upper ? dza : -dza);
    double dz = z_obs - z_pred_current;
    double dz_sq = dz * dz;

    // For the flipped hypothesis the layer is inverted
    bool flipped_is_upper  = !current_is_upper;
    double z_pred_flipped  = z_mean + (flipped_is_upper ? dza : -dza);

    FrameRecord rec;
    rec.panel_id        = panel_id;
    rec.z_obs           = z_obs;
    rec.z_mean          = z_mean;
    rec.dza             = dza;
    rec.current_is_upper = current_is_upper;
    rec.dz_sq           = dz_sq;
    rec.z_pred_flipped  = z_pred_flipped;

    buffer_.push_back(rec);
    if (static_cast<int>(buffer_.size()) > W_) buffer_.pop_front();

    if (static_cast<int>(buffer_.size()) < W_) return {};  // warm-up

    // ── Step 2: coarse filter ──
    double dz_sq_mean = compute_dz_sq_mean();
    if (dz_sq_mean < T1_) {
      consecutive_suspect_ = 0;
      return {};
    }

    // ── Step 3: compute J_current and J_flipped ──
    double J_current = 0.0, J_flipped = 0.0;
    for (const auto &f : buffer_) {
      J_current += f.dz_sq;
      double d_flip = f.z_obs - f.z_pred_flipped;
      J_flipped += d_flip * d_flip;
    }

    // ── Step 4: hypothesis selection ──
    bool prefer_flipped = (J_flipped < J_current);

    // Require a meaningful cost advantage (> 20%) to avoid noise-driven flips
    double ratio = (J_current > 1e-12)
                       ? (J_current - J_flipped) / J_current
                       : 0.0;
    if (prefer_flipped && ratio < 0.20) {
      // Cost difference too small → inconclusive
      consecutive_suspect_ = 0;
      return {};
    }

    // ── Step 5: parity check ──
    int current_parity  = panel_id % 2;
    int selected_parity = prefer_flipped ? (1 - current_parity) : current_parity;

    if (current_parity != selected_parity) {
      ++consecutive_suspect_;
    } else {
      consecutive_suspect_ = 0;
      return {};
    }

    // Determine action
    if (consecutive_suspect_ >= N_reinit_) {
      int new_pid = panel_id ^ 1;
      double conf = std::min(1.0, 0.8 + ratio * 0.2);
      std::fprintf(stderr,
          "[PanelMismatchDetector] REINIT: panel %d→%d  "
          "J_cur=%.5f J_flip=%.5f ratio=%.2f dz²_mean=%.5f\n",
          panel_id, new_pid, J_current, J_flipped, ratio, dz_sq_mean);
      consecutive_suspect_ = 0;
      buffer_.clear();
      return {Action::REINIT, new_pid, conf};
    }

    if (consecutive_suspect_ >= N_confirm_) {
      int new_pid = panel_id ^ 1;
      double conf = std::min(1.0, 0.6 + ratio * 0.4);
      std::fprintf(stderr,
          "[PanelMismatchDetector] PATCH: panel %d→%d  "
          "J_cur=%.5f J_flip=%.5f ratio=%.2f dz²_mean=%.5f\n",
          panel_id, new_pid, J_current, J_flipped, ratio, dz_sq_mean);
      // Keep consecutive_suspect_ accumulating toward REINIT threshold
      return {Action::PATCH, new_pid, conf};
    }

    return {};
  }

  void reset() {
    buffer_.clear();
    consecutive_suspect_ = 0;
  }

  bool is_enabled() const { return enabled_; }
  void set_enabled(bool v) { enabled_ = v; }

 private:
  struct FrameRecord {
    int    panel_id;
    double z_obs;
    double z_mean;
    double dza;
    bool   current_is_upper;
    double dz_sq;
    double z_pred_flipped;
  };

  double compute_dz_sq_mean() const {
    if (buffer_.empty()) return 0.0;
    double s = 0.0;
    for (const auto &f : buffer_) s += f.dz_sq;
    return s / static_cast<double>(buffer_.size());
  }

  int    W_;
  double T1_;
  int    N_confirm_;
  int    N_reinit_;
  bool   enabled_;

  std::deque<FrameRecord> buffer_;
  int consecutive_suspect_ = 0;
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_ASSOCIATION_PANEL_MISMATCH_DETECTOR_HPP_
