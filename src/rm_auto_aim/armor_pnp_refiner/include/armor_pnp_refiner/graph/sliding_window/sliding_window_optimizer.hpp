#pragma once

#include <vector>

#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"
#include "armor_pnp_refiner/graph/sliding_window/window_state.hpp"

namespace armor_pnp_refiner {

// Phase2: Sliding window xyz-yaw g2o optimizer.
// Uses trailing window: only past frames, no future frames.
// Outputs refined PnP for the current (last) frame in window.
class SlidingWindowOptimizer {
public:
  explicit SlidingWindowOptimizer(const PnpRefinerConfig& config);

  // Run optimization on a window of frames.
  // Returns refined output for the last frame.
  PnpRefineOutput refine(const std::vector<WindowFrame>& window);

private:
  PnpRefinerConfig config_;
};

// Manages per-track sliding windows.
class PnpRefineWindowManager {
public:
  explicit PnpRefineWindowManager(const PnpRefinerConfig& config);

  // Add a new frame to the window for a given track.
  void push(int refine_track_id, const PnpRefineInput& input);

  // Get the current window frames for a track.
  std::vector<WindowFrame> getWindow(int refine_track_id) const;

  // Reset (clear) a specific track's window.
  void resetTrack(int refine_track_id);

  // Remove expired tracks based on time gap.
  void pruneExpired(double now_sec);

  // Reset all windows.
  void resetAll();

  // Get window size for a track.
  int windowSize(int refine_track_id) const;

private:
  PnpRefinerConfig config_;
  std::vector<RefineWindow> windows_;

  RefineWindow* findWindow(int refine_track_id);
  const RefineWindow* findWindow(int refine_track_id) const;
  RefineWindow& getOrCreateWindow(int refine_track_id);
};

}  // namespace armor_pnp_refiner
