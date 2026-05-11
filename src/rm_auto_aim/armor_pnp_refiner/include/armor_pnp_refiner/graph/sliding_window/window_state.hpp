#pragma once

#include <vector>
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"

namespace armor_pnp_refiner {

// Per-frame entry in a sliding window.
struct WindowFrame {
  int frame_index{-1};
  double stamp_sec{0.0};
  PnpRefineInput input;

  // Initial estimate for this frame's xyz-yaw state.
  Eigen::Vector4d xyz_yaw_initial{Eigen::Vector4d::Zero()};

  // After optimization: refined state.
  Eigen::Vector4d xyz_yaw_refined{Eigen::Vector4d::Zero()};
  bool refined{false};
};

// Sliding window state for a single refine_track_id.
struct RefineWindow {
  int refine_track_id{-1};
  std::vector<WindowFrame> frames;
  int hits{0};
  int misses{0};
  bool confirmed{false};
  double last_stamp_sec{0.0};

  bool isConfirmed() const { return hits >= 2; }
  bool isExpired(double max_gap_ms, double now_sec) const {
    return (now_sec - last_stamp_sec) * 1000.0 > max_gap_ms;
  }
  int size() const { return static_cast<int>(frames.size()); }
};

}  // namespace armor_pnp_refiner
