#pragma once

#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"

namespace armor_pnp_refiner {

// Bridge to convert armor_detector_nn 2D tracker results into refiner input.
// Passes through external_track_id when available from the NN detector's
// 2D tracker output.
class Track2dBridge {
public:
  Track2dBridge() = default;

  // Attach external track_id to a refiner input.
  // Call this when armor_detector_nn provides a track_id.
  static void attachTrackId(PnpRefineInput& input, int track2d_id);

  // Check if the input has a usable external track id.
  static bool hasExternalTrackId(const PnpRefineInput& input);
};

}  // namespace armor_pnp_refiner
