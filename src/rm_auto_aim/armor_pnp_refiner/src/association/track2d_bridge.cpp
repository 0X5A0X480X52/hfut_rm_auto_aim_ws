#include "armor_pnp_refiner/association/track2d_bridge.hpp"

namespace armor_pnp_refiner {

void Track2dBridge::attachTrackId(PnpRefineInput& input, int track2d_id)
{
  input.external_track_id = track2d_id;
}

bool Track2dBridge::hasExternalTrackId(const PnpRefineInput& input)
{
  return input.external_track_id.has_value();
}

}  // namespace armor_pnp_refiner
