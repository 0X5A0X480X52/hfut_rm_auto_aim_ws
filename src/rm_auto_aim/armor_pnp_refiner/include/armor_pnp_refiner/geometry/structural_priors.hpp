#pragma once

#include <cmath>
#include "armor_pnp_refiner/core/pnp_refiner_config.hpp"
#include "armor_pnp_refiner/core/pnp_refiner_types.hpp"

namespace armor_pnp_refiner {
namespace geometry {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// Phase1/Phase2 temporary structural prior:
// armor roll in odom is assumed 0 deg.
// armor pitch in odom is assumed +15 deg for outpost and -15 deg for others.
// TODO: move these values to armor_pnp_refiner config after the first version is validated.
inline double getStructuralRollRad(const PnpRefinerConfig& config) {
  return config.roll_odom_deg * kDegToRad;
}

inline double getStructuralPitchRad(const PnpRefinerConfig& config,
                                     ArmorSizeType type) {
  double deg = (type == ArmorSizeType::OUTPOST)
    ? config.pitch_odom_outpost_deg
    : config.pitch_odom_default_deg;
  return deg * kDegToRad;
}

}  // namespace geometry
}  // namespace armor_pnp_refiner
