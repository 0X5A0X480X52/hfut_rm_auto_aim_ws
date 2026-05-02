// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_OBSERVATION_FRONTEND_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_OBSERVATION_FRONTEND_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_runtime_context.hpp"

namespace fyt::auto_aim::outpost_v2 {

struct BindingCandidate {
  int candidate_panel_id = -1;
  double candidate_prob = 0.0;
  double candidate_margin = 0.0;
  double selected_yaw_err = std::numeric_limits<double>::quiet_NaN();
  double selected_xy_residual = std::numeric_limits<double>::quiet_NaN();
  double z_jump = std::numeric_limits<double>::quiet_NaN();
  bool has_z_jump = false;
  double entropy_norm = 1.0;
  double max_prob = 0.0;
  std::array<double, 3> costs{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
  std::array<double, 3> probs{{
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()}};
};

class ObservationFrontend {
 public:
  explicit ObservationFrontend(const UnifiedConfig & cfg);

  const ObservationData * select_primary_observation(
      const std::vector<ObservationData> & obs,
      const OutpostRuntimeContext & ctx) const;

  BindingCandidate build_binding_candidate(
      const ObservationData & obs,
      const OutpostRuntimeContext & ctx) const;

 private:
  UnifiedConfig cfg_;
  double radius_ = 0.26;
  std::array<double, 3> z_offsets_{{0.06, 0.0, -0.06}};
  std::array<double, 3> panel_angles_{{0.0, 2.0 * M_PI / 3.0, -2.0 * M_PI / 3.0}};
};

}  // namespace fyt::auto_aim::outpost_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_OBSERVATION_FRONTEND_HPP_
