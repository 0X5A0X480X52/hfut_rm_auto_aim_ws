// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_STRUCTURED_BACKEND_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_STRUCTURED_BACKEND_HPP_

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/outpost_spin_ukf.hpp"
#include "max_entropy_tracker/trackers/outpost_v2/outpost_backend_interface.hpp"

namespace fyt::auto_aim::outpost_v2 {

class OutpostStructuredBackend : public IOutpostBackend {
 public:
  OutpostStructuredBackend(const UnifiedConfig & cfg, double dt);

  void reset(const ObservationData & obs, int panel_id) override;
  void predict(double dt) override;
  bool update(const ObservationData & obs,
              const BackendUpdateHint & hint) override;
  BackendStateSnapshot snapshot() const override;

  OutpostSpinUKF & ukf() { return ukf_; }
  const OutpostSpinUKF & ukf() const { return ukf_; }

 private:
  int current_panel_id_ = 0;
  OutpostSpinUKF ukf_;
  bool initialized_ = false;
};

}  // namespace fyt::auto_aim::outpost_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V2_OUTPOST_STRUCTURED_BACKEND_HPP_
