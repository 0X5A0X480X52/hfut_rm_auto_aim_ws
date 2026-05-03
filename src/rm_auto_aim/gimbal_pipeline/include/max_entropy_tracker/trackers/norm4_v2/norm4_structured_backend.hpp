// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_STRUCTURED_BACKEND_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_STRUCTURED_BACKEND_HPP_

#include <string>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/filters/dual_radius_spin_ukf.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_backend_interface.hpp"

namespace fyt::auto_aim::norm4_v2 {

class Norm4StructuredBackend : public INorm4Backend {
 public:
  Norm4StructuredBackend(const UnifiedConfig &cfg, double dt);

  void reset(const ObservationData &obs, int panel_id, double r1, double r2,
             double dza) override;
  void predict(double dt) override;
  bool update(const ObservationData &obs,
              const BackendUpdateHint &hint) override;
  BackendStateSnapshot snapshot() const override;

  bool update_dual(const ObservationData &obs1, const ObservationData &obs2,
                   int panel_id_1, int panel_id_2, const std::string &layer_1,
                   const std::string &layer_2, double height_confidence);

  void apply_panel_correction(int new_panel_id, double armor_yaw);

  DualRadiusSpinUKF &ukf() { return ukf_; }
  const DualRadiusSpinUKF &ukf() const { return ukf_; }
  int current_panel_id() const { return current_panel_id_; }
  bool initialized() const { return initialized_; }

 private:
  static std::string panel_r_type(int panel_id);
  static std::string default_layer(int panel_id);
  static std::string layer_from_label(binder::HeightLabel label, int panel_id);

  UnifiedConfig cfg_;
  DualRadiusSpinUKF ukf_;
  bool initialized_ = false;
  int current_panel_id_ = 0;
  double default_r1_ = 0.15;
  double default_r2_ = 0.20;
  double default_dza_ = 0.0;
};

}  // namespace fyt::auto_aim::norm4_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_STRUCTURED_BACKEND_HPP_
