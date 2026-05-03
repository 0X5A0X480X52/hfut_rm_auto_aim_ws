// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_OBSERVATION_FRONTEND_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_OBSERVATION_FRONTEND_HPP_

#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "max_entropy_tracker/association/panel_associator.hpp"
#include "max_entropy_tracker/binder/model/binder_enums.hpp"
#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/norm4_v2/norm4_runtime_context.hpp"

namespace fyt::auto_aim::norm4_v2 {

struct BindingCandidate {
  int candidate_panel_id = -1;
  binder::HeightLabel candidate_height_label = binder::HeightLabel::UNKNOWN;
  double height_confidence = 0.0;

  double candidate_prob = 0.0;
  double candidate_margin = 0.0;
  double selected_yaw_err = std::numeric_limits<double>::quiet_NaN();
  double selected_xy_residual = std::numeric_limits<double>::quiet_NaN();
  double cost_margin = std::numeric_limits<double>::quiet_NaN();
  double z_jump = std::numeric_limits<double>::quiet_NaN();
  bool has_z_jump = false;
  double entropy_norm = 1.0;
  double max_prob = 0.0;

  std::vector<int> obs_panel_ids;
  std::vector<binder::HeightLabel> obs_height_labels;

  PanelAssociator::AssociationDiagnostics assoc_diag{};
};

struct DualObservationAssignment {
  bool valid = false;
  int panel_id_1 = -1;
  int panel_id_2 = -1;
  binder::HeightLabel label_1 = binder::HeightLabel::UNKNOWN;
  binder::HeightLabel label_2 = binder::HeightLabel::UNKNOWN;
  std::string layer_1;
  std::string layer_2;
  double height_confidence = 0.0;
  double cost = std::numeric_limits<double>::quiet_NaN();
};

class ObservationFrontend {
 public:
  explicit ObservationFrontend(const UnifiedConfig &cfg);

  void reset_history();

  const ObservationData *select_primary_observation(
      const std::vector<ObservationData> &obs,
      const Norm4RuntimeContext &ctx) const;

  BindingCandidate build_binding_candidate(const ObservationData &obs,
                                           const Norm4RuntimeContext &ctx);

  int infer_panel_for_observation(const ObservationData &obs,
                                  const Norm4RuntimeContext &ctx,
                                  PanelAssociator::AssociationDiagnostics *diag =
                                      nullptr);

  DualObservationAssignment assign_dual_observations(
      const ObservationData &obs1, const ObservationData &obs2,
      const Norm4RuntimeContext &ctx) const;

  void apply_forced_assignment(BindingCandidate *candidate,
                               const ObservationData &obs,
                               const Norm4RuntimeContext &ctx,
                               int panel_id,
                               binder::HeightLabel label) const;

 private:
  binder::HeightLabel default_height_label(int panel_id) const;
  static std::string layer_from_label(binder::HeightLabel label);
  double radius_for_panel(int panel_id, const Norm4RuntimeContext &ctx) const;

  UnifiedConfig cfg_;
  PanelAssociator panel_associator_;
  std::array<double, 4> panel_angles_{{0.0, M_PI / 2.0, M_PI, -M_PI / 2.0}};
};

}  // namespace fyt::auto_aim::norm4_v2

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V2_NORM4_OBSERVATION_FRONTEND_HPP_
