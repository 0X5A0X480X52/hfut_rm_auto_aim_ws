// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_TRACKER_V2_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_TRACKER_V2_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/norm4_hypothesis_generator.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/norm4_hypothesis_types.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/norm4_ukf_backend_v1.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"

namespace fyt::auto_aim {

class Norm4ArmorTrackerV2 : public BaseTracker {
 public:
  explicit Norm4ArmorTrackerV2(const UnifiedConfig &config, double dt = 0.05,
                               bool enable_oscillation = false);

  void initialize(const std::vector<ObservationData> &obs, double r1 = 0.15,
                  double r2 = 0.20, double dza = 0.0) override;
  void predict(std::optional<double> target_time = std::nullopt) override;
  bool update(const std::vector<ObservationData> &obs) override;

  Eigen::Vector3d get_center_position() const override;
  double get_yaw() const override;
  std::pair<double, double> get_radii() const override;
  SpinFilterInterface &spin_filter() override;
  const SpinFilterInterface &spin_filter() const override;
  ManeuverResult assess_maneuver() const override;

  Eigen::Vector3d get_publish_velocity() const override;
  bool is_ambiguous_single_mode() const override;
  bool supports_ambiguous_single_semantics() const override { return true; }
  int effective_num_armors() const override;
  double confidence_scale() const override;
  std::vector<geometry_msgs::msg::Pose> build_armors_offset_for_message() const override;

  const norm4_v3::HypothesisDebugFrame &last_hypothesis_debug() const {
    return last_hypothesis_debug_;
  }

 private:
  void select_topk(std::vector<norm4_v3::MeasurementEval> &evals,
                   const std::vector<norm4_v3::Hypothesis> &hyps,
                   int topk_count,
                   std::vector<norm4_v3::TopKEntry> *topk_out,
                   double *confidence_out, double *margin_out) const;

  UnifiedConfig config_;
  std::unique_ptr<norm4_v3::Norm4UkfBackendV1> backend_;
  norm4_v3::HypothesisGenerator hypothesis_generator_;
  ManeuverDetector maneuver_detector_;

  double default_r1_ = 0.15;
  double default_r2_ = 0.20;
  double default_dza_ = 0.0;

  int current_panel_id_ = -1;

  norm4_v3::HypothesisDebugFrame last_hypothesis_debug_{};
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_TRACKER_V2_HPP_
