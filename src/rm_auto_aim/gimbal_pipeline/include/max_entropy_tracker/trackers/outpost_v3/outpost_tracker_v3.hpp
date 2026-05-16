// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_TRACKER_V3_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_TRACKER_V3_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/hypothesis/norm4_hypothesis_types.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/interfaces/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/outpost_v3/outpost_hypothesis_generator.hpp"
#include "max_entropy_tracker/trackers/outpost_v3/outpost_hypothesis_types.hpp"
#include "max_entropy_tracker/trackers/outpost_v3/outpost_inekf_backend.hpp"

namespace fyt::auto_aim {

class OutpostTrackerV3 : public BaseTracker {
 public:
  struct DebugSnapshot {
    bool valid = false;
    int mode_state = 1;  // 0=STRUCTURED, 1=AMBIGUOUS
    int current_panel_id = -1;
    int candidate_panel_id = -1;
    double candidate_prob = 0.0;
    double candidate_margin = 0.0;
    double entropy_norm = 1.0;
    double max_prob = 0.0;
    bool committed = false;
    double top1_confidence = 0.0;
    double top1_top2_margin = 0.0;
    double top1_nis = -1.0;
    std::string decision_reason;
    int consecutive_degraded = 0;
    int consecutive_stable = 0;
  };

  explicit OutpostTrackerV3(const outpost_v3::OutpostV3Config &cfg,
                            double dt = 0.05);

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

  const DebugSnapshot &debug_snapshot() const { return debug_snapshot_; }

 private:
  void select_topk(
      std::vector<norm4_v3::MeasurementEval> &evals,
      const std::vector<outpost_v3::OutpostHypothesis> &hyps,
      int topk_count,
      std::vector<norm4_v3::TopKEntry> *topk_out,
      double *confidence_out, double *margin_out) const;

  void update_mode_routing(double confidence, double margin, bool committed);
  void populate_debug_snapshot(
      bool committed, const std::vector<norm4_v3::TopKEntry> &topk,
      double top1_confidence, double top1_top2_margin,
      const std::string &decision_reason);

  outpost_v3::OutpostV3Config cfg_;
  std::unique_ptr<outpost_v3::OutpostInEKFBackend> backend_;

  outpost_v3::OutpostHypothesisGenerator hypothesis_generator_;

  outpost_v3::OutpostV3Mode mode_ = outpost_v3::OutpostV3Mode::AMBIGUOUS;
  int current_panel_id_ = -1;
  int consecutive_degraded_ = 0;
  int consecutive_stable_ = 0;

  DebugSnapshot debug_snapshot_{};
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_TRACKER_V3_HPP_
