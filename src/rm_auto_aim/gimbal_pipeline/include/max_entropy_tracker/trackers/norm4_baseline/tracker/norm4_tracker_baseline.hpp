// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_TRACKER_BASELINE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_TRACKER_BASELINE_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "max_entropy_tracker/core/config.hpp"
#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/evidence/evidence_builder.hpp"
#include "max_entropy_tracker/trackers/base_tracker.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/hypothesis/norm4_hypothesis_generator.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/hypothesis/norm4_hypothesis_types.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/interfaces/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/backends/norm4_backend_factory.hpp"
#include "max_entropy_tracker/utils/maneuver_detector.hpp"

namespace fyt::auto_aim {

class Norm4TrackerBaseline : public BaseTracker {
 public:
  explicit Norm4TrackerBaseline(const UnifiedConfig &config, double dt = 0.05,
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

  const norm4_baseline::HypothesisDebugFrame &last_hypothesis_debug() const {
    return last_hypothesis_debug_;
  }

  const norm4_baseline::BaselineDebugSnapshot &debug_snapshot() const { return debug_snapshot_; }
  const evidence::ArmorEvidenceFrame &last_evidence_frame() const { return evidence_frame_; }
  norm4_baseline::Norm4BaselineMode current_mode() const { return mode_; }

 private:
  void populate_debug_snapshot();

  // Warmup (0/1 dual-seed)
  void init_warmup(const std::vector<ObservationData> &obs, double r1, double r2, double dza);
  bool run_warmup(const std::vector<ObservationData> &obs);
  void promote_warmup_winner();

  // Mode routing
  void set_mode(norm4_baseline::Norm4BaselineMode m);
  void apply_mode_routing();
  void select_topk(std::vector<norm4_baseline::MeasurementEval> &evals,
                   const std::vector<norm4_baseline::Hypothesis> &hyps,
                   int topk_count,
                   std::vector<norm4_baseline::TopKEntry> *topk_out,
                   double *confidence_out, double *margin_out) const;

  UnifiedConfig config_;
  std::unique_ptr<norm4_baseline::IStructuredBackend> backend_;
  norm4_baseline::HypothesisGenerator hypothesis_generator_;
  ManeuverDetector maneuver_detector_;

  double default_r1_ = 0.15;
  double default_r2_ = 0.20;
  double default_dza_ = 0.0;
  std::optional<ObservationData> warmup_last_obs_;

  int current_panel_id_ = -1;
  norm4_baseline::Norm4BaselineMode mode_ = norm4_baseline::Norm4BaselineMode::AMBIGUOUS;
  norm4_baseline::WarmupState warmup_state_{};

  norm4_baseline::HypothesisDebugFrame last_hypothesis_debug_{};
  norm4_baseline::BaselineDebugSnapshot debug_snapshot_{};
  evidence::ArmorEvidenceFrame evidence_frame_{};
};

}  // namespace fyt::auto_aim

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_TRACKER_BASELINE_HPP_
