// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_UKF_BACKEND_BASELINE_ADAPTER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_UKF_BACKEND_BASELINE_ADAPTER_HPP_

#include "max_entropy_tracker/trackers/norm4_baseline/interfaces/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_baseline/backends/norm4_ukf_backend_baseline.hpp"

namespace fyt::auto_aim::norm4_baseline {

class UkfBackendBaselineAdapter : public IStructuredBackend {
 public:
  explicit UkfBackendBaselineAdapter(const UnifiedConfig &config, double dt = 0.05)
      : baseline_(config, dt) {}

  void reset(const ObservationData &obs, int panel_id, double r1, double r2,
             double dza) override {
    baseline_.reset(obs, panel_id, r1, r2, dza);
  }

  void predict(double dt) override { baseline_.predict(dt); }

  bool initialized() const override { return baseline_.initialized(); }

  PredictContext buildPredictContext() const override {
    return baseline_.buildPredictContext();
  }

  MeasurementEval evaluateSingle(const PredictContext &ctx,
                                  const ObservationData &obs,
                                  int panel_id) const override {
    return baseline_.evaluateSingle(ctx, obs, panel_id);
  }

  MeasurementEval evaluateDual(const PredictContext &ctx,
                                const ObservationData &obs0,
                                const ObservationData &obs1,
                                int panel_id_0, int panel_id_1) const override {
    return baseline_.evaluateDual(ctx, obs0, obs1, panel_id_0, panel_id_1);
  }

  UkfTrial tryUpdateSingle(const PredictContext &ctx,
                            const ObservationData &obs,
                            int panel_id) const override {
    return baseline_.tryUpdateSingle(ctx, obs, panel_id);
  }

  UkfTrial tryUpdateDual(const PredictContext &ctx,
                          const ObservationData &obs0,
                          const ObservationData &obs1,
                          int panel_id_0, int panel_id_1) const override {
    return baseline_.tryUpdateDual(ctx, obs0, obs1, panel_id_0, panel_id_1);
  }

  void commit(const UkfTrial &trial) override { baseline_.commit(trial); }

  BackendSnapshot snapshot() const override {
    BackendSnapshot snap;
    snap.x = baseline_.x();
    snap.P = baseline_.P();
    snap.k = baseline_.get_k();
    snap.last_k = baseline_.get_k();
    snap.current_panel_id = -1;
    snap.last_nis = baseline_.last_nis();
    snap.last_innov_xyz = baseline_.last_innov_xyz();
    snap.last_innov_yaw = baseline_.last_innov_yaw();
    snap.last_update_type = baseline_.last_update_type();
    return snap;
  }

  SpinFilterInterface &spin_filter() override { return baseline_; }
  const SpinFilterInterface &spin_filter() const override { return baseline_; }

  Norm4UkfBackendBaseline &baseline() { return baseline_; }
  const Norm4UkfBackendBaseline &baseline() const { return baseline_; }

 private:
  Norm4UkfBackendBaseline baseline_;
};

}  // namespace fyt::auto_aim::norm4_baseline

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_BASELINE_NORM4_UKF_BACKEND_BASELINE_ADAPTER_HPP_
