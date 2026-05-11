// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_UKF_BACKEND_V1_ADAPTER_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_UKF_BACKEND_V1_ADAPTER_HPP_

#include "max_entropy_tracker/trackers/norm4_v3/interfaces/norm4_backend_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/backends/norm4_ukf_backend_v1.hpp"

namespace fyt::auto_aim::norm4_v3 {

class UkfBackendV1Adapter : public IStructuredBackend {
 public:
  explicit UkfBackendV1Adapter(const UnifiedConfig &config, double dt = 0.05)
      : v1_(config, dt) {}

  void reset(const ObservationData &obs, int panel_id, double r1, double r2,
             double dza) override {
    v1_.reset(obs, panel_id, r1, r2, dza);
  }

  void predict(double dt) override { v1_.predict(dt); }

  bool initialized() const override { return v1_.initialized(); }

  PredictContext buildPredictContext() const override {
    return v1_.buildPredictContext();
  }

  MeasurementEval evaluateSingle(const PredictContext &ctx,
                                  const ObservationData &obs,
                                  int panel_id) const override {
    return v1_.evaluateSingle(ctx, obs, panel_id);
  }

  MeasurementEval evaluateDual(const PredictContext &ctx,
                                const ObservationData &obs0,
                                const ObservationData &obs1,
                                int panel_id_0, int panel_id_1) const override {
    return v1_.evaluateDual(ctx, obs0, obs1, panel_id_0, panel_id_1);
  }

  UkfTrial tryUpdateSingle(const PredictContext &ctx,
                            const ObservationData &obs,
                            int panel_id) const override {
    return v1_.tryUpdateSingle(ctx, obs, panel_id);
  }

  UkfTrial tryUpdateDual(const PredictContext &ctx,
                          const ObservationData &obs0,
                          const ObservationData &obs1,
                          int panel_id_0, int panel_id_1) const override {
    return v1_.tryUpdateDual(ctx, obs0, obs1, panel_id_0, panel_id_1);
  }

  void commit(const UkfTrial &trial) override { v1_.commit(trial); }

  BackendSnapshot snapshot() const override {
    BackendSnapshot snap;
    snap.x = v1_.x();
    snap.P = v1_.P();
    snap.k = v1_.get_k();
    snap.last_k = v1_.get_k();
    snap.current_panel_id = -1;
    snap.last_nis = v1_.last_nis();
    snap.last_innov_xyz = v1_.last_innov_xyz();
    snap.last_innov_yaw = v1_.last_innov_yaw();
    snap.last_update_type = v1_.last_update_type();
    return snap;
  }

  SpinFilterInterface &spin_filter() override { return v1_; }
  const SpinFilterInterface &spin_filter() const override { return v1_; }

  Norm4UkfBackendV1 &v1() { return v1_; }
  const Norm4UkfBackendV1 &v1() const { return v1_; }

 private:
  Norm4UkfBackendV1 v1_;
};

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_UKF_BACKEND_V1_ADAPTER_HPP_
