// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_BACKEND_INTERFACE_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_BACKEND_INTERFACE_HPP_

#include <Eigen/Dense>

#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/filters/spin_filter_interface.hpp"
#include "max_entropy_tracker/trackers/norm4_v3/hypothesis/norm4_hypothesis_types.hpp"

namespace fyt::auto_aim::norm4_v3 {

struct BackendSnapshot {
  Eigen::VectorXd x;
  Eigen::MatrixXd P;
  int k = 0;
  int last_k = 0;
  int current_panel_id = -1;
  double last_nis = -1.0;
  Eigen::VectorXd last_innov_xyz;
  double last_innov_yaw = 0.0;
  int last_update_type = 0;
};

class IStructuredBackend {
 public:
  virtual ~IStructuredBackend() = default;

  virtual void reset(const ObservationData &obs, int panel_id, double r1,
                     double r2, double dza) = 0;
  virtual void predict(double dt) = 0;
  virtual bool initialized() const = 0;

  virtual PredictContext buildPredictContext() const = 0;

  virtual MeasurementEval evaluateSingle(const PredictContext &ctx,
                                          const ObservationData &obs,
                                          int panel_id) const = 0;

  virtual MeasurementEval evaluateDual(const PredictContext &ctx,
                                        const ObservationData &obs0,
                                        const ObservationData &obs1,
                                        int panel_id_0,
                                        int panel_id_1) const = 0;

  virtual UkfTrial tryUpdateSingle(const PredictContext &ctx,
                                    const ObservationData &obs,
                                    int panel_id) const = 0;

  virtual UkfTrial tryUpdateDual(const PredictContext &ctx,
                                  const ObservationData &obs0,
                                  const ObservationData &obs1,
                                  int panel_id_0, int panel_id_1) const = 0;

  virtual void commit(const UkfTrial &trial) = 0;

  virtual BackendSnapshot snapshot() const = 0;

  virtual SpinFilterInterface &spin_filter() = 0;
  virtual const SpinFilterInterface &spin_filter() const = 0;
};

}  // namespace fyt::auto_aim::norm4_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_NORM4_V3_NORM4_BACKEND_INTERFACE_HPP_
