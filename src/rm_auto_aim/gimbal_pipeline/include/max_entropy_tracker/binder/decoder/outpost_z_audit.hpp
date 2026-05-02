// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_DECODER_OUTPOST_Z_AUDIT_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_DECODER_OUTPOST_Z_AUDIT_HPP_

#include <array>
#include <limits>

#include "max_entropy_tracker/binder/model/outpost_binding_types.hpp"
#include "max_entropy_tracker/core/observation.hpp"

namespace fyt::auto_aim::binder {

class OutpostZAudit {
 public:
  explicit OutpostZAudit(const std::array<double, 3> & z_offsets);

  void reset();
  OutpostZAuditResult update(const ObservationData & obs);

  double confidence() const { return confidence_; }

 private:
  std::array<double, 3> z_offsets_;
  bool initialized_ = false;
  double center_est_ = std::numeric_limits<double>::quiet_NaN();
  double prev_obs_z_ = std::numeric_limits<double>::quiet_NaN();
  int prev_panel_id_ = -1;
  double confidence_ = 0.0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_DECODER_OUTPOST_Z_AUDIT_HPP_
