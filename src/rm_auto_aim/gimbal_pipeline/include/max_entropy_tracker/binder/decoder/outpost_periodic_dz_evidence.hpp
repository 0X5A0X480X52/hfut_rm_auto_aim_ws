// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_DECODER_OUTPOST_PERIODIC_DZ_EVIDENCE_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_DECODER_OUTPOST_PERIODIC_DZ_EVIDENCE_HPP_

#include <array>
#include <deque>
#include <limits>

#include "max_entropy_tracker/binder/model/binding_hypothesis.hpp"
#include "max_entropy_tracker/core/config.hpp"

namespace fyt::auto_aim::binder {

class OutpostPeriodicDzEvidence {
 public:
  OutpostPeriodicDzEvidence(const UnifiedConfig & config,
                            const std::array<double, 3> & z_offsets);

  void reset();
  void update(double z_jump, bool allow_model_update, double yaw_rate_est);
  void apply_prior(std::array<BindingHypothesis, 3> & hyps, double z_jump,
                   int bound_panel_id) const;

  double confidence() const { return period_confidence_; }
  int phase() const { return period_phase_index_; }
  int spin_direction() const { return spin_direction_; }
  double dz_small_est() const { return dz_small_est_; }
  double dz_large_est() const { return dz_large_est_; }

 private:
  std::array<double, 3> periodic_template_for_spin() const;
  double compute_period_confidence_for_phase(
      int phase, const std::array<double, 3> & templ, int sample_count) const;

  UnifiedConfig config_;
  std::array<double, 3> z_offsets_;
  std::deque<double> dz_jump_history_;
  double dz_small_est_ = std::numeric_limits<double>::quiet_NaN();
  double dz_large_est_ = std::numeric_limits<double>::quiet_NaN();
  double period_confidence_ = 0.0;
  int period_phase_index_ = -1;
  int spin_direction_ = 0;
  int pending_spin_direction_ = 0;
  int pending_spin_direction_count_ = 0;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_DECODER_OUTPOST_PERIODIC_DZ_EVIDENCE_HPP_
