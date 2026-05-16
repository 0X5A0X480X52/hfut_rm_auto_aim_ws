// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_HYPOTHESIS_GENERATOR_HPP_
#define MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_HYPOTHESIS_GENERATOR_HPP_

#include <vector>

#include "max_entropy_tracker/core/observation.hpp"
#include "max_entropy_tracker/trackers/outpost_v3/outpost_hypothesis_types.hpp"

namespace fyt::auto_aim::outpost_v3 {

class OutpostHypothesisGenerator {
 public:
  OutpostHypothesisGenerator() = default;

  std::vector<OutpostHypothesis> generate(
      const std::vector<ObservationData> &observations) const;

  void attach_prior(std::vector<OutpostHypothesis> *hypotheses,
                    int last_committed_panel,
                    double switch_penalty) const;

 private:
  std::vector<OutpostHypothesis> generate_single(int obs_index) const;
};

}  // namespace fyt::auto_aim::outpost_v3

#endif  // MAX_ENTROPY_TRACKER_TRACKERS_OUTPOST_V3_OUTPOST_HYPOTHESIS_GENERATOR_HPP_
