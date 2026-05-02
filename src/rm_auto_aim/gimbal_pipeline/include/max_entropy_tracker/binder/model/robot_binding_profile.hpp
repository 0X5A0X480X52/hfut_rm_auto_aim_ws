// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_MODEL_ROBOT_BINDING_PROFILE_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_MODEL_ROBOT_BINDING_PROFILE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "max_entropy_tracker/binder/model/binder_enums.hpp"

namespace fyt::auto_aim::binder {

struct RobotBindingProfile {
  uint8_t robot_type = 255;
  int panel_count = 4;
  std::vector<double> z_offsets;
  std::vector<int> cyclic_order;
  bool has_dual_obs_capability = true;
  JumpKind primary_jump_kind = JumpKind::DZ;
  int height_levels = 2;

  HeightLabel height_label_for(int panel_id) const;
};

class RobotBindingProfileProvider {
 public:
  static RobotBindingProfile from_robot_id(
      const std::string & robot_id,
      const std::vector<double> & armors_offset = {});

  static RobotBindingProfile from_robot_type(uint8_t robot_type,
                                             int num_armors = 4);

  static bool validate(const RobotBindingProfile & profile);
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_MODEL_ROBOT_BINDING_PROFILE_HPP_
