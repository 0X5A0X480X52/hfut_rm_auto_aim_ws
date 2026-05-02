// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_FACTORY_BINDER_FACTORY_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_FACTORY_BINDER_FACTORY_HPP_

#include <memory>

#include "max_entropy_tracker/binder/model/robot_binding_profile.hpp"
#include "max_entropy_tracker/binder/pipeline/binder_pipeline.hpp"
#include "max_entropy_tracker/core/config.hpp"

namespace fyt::auto_aim::binder {

class BinderFactory {
 public:
  static std::unique_ptr<BinderPipeline> create(
      const RobotBindingProfile & profile,
      const BinderConfig & config);
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_FACTORY_BINDER_FACTORY_HPP_
