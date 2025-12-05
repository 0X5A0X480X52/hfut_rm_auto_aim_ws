// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ballistic_solver/compensator/ideal_compensator.hpp"
#include "ballistic_solver/compensator/resistance_compensator.hpp"
#include "ballistic_solver/compensator/trajectory_compensator.hpp"

// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Modified by Amatrix (HFUT RM SKT GROUP)
// Copyright (C) FYT Vision Group. All rights reserved.

namespace ballistic_solver {

std::unique_ptr<TrajectoryCompensator> CompensatorFactory::createCompensator(
  const std::string &type) {
  if (type == "ideal") {
    return std::make_unique<IdealCompensator>();
  } else if (type == "resistance") {
    return std::make_unique<ResistanceCompensator>();
  } else {
    // 默认返回考虑空气阻力的补偿器
    return std::make_unique<ResistanceCompensator>();
  }
}

}  // namespace ballistic_solver
