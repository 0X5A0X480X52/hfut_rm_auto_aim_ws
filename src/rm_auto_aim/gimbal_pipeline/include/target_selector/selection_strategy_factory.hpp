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

#ifndef TARGET_SELECTOR__SELECTION_STRATEGY_FACTORY_HPP_
#define TARGET_SELECTOR__SELECTION_STRATEGY_FACTORY_HPP_

#include <atomic>
#include <memory>
#include <string>

#include "target_selector/selection_strategy.hpp"

namespace fyt::auto_aim {

class SelectionStrategyFactory {
 public:
  SelectionStrategyFactory() = delete;

  static SelectionStrategyPtr create(
      const std::string& name,
      const std::atomic_bool* attack_outpost_first = nullptr);
};

}  // namespace fyt::auto_aim

#endif  // TARGET_SELECTOR__SELECTION_STRATEGY_FACTORY_HPP_
