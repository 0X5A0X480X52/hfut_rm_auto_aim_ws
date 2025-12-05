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

#ifndef ARMOR_TRACKER__TRACKING_STRATEGY_HPP_
#define ARMOR_TRACKER__TRACKING_STRATEGY_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "armor_tracker/armor_types.hpp"

namespace fyt::auto_aim {

/**
 * @brief 跟踪策略接口
 * 
 * 策略模式的抽象基类，定义了处理不同来源装甲板的策略接口。
 * 通过继承此类，可以实现不同的处理逻辑，便于扩展。
 */
class ITrackingStrategy {
public:
  virtual ~ITrackingStrategy() = default;
  
  /**
   * @brief 获取策略名称
   */
  virtual std::string getName() const = 0;
  
  /**
   * @brief 判断策略是否适用于指定的来源类型
   * @param source 装甲板来源类型
   * @return true 如果策略可以处理该来源
   */
  virtual bool canHandle(ArmorSourceType source) const = 0;
  
  /**
   * @brief 预处理观测数据
   * @param observation 原始观测数据
   * @return 预处理后的观测数据
   */
  virtual ArmorObservation preprocess(const ArmorObservation& observation) const {
    return observation;  // 默认不做处理
  }
  
  /**
   * @brief 计算观测权重/置信度
   * @param observation 观测数据
   * @param current_state 当前跟踪状态 (可选)
   * @return 权重值 [0, 1]
   */
  virtual double computeWeight(
    const ArmorObservation& observation,
    const TrackedArmorState* current_state = nullptr) const = 0;
  
  /**
   * @brief 判断是否应该更新跟踪器
   * @param observation 观测数据
   * @param current_state 当前跟踪状态
   * @return true 如果应该执行更新
   */
  virtual bool shouldUpdate(
    const ArmorObservation& observation,
    const TrackedArmorState& current_state) const {
    return true;  // 默认总是更新
  }
  
  /**
   * @brief 后处理跟踪结果
   * @param state 跟踪状态 (可修改)
   */
  virtual void postprocess(TrackedArmorState& state) const {
    // 默认不做处理
  }
};

/**
 * @brief 检测来源策略
 * 处理来自上游检测节点的装甲板观测
 */
// Concrete strategy classes and the manager are declared in individual headers under include/armor_tracker/strategies/
// - armor_tracker/strategies/detect_source_strategy.hpp
// - armor_tracker/strategies/estimate_source_strategy.hpp
// - armor_tracker/strategies/predict_source_strategy.hpp
// - armor_tracker/strategies/tracking_strategy_manager.hpp

}  // namespace fyt::auto_aim

#endif  // ARMOR_TRACKER__TRACKING_STRATEGY_HPP_
