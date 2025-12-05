// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
// Modified by Amatrix (HFUT RM SKT GROUP)
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

#ifndef BALLISTIC_SOLVER__COMPENSATOR__MANUAL_COMPENSATOR_HPP_
#define BALLISTIC_SOLVER__COMPENSATOR__MANUAL_COMPENSATOR_HPP_

#include <algorithm>
#include <sstream>
#include <string>
#include <vector>

namespace ballistic_solver {

/// 手动补偿配置字符串的标准参数数量
constexpr std::size_t MANUAL_COMPENSATOR_STR_NUM = 6;

/**
 * @brief 线性区间类
 * 
 * 用于表示一个一维区间 [lower, upper]，支持点检测和区间交集检测
 */
class LineRegion {
public:
  /**
   * @brief 构造函数
   * @param lower 区间下界
   * @param upper 区间上界
   */
  LineRegion(const double lower, const double upper) : lower_(lower), upper_(upper) {}

  /**
   * @brief 检查点是否在区间内
   * @param point 待检查的点
   * @return 点是否在区间内 (开区间)
   */
  bool checkPoint(const double point) const { return (point > lower_ && point < upper_); }

  /**
   * @brief 检查是否与另一区间有交集
   * @param other 另一个区间
   * @return 是否有交集
   */
  bool checkIntersection(const LineRegion &other) const {
    return checkPoint(other.lower_) || checkPoint(other.upper_);
  }

  double getLower() const { return lower_; }
  double getUpper() const { return upper_; }

private:
  double lower_;  ///< 区间下界
  double upper_;  ///< 区间上界
};

/**
 * @brief 手动补偿器
 * 
 * 基于距离-高度二维映射表的角度补偿器。
 * 用于根据目标距离和高度查询预设的 pitch/yaw 补偿值。
 * 
 * 映射表结构:
 * - 第一级: 距离区间 -> 高度映射列表
 * - 第二级: 高度区间 -> (pitch_offset, yaw_offset)
 * 
 * 使用场景:
 * - 弥补机械误差
 * - 针对特定距离/高度的微调
 * - 实验数据的离散补偿
 */
class ManualCompensator {
public:
  /**
   * @brief 高度映射节点
   */
  struct HeightMapNode {
    HeightMapNode(const LineRegion &region, const double pitch, const double yaw)
    : height_region(region), pitch_offset(pitch), yaw_offset(yaw) {}

    LineRegion height_region;  ///< 高度区间
    double pitch_offset;       ///< pitch 补偿值 (弧度)
    double yaw_offset;         ///< yaw 补偿值 (弧度)
  };

  /**
   * @brief 距离映射节点
   */
  struct DistMapNode {
    DistMapNode(const LineRegion &region, const std::vector<HeightMapNode> &h_nodes)
    : dist_region(region), height_map(h_nodes) {}

    LineRegion dist_region;                 ///< 距离区间
    std::vector<HeightMapNode> height_map;  ///< 对应的高度映射列表
  };

  ManualCompensator() = default;
  ~ManualCompensator() = default;

  /**
   * @brief 根据距离和高度查询补偿角度
   * @param dist 目标距离 (米)
   * @param height 目标高度 (米)
   * @return 补偿值 {pitch_offset, yaw_offset}，若未找到返回 {0.0, 0.0}
   */
  std::vector<double> angleHardCorrect(const double dist, const double height) const;

  /**
   * @brief 更新补偿映射表
   * @param d_region 距离区间
   * @param h_region 高度区间
   * @param pitch_offset pitch 补偿值
   * @param yaw_offset yaw 补偿值
   * @return 是否成功添加 (若区间重叠则返回 false)
   */
  bool updateMap(const LineRegion &d_region,
                 const LineRegion &h_region,
                 const double pitch_offset,
                 const double yaw_offset);

  /**
   * @brief 通过字符串更新映射表
   * 
   * 字符串格式: "dist_lower dist_upper height_lower height_upper pitch_offset yaw_offset"
   * 
   * @param str 配置字符串
   * @return 是否成功解析和添加
   */
  bool updateMapByStr(const std::string &str);

  /**
   * @brief 批量更新映射表
   * @param strs 配置字符串列表
   * @return 是否全部成功
   */
  bool updateMapFlow(const std::vector<std::string> &strs) {
    for (const auto &str : strs) {
      if (!updateMapByStr(str)) {
        return false;
      }
    }
    return true;
  }

  /**
   * @brief 清空映射表
   */
  void clearMap() { angle_offset_map_.clear(); }

  /**
   * @brief 获取当前映射表大小
   * @return 距离区间数量
   */
  std::size_t getMapSize() const { return angle_offset_map_.size(); }

private:
  /**
   * @brief 解析配置字符串
   * @param str 输入字符串
   * @param nums 输出数值列表
   * @return 是否成功解析 (需要恰好 6 个数值)
   */
  bool parseStr(const std::string &str, std::vector<double> &nums) const;

  std::vector<DistMapNode> angle_offset_map_;  ///< 角度补偿映射表
};

}  // namespace ballistic_solver

#endif  // BALLISTIC_SOLVER__COMPENSATOR__MANUAL_COMPENSATOR_HPP_
