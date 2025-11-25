#ifndef MUIT_OBJ_TRACKER_HUNGARIAN_HPP
#define MUIT_OBJ_TRACKER_HUNGARIAN_HPP

#include <vector>

namespace muit_obj_tracker {

/**
 * @class HungarianAlgorithm
 * @brief 匈牙利算法实现类。
 * 
 * 用于解决二分图最大权匹配问题，常用于多目标跟踪中的数据关联。
 */
class HungarianAlgorithm {
public:
    /**
     * @brief 求解分配问题。
     * @param distMatrix 代价矩阵（距离矩阵）。
     * @param assignment 输出的分配结果，assignment[i] 表示第 i 行匹配到的列索引。
     * @return 最小总代价。
     */
    double Solve(const std::vector<std::vector<double>>& distMatrix, std::vector<int>& assignment);
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_HUNGARIAN_HPP
