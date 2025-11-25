#ifndef MUIT_OBJ_TRACKER_TRACKER_MANAGER_HPP
#define MUIT_OBJ_TRACKER_TRACKER_MANAGER_HPP

#include "muit_obj_tracker/tracker/itracker.hpp"
#include "models/model_config_loader.h"
#include <memory>
#include <string>

namespace muit_obj_tracker {

/**
 * @enum TrackerType
 * @brief 支持的跟踪器类型枚举。
 */
enum class TrackerType { SORT, DEEPSORT, COMPOSITE, KALMANNET, POINT };

/**
 * @class TrackerManager
 * @brief 跟踪器管理器。
 * 
 * 负责管理具体的跟踪器实例，支持运行时切换跟踪算法，并提供统一的外部接口。
 */
class TrackerManager {
private:
    std::unique_ptr<ITracker> tracker; ///< 当前使用的跟踪器实例
    TrackerType current_type;          ///< 当前跟踪器类型
    std::string config_file;           ///< 配置文件路径
    std::string model_name;            ///< 使用的模型名称

public:
    /**
     * @brief 构造函数。
     * @param t 初始跟踪器类型。
     * @param config_file 配置文件路径。
     * @param model_name 使用的模型名称（默认为 "CV_KF"）。
     */
    TrackerManager(TrackerType t, const std::string& config_file = "", const std::string& model_name = "CV_KF");

    /**
     * @brief 设置跟踪器类型。
     * 会重新创建跟踪器实例，原有的跟踪状态将丢失。
     * @param t 目标跟踪器类型。
     */
    void setTrackerType(TrackerType t);

    /**
     * @brief 加载配置文件并重新初始化跟踪器。
     * @param config_file 配置文件路径。
     * @param model_name 模型名称。
     */
    void loadConfig(const std::string& config_file, const std::string& model_name);

    /**
     * @brief 执行预测。
     */
    void predict();

    /**
     * @brief 执行更新。
     * @param detections 检测结果列表。
     */
    void update(const std::vector<Detection>& detections);

    /**
     * @brief 获取跟踪结果。
     * @return 跟踪结果列表。
     */
    std::vector<TrackResult> getTracks() const;
};

} // namespace muit_obj_tracker

#endif // MUIT_OBJ_TRACKER_TRACKER_MANAGER_HPP
