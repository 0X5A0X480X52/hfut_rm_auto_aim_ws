# Armor Tracker

## 概述

多装甲板跟踪包，基于 `muit_obj_tracker` 实现多目标跟踪功能。

## 版本说明

- **v2.0 (当前)**: 基于 `muit_obj_tracker` 的新架构
- **v1.0 (Legacy)**: 原有 EKF 实现，保留用于对比

## 功能特性

### 核心功能
- ✅ 多目标跟踪: 同时跟踪多个装甲板
- ✅ 匈牙利算法: 优化的数据关联
- ✅ 异步更新: 支持帧率不稳定的输入
- ✅ 多来源融合: 检测/估计/预测三种来源
- ✅ 策略模式: 灵活的处理策略

### 新增特性 (v2.0)
- ✅ 基于 `muit_obj_tracker` 的跟踪核心
- ✅ 异步预测更新（定时器触发）
- ✅ 多来源输入支持
  - `DETECT`: 上游检测节点的位置估计
  - `ESTIMATE`: robot_pose_estimator 的估计装甲板
  - `PREDICT`: 纯模型预测
- ✅ 策略模式处理不同来源
- ✅ 预留视觉跟踪接口

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/armor_detector/armors` | `rm_interfaces/msg/Armors` | 装甲板检测结果 |
| `/robot_pose_estimator/virtual_armors` | `rm_interfaces/msg/Armors` | 估计的虚拟装甲板 (预留) |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/armor_tracker/tracked_armors` | `rm_interfaces/msg/TrackedArmors` | 所有被跟踪装甲板的状态 |
| `/armor_tracker/markers` | `visualization_msgs/msg/MarkerArray` | 可视化标记 (调试模式) |

## 参数

### 异步更新参数
| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `predict_rate` | double | 100.0 | 预测更新频率 (Hz) |
| `publish_rate` | double | 100.0 | 发布频率 (Hz) |
| `detection_timeout` | double | 0.5 | 检测超时时间 (s) |

### 数据关联参数
| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `max_match_distance` | double | 0.5 | 最大匹配距离 (米) |
| `max_match_yaw_diff` | double | 1.0 | 最大匹配yaw角差 (弧度) |

### 状态机参数
| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `tracking_threshold` | int | 3 | 进入TRACKING状态的最小匹配次数 |
| `lost_threshold` | int | 30 | 进入LOST状态的最大丢失帧数 |
| `max_trackers` | int | 20 | 最大跟踪器数量 |

### 模型配置
| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `model.name` | string | "CV_KF" | 滤波模型名称 |
| `model.config_file` | string | "" | 模型配置文件路径 |

## 状态标记

### 来源类型 (source_type)
| 值 | 名称 | 说明 |
|----|------|------|
| 0 | DETECT | 融合了上游检测节点的位置估计 |
| 1 | ESTIMATE | 融合了robot_pose_estimator的估计装甲板位置 |
| 2 | PREDICT | 无观测输入，仅通过模型预测 |

### 跟踪状态 (tracking_state)
| 值 | 名称 | 说明 |
|----|------|------|
| 0 | LOST | 跟踪丢失，即将被移除 |
| 1 | DETECTING | 检测中，尚未确认跟踪 |
| 2 | TRACKING | 正在跟踪 |
| 3 | TEMP_LOST | 临时丢失，使用预测维持 |

## 使用方法

```bash
# 启动跟踪器 (v2.0)
ros2 launch armor_tracker armor_tracker.launch.py

# 启动旧版跟踪器 (对比测试)
ros2 launch armor_tracker armor_tracker_legacy.launch.py

# 查看跟踪结果
ros2 topic echo /armor_tracker/tracked_armors

# RViz可视化
rviz2
# 添加 MarkerArray, Topic: /armor_tracker/markers
```

## 架构说明

### 核心类

1. **ArmorTrackerNode**: 主节点，管理订阅、发布和定时器
2. **ArmorTrackerCore**: 核心跟踪逻辑，封装 `muit_obj_tracker`
3. **TrackingStrategyManager**: 策略管理器
4. **ITrackingStrategy**: 策略接口
   - `DetectSourceStrategy`: 处理检测来源
   - `EstimateSourceStrategy`: 处理估计来源
   - `PredictSourceStrategy`: 处理预测来源

### 数据流

```
/armor_detector/armors (检测) ──┐
                                │
                                ├──> ArmorTrackerNode ──> ArmorTrackerCore
                                │         │                    │
/robot_pose_estimator/         │         │                    │
  virtual_armors (估计) ────────┘         │                    │
                                          │                    ↓
                                          │           muit_obj_tracker
                                          │           (PointTracker)
                                          │                    │
                                          │                    ↓
                                          └──> 发布 ──> /armor_tracker/tracked_armors
```

### 异步更新机制

```
检测回调 ────────────────────────> 更新跟踪器 (source: DETECT)
                                          ↑
估计回调 ────────────────────────> 更新跟踪器 (source: ESTIMATE)
                                          ↑
预测定时器 (predict_rate Hz) ────> 预测步骤 (若无观测: source: PREDICT)
                                          │
发布定时器 (publish_rate Hz) ────> 发布跟踪结果
```

## 扩展指南

### 添加新的跟踪策略

```cpp
class MyCustomStrategy : public ITrackingStrategy {
public:
  std::string getName() const override { return "MyCustom"; }
  
  bool canHandle(ArmorSourceType source) const override {
    // 定义策略适用的来源类型
    return source == ArmorSourceType::DETECT;
  }
  
  double computeWeight(
    const ArmorObservation& observation,
    const TrackedArmorState* current_state = nullptr) const override {
    // 实现权重计算逻辑
    return observation.confidence;
  }
};

// 注册策略
strategy_manager_->registerStrategy(std::make_shared<MyCustomStrategy>());
```

### 添加视觉跟踪支持

预留了视觉跟踪接口，可通过以下方式扩展：

1. 订阅视觉跟踪结果话题
2. 实现新的来源类型策略
3. 融合视觉特征到观测数据

## 待优化

- [ ] 添加视觉特征 ReID 支持
- [ ] 实现自适应噪声参数
- [ ] 支持装甲板融合（同一机器人的多个装甲板）
- [ ] 添加跟踪质量评估指标
