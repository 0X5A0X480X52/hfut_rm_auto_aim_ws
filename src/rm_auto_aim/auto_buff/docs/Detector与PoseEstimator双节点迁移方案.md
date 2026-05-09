# auto_buff Detector 与 PoseEstimator 双节点迁移方案

日期：2026-05-09

## 1. 方案结论

采用双节点架构：

1. `buff_detector_node`（ROS2化）
2. `buff_pose_estimator_node`（新增）

并将当前 `src/ros2/rune_target_tracker_adapter_node.cpp` 的功能并入 `buff_pose_estimator_node`，后续下线 adapter 节点。

## 2. 数据链路

### 前驱输入

- 相机图像：`sensor_msgs/msg/Image`
- 相机参数：`sensor_msgs/msg/CameraInfo`

### 节点职责拆分

1. `buff_detector_node`
- 订阅图像
- 执行 rune 检测（ONNX/OpenVINO）
- 发布 `rm_interfaces/msg/RuneTarget`

2. `buff_pose_estimator_node`
- 订阅 `RuneTarget + CameraInfo`
- 可选订阅 `SerialReceiveData`（模式语义）
- 执行 PnP 姿态估计 + TF 变换
- 发布 `rm_interfaces/msg/TrackedRobot` 到 `/auto_buff/tracked_robot`

### 后继接入

- `gimbal_pipeline` 通过已实现的 external target adapter 订阅 `/auto_buff/tracked_robot` 并合并进主链路。

## 3. 为什么这样更好

- 比单 adapter 节点链路更短、时延更低。
- Detector 和 PoseEstimator 职责清晰，便于独立调试与回放。
- 复用现有 `buff_detector` 代码，迁移风险更低。
- 与现有 `gimbal_pipeline` 接口完全对齐。

## 4. 文件级改造计划

### A. Detector ROS2 化（保留文件，改实现）

- `buff_detector/include/buff_detector_node.hpp`
- `buff_detector/src/buff_detector_node.cpp`

改造要点：
- 去掉 iceoryx 订阅/发布与 listener 依赖。
- 替换为 ROS2 `rclcpp::Node` + image 订阅 + RuneTarget 发布。
- 保留 detector 核心推理/后处理逻辑。

### B. 新增 PoseEstimator 节点

- 新增：`src/ros2/buff_pose_estimator_node.cpp`（或 `buff_pose_estimator/` 子目录）

改造要点：
- 集成当前 `rune_target_tracker_adapter_node.cpp` 的核心功能：
  - RuneTarget 到 TrackedRobot 映射
  - PnP 姿态估计
  - TF 到 `target_frame`
  - 模式语义（small/big buff）映射

### C. 过渡与下线

- 保留 `rune_target_tracker_adapter_node.cpp` 仅作过渡验证。
- 当 `buff_pose_estimator_node` 验证通过后，下线 adapter 节点并更新 launch。

## 5. 配置与资产

保留并使用：

- `model/yolox_rune_3.6m.onnx`
- `model/0526.onnx`
- `config/buff_detector.jlu_reference.yaml`
- `config/buff_tracker.jlu_reference.yaml`

新增/更新：

- detector 节点参数（模型路径、阈值、设备）
- pose_estimator 节点参数（PnP、TF、模式映射、topic）

## 6. 验收标准

1. `auto_buff` 包可编译通过。
2. `buff_detector_node` 能稳定发布 `RuneTarget`。
3. `buff_pose_estimator_node` 能稳定发布 `/auto_buff/tracked_robot`。
4. `gimbal_pipeline` 在 mode `2/3/4/5` 下可按白名单接收 `small_buff/big_buff`。
5. 自瞄模式 `0/1` 不受影响。

## 7. buff_tracker 语义收敛

`buff_tracker` 目录中的代码后续按“仅状态估计”收敛：

- 保留：状态估计、拟合、因子图相关实现。
- 删除：云台控制、弹道、AimCommand 发布及其依赖链。

这使 `auto_buff` 包的上游职责聚焦为“检测 + 姿态估计 + 状态输出”，控制策略统一下沉到 `gimbal_pipeline`。
