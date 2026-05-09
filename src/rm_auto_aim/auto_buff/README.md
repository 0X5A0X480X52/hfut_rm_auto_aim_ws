# 能量机关组件

包含：

- 扇叶识别节点
- 能量机关追踪即弹道解算节点

详见各个节点自己的README。

## 当前 ROS2 适配实现

已落地的最小可运行链路：

- 节点：`buff_pose_estimator_node`
  - 输入：`/rune_target` (`rm_interfaces/msg/RuneTarget`)
  - 输出：`/auto_buff/tracked_robot` (`rm_interfaces/msg/TrackedRobot`)
  - 目标：给 `gimbal_pipeline` 的 buff 外部目标接入链路供数

- 配置：
  - `config/buff_pose_estimator.yaml`

- 启动：
  - `launch/buff_pose_estimator.launch.py`
  - `launch/rune_target_tracker_adapter.launch.py`（兼容入口，内部已转发到 `buff_pose_estimator_node`）

## 迁移资产

从 `tmp/jlu_vision_26` 同步了模型与参考配置：

- `model/yolox_rune_3.6m.onnx`
- `model/0526.onnx`
- `config/buff_detector.jlu_reference.yaml`
- `config/buff_tracker.jlu_reference.yaml`

## 说明

- `buff_detector/` 与 `buff_tracker/` 下旧 JLU 代码已保留作迁移参考，但当前不参与编译。
- 依赖收敛和替换决策见：`docs/JLU公共模块移植与ROS2替换调研.md`
