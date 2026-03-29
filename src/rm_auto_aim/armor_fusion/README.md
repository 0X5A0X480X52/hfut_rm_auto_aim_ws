# Armor Fusion - 多摄像头装甲板检测融合

## 概述

`armor_fusion` 是一个 ROS2 包，用于融合多个摄像头的装甲板检测结果。它通过以下步骤实现多摄像头的精确目标感知：

1. **订阅多路检测结果**：订阅多个 `armor_detector` 节点发布的装甲板检测话题
2. **坐标转换**：将各摄像头坐标系下的检测结果统一转换到 `base_link` 坐标系
3. **DBSCAN 聚类**：使用 DBSCAN 算法对所有测量点进行聚类，识别同一目标的多摄像头观测
4. **Bundle Adjustment 优化**：在每个聚类内通过小型 BA 优化融合各点测量
5. **结果发布**：将融合后的高精度目标位置发布给 `armor_solver`

## 功能特性

- ✅ 多摄像头装甲板检测融合
- ✅ 基于 DBSCAN 的智能聚类
- ✅ 处理噪声导致的聚类分裂（假定同一目标最多因噪声出现两个点）
- ✅ Bundle Adjustment 优化位置估计
- ✅ 加权融合考虑测量不确定性
- ✅ RViz 可视化支持
- ✅ 时间同步和TF坐标转换

## 依赖

- ROS2 (Humble/Foxy或更高版本)
- Python 3
- numpy
- scikit-learn (用于DBSCAN)
- scipy (用于优化)
- tf2_ros
- rm_interfaces

## 安装

### 安装Python依赖

```bash
pip3 install numpy scikit-learn scipy
```

### 编译包

```bash
cd ~/hfut_rm_auto_aim_ws
colcon build --packages-select armor_fusion
source install/setup.bash
```

## 配置

### 参数配置文件

编辑 `config/fusion_params.yaml` 来配置融合节点：

```yaml
camera_topics:
  - "/camera1/armor_detector/armors"  # 摄像头1的装甲板话题
  - "/camera2/armor_detector/armors"  # 摄像头2的装甲板话题

target_frame: "base_link"  # 目标坐标系

dbscan_eps: 0.3              # DBSCAN聚类半径（米）
dbscan_min_samples: 1        # 最小样本数
max_cluster_noise: 0.5       # 合并近距离聚类的阈值（米）

publish_rate: 100.0          # 发布频率 (Hz)
enable_visualization: true   # 是否启用RViz可视化
sync_timeout: 0.05          # 同步超时（秒）
```

### 关键参数说明

- **camera_topics**: 需要订阅的所有摄像头装甲板话题列表
- **dbscan_eps**: DBSCAN 的 ε 参数，定义邻域半径。根据实际场景调整：
  - 值太小：同一目标可能被分成多个聚类
  - 值太大：不同目标可能被错误地聚在一起
  - 建议：0.2-0.5米
- **max_cluster_noise**: 用于合并因噪声分裂的聚类，假定同一目标最多因噪声产生两个聚类
- **sync_timeout**: 消息同步超时时间，超过此时间的消息将被丢弃

## 使用方法

### 启动融合节点

```bash
ros2 launch armor_fusion fusion.launch.py
```

### 自定义配置启动

```bash
ros2 launch armor_fusion fusion.launch.py config_file:=/path/to/your/config.yaml
```

### 单独运行节点

```bash
ros2 run armor_fusion armor_fusion_node --ros-args --params-file config/fusion_params.yaml
```

Python 版本仍可用：

```bash
ros2 run armor_fusion multi_camera_fusion_node.py --ros-args --params-file config/fusion_params.yaml
```

## C++ 代码结构（按功能拆分）

- `include/armor_fusion/measurement_types.hpp`
  - 融合内部统一测量类型定义
- `include/armor_fusion/transform_utils.hpp` + `src/transform_utils.cpp`
  - TF变换与变换失败回退逻辑
- `include/armor_fusion/clustering_utils.hpp` + `src/clustering_utils.cpp`
  - 聚类与聚类合并逻辑
- `include/armor_fusion/fusion_utils.hpp` + `src/fusion_utils.cpp`
  - 单聚类融合（位置加权融合、ID/类型聚合、四元数平均）
- `include/armor_fusion/visualization_utils.hpp` + `src/visualization_utils.cpp`
  - RViz Marker 构建
- `include/armor_fusion/multi_camera_fusion_node.hpp` + `src/multi_camera_fusion_node.cpp`
  - 节点编排流程（订阅/缓冲/调度/发布）

## 话题接口

### 订阅话题

- **多个摄像头装甲板话题** (`rm_interfaces/msg/Armors`)
  - 默认: `/camera1/armor_detector/armors`, `/camera2/armor_detector/armors`
  - 可在配置文件中修改

### 发布话题

- **`/armor_detector/armors`** (`rm_interfaces/msg/Armors`)
  - 融合后的装甲板检测结果
  - 在 `base_link` 坐标系下
  - 与原 detector 输出话题保持一致，tracker/solver 无需改代码

- **`/armor_fusion/markers`** (`visualization_msgs/msg/MarkerArray`)
  - 可视化标记（当 `enable_visualization=true` 时）
  - 绿色小球：原始测量点
  - 红色大球：融合后的目标位置
  - 白色文本：装甲板编号

## 系统集成

### 与现有系统集成

默认配置下不需要修改 `armor_solver` 或 `armor_tracker` 订阅代码。

- 融合节点输出直接发布到 `/armor_detector/armors`
- 下游节点保持原有订阅即可

如需并行对照测试，可通过参数覆盖输出话题：

```bash
ros2 launch armor_fusion fusion.launch.py output_topic:=/armor_fusion/armors
```

### 完整启动示例

```xml
<!-- launch文件示例 -->
<launch>
  <!-- 摄像头1检测器 -->
  <node pkg="armor_detector" exec="armor_detector_node" name="camera1_detector">
    <remap from="image_raw" to="/camera1/image_raw"/>
    <remap from="camera_info" to="/camera1/camera_info"/>
    <remap from="armor_detector/armors" to="/camera1/armor_detector/armors"/>
  </node>
  
  <!-- 摄像头2检测器 -->
  <node pkg="armor_detector" exec="armor_detector_node" name="camera2_detector">
    <remap from="image_raw" to="/camera2/image_raw"/>
    <remap from="camera_info" to="/camera2/camera_info"/>
    <remap from="armor_detector/armors" to="/camera2/armor_detector/armors"/>
  </node>
  
  <!-- 融合节点 -->
  <include file="$(find-pkg-share armor_fusion)/launch/fusion.launch.py"/>
  
  <!-- Solver节点（订阅融合结果） -->
  <node pkg="armor_solver" exec="armor_solver_node" name="armor_solver"/>
</launch>
```

## 算法原理

### 1. 坐标转换

使用 TF2 将各摄像头坐标系下的装甲板位置转换到统一的 `base_link` 坐标系。

### 2. DBSCAN 聚类

- **输入**: 所有测量点的3D位置
- **算法**: DBSCAN（基于密度的聚类）
- **目标**: 将同一目标的多摄像头观测分为一类

### 3. 聚类合并

针对"同一目标最多因噪声出现两个点"的假设：
- 计算聚类中心
- 如果两个聚类中心距离 < `max_cluster_noise`，则合并
- 这样可以处理边界情况和测量噪声导致的聚类分裂

### 4. Bundle Adjustment 优化

对每个聚类内的测量：
- **目标**: 最小化加权重投影误差
- **权重**: 基于测量的协方差矩阵（距离越远，不确定性越大）
- **优化器**: Levenberg-Marquardt 算法
- **输出**: 最优的3D位置估计

### 5. 方向融合

使用四元数平均算法融合多个方向测量。

## RViz 可视化

在 RViz 中添加以下显示项：

1. **MarkerArray** - `/armor_fusion/markers`
   - 查看原始测量点和融合结果
   
2. **TF** - 查看各摄像头和 base_link 的坐标系关系

## 调试

### 查看日志

```bash
ros2 run armor_fusion multi_camera_fusion_node.py --ros-args --log-level DEBUG
```

### 查看话题

```bash
# 查看订阅的话题
ros2 topic list | grep armors

# 查看融合输出
ros2 topic echo /armor_fusion/armors

# 查看消息频率
ros2 topic hz /armor_fusion/armors
```

### 检查TF树

```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

## 性能优化建议

1. **调整 `publish_rate`**: 根据计算资源调整发布频率
2. **调整 `dbscan_eps`**: 根据实际场景的目标间距调整
3. **调整 `sync_timeout`**: 根据系统延迟调整同步窗口
4. **减少可视化开销**: 在生产环境中设置 `enable_visualization: false`

## 故障排除

### 问题：没有融合输出

- 检查是否正确订阅了所有摄像头话题
- 检查 TF 树是否完整
- 查看日志中的 TF 转换错误

### 问题：聚类效果不好

- 增大 `dbscan_eps` 如果目标被分成多个聚类
- 减小 `dbscan_eps` 如果不同目标被错误聚在一起
- 调整 `max_cluster_noise` 以更好地合并噪声分裂的聚类

### 问题：延迟太大

- 减小 `publish_rate`
- 减小 `measurement_buffer_size`
- 关闭可视化

## 未来改进方向

- [ ] 支持动态添加/移除摄像头
- [ ] 更复杂的时间同步策略
- [ ] GPU 加速的 BA 优化
- [ ] 基于跟踪历史的自适应聚类
- [ ] 更精确的测量不确定性模型

## 许可证

Apache License 2.0

## 贡献

欢迎提交 Issue 和 Pull Request！
