# 快速入门指南

## 1. 安装依赖

```bash
# 安装Python依赖
pip3 install numpy scikit-learn scipy
```

## 2. 编译包

```bash
cd ~/hfut_rm_auto_aim_ws
colcon build --packages-select armor_fusion
source install/setup.bash
```

## 3. 测试融合功能

### 3.1 启动测试发布器（模拟两个摄像头）

在终端1中运行：

```bash
ros2 run armor_fusion test_publisher
```

这将发布模拟的装甲板检测数据：
- 两个目标
- 每个目标被两个摄像头同时观测
- 添加随机噪声模拟真实情况

### 3.2 启动融合节点

在终端2中运行：

```bash
ros2 run armor_fusion multi_camera_fusion_node.py --ros-args --params-file src/rm_auto_aim/armor_fusion/config/fusion_params.yaml
```

### 3.3 查看融合结果

在终端3中运行：

```bash
# 查看融合后的装甲板话题
ros2 topic echo /armor_fusion/armors

# 查看消息频率
ros2 topic hz /armor_fusion/armors
```

### 3.4 可视化（可选）

启动RViz：

```bash
rviz2
```

在RViz中：
1. 设置Fixed Frame为 `base_link`
2. 添加 MarkerArray 显示项，话题选择 `/armor_fusion/markers`
3. 你将看到：
   - 绿色小球：原始测量点（每个目标2个点，来自两个摄像头）
   - 红色大球：融合后的目标位置（每个目标1个点）

## 4. 与实际系统集成

### 4.1 配置摄像头话题

编辑 `config/fusion_params.yaml`：

```yaml
camera_topics:
  - "/camera1/armor_detector/armors"  # 替换为你的摄像头1话题
  - "/camera2/armor_detector/armors"  # 替换为你的摄像头2话题
  - "/camera3/armor_detector/armors"  # 可以添加更多摄像头
```

### 4.2 修改 armor_solver 订阅

方法1：修改launch文件，使用话题重映射

```python
solver_node = Node(
    package='armor_solver',
    executable='armor_solver_node',
    remappings=[
        ('armor_detector/armors', 'armor_fusion/armors'),
    ]
)
```

方法2：直接在命令行使用重映射

```bash
ros2 run armor_solver armor_solver_node --ros-args -r armor_detector/armors:=armor_fusion/armors
```

### 4.3 完整系统启动

使用提供的完整启动文件：

```bash
ros2 launch armor_fusion multi_camera_system.launch.py
```

## 5. 参数调优

### 5.1 DBSCAN聚类参数

在 `config/fusion_params.yaml` 中调整：

```yaml
# 聚类半径（米）
dbscan_eps: 0.3  # 增大此值使聚类更宽松，减小使聚类更严格

# 最小样本数
dbscan_min_samples: 1  # 通常设为1即可

# 最大噪声距离（米）
max_cluster_noise: 0.5  # 用于合并因噪声分裂的聚类
```

**调优建议**：
- 如果同一目标被分成多个聚类 → 增大 `dbscan_eps`
- 如果不同目标被错误聚在一起 → 减小 `dbscan_eps`
- 如果噪声导致的分裂没有被正确合并 → 增大 `max_cluster_noise`

### 5.2 性能参数

```yaml
# 发布频率（Hz）
publish_rate: 100.0  # 根据CPU性能调整

# 同步超时（秒）
sync_timeout: 0.05  # 根据系统延迟调整
```

## 6. 故障排除

### 问题：没有输出

**检查步骤**：

1. 确认摄像头话题正常发布
   ```bash
   ros2 topic list | grep armors
   ros2 topic hz /camera1/armor_detector/armors
   ```

2. 检查TF树
   ```bash
   ros2 run tf2_tools view_frames
   ```

3. 查看节点日志
   ```bash
   ros2 run armor_fusion multi_camera_fusion_node.py --ros-args --log-level DEBUG
   ```

### 问题：聚类效果不好

查看日志中的聚类信息：
```bash
ros2 run armor_fusion multi_camera_fusion_node.py --ros-args --log-level DEBUG
```

调整 `dbscan_eps` 和 `max_cluster_noise` 参数。

### 问题：TF转换失败

确保所有摄像头的TF已正确发布：
```bash
ros2 run tf2_ros tf2_echo base_link camera1_optical_frame
ros2 run tf2_ros tf2_echo base_link camera2_optical_frame
```

## 7. 性能监控

```bash
# 查看CPU使用率
top -p $(pgrep -f multi_camera_fusion)

# 查看话题延迟
ros2 topic delay /armor_fusion/armors

# 查看节点统计信息
ros2 node info /multi_camera_fusion
```

## 8. 下一步

- 阅读完整的 [README.md](README.md) 了解更多细节
- 根据实际场景调整参数
- 集成到完整的自瞄系统中
- 在实际机器人上测试

## 需要帮助？

- 查看日志中的错误信息
- 使用 `--ros-args --log-level DEBUG` 获取详细日志
- 在可视化中观察聚类行为
- 检查README中的"故障排除"部分
