# PointTracker 测试

本目录包含 `PointTracker` 的测试程序和配置文件。

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select muit_obj_tracker
```

## 测试配置文件

测试配置文件位于 `config/` 目录，支持以下卡尔曼滤波器：

| 配置文件 | 滤波器类型 | 说明 |
|---------|-----------|------|
| `cv_kf_2d.yaml` | CV_KF | 2D 恒定速度卡尔曼滤波器 |
| `ca_kf_2d.yaml` | CA_KF | 2D 恒定加速度卡尔曼滤波器 |
| `cs_kf_2d.yaml` | CS_KF | 2D 当前统计模型卡尔曼滤波器 |

### 配置文件格式

```yaml
# 滤波器参数
filter:
  type: "CV_KF"           # 滤波器类型
  T: 0.033                # 采样时间 (秒)
  Dim: 2                  # 状态维度 (2D)
  R:                      # 测量噪声协方差矩阵
    - [1.0, 0.0]
    - [0.0, 1.0]
  X_0: [0.0, 0.0, 0.0, 0.0]  # 初始状态向量
  process_noise_std: 0.5  # 过程噪声标准差

# 跟踪器参数
tracker:
  max_age: 30             # 最大未更新帧数
  min_hits: 3             # 最小匹配次数
  distance_threshold: 100.0  # 距离匹配阈值 (像素)
```

## 运行测试

### 使用脚本运行

```bash
# 赋予执行权限
chmod +x test/run_test.sh test/run_all_tests.sh

# 运行单个测试
./test/run_test.sh cv_kf_2d 0              # CV_KF + 直线运动
./test/run_test.sh ca_kf_2d 1 --visualize  # CA_KF + 圆周运动 + 可视化

# 运行所有测试
./test/run_all_tests.sh
```

### 直接运行可执行文件

```bash
# Source ROS2 环境
source install/setup.bash

# 运行测试
./build/muit_obj_tracker/point_tracker_test <config_file> [options]
```

## 命令行参数

| 参数 | 说明 | 默认值 |
|-----|------|--------|
| `<config_file>` | YAML 配置文件路径 | 必需 |
| `--visualize` | 启用可视化显示 | 关闭 |
| `--test-type N` | 测试类型 | 0 |
| `--frames N` | 测试帧数 | 200 |
| `--noise N` | 测量噪声标准差 | 2.0 |

## 测试类型

| 类型 | 编号 | 说明 |
|-----|-----|------|
| 直线运动 | 0 | 目标沿直线匀速运动 |
| 圆周运动 | 1 | 目标做圆周运动 |
| 之字形运动 | 2 | 目标做正弦曲线运动 |
| 多目标跟踪 | 3 | 同时跟踪多个目标 |
| 遮挡恢复 | 4 | 目标短暂消失后恢复跟踪 |

## 测试输出

测试程序会输出以下统计指标：

- **总帧数**: 测试的总帧数
- **成功跟踪帧数**: 有有效跟踪输出的帧数
- **跟踪率**: 成功跟踪帧数 / 总帧数
- **ID 切换次数**: 目标 ID 发生变化的次数
- **平均位置误差**: 估计位置与检测位置的平均欧氏距离
- **最大位置误差**: 最大的位置误差
- **处理时间**: 总处理时间和平均每帧处理时间

## 添加新的滤波器配置

1. 在 `config/` 目录创建新的 YAML 配置文件
2. 设置 `filter.type` 为已注册的滤波器类型
3. 根据滤波器类型配置相应的参数

支持的滤波器类型请参考 `src/kalmanFilters/filters` 目录。

## 可视化说明

启用 `--visualize` 选项后：

- **绿色框/点**: 检测结果
- **红色框/点**: 跟踪估计
- 按 `ESC` 键退出可视化

## 示例

```bash
# 测试 CV_KF 在直线运动场景下的性能
./test/run_test.sh cv_kf_2d 0 --frames 300 --noise 3.0

# 测试 CA_KF 在圆周运动场景下的性能并可视化
./test/run_test.sh ca_kf_2d 1 --visualize

# 测试 CS_KF 在多目标场景下的性能
./test/run_test.sh cs_kf_2d 3 --frames 200
```
