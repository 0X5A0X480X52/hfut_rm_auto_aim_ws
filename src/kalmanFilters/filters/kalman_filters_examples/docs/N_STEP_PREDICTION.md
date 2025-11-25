# N步预测准确率分析

## 功能说明

本功能用于评估卡尔曼滤波器在不同时间步长下的预测准确率。通过分析滤波器在1步、5步、10步、20步、50步等不同预测视野（prediction horizon）下的性能，可以全面了解滤波器的预测能力。

## 预测方法

### 有速度估计的滤波器（CV, CA, CS, Singer, CTRV）

使用**恒速模型**进行N步预测：

```
predicted_position(t+N) = current_position(t) + current_velocity(t) × (N × dt)
```

### 无速度估计的滤波器（IMM）

使用**恒定位置模型**进行N步预测：

```
predicted_position(t+N) = current_position(t)
```

## 评估指标

对每个预测视野N，计算以下指标：

1. **RMSE (Root Mean Square Error)** - 均方根误差
   - 衡量预测值与真实值的整体偏差
   - 对大误差更敏感

2. **MAE (Mean Absolute Error)** - 平均绝对误差
   - 衡量预测值与真实值的平均偏差
   - 对所有误差一视同仁

3. **STD (Standard Deviation)** - 标准差
   - 衡量误差的离散程度
   - 反映预测的稳定性

每个指标都计算：

- **Per-axis** (X, Y, Z轴各自的误差)
- **Overall** (综合位置误差: √(x²+y²+z²))

## 使用方法

### 方法1：使用Python脚本

```bash
# 基本用法
python3 scripts/analyze_results.py \
    test_data/cv_3d.csv \
    results/cv_kf_3d_result.csv \
    --output analysis/cv_kf_3d \
    --n-step

# 自定义预测视野
python3 scripts/analyze_results.py \
    test_data/cv_3d.csv \
    results/cv_kf_3d_result.csv \
    --output analysis/cv_kf_3d \
    --n-step \
    --horizons 1 5 10 20 50 100
```

### 方法2：使用便捷脚本

```bash
# 测试CV滤波器
./scripts/test_n_step_prediction.sh cv_kf_3d

# 测试CA滤波器
./scripts/test_n_step_prediction.sh ca_kf_3d

# 测试IMM滤波器
./scripts/test_n_step_prediction.sh imm_cv_ca_cs_3d
```

## 输出文件

运行分析后，会在输出目录生成以下文件：

### 1. n_step_predictions.png

可视化图表，包含6个子图：

- **RMSE vs Prediction Horizon**: 整体RMSE随预测步数变化
- **MAE vs Prediction Horizon**: 整体MAE随预测步数变化
- **Per-Axis RMSE**: X/Y/Z各轴RMSE对比
- **Error Distribution (短期)**: 1步预测误差分布直方图
- **Error Distribution (中期)**: 中间步数预测误差分布直方图
- **Error Distribution (长期)**: 最大步数预测误差分布直方图

### 2. n_step_predictions.json

详细的统计数据，JSON格式：

```json
{
  "1_steps": {
    "horizon": 1,
    "num_samples": 999,
    "overall": {
      "rmse": 0.0656,
      "mae": 0.0571,
      "std": 0.0324
    },
    "x": {
      "rmse": 0.0419,
      "mae": 0.0307,
      "std": 0.0417
    },
    ...
  },
  ...
}
```

## 结果解读

### 1. RMSE趋势分析

- **线性增长**: 表明滤波器使用恒速模型，预测误差随时间线性累积
- **指数增长**: 表明目标运动复杂，恒速假设不适用
- **平缓增长**: 表明滤波器预测能力较好

### 2. 不同滤波器对比

| 滤波器 | 1步RMSE | 20步RMSE | 特点 |
|--------|---------|----------|------|
| CV_KF  | ~0.06m  | ~0.14m   | 适用于匀速运动 |
| CA_KF  | ~0.06m  | ~0.13m   | 考虑加速度，预测更准 |
| IMM    | ~0.06m  | ~0.35m   | 无速度信息，长期预测差 |

### 3. 应用建议

- **1-5步**: 适用于实时跟踪和控制
- **10-20步**: 适用于短期轨迹预测
- **50步以上**: 仅当目标运动非常规律时使用

## 实际应用示例

### 场景1: 机器人目标跟踪

```bash
# 评估跟踪器的1-10步预测能力
python3 scripts/analyze_results.py \
    test_data/robot_tracking.csv \
    results/tracking_result.csv \
    --n-step --horizons 1 2 3 5 10
```

### 场景2: 弹道预测

```bash
# 评估弹道模型的长期预测
python3 scripts/analyze_results.py \
    test_data/ballistic.csv \
    results/ballistic_result.csv \
    --n-step --horizons 5 10 20 50 100
```

## 技术细节

### 采样时间处理

- 自动从数据中检测采样时间间隔 dt
- N步预测时间 = N × dt

### 边界处理

- 对于N步预测，只使用前 (总样本数 - N) 个点
- 确保每个预测都有对应的真值进行对比

### 维度自动检测

- 自动检测2D或3D数据
- 相应调整误差计算方法

## 常见问题

### Q1: 为什么IMM的长期预测误差很大？

A: IMM滤波器输出中只有位置估计，没有速度信息，因此使用恒定位置模型预测，长期预测误差会迅速累积。

### Q2: 如何选择合适的预测视野？

A: 根据应用需求：

- 控制应用：1-5步
- 碰撞预警：10-20步
- 轨迹规划：20-50步

### Q3: RMSE和MAE哪个更重要？

A:

- RMSE对大误差更敏感，适合评估最坏情况
- MAE反映平均性能，适合评估整体水平
- 建议两者结合分析

## 参考资料

1. Kalman Filter N-step Prediction Theory
2. Multi-step Ahead Forecasting in State Space Models
3. Performance Metrics for Prediction Systems
