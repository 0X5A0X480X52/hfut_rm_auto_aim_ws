# N步预测准确率分析功能

## 功能概述

新增了N步预测（N-step ahead prediction）准确率评估功能，可以量化评估卡尔曼滤波器在未来多个时间步的预测能力。

## 核心特性

### 1. **多视野预测评估**
- 支持同时评估多个预测视野（如1步、5步、10步、20步、50步）
- 自动计算每个视野下的RMSE、MAE和标准差
- 分别统计X、Y、Z各轴和整体位置误差

### 2. **智能预测模型**
- **有速度估计的滤波器**（CV、CA、CS、Singer、CTRV）：使用恒速模型
  ```
  predicted_pos(t+N) = current_pos(t) + current_vel(t) × (N × dt)
  ```
- **无速度估计的滤波器**（IMM）：使用恒定位置模型
  ```
  predicted_pos(t+N) = current_pos(t)
  ```

### 3. **丰富的可视化**
每次分析生成6张图表：
- RMSE vs 预测步数曲线
- MAE vs 预测步数曲线
- X/Y/Z各轴RMSE对比
- 3个不同预测视野的误差分布直方图

### 4. **多滤波器性能对比**
专门的对比工具，生成包含4个子图的综合对比：
- RMSE对比曲线
- MAE对比曲线
- 归一化误差增长率
- 性能汇总表格

## 使用方法

### 基础用法

```bash
# 1. 运行滤波器测试（如果还没有）
cd kalman_filters_examples
./scripts/run_all_tests.sh

# 2. 对单个滤波器进行N步预测分析
python3 scripts/analyze_results.py \
    test_data/cv_3d.csv \
    results/cv_kf_3d_result.csv \
    --output analysis/cv_kf_3d \
    --n-step

# 3. 自定义预测视野
python3 scripts/analyze_results.py \
    test_data/cv_3d.csv \
    results/cv_kf_3d_result.csv \
    --n-step \
    --horizons 1 5 10 20 50 100

# 4. 对比多个滤波器
python3 scripts/compare_n_step_predictions.py \
    --filters cv_kf_3d ca_kf_3d imm_cv_ca_cs_3d \
    --output analysis/comparison.png
```

### 使用便捷脚本

```bash
# 测试特定滤波器
./scripts/test_n_step_prediction.sh cv_kf_3d
./scripts/test_n_step_prediction.sh imm_cv_ca_cs_3d

# 运行完整演示
./scripts/demo_n_step_prediction.sh
```

## 输出文件说明

每次N步预测分析会在输出目录生成：

1. **n_step_predictions.png** - 6张子图的可视化
2. **n_step_predictions.json** - 详细统计数据（JSON格式）
3. **n_step_comparison.png** - 多滤波器对比图（可选）
4. **n_step_comparison.pdf** - 高质量PDF版本（可选）

## 典型结果示例

### CV滤波器（有速度估计）
```
1-step prediction:  RMSE: 0.0656 m,  MAE: 0.0571 m
5-step prediction:  RMSE: 0.0804 m,  MAE: 0.0687 m
10-step prediction: RMSE: 0.1005 m,  MAE: 0.0843 m
20-step prediction: RMSE: 0.1431 m,  MAE: 0.1167 m
```

### IMM滤波器（无速度估计）
```
1-step prediction:  RMSE: 0.0635 m,  MAE: 0.0562 m
5-step prediction:  RMSE: 0.1071 m,  MAE: 0.0989 m
10-step prediction: RMSE: 0.1828 m,  MAE: 0.1728 m
20-step prediction: RMSE: 0.3457 m,  MAE: 0.3302 m
50-step prediction: RMSE: 0.8461 m,  MAE: 0.8116 m ← 急剧增长
```

## 关键发现

### 1. 有速度 vs 无速度
- **CV/CA/CS滤波器**：20步RMSE约0.14-0.16m（线性增长）
- **IMM滤波器**：20步RMSE约0.35m（快速增长）
- 速度信息对长期预测至关重要

### 2. 预测视野建议
| 应用场景 | 推荐视野 | 典型RMSE |
|---------|---------|---------|
| 实时控制 | 1-3步 | <0.1m |
| 短期规划 | 5-10步 | 0.1-0.15m |
| 碰撞预警 | 10-20步 | 0.15-0.35m |
| 长期预测 | 20-50步 | >0.3m（仅CV/CA可靠）|

### 3. 误差增长模式
- **线性增长**：恒速假设合理，预测可靠
- **二次增长**：加速度影响显著
- **指数增长**：运动模式复杂，预测不可靠

## 命令行参数详解

### analyze_results.py
```
--n-step              启用N步预测分析
--horizons [N...]     指定预测视野列表（默认：1 5 10 20 50）
```

### compare_n_step_predictions.py
```
--analysis-dir DIR    分析结果基础目录（默认：./analysis）
--output FILE         输出文件路径
--filters [F...]      要对比的滤波器列表
```

## 实际应用示例

### 示例1: 评估实时跟踪性能
```bash
# 只关注1-5步的短期预测
python3 scripts/analyze_results.py \
    data.csv result.csv --n-step --horizons 1 2 3 4 5
```

### 示例2: 选择最佳滤波器
```bash
# 先对每个候选滤波器做N步分析
for filter in cv_kf ca_kf cs_kf; do
    python3 scripts/analyze_results.py \
        test_data/${filter}_3d.csv \
        results/${filter}_3d_result.csv \
        --n-step --horizons 1 5 10 20
done

# 然后对比
python3 scripts/compare_n_step_predictions.py \
    --filters cv_kf_3d ca_kf_3d cs_kf_3d
```

### 示例3: 参数调优
```bash
# 调整滤波器参数后，对比N步预测性能变化
python3 scripts/analyze_results.py \
    data.csv result_old.csv -o analysis/old --n-step
python3 scripts/analyze_results.py \
    data.csv result_new.csv -o analysis/new --n-step

# 查看JSON文件对比数值变化
```

## 技术细节

### 计算方法
1. 对于时刻t，使用滤波器在t时刻的估计状态
2. 根据模型预测t+N时刻的位置
3. 与t+N时刻的真实位置对比，计算误差
4. 对所有可用时刻重复，统计RMSE/MAE

### 采样点处理
- 自动检测数据采样时间dt
- 对于N步预测，使用前(总数-N)个点
- 确保每个预测都有对应真值

### 维度兼容性
- 自动检测2D/3D数据
- 支持有/无速度估计的滤波器
- 智能处理不同列名格式（est_x vs est_state_0）

## 相关文档

- **详细说明**：[N_STEP_PREDICTION.md](./N_STEP_PREDICTION.md)
- **快速参考**：[N_STEP_QUICK_REF.md](./N_STEP_QUICK_REF.md)

## 常见问题

**Q: 为什么我的IMM长期预测误差特别大？**
A: IMM输出只包含位置，没有速度信息，只能用恒定位置预测，误差会快速累积。

**Q: 如何选择合适的预测视野？**
A: 根据应用需求和可接受的误差范围。一般1-5步用于控制，10-20步用于规划。

**Q: RMSE突然跳跃说明什么？**
A: 可能数据中存在急转弯或突变，滤波器在这些点的预测性能下降。

**Q: 能否用于2D数据？**
A: 完全支持，自动检测维度并相应调整计算。

## 版本历史

- **v1.0** (2025-11-25): 初始版本
  - 基础N步预测分析
  - 多视野评估
  - 可视化和统计输出
  - 多滤波器对比工具
