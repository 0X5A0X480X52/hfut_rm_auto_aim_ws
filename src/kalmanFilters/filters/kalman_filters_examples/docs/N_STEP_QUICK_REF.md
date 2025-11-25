# N步预测分析 - 快速参考

## 一句话说明
评估卡尔曼滤波器在未来N个时间步的位置预测准确率，支持多种预测视野对比。

## 快速使用

### 最简单的用法
```bash
cd kalman_filters_examples
python3 scripts/analyze_results.py \
    test_data/cv_3d.csv \
    results/cv_kf_3d_result.csv \
    --n-step
```

### 自定义预测步数
```bash
python3 scripts/analyze_results.py \
    test_data/cv_3d.csv \
    results/cv_kf_3d_result.csv \
    --n-step \
    --horizons 1 5 10 20 50
```

## 关键输出

### 1. 可视化图表 (n_step_predictions.png)
- **左上**: RMSE vs 预测步数（越平缓越好）
- **中上**: MAE vs 预测步数
- **右上**: X/Y/Z各轴RMSE对比
- **下方三图**: 不同预测步数的误差分布

### 2. 统计数据 (n_step_predictions.json)
```json
{
  "5_steps": {
    "overall": {
      "rmse": 0.0804,  // 5步预测的整体RMSE
      "mae": 0.0687    // 5步预测的整体MAE
    }
  }
}
```

## 结果解读

| 预测步数 | 典型RMSE | 适用场景 |
|---------|---------|---------|
| 1-5步   | 0.06-0.08m | 实时跟踪、闭环控制 |
| 10-20步 | 0.10-0.15m | 短期轨迹预测、碰撞预警 |
| 50步+   | 0.30m+     | 长期规划（需要高质量滤波器）|

## 不同滤波器对比

| 滤波器 | 有速度估计 | 20步RMSE | 备注 |
|--------|-----------|----------|------|
| CV/CA/CS | ✓ | ~0.14m | 使用恒速模型，预测较准 |
| IMM      | ✗ | ~0.35m | 只能恒定位置，预测较差 |

## 常用命令

```bash
# 仅分析短期预测（1-10步）
python3 scripts/analyze_results.py data.csv result.csv --n-step --horizons 1 2 3 5 10

# 分析长期预测（包括100步）
python3 scripts/analyze_results.py data.csv result.csv --n-step --horizons 1 5 10 20 50 100

# 使用便捷脚本测试CV滤波器
./scripts/test_n_step_prediction.sh cv_kf_3d

# 运行演示
./scripts/demo_n_step_prediction.sh
```

## 核心指标含义

- **RMSE**: 均方根误差，对大误差敏感，用于评估最坏情况
- **MAE**: 平均绝对误差，反映整体平均水平
- **STD**: 标准差，衡量预测的稳定性

## 何时使用

✅ **适用场景**：
- 需要评估滤波器的预测能力
- 对比不同滤波器的长期性能
- 确定合适的预测视野范围
- 优化滤波器参数

❌ **不适用场景**：
- 只关心当前时刻的估计精度（用标准分析即可）
- 数据量太小（<100个样本）

## 详细文档
参见：[N_STEP_PREDICTION.md](./N_STEP_PREDICTION.md)
