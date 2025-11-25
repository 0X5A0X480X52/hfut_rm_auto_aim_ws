# Quick Start Guide - 快速入门指南

## 1. 编译包

```bash
cd /home/amatrix02/hfut_rm_auto_aim_ws
colcon build --packages-select kalman_filters_examples
source install/setup.bash
```

## 2. 进入示例目录

```bash
cd src/kalmanFilters/filters/kalman_filters_examples
```

## 3. 生成测试数据

```bash
python3 scripts/generate_test_data.py --output-dir test_data
```

这将生成7个CSV文件：

- cv_2d.csv, ca_2d.csv, circular_2d.csv (2D轨迹)
- cv_3d.csv, ca_3d.csv, helix_3d.csv, mixed_3d.csv (3D轨迹)

## 4. 运行测试示例

### 示例1: 2D恒定速度滤波器

```bash
# 运行滤波器
filter_test \
    config/cv_kf_2d.yaml \
    test_data/cv_2d.csv \
    results/cv_2d_result.csv

# 分析结果
python3 scripts/analyze_results.py \
    test_data/cv_2d.csv \
    results/cv_2d_result.csv \
    --output analysis/cv_2d
```

查看结果：

- `analysis/cv_2d/trajectory.png` - 轨迹对比图
- `analysis/cv_2d/errors.png` - 误差分析图
- `analysis/cv_2d/statistics.json` - 统计数据

### 示例2: 3D恒定加速度滤波器

```bash
filter_test \
    config/ca_kf_3d.yaml \
    test_data/ca_3d.csv \
    results/ca_3d_result.csv

python3 scripts/analyze_results.py \
    test_data/ca_3d.csv \
    results/ca_3d_result.csv \
    --output analysis/ca_3d
```

### 示例3: IMM滤波器 (混合运动)

```bash
filter_test \
    config/imm_cv_ca_cs_3d.yaml \
    test_data/mixed_3d.csv \
    results/imm_result.csv

python3 scripts/analyze_results.py \
    test_data/mixed_3d.csv \
    results/imm_result.csv \
    --output analysis/imm
```

## 5. 批量运行所有测试

```bash
chmod +x scripts/run_all_tests.sh
./scripts/run_all_tests.sh
```

## 测试配置说明

### 2D滤波器测试

| 滤波器 | 配置文件 | 测试数据 | 说明 | 状态 |
|--------|----------|----------|------|------|
| CV_KF | cv_kf_2d.yaml | cv_2d.csv | 2D恒定速度 | ✅ 通过 |
| CA_KF | ca_kf_2d.yaml | ca_2d.csv | 2D恒定加速度 | ✅ 通过 |
| CTRV_EKF | ctrv_ekf_2d.yaml | circular_2d.csv | 2D圆周运动 | ⚠️ 失败* |

### 3D滤波器测试

| 滤波器 | 配置文件 | 测试数据 | 说明 | 状态 |
|--------|----------|----------|------|------|
| CV_KF | cv_kf_3d.yaml | cv_3d.csv | 3D恒定速度 | ✅ 通过 |
| CA_KF | ca_kf_3d.yaml | ca_3d.csv | 3D恒定加速度 | ✅ 通过 |
| CS_KF | cs_kf_3d.yaml | ca_3d.csv | 3D Current Statistical | ✅ 通过 |
| Singer_KF | singer_kf_3d.yaml | ca_3d.csv | 3D Singer模型 | ✅ 通过 |
| IMM | imm_cv_ca_cs_3d.yaml | mixed_3d.csv | 3D交互式多模型 | ⚠️ 失败* |

*注: CTRV_EKF 和 IMM 测试失败是由于原有滤波器代码中的 Eigen 矩阵维度问题，不是测试框架的问题。

## 常见问题

### Q: 找不到 filter_test 命令

A: 确保已经source了环境：

```bash
source /home/amatrix02/hfut_rm_auto_aim_ws/install/setup.bash
```

### Q: Python脚本执行失败

A: 安装依赖：

```bash
# recommended: create virtual environment and install pinned requirements
# this project provides requirements.txt and a helper script:
#   scripts/setup_venv.sh
# or install explicitly:
pip3 install "numpy<2" pandas matplotlib
```

### Q: 结果分析脚本报错 "No module named 'pandas'"

A: 安装 Python 数据分析库：

```bash
pip3 install numpy pandas matplotlib
```

### Q: 如何修改滤波器参数？

A: 编辑 config/ 目录下的YAML文件，修改参数后重新运行测试

### Q: 如何生成自己的测试数据？

A: 编辑 `scripts/generate_test_data.py`，添加新的轨迹生成类
