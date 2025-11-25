# Kalman Filters Examples - 卡尔曼滤波器测试示例

这个包提供了完整的卡尔曼滤波器测试框架，用于测试 `basic_models` 和 `combined_models` 包中实现的各种滤波器模型。

## 功能特性

- **配置文件**: YAML格式的滤波器配置，支持所有滤波器类型
- **测试数据生成**: Python脚本生成2D/3D运动轨迹测试数据
- **滤波器测试**: C++程序读取CSV数据，运行滤波器，输出结果
- **结果分析**: Python脚本对结果进行误差分析和可视化

## 支持的滤波器

### 基础模型 (Basic Models)

1. **CV_KF** - 恒定速度卡尔曼滤波器 (2D/3D)
2. **CA_KF** - 恒定加速度卡尔曼滤波器 (2D/3D)
3. **CS_KF** - "当前"统计模型卡尔曼滤波器 (3D)
4. **Singer_KF** - Singer模型卡尔曼滤波器 (3D)
5. **CTRV_EKF** - 恒定转弯率和速度扩展卡尔曼滤波器 (2D)

### 组合模型 (Combined Models)

1. **IMM_CV_CA_CS_3Dim** - 交互式多模型滤波器 (CV+CA+CS)
2. 其他IMM变体

## 目录结构

```
kalman_filters_examples/
├── config/                 # YAML配置文件
│   ├── cv_kf_2d.yaml
│   ├── cv_kf_3d.yaml
│   ├── ca_kf_2d.yaml
│   ├── ca_kf_3d.yaml
│   ├── cs_kf_3d.yaml
│   ├── singer_kf_3d.yaml
│   ├── ctrv_ekf_2d.yaml
│   └── imm_cv_ca_cs_3d.yaml
├── scripts/                # Python脚本
│   ├── generate_test_data.py   # 生成测试数据
│   ├── analyze_results.py      # 分析结果
│   └── run_all_tests.sh        # 批量运行测试
├── include/                # C++头文件
│   ├── yaml_config_reader.hpp
│   ├── csv_reader.hpp
│   └── csv_writer.hpp
├── src/                    # C++源文件
│   └── filter_test.cpp     # 滤波器测试程序
├── test_data/              # 测试数据 (自动生成)
├── results/                # 测试结果 (自动生成)
└── analysis/               # 分析结果 (自动生成)
```

## 使用方法

### 1. 编译

```bash
cd /home/amatrix02/hfut_rm_auto_aim_ws
colcon build --packages-select kalman_filters_examples
source install/setup.bash
```

### 2. 生成测试数据

```bash
cd src/kalmanFilters/filters/kalman_filters_examples

# 使用默认参数
python3 scripts/generate_test_data.py

# 自定义参数
python3 scripts/generate_test_data.py \
    --output-dir ./test_data \
    --duration 10.0 \
    --dt 0.01 \
    --noise 0.1
```

生成的数据文件：
- `cv_2d.csv` - 2D恒定速度
- `ca_2d.csv` - 2D恒定加速度
- `circular_2d.csv` - 2D圆周运动
- `cv_3d.csv` - 3D恒定速度
- `ca_3d.csv` - 3D恒定加速度
- `helix_3d.csv` - 3D螺旋运动
- `mixed_3d.csv` - 3D混合运动

### 3. 运行单个滤波器测试

```bash
# 格式: filter_test <config_file> <input_csv> <output_csv>

# 示例: 2D CV滤波器
filter_test \
    config/cv_kf_2d.yaml \
    test_data/cv_2d.csv \
    results/cv_kf_2d_result.csv

# 示例: 3D CA滤波器
filter_test \
    config/ca_kf_3d.yaml \
    test_data/ca_3d.csv \
    results/ca_kf_3d_result.csv

# 示例: IMM滤波器
filter_test \
    config/imm_cv_ca_cs_3d.yaml \
    test_data/mixed_3d.csv \
    results/imm_result.csv
```

### 4. 分析结果

```bash
# 格式: analyze_results.py <test_data> <result> --output <output_dir>

python3 scripts/analyze_results.py \
    test_data/cv_2d.csv \
    results/cv_kf_2d_result.csv \
    --output analysis/cv_kf_2d
```

分析输出：
- `trajectory.png` - 轨迹对比图
- `errors.png` - 误差分析图
- `statistics.json` - 误差统计数据

### 5. 批量运行所有测试

```bash
# 自动生成数据、运行测试、分析结果
chmod +x scripts/run_all_tests.sh
./scripts/run_all_tests.sh
```

## 配置文件格式

YAML配置文件示例：

```yaml
filter:
  type: "CA_KF"           # 滤波器类型
  T: 0.01                 # 采样时间 (秒)
  Dim: 3                  # 维度 (2D或3D)
  
  # 测量噪声协方差矩阵 R
  R:
    - [0.01, 0.0, 0.0]
    - [0.0, 0.01, 0.0]
    - [0.0, 0.0, 0.01]
  
  # 初始状态向量
  X_0: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  
  # 额外参数 (可选)
  extra_params:
    alpha: 0.5
    a_max: 5.0
  
  # 转移概率矩阵 (仅用于IMM)
  transform_rate_mat:
    - [0.90, 0.05, 0.05]
    - [0.05, 0.90, 0.05]
    - [0.05, 0.05, 0.90]
```

## CSV数据格式

### 测试数据 (输入)

```csv
time,true_x,true_y,true_z,true_vx,true_vy,true_vz,true_ax,true_ay,true_az,meas_x,meas_y,meas_z
0.00,0.0,0.0,0.0,1.0,0.5,0.3,0.5,0.3,0.2,0.012,-0.008,0.015
0.01,0.01,0.005,0.003,1.005,0.503,0.302,0.5,0.3,0.2,-0.003,0.011,0.008
...
```

### 结果数据 (输出)

```csv
time,meas_x,meas_y,meas_z,est_x,est_vx,est_ax,est_y,est_vy,est_ay,est_z,est_vz,est_az
0.00,0.012,-0.008,0.015,0.012,0.0,0.0,-0.008,0.0,0.0,0.015,0.0,0.0
0.01,-0.003,0.011,0.008,0.008,0.15,0.23,0.006,0.12,0.18,0.010,0.08,0.12
...
```

## 误差指标

分析脚本计算以下误差指标：

- **RMSE** (Root Mean Square Error) - 均方根误差
- **MAE** (Mean Absolute Error) - 平均绝对误差
- **STD** (Standard Deviation) - 标准差

对于位置和速度的每个轴单独计算，以及整体3D/2D误差。

## 示例结果

运行测试后，您将得到：

1. **轨迹可视化**
   - 真实轨迹 vs 滤波后轨迹 vs 测量值
   - 2D轨迹图或3D轨迹图

2. **误差分析**
   - 随时间变化的误差
   - 误差分布直方图
   - 误差百分位数

3. **统计数据**
   - JSON格式的详细统计
   - 可用于性能对比

## 依赖项

### C++ 依赖
- Eigen3
- yaml-cpp
- models, basic_models, combined_models (本项目的其他包)

### Python 依赖
```bash
# recommended: create virtual environment and install pinned requirements
# this project provides requirements.txt and a helper script:
#   scripts/setup_venv.sh
# or install explicitly:
pip3 install "numpy<2" pandas matplotlib
```

## 故障排除

### 找不到配置文件
确保配置文件路径正确，使用绝对路径或相对于当前工作目录的路径。

### 找不到测试数据
先运行 `generate_test_data.py` 生成测试数据。

### Python依赖错误
安装所需的Python包：
```bash
pip3 install numpy pandas matplotlib
```

### 编译错误
确保已安装yaml-cpp：
```bash
sudo apt-get install libyaml-cpp-dev
```

## 高级使用

### 自定义测试数据

编辑 `generate_test_data.py` 添加新的轨迹类型：

```python
class MyCustomTrajectory(TrajectoryGenerator):
    def generate(self, ...):
        # 实现自定义轨迹生成
        ...
```

### 添加新的分析指标

编辑 `analyze_results.py` 的 `calculate_errors()` 方法：

```python
def calculate_errors(self):
    errors = {}
    # 添加自定义误差计算
    ...
    return errors
```

## 许可证

参见项目根目录的 LICENSE 文件。
