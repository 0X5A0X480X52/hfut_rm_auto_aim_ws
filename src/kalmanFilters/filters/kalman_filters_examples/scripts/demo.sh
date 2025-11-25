#!/bin/bash
# Demo script - Quick demonstration of the filter test framework
# 演示脚本 - 快速演示滤波器测试框架

set -e  # Exit on error

echo "==================================================================="
echo "Kalman Filter Test Framework Demo"
echo "卡尔曼滤波器测试框架演示"
echo "==================================================================="
echo ""

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PACKAGE_DIR="$( cd "${SCRIPT_DIR}/.." && pwd )"

cd "${PACKAGE_DIR}"

# Create directories
echo "Creating directories..."
mkdir -p test_data results analysis
echo ""

# Step 1: Generate test data
echo "==================================================================="
echo "Step 1: Generating test data (2D and 3D trajectories)"
echo "步骤1: 生成测试数据（2D和3D轨迹）"
echo "==================================================================="
echo ""

python3 scripts/generate_test_data.py \
    --output-dir test_data \
    --duration 5.0 \
    --dt 0.01 \
    --noise 0.05

echo ""
echo "Generated test data files:"
ls -lh test_data/*.csv
echo ""

# Step 2: Run a simple 2D CV filter test
echo "==================================================================="
echo "Step 2: Testing 2D Constant Velocity Filter"
echo "步骤2: 测试2D恒定速度滤波器"
echo "==================================================================="
echo ""

echo "Running CV_KF on 2D constant velocity data..."
filter_test \
    config/cv_kf_2d.yaml \
    test_data/cv_2d.csv \
    results/demo_cv_2d_result.csv

echo ""
echo "Analyzing results..."
python3 scripts/analyze_results.py \
    test_data/cv_2d.csv \
    results/demo_cv_2d_result.csv \
    --output analysis/demo_cv_2d

echo ""
echo "Results:"
echo "  - Filter output: results/demo_cv_2d_result.csv"
echo "  - Trajectory plot: analysis/demo_cv_2d/trajectory.png"
echo "  - Error analysis: analysis/demo_cv_2d/errors.png"
echo "  - Statistics: analysis/demo_cv_2d/statistics.json"
echo ""

# Step 3: Run a 3D CA filter test
echo "==================================================================="
echo "Step 3: Testing 3D Constant Acceleration Filter"
echo "步骤3: 测试3D恒定加速度滤波器"
echo "==================================================================="
echo ""

echo "Running CA_KF on 3D constant acceleration data..."
filter_test \
    config/ca_kf_3d.yaml \
    test_data/ca_3d.csv \
    results/demo_ca_3d_result.csv

echo ""
echo "Analyzing results..."
python3 scripts/analyze_results.py \
    test_data/ca_3d.csv \
    results/demo_ca_3d_result.csv \
    --output analysis/demo_ca_3d

echo ""
echo "Results:"
echo "  - Filter output: results/demo_ca_3d_result.csv"
echo "  - Trajectory plot: analysis/demo_ca_3d/trajectory.png"
echo "  - Error analysis: analysis/demo_ca_3d/errors.png"
echo "  - Statistics: analysis/demo_ca_3d/statistics.json"
echo ""

# Step 4: Run IMM filter on mixed motion
echo "==================================================================="
echo "Step 4: Testing IMM Filter on Mixed Motion"
echo "步骤4: 测试IMM滤波器（混合运动）"
echo "==================================================================="
echo ""

echo "Running IMM filter on 3D mixed motion data..."
filter_test \
    config/imm_cv_ca_cs_3d.yaml \
    test_data/mixed_3d.csv \
    results/demo_imm_result.csv

echo ""
echo "Analyzing results..."
python3 scripts/analyze_results.py \
    test_data/mixed_3d.csv \
    results/demo_imm_result.csv \
    --output analysis/demo_imm

echo ""
echo "Results:"
echo "  - Filter output: results/demo_imm_result.csv"
echo "  - Trajectory plot: analysis/demo_imm/trajectory.png"
echo "  - Error analysis: analysis/demo_imm/errors.png"
echo "  - Statistics: analysis/demo_imm/statistics.json"
echo ""

# Summary
echo "==================================================================="
echo "Demo Complete!"
echo "演示完成！"
echo "==================================================================="
echo ""
echo "Summary of generated files:"
echo ""
echo "Test Data (test_data/):"
ls -1 test_data/*.csv
echo ""
echo "Filter Results (results/):"
ls -1 results/demo_*.csv
echo ""
echo "Analysis Results (analysis/):"
ls -1 analysis/
echo ""
echo "To view the plots, open the PNG files in analysis/*/"
echo "要查看图表，请打开 analysis/*/ 目录中的PNG文件"
echo ""
echo "To see detailed statistics:"
echo "查看详细统计信息："
echo "  cat analysis/demo_cv_2d/statistics.json"
echo "  cat analysis/demo_ca_3d/statistics.json"
echo "  cat analysis/demo_imm/statistics.json"
echo ""
