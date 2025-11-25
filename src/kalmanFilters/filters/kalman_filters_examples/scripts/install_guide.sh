#!/bin/bash
# Installation and usage guide
# 安装和使用指南

echo "==================================================================="
echo "Kalman Filter Test Framework - Installation Guide"
echo "卡尔曼滤波器测试框架 - 安装指南"
echo "==================================================================="
echo ""

echo "Prerequisites / 前提条件:"
echo "  - ROS2 workspace set up / ROS2工作空间已设置"
echo "  - Eigen3, yaml-cpp installed / 已安装Eigen3和yaml-cpp"
echo "  - Python 3 with numpy, pandas, matplotlib / Python3及相关库"
echo ""

echo "Step 1: Install Python dependencies / 安装Python依赖"
echo "-------------------------------------------------------------------"
echo "pip3 install numpy pandas matplotlib"
echo ""

echo "Step 2: Build the package / 编译包"
echo "-------------------------------------------------------------------"
echo "cd /home/amatrix02/hfut_rm_auto_aim_ws"
echo "colcon build --packages-select kalman_filters_examples"
echo "source install/setup.bash"
echo ""

echo "Step 3: Navigate to package directory / 进入包目录"
echo "-------------------------------------------------------------------"
echo "cd src/kalmanFilters/filters/kalman_filters_examples"
echo ""

echo "Step 4: Run the demo / 运行演示"
echo "-------------------------------------------------------------------"
echo "./scripts/demo.sh"
echo ""
echo "This will:"
echo "  1. Generate test data / 生成测试数据"
echo "  2. Run 3 filter tests / 运行3个滤波器测试"
echo "  3. Analyze results / 分析结果"
echo "  4. Generate plots and statistics / 生成图表和统计"
echo ""

echo "==================================================================="
echo "Quick Test Examples / 快速测试示例"
echo "==================================================================="
echo ""

echo "Example 1: 2D CV Filter"
echo "-------------------------------------------------------------------"
cat << 'EOF'
# Generate data
python3 scripts/generate_test_data.py --output-dir test_data

# Run filter
filter_test \
    config/cv_kf_2d.yaml \
    test_data/cv_2d.csv \
    results/cv_2d_result.csv

# Analyze
python3 scripts/analyze_results.py \
    test_data/cv_2d.csv \
    results/cv_2d_result.csv \
    --output analysis/cv_2d

# View results
ls analysis/cv_2d/
EOF
echo ""

echo "Example 2: 3D IMM Filter"
echo "-------------------------------------------------------------------"
cat << 'EOF'
# Generate data (if not already done)
python3 scripts/generate_test_data.py --output-dir test_data

# Run filter
filter_test \
    config/imm_cv_ca_cs_3d.yaml \
    test_data/mixed_3d.csv \
    results/imm_result.csv

# Analyze
python3 scripts/analyze_results.py \
    test_data/mixed_3d.csv \
    results/imm_result.csv \
    --output analysis/imm

# View results
cat analysis/imm/statistics.json
EOF
echo ""

echo "==================================================================="
echo "Available Filters / 可用的滤波器"
echo "==================================================================="
echo ""
echo "2D Filters:"
echo "  - CV_KF (config/cv_kf_2d.yaml)        恒定速度"
echo "  - CA_KF (config/ca_kf_2d.yaml)        恒定加速度"
echo "  - CTRV_EKF (config/ctrv_ekf_2d.yaml)  恒定转弯率"
echo ""
echo "3D Filters:"
echo "  - CV_KF (config/cv_kf_3d.yaml)        恒定速度"
echo "  - CA_KF (config/ca_kf_3d.yaml)        恒定加速度"
echo "  - CS_KF (config/cs_kf_3d.yaml)        Current Statistical"
echo "  - Singer_KF (config/singer_kf_3d.yaml) Singer模型"
echo "  - IMM (config/imm_cv_ca_cs_3d.yaml)   交互式多模型"
echo ""

echo "==================================================================="
echo "Test Data Types / 测试数据类型"
echo "==================================================================="
echo ""
echo "2D Data:"
echo "  - cv_2d.csv         恒定速度"
echo "  - ca_2d.csv         恒定加速度"
echo "  - circular_2d.csv   圆周运动"
echo ""
echo "3D Data:"
echo "  - cv_3d.csv         恒定速度"
echo "  - ca_3d.csv         恒定加速度"
echo "  - helix_3d.csv      螺旋运动"
echo "  - mixed_3d.csv      混合运动"
echo ""

echo "==================================================================="
echo "Documentation / 文档"
echo "==================================================================="
echo ""
echo "  - README.md       Complete documentation / 完整文档"
echo "  - QUICKSTART.md   Quick start guide / 快速入门"
echo "  - SUMMARY.md      Project summary / 项目总结"
echo ""

echo "==================================================================="
echo "For more information, see README.md"
echo "更多信息请参见 README.md"
echo "==================================================================="
