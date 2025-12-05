#!/bin/bash
# 3D 复杂轨迹跟踪测试脚本
# 测试 kalman_filters_examples/config 下的所有配置文件

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../../../../.." && pwd)"
TEST_DIR="$SCRIPT_DIR/.."
DATA_DIR="$TEST_DIR/data"
CONFIG_DIR="$TEST_DIR/config"
RESULTS_DIR="$DATA_DIR/results_3d"

# 配置文件目录
EXAMPLE_CONFIG_DIR="$WS_DIR/src/kalmanFilters/filters/kalman_filters_examples/config"

echo "=============================================="
echo "3D 复杂轨迹跟踪测试"
echo "=============================================="
echo ""
echo "工作空间: $WS_DIR"
echo "测试目录: $TEST_DIR"
echo "数据目录: $DATA_DIR"
echo "配置目录: $EXAMPLE_CONFIG_DIR"
echo ""

# 创建结果目录
mkdir -p "$RESULTS_DIR"

# 进入工作空间
cd "$WS_DIR"

# Source ROS2 环境
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

# 1. 生成 3D 测试数据
echo "=============================================="
echo "步骤 1: 生成 3D 复杂轨迹数据"
echo "=============================================="

if [ ! -f "$DATA_DIR/complex_detections_3d.csv" ]; then
    echo "正在生成 3D 轨迹数据..."
    python3 "$SCRIPT_DIR/generate_complex_trajectory_3d.py" \
        --frames 500 \
        --dt 0.01 \
        --noise 0.02 \
        --detection-prob 0.95 \
        --output-dir "$DATA_DIR" \
        --seed 42
else
    echo "3D 轨迹数据已存在，跳过生成"
fi

echo ""

# 2. 编译测试程序
echo "=============================================="
echo "步骤 2: 编译测试程序"
echo "=============================================="

colcon build --packages-select muit_obj_tracker --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

echo ""

# 3. 测试本地 3D 配置文件
echo "=============================================="
echo "步骤 3: 测试本地 3D 配置文件"
echo "=============================================="

LOCAL_3D_CONFIGS=(
    "cv_kf_3d.yaml"
    "ca_kf_3d.yaml"
    "cs_kf_3d.yaml"
    "singer_kf_3d.yaml"
)

for config in "${LOCAL_3D_CONFIGS[@]}"; do
    config_path="$CONFIG_DIR/$config"
    if [ -f "$config_path" ]; then
        filter_name=$(echo "$config" | sed 's/_3d\.yaml//' | tr '[:lower:]' '[:upper:]')
        echo ""
        echo "--- 测试: $filter_name (本地配置) ---"
        
        output_file="$RESULTS_DIR/results_${filter_name,,}_local.csv"
        
        ./build/muit_obj_tracker/point_tracker_test_3d \
            "$config_path" \
            --csv "$DATA_DIR/complex_detections_3d.csv" \
            --gt "$DATA_DIR/complex_ground_truth_3d.csv" \
            --output "$output_file" \
            2>&1 | grep -E "(滤波器类型|维度|帧数|总检测数|跟踪率|ID 切换|平均位置误差|最大位置误差|处理时间|完成)"
    else
        echo "配置文件不存在: $config_path"
    fi
done

echo ""

# 4. 测试 kalman_filters_examples/config 下的 3D 配置文件
echo "=============================================="
echo "步骤 4: 测试 kalman_filters_examples 3D 配置"
echo "=============================================="

EXAMPLE_3D_CONFIGS=(
    "cv_kf_3d.yaml"
    "ca_kf_3d.yaml"
    "cs_kf_3d.yaml"
    "singer_kf_3d.yaml"
    "imm_cv_ca_cs_3d.yaml"
)

for config in "${EXAMPLE_3D_CONFIGS[@]}"; do
    config_path="$EXAMPLE_CONFIG_DIR/$config"
    if [ -f "$config_path" ]; then
        filter_name=$(echo "$config" | sed 's/_3d\.yaml//' | tr '[:lower:]' '[:upper:]')
        echo ""
        echo "--- 测试: $filter_name (kalman_filters_examples) ---"
        
        output_file="$RESULTS_DIR/results_${filter_name,,}_example.csv"
        
        ./build/muit_obj_tracker/point_tracker_test_3d \
            "$config_path" \
            --csv "$DATA_DIR/complex_detections_3d.csv" \
            --gt "$DATA_DIR/complex_ground_truth_3d.csv" \
            --output "$output_file" \
            2>&1 | grep -E "(滤波器类型|维度|帧数|总检测数|跟踪率|ID 切换|平均位置误差|最大位置误差|处理时间|完成)" || true
    else
        echo "配置文件不存在: $config_path"
    fi
done

echo ""

# 5. 测试 2D 配置文件 (使用 2D 数据)
echo "=============================================="
echo "步骤 5: 测试 2D 配置文件"
echo "=============================================="

# 生成 2D 数据（如果不存在）
if [ ! -f "$DATA_DIR/complex_detections.csv" ]; then
    echo "正在生成 2D 轨迹数据..."
    python3 "$SCRIPT_DIR/generate_complex_trajectory.py" \
        --frames 500 \
        --dt 0.033 \
        --noise 2.0 \
        --detection-prob 0.95 \
        --output-dir "$DATA_DIR" \
        --seed 42
fi

EXAMPLE_2D_CONFIGS=(
    "cv_kf_2d.yaml"
    "ca_kf_2d.yaml"
    "ctrv_ekf_2d.yaml"
)

for config in "${EXAMPLE_2D_CONFIGS[@]}"; do
    config_path="$EXAMPLE_CONFIG_DIR/$config"
    if [ -f "$config_path" ]; then
        filter_name=$(echo "$config" | sed 's/_2d\.yaml//' | tr '[:lower:]' '[:upper:]')
        echo ""
        echo "--- 测试: $filter_name (2D, kalman_filters_examples) ---"
        
        output_file="$RESULTS_DIR/results_${filter_name,,}_2d.csv"
        
        ./build/muit_obj_tracker/point_tracker_test \
            "$config_path" \
            --test-type 5 \
            --csv "$DATA_DIR/complex_detections.csv" \
            --output "$output_file" \
            2>&1 | grep -E "(滤波器类型|维度|帧数|总检测数|跟踪率|ID 切换|平均位置误差|最大位置误差|处理时间|完成)" || true
    else
        echo "配置文件不存在: $config_path"
    fi
done

echo ""
echo "=============================================="
echo "测试完成！"
echo "=============================================="
echo ""
echo "结果文件保存在: $RESULTS_DIR"
ls -la "$RESULTS_DIR"
