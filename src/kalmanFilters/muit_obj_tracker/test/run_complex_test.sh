#!/bin/bash
# 复杂轨迹测试脚本
# 
# 该脚本会:
# 1. 使用 Python 生成复杂轨迹测试数据
# 2. 使用不同的滤波器配置进行测试
# 3. 保存测试结果

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
DATA_DIR="${SCRIPT_DIR}/data"
CONFIG_DIR="${SCRIPT_DIR}/config"

echo "=========================================="
echo "复杂轨迹跟踪测试"
echo "=========================================="

# Source ROS2 环境
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

# 创建数据目录
mkdir -p "${DATA_DIR}"
mkdir -p "${DATA_DIR}/results"

# 检测参数
FRAMES="${1:-500}"
NOISE="${2:-2.0}"
DETECTION_PROB="${3:-0.95}"

echo ""
echo "参数设置:"
echo "  帧数: ${FRAMES}"
echo "  测量噪声: ${NOISE}"
echo "  检测概率: ${DETECTION_PROB}"
echo ""

# 1. 生成测试数据
echo "=========================================="
echo "步骤 1: 生成复杂轨迹测试数据"
echo "=========================================="

python3 "${SCRIPT_DIR}/scripts/generate_complex_trajectory.py" \
    --frames "${FRAMES}" \
    --noise "${NOISE}" \
    --detection-prob "${DETECTION_PROB}" \
    --output-dir "${DATA_DIR}" \
    --seed 42

if [ $? -ne 0 ]; then
    echo "错误: 数据生成失败"
    exit 1
fi

echo ""

# 2. 获取测试可执行文件
EXECUTABLE="${WS_DIR}/install/muit_obj_tracker/lib/muit_obj_tracker/point_tracker_test"
if [ ! -f "${EXECUTABLE}" ]; then
    EXECUTABLE="${WS_DIR}/build/muit_obj_tracker/point_tracker_test"
fi

if [ ! -f "${EXECUTABLE}" ]; then
    echo "错误: 找不到测试可执行文件"
    echo "请先编译项目: colcon build --packages-select muit_obj_tracker"
    exit 1
fi

# 3. 运行不同滤波器配置的测试
echo "=========================================="
echo "步骤 2: 运行滤波器测试"
echo "=========================================="

declare -a FILTERS=("cv_kf_2d" "ca_kf_2d" "cs_kf_2d")
declare -a FILTER_NAMES=("CV_KF (恒定速度)" "CA_KF (恒定加速度)" "CS_KF (当前统计)")

for i in "${!FILTERS[@]}"; do
    filter="${FILTERS[$i]}"
    name="${FILTER_NAMES[$i]}"
    config_file="${CONFIG_DIR}/${filter}.yaml"
    
    if [ ! -f "${config_file}" ]; then
        echo "警告: 配置文件不存在: ${config_file}"
        continue
    fi
    
    echo ""
    echo "----------------------------------------"
    echo "测试滤波器: ${name}"
    echo "配置文件: ${config_file}"
    echo "----------------------------------------"
    
    output_file="${DATA_DIR}/results/tracking_${filter}.csv"
    
    "${EXECUTABLE}" "${config_file}" \
        --test-type 5 \
        --csv "${DATA_DIR}/complex_detections.csv" \
        --gt "${DATA_DIR}/complex_ground_truth.csv" \
        --output "${output_file}"
    
    echo ""
done

# 4. 生成对比报告
echo "=========================================="
echo "步骤 3: 生成测试报告"
echo "=========================================="

echo ""
echo "测试数据文件:"
echo "  检测数据: ${DATA_DIR}/complex_detections.csv"
echo "  真值数据: ${DATA_DIR}/complex_ground_truth.csv"
echo "  轨迹信息: ${DATA_DIR}/complex_trajectory_info.csv"
echo "  轨迹图: ${DATA_DIR}/trajectory_overview.png"
echo ""
echo "跟踪结果:"
for filter in "${FILTERS[@]}"; do
    result_file="${DATA_DIR}/results/tracking_${filter}.csv"
    if [ -f "${result_file}" ]; then
        echo "  ${filter}: ${result_file}"
    fi
done

echo ""
echo "=========================================="
echo "测试完成!"
echo "=========================================="
