#!/bin/bash
# 运行所有滤波器的测试
# 
# 该脚本会对所有可用的滤波器配置运行完整的测试套件

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/config"
WS_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

# 测试类型说明
declare -a TEST_NAMES=("直线运动" "圆周运动" "之字形运动" "多目标跟踪" "遮挡恢复")

echo "=========================================="
echo "PointTracker 完整测试套件"
echo "=========================================="
echo ""

# Source ROS2 环境
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

# 查找可执行文件
EXECUTABLE="${WS_DIR}/install/muit_obj_tracker/lib/muit_obj_tracker/point_tracker_test"
if [ ! -f "${EXECUTABLE}" ]; then
    EXECUTABLE="${WS_DIR}/build/muit_obj_tracker/point_tracker_test"
fi

if [ ! -f "${EXECUTABLE}" ]; then
    echo "错误: 找不到测试可执行文件"
    echo "请先编译项目: colcon build --packages-select muit_obj_tracker"
    exit 1
fi

# 结果汇总
declare -A RESULTS

# 遍历所有配置文件
for config_file in "${CONFIG_DIR}"/*.yaml; do
    if [ ! -f "$config_file" ]; then
        continue
    fi
    
    filter_name=$(basename "$config_file" .yaml)
    
    echo ""
    echo "=========================================="
    echo "测试滤波器: ${filter_name}"
    echo "=========================================="
    
    # 运行所有测试类型
    for test_type in 0 1 2 3 4; do
        echo ""
        echo "--- 测试 ${test_type}: ${TEST_NAMES[$test_type]} ---"
        
        output=$("${EXECUTABLE}" "${config_file}" --test-type ${test_type} --frames 100 2>&1)
        
        # 提取关键指标
        track_rate=$(echo "$output" | grep "跟踪率" | awk '{print $2}')
        avg_error=$(echo "$output" | grep "平均位置误差" | awk '{print $2}')
        
        echo "  跟踪率: ${track_rate:-N/A}"
        echo "  平均位置误差: ${avg_error:-N/A}"
        
        RESULTS["${filter_name}_${test_type}"]="${track_rate:-0%}|${avg_error:-N/A}"
    done
done

echo ""
echo "=========================================="
echo "测试汇总"
echo "=========================================="
echo ""
printf "%-15s | %-10s | %-10s | %-10s | %-10s | %-10s\n" \
    "滤波器" "直线" "圆周" "之字形" "多目标" "遮挡"
echo "--------------------------------------------------------------------------------"

for config_file in "${CONFIG_DIR}"/*.yaml; do
    if [ ! -f "$config_file" ]; then
        continue
    fi
    
    filter_name=$(basename "$config_file" .yaml)
    
    r0=$(echo "${RESULTS[${filter_name}_0]}" | cut -d'|' -f1)
    r1=$(echo "${RESULTS[${filter_name}_1]}" | cut -d'|' -f1)
    r2=$(echo "${RESULTS[${filter_name}_2]}" | cut -d'|' -f1)
    r3=$(echo "${RESULTS[${filter_name}_3]}" | cut -d'|' -f1)
    r4=$(echo "${RESULTS[${filter_name}_4]}" | cut -d'|' -f1)
    
    printf "%-15s | %-10s | %-10s | %-10s | %-10s | %-10s\n" \
        "$filter_name" "${r0:-N/A}" "${r1:-N/A}" "${r2:-N/A}" "${r3:-N/A}" "${r4:-N/A}"
done

echo ""
echo "测试完成!"
