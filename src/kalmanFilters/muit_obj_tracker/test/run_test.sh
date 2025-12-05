#!/bin/bash
# PointTracker 测试运行脚本
# 
# 使用方法:
#   ./run_test.sh [filter_type] [test_type] [options]
#
# 示例:
#   ./run_test.sh cv_kf_2d 0           # 使用 CV_KF 测试直线运动
#   ./run_test.sh ca_kf_2d 1 --visualize  # 使用 CA_KF 测试圆周运动并可视化

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/config"

# 默认参数
FILTER_TYPE="${1:-cv_kf_2d}"
TEST_TYPE="${2:-0}"
shift 2 2>/dev/null || true

# 获取 ROS2 工作空间根目录
WS_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"

echo "=========================================="
echo "PointTracker 测试"
echo "=========================================="
echo "滤波器配置: ${FILTER_TYPE}"
echo "测试类型: ${TEST_TYPE}"
echo "额外参数: $@"
echo ""

# 检查配置文件是否存在
CONFIG_FILE="${CONFIG_DIR}/${FILTER_TYPE}.yaml"
if [ ! -f "${CONFIG_FILE}" ]; then
    echo "错误: 配置文件不存在: ${CONFIG_FILE}"
    echo ""
    echo "可用的配置文件:"
    ls -1 "${CONFIG_DIR}"/*.yaml 2>/dev/null | while read f; do
        basename "$f" .yaml
    done
    exit 1
fi

# Source ROS2 环境
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
else
    echo "警告: 未找到 ROS2 工作空间 setup.bash"
fi

# 运行测试
EXECUTABLE="${WS_DIR}/install/muit_obj_tracker/lib/muit_obj_tracker/point_tracker_test"
if [ ! -f "${EXECUTABLE}" ]; then
    # 尝试在 build 目录查找
    EXECUTABLE="${WS_DIR}/build/muit_obj_tracker/point_tracker_test"
fi

if [ ! -f "${EXECUTABLE}" ]; then
    echo "错误: 找不到测试可执行文件"
    echo "请先编译项目: colcon build --packages-select muit_obj_tracker"
    exit 1
fi

echo "运行: ${EXECUTABLE} ${CONFIG_FILE} --test-type ${TEST_TYPE} $@"
echo ""

"${EXECUTABLE}" "${CONFIG_FILE}" --test-type "${TEST_TYPE}" "$@"
