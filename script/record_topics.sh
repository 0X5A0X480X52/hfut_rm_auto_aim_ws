#!/bin/bash

# ROS2 Bag 录制脚本
# 用于录制指定的 ROS2 话题

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 默认配置
DEFAULT_OUTPUT_DIR="$HOME/ros2_bags"
DEFAULT_BAG_NAME="ros2_bag_$(date +%Y%m%d_%H%M%S)"
DEFAULT_COMPRESSION=false
DEFAULT_RECORD_ALL=false
DEFAULT_DURATION=0
CONFIG_TOPICS=()

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 从 YAML 配置文件加载设置
load_yaml_config() {
    local yaml_file="$SCRIPT_DIR/record_config.yaml"
    
    if [ ! -f "$yaml_file" ]; then
        return 0
    fi
    
    # 使用 Python 解析 YAML 并导出为 shell 变量
    local yaml_vars
    yaml_vars=$(python3 -c "
import yaml
import sys
import os

try:
    with open('$yaml_file', 'r') as f:
        config = yaml.safe_load(f)
    
    # 输出目录
    if config.get('output_dir'):
        print(f'DEFAULT_OUTPUT_DIR={os.path.expanduser(config[\"output_dir\"])}')
    
    # bag 名称
    if config.get('bag_name'):
        print(f'DEFAULT_BAG_NAME={config[\"bag_name\"]}')
    
    # 压缩
    if config.get('compression') is not None:
        print(f'DEFAULT_COMPRESSION={str(config[\"compression\"]).lower()}')
    
    # 录制所有话题
    if config.get('record_all') is not None:
        print(f'DEFAULT_RECORD_ALL={str(config[\"record_all\"]).lower()}')
    
    # 时长
    if config.get('duration') is not None:
        print(f'DEFAULT_DURATION={config[\"duration\"]}')
    
    # 话题列表
    if config.get('topics'):
        topics = '|'.join(config['topics'])
        # 使用单引号防止 eval 时被 shell 解析
        print(f\"CONFIG_TOPICS_STR='{topics}'\")
        
except Exception as e:
    print(f'# Error loading YAML: {e}', file=sys.stderr)
" 2>/dev/null)
    
    if [ -n "$yaml_vars" ]; then
        eval "$yaml_vars"
        echo -e "${GREEN}✓ 已加载配置: $yaml_file${NC}"
        
        # 解析话题列表字符串为数组（使用管道符分隔）
        if [ -n "$CONFIG_TOPICS_STR" ]; then
            IFS='|' read -ra CONFIG_TOPICS <<< "$CONFIG_TOPICS_STR"
        fi
    fi
}

# 加载配置
load_yaml_config

# 显示使用说明
show_usage() {
    echo "用法: $0 [选项] <话题列表>"
    echo ""
    echo "选项:"
    echo "  -o, --output DIR      指定输出目录 (默认: $DEFAULT_OUTPUT_DIR)"
    echo "  -n, --name NAME       指定 bag 文件名 (默认: 自动生成时间戳)"
    echo "  -a, --all             录制所有话题"
    echo "  -d, --duration SEC    录制指定秒数后自动停止"
    echo "  -c, --compression     使用压缩存储"
    echo "  -h, --help            显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 /camera/image_raw /lidar/points"
    echo "  $0 -o ./my_bags -n test_bag /camera/image_raw"
    echo "  $0 -a -d 30  # 录制所有话题30秒"
    echo "  $0 -c /camera/image_raw  # 使用压缩"
}

# 检查 ROS2 环境
check_ros2_env() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}错误: 未找到 ros2 命令。请先 source ROS2 环境。${NC}"
        exit 1
    fi
}

# 解析参数
OUTPUT_DIR="$DEFAULT_OUTPUT_DIR"
BAG_NAME="$DEFAULT_BAG_NAME"
RECORD_ALL=$DEFAULT_RECORD_ALL
DURATION="$DEFAULT_DURATION"
COMPRESSION=""
TOPICS=()
USER_PROVIDED_TOPICS=false

# 如果配置指定了压缩
if [ "$DEFAULT_COMPRESSION" = "true" ]; then
    COMPRESSION="--compression-mode file"
fi

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -n|--name)
            BAG_NAME="$2"
            shift 2
            ;;
        -a|--all)
            RECORD_ALL=true
            shift
            ;;
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -c|--compression)
            COMPRESSION="--compression-mode file"
            shift
            ;;
        -*)
            echo -e "${RED}错误: 未知选项 $1${NC}"
            show_usage
            exit 1
            ;;
        *)
            TOPICS+=("$1")
            USER_PROVIDED_TOPICS=true
            shift
            ;;
    esac
done

# 如果用户未提供话题且配置文件中有话题列表，使用配置文件中的话题
if [ "$USER_PROVIDED_TOPICS" = false ] && [ ${#CONFIG_TOPICS[@]} -gt 0 ]; then
    TOPICS=("${CONFIG_TOPICS[@]}")
    echo -e "${BLUE}使用配置文件中的话题列表${NC}"
fi

# 检查环境
check_ros2_env

# 创建输出目录
mkdir -p "$OUTPUT_DIR"

# 构建完整路径
BAG_PATH="$OUTPUT_DIR/$BAG_NAME"

# 构建录制命令
CMD="ros2 bag record"

if [ "$RECORD_ALL" = true ]; then
    CMD="$CMD -a"
    echo -e "${GREEN}准备录制所有话题...${NC}"
else
    if [ ${#TOPICS[@]} -eq 0 ]; then
        echo -e "${RED}错误: 请指定要录制的话题，或使用 -a 录制所有话题${NC}"
        show_usage
        exit 1
    fi
    CMD="$CMD ${TOPICS[*]}"
    echo -e "${GREEN}准备录制话题:${NC}"
    for topic in "${TOPICS[@]}"; do
        echo "  - $topic"
    done
fi

# 添加压缩选项
if [ -n "$COMPRESSION" ]; then
    CMD="$CMD $COMPRESSION"
    echo -e "${YELLOW}使用压缩模式${NC}"
fi

# 添加输出路径
CMD="$CMD -o $BAG_PATH"

# 添加持续时间（大于0才添加）
if [ -n "$DURATION" ] && [ "$DURATION" -gt 0 ] 2>/dev/null; then
    CMD="$CMD -d $DURATION"
    echo -e "${YELLOW}录制时长: ${DURATION}秒${NC}"
fi

echo -e "${GREEN}输出路径: $BAG_PATH${NC}"
echo ""
echo -e "${YELLOW}开始录制... (按 Ctrl+C 停止)${NC}"
echo ""

# 执行录制命令
$CMD

echo ""
echo -e "${GREEN}录制完成！${NC}"
echo -e "${GREEN}Bag 文件保存在: $BAG_PATH${NC}"

# 显示 bag 信息
if [ -d "$BAG_PATH" ]; then
    echo ""
    echo -e "${YELLOW}Bag 信息:${NC}"
    ros2 bag info "$BAG_PATH"
fi
