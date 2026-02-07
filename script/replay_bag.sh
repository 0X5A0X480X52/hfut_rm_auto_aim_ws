#!/bin/bash

# ROS2 Bag 重放脚本
# 用于重放录制的 ROS2 bag 文件

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认配置
DEFAULT_BAG_DIR="$HOME/ros2_bags"
DEFAULT_BAG_PATH=""
DEFAULT_RATE="1.0"
DEFAULT_LOOP=false
DEFAULT_START_OFFSET=0
DEFAULT_DURATION=0
CONFIG_TOPICS=()

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 从 YAML 配置文件加载设置
load_yaml_config() {
    local yaml_file="$SCRIPT_DIR/replay_config.yaml"
    
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
    
    # bag 目录
    if config.get('bag_dir'):
        print(f'DEFAULT_BAG_DIR={os.path.expanduser(config[\"bag_dir\"])}')
    
    # bag 路径
    if config.get('bag_path'):
        print(f'DEFAULT_BAG_PATH={os.path.expanduser(config[\"bag_path\"])}')
    
    # 速率
    if config.get('rate') is not None:
        print(f'DEFAULT_RATE={config[\"rate\"]}')
    
    # 循环
    if config.get('loop') is not None:
        print(f'DEFAULT_LOOP={str(config[\"loop\"]).lower()}')
    
    # 起始偏移
    if config.get('start_offset') is not None:
        print(f'DEFAULT_START_OFFSET={config[\"start_offset\"]}')
    
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
    echo "用法: $0 [选项] <bag文件路径>"
    echo ""
    echo "选项:"
    echo "  -r, --rate RATE       播放速率 (默认: 1.0)"
    echo "  -l, --loop            循环播放"
    echo "  -s, --start SEC       从指定秒数开始播放"
    echo "  -d, --duration SEC    播放指定秒数"
    echo "  -t, --topics TOPICS   仅播放指定话题 (用空格分隔)"
    echo "  -i, --info            仅显示 bag 信息，不播放"
    echo "  -L, --list            列出默认目录中的所有 bag 文件"
    echo "  -h, --help            显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 ~/ros2_bags/my_bag"
    echo "  $0 -r 0.5 ~/ros2_bags/my_bag  # 以0.5倍速播放"
    echo "  $0 -l ~/ros2_bags/my_bag  # 循环播放"
    echo "  $0 -s 10 -d 30 ~/ros2_bags/my_bag  # 从第10秒播放30秒"
    echo "  $0 -t '/camera/image_raw /lidar/points' ~/ros2_bags/my_bag"
    echo "  $0 -i ~/ros2_bags/my_bag  # 显示信息"
    echo "  $0 -L  # 列出所有bag文件"
}

# 检查 ROS2 环境
check_ros2_env() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}错误: 未找到 ros2 命令。请先 source ROS2 环境。${NC}"
        exit 1
    fi
}

# 列出 bag 文件
list_bags() {
    echo -e "${BLUE}在 $DEFAULT_BAG_DIR 中找到的 bag 文件:${NC}"
    echo ""
    
    if [ ! -d "$DEFAULT_BAG_DIR" ]; then
        echo -e "${YELLOW}目录不存在: $DEFAULT_BAG_DIR${NC}"
        return
    fi
    
    local count=0
    for bag in "$DEFAULT_BAG_DIR"/*; do
        if [ -d "$bag" ] && [ -f "$bag/metadata.yaml" ]; then
            count=$((count + 1))
            echo -e "${GREEN}[$count] $(basename "$bag")${NC}"
            
            # 获取简要信息
            local duration=$(ros2 bag info "$bag" 2>/dev/null | grep "Duration" | awk '{print $2}')
            local size=$(du -sh "$bag" 2>/dev/null | awk '{print $1}')
            
            echo "    路径: $bag"
            echo "    大小: $size"
            echo "    时长: $duration"
            echo ""
        fi
    done
    
    if [ $count -eq 0 ]; then
        echo -e "${YELLOW}未找到 bag 文件${NC}"
    fi
}

# 显示 bag 信息
show_bag_info() {
    local bag_path="$1"
    
    echo -e "${BLUE}=== Bag 文件信息 ===${NC}"
    echo ""
    
    ros2 bag info "$bag_path"
    
    echo ""
    echo -e "${BLUE}=== 话题列表 ===${NC}"
    ros2 bag info "$bag_path" | grep "Topic:" | awk '{print $2}' | while read topic; do
        echo "  - $topic"
    done
}

# 解析参数
RATE="$DEFAULT_RATE"
LOOP=$DEFAULT_LOOP
START_OFFSET="$DEFAULT_START_OFFSET"
DURATION="$DEFAULT_DURATION"
TOPICS=""
INFO_ONLY=false
LIST_ONLY=false
BAG_PATH="$DEFAULT_BAG_PATH"

# 如果配置文件中有话题列表，转换为字符串
if [ ${#CONFIG_TOPICS[@]} -gt 0 ]; then
    TOPICS="${CONFIG_TOPICS[*]}"
fi

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_usage
            exit 0
            ;;
        -r|--rate)
            RATE="$2"
            shift 2
            ;;
        -l|--loop)
            LOOP=true
            shift
            ;;
        -s|--start)
            START_OFFSET="$2"
            shift 2
            ;;
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -t|--topics)
            TOPICS="$2"
            shift 2
            ;;
        -i|--info)
            INFO_ONLY=true
            shift
            ;;
        -L|--list)
            LIST_ONLY=true
            shift
            ;;
        -*)
            echo -e "${RED}错误: 未知选项 $1${NC}"
            show_usage
            exit 1
            ;;
        *)
            BAG_PATH="$1"
            shift
            ;;
    esac
done

# 检查环境
check_ros2_env

# 如果只是列出文件
if [ "$LIST_ONLY" = true ]; then
    list_bags
    exit 0
fi

# 检查是否提供了 bag 路径
if [ -z "$BAG_PATH" ]; then
    echo -e "${RED}错误: 请指定 bag 文件路径${NC}"
    echo ""
    show_usage
    exit 1
fi

# 检查 bag 文件是否存在
if [ ! -d "$BAG_PATH" ]; then
    echo -e "${RED}错误: Bag 文件不存在: $BAG_PATH${NC}"
    exit 1
fi

if [ ! -f "$BAG_PATH/metadata.yaml" ]; then
    echo -e "${RED}错误: 无效的 bag 文件 (缺少 metadata.yaml): $BAG_PATH${NC}"
    exit 1
fi

# 如果只显示信息
if [ "$INFO_ONLY" = true ]; then
    show_bag_info "$BAG_PATH"
    exit 0
fi

# 显示将要播放的信息
echo -e "${GREEN}=== 准备重放 Bag ===${NC}"
echo -e "${BLUE}Bag 路径:${NC} $BAG_PATH"
echo -e "${BLUE}播放速率:${NC} ${RATE}x"

if [ "$LOOP" = true ]; then
    echo -e "${BLUE}循环模式:${NC} 启用"
fi

if [ -n "$START_OFFSET" ]; then
    echo -e "${BLUE}起始偏移:${NC} ${START_OFFSET}秒"
fi

if [ -n "$DURATION" ]; then
    echo -e "${BLUE}播放时长:${NC} ${DURATION}秒"
fi

if [ -n "$TOPICS" ]; then
    echo -e "${BLUE}指定话题:${NC}"
    for topic in $TOPICS; do
        echo "  - $topic"
    done
fi

echo ""

# 构建重放命令
CMD="ros2 bag play $BAG_PATH"

# 添加播放速率
CMD="$CMD --rate $RATE"

# 添加循环选项
if [ "$LOOP" = true ]; then
    CMD="$CMD --loop"
fi

# 添加起始偏移（大于0才添加）
if [ -n "$START_OFFSET" ] && [ "$START_OFFSET" -gt 0 ] 2>/dev/null; then
    CMD="$CMD --start-offset $START_OFFSET"
fi

# 添加持续时间（大于0才添加）
if [ -n "$DURATION" ] && [ "$DURATION" -gt 0 ] 2>/dev/null; then
    CMD="$CMD --duration $DURATION"
fi

# 添加话题过滤
if [ -n "$TOPICS" ]; then
    CMD="$CMD --topics $TOPICS"
fi

echo -e "${YELLOW}开始播放... (按 Ctrl+C 停止)${NC}"
echo ""

# 执行重放命令
$CMD

echo ""
echo -e "${GREEN}播放完成！${NC}"
