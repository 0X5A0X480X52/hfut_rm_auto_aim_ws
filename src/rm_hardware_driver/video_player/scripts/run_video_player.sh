#!/bin/bash

# Video Player 使用示例脚本
# 此脚本演示如何启动 video_player 节点

echo "========================================"
echo "Video Player - 使用示例"
echo "========================================"
echo ""

# 检查是否提供了视频路径
if [ -z "$1" ]; then
    echo "错误：请提供视频文件路径"
    echo ""
    echo "使用方法："
    echo "  $0 /path/to/video.mp4 [fps] [loop]"
    echo ""
    echo "示例："
    echo "  $0 ~/Videos/test.mp4 30 true"
    echo "  $0 ~/Videos/test.mp4 60 false"
    echo ""
    exit 1
fi

VIDEO_PATH="$1"
FPS="${2:-30.0}"
LOOP="${3:-true}"

# 检查视频文件是否存在
if [ ! -f "$VIDEO_PATH" ]; then
    echo "错误：视频文件不存在: $VIDEO_PATH"
    exit 1
fi

echo "视频文件: $VIDEO_PATH"
echo "帧率: $FPS"
echo "循环播放: $LOOP"
echo ""
echo "启动 video_player 节点..."
echo ""

# Source ROS2 环境
source /home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash

# 启动节点
ros2 launch video_player video_player_launch.py \
    video_path:="$VIDEO_PATH" \
    fps:="$FPS" \
    loop_playback:="$LOOP"
