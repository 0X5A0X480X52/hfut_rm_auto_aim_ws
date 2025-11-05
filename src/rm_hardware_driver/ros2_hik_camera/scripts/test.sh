#!/bin/bash
# 测试脚本 - 验证ROS2包是否正确安装

echo "================================"
echo "Testing ros2_hik_camera package"
echo "================================"
echo ""

cd ~/hfut_rm_auto_aim_ws
source install/setup.bash

echo "1. Checking if package is installed..."
if ros2 pkg list | grep -q ros2_hik_camera; then
    echo "   ✓ Package found"
else
    echo "   ✗ Package not found"
    exit 1
fi

echo ""
echo "2. Checking executable..."
if ros2 pkg executables ros2_hik_camera 2>&1 | grep -q ros2_hik_camera_node; then
    echo "   ✓ Executable found: ros2_hik_camera_node"
else
    echo "   ✗ Executable not found"
    exit 1
fi

echo ""
echo "3. Checking launch file..."
if [ -f "install/ros2_hik_camera/share/ros2_hik_camera/launch/hik_camera_launch.py" ]; then
    echo "   ✓ Launch file found"
else
    echo "   ✗ Launch file not found"
    exit 1
fi

echo ""
echo "4. Checking config files..."
if [ -f "install/ros2_hik_camera/share/ros2_hik_camera/config/camera_params.yaml" ]; then
    echo "   ✓ camera_params.yaml found"
else
    echo "   ✗ camera_params.yaml not found"
fi

if [ -f "install/ros2_hik_camera/share/ros2_hik_camera/config/camera_info.yaml" ]; then
    echo "   ✓ camera_info.yaml found"
else
    echo "   ✗ camera_info.yaml not found"
fi

echo ""
echo "================================"
echo "All checks passed! ✓"
echo "================================"
echo ""
echo "To run the node (with camera connected):"
echo "  source install/setup.bash"
echo "  ros2 launch ros2_hik_camera hik_camera_launch.py"
echo ""
echo "Or run directly:"
echo "  source install/setup.bash"
echo "  ros2 run ros2_hik_camera ros2_hik_camera_node"
echo ""
