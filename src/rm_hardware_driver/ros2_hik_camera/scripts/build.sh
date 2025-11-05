#!/bin/bash
# 构建和测试脚本

echo "================================"
echo "Building ros2_hik_camera package"
echo "================================"

cd /home/amatrix/hfut_rm_auto_aim_ws

# 清理之前的构建（可选）
# rm -rf build/ros2_hik_camera install/ros2_hik_camera

# 构建包
colcon build --packages-select ros2_hik_camera --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "================================"
    echo "Build successful!"
    echo "================================"
    echo ""
    echo "To use the package, run:"
    echo "  source install/setup.bash"
    echo "  ros2 launch ros2_hik_camera hik_camera_launch.py"
    echo ""
else
    echo ""
    echo "================================"
    echo "Build failed! Check errors above."
    echo "================================"
    echo ""
    exit 1
fi
