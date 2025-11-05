#!/bin/bash
# 完整的相机测试和启动脚本

echo "================================"
echo "HIKVision Camera Test & Launch"
echo "================================"
echo ""

cd ~/hfut_rm_auto_aim_ws
source install/setup.bash

# 测试1: 低帧率 (10fps)
echo "Test 1: Running with 10 FPS (conservative)"
echo "-------------------------------------------"
ros2 run ros2_hik_camera ros2_hik_camera_node --ros-args \
  -p frame_rate:=10.0 \
  -p exposure_time:=5000.0 \
  -p gain:=8.0 &

PID=$!
sleep 15

if ps -p $PID > /dev/null; then
  echo "✓ Camera running successfully at 10 FPS!"
  echo ""
  echo "Checking topics..."
  ros2 topic list | grep -E "image|camera"
  echo ""
  echo "Getting image info..."
  timeout 2 ros2 topic echo /image_raw --no-arr | head -10
  
  kill $PID
  wait $PID 2>/dev/null
else
  echo "✗ Camera failed at 10 FPS"
fi

sleep 2
echo ""
echo "================================"
echo ""

# 测试2: 中等帧率 (30fps)
echo "Test 2: Running with 30 FPS (default)"
echo "--------------------------------------"
ros2 run ros2_hik_camera ros2_hik_camera_node --ros-args \
  -p frame_rate:=30.0 \
  -p exposure_time:=3000.0 \
  -p gain:=8.0 &

PID=$!
sleep 15

if ps -p $PID > /dev/null; then
  echo "✓ Camera running successfully at 30 FPS!"
  echo ""
  echo "Checking frame rate..."
  timeout 5 ros2 topic hz /image_raw
  
  kill $PID
  wait $PID 2>/dev/null
else
  echo "✗ Camera failed at 30 FPS"
fi

echo ""
echo "================================"
echo "Test Complete"
echo "================================"
echo ""
echo "If 10 FPS works but 30 FPS doesn't, use:"
echo "  ros2 run ros2_hik_camera ros2_hik_camera_node --ros-args -p frame_rate:=10.0"
echo ""
echo "Or modify config/camera_params.yaml to set frame_rate: 10.0"
echo ""
