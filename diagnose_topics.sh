#!/bin/bash
# 诊断脚本：检查话题连接和数据流

echo "=== Checking topic list ==="
ros2 topic list | grep -E "(tracked_robots|selected_target|gimbal_cmd)"

echo -e "\n=== Checking TrackedRobots publishers/subscribers ==="
ros2 topic info /max_entropy_tracker/tracked_robots

echo -e "\n=== Checking SelectedTarget publishers/subscribers ==="
ros2 topic info /target_selector/selected_target

echo -e "\n=== Checking GimbalCmd publishers/subscribers ==="
ros2 topic info /trajectory_planner/gimbal_cmd

echo -e "\n=== Echoing SelectedTarget (5 seconds) ==="
timeout 5 ros2 topic echo /target_selector/selected_target --once

echo -e "\n=== Echoing GimbalCmd (5 seconds) ==="
timeout 5 ros2 topic echo /trajectory_planner/gimbal_cmd --once

echo -e "\n=== Checking service list ==="
ros2 service list | grep ballistic

echo -e "\n=== Done ==="
