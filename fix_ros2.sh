#!/bin/bash
set -e

echo "🔧 Restarting ROS2 daemon and DDS..."

# 定义超时时间（秒）
DAEMON_STOP_TIMEOUT=5

# Function: stop ros2 daemon with timeout
stop_ros2_daemon() {
    echo "🛑 Stopping ROS2 daemon..."
    
    # 后台执行 stop
    ros2 daemon stop &
    STOP_PID=$!
    
    # 等待指定超时时间
    SECONDS_PASSED=0
    while kill -0 $STOP_PID 2>/dev/null; do
        sleep 1
        ((SECONDS_PASSED++))
        if [ $SECONDS_PASSED -ge $DAEMON_STOP_TIMEOUT ]; then
            echo "⚠️ ros2 daemon stop timeout after ${DAEMON_STOP_TIMEOUT}s"
            
            # 检查是否存在 ros2daemon 进程
            DAEMON_PID=$(pgrep -f ros2daemon || true)
            if [ -n "$DAEMON_PID" ]; then
                echo "💀 ros2daemon process still running (PID: $DAEMON_PID), killing..."
                sudo kill -9 $DAEMON_PID
            else
                echo "✅ No ros2daemon process found"
            fi
            
            # 杀掉后台 stop 命令
            kill -9 $STOP_PID 2>/dev/null || true
            break
        fi
    done

    wait $STOP_PID 2>/dev/null || true
    echo "🛑 ROS2 daemon stopped."
}

stop_ros2_daemon

set +e

echo "🛑 Stopping existing ROS2 and DDS processes..."
sudo pkill -9 -f fastdds || true
sudo pkill -9 -f cyclonedds || true
sudo pkill -9 -f rmw || true
sudo pkill -9 -f ros2 || true

echo "🧹 Cleaning up shared memory segments..."
sudo rm -rf /dev/shm/ros2* || true
sudo rm -rf /dev/shm/fast* || true

echo "🚀 Starting ROS2 daemon..."
ros2 daemon start

echo "✅ ROS2 services restarted."

echo "ℹ️ Listing ROS2 topics..."
ros2 topic list
