#!/bin/bash
# watch_dog.sh - ROS2 watchdog for HFUT RM Auto Aim Project
# Optimized: clean SHM, disable FastDDS SHM, restart on heartbeat loss

echo "[WATCHDOG] START DOMAIN=$ROS_DOMAIN_ID"

# ===========================
# Configurable parameters
# ===========================
TIMEOUT=10                  # 心跳检测间隔（秒）
NAMESPACE=""                # ROS2 命名空间，例如 "/infantry_3"
NODE_NAMES=("armor_detector" "serial_driver" "gimbal_pipeline")  # 监控节点列表，用空格分隔
USER="$(whoami)"
HOME_DIR=$(eval echo ~$USER)
WORKING_DIR="$HOME_DIR/hfut_rm_auto_aim_ws/"  # 代码目录
LAUNCH_FILE="rm_bringup bringup_pipeline.launch.py"  # ROS2 launch 文件
OUTPUT_FILE="$WORKING_DIR/screen.output"  # 启动日志

# ===========================
# ROS2 / RMW 环境
# ===========================
rmw="rmw_fastrtps_cpp"          # RMW 实现，可改为 rmw_cyclonedds_cpp
export RMW_IMPLEMENTATION="$rmw"
export FASTDDS_SHM_DISABLE=1     # 禁用 FastDDS SHM 避免锁死问题
export ROS_DOMAIN_ID=10

export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"

source /opt/ros/humble/setup.bash
source $WORKING_DIR/install/setup.bash

# 可选 RMW 配置文件
rmw_config=""
if [[ "$rmw" == "rmw_fastrtps_cpp" && ! -z "$rmw_config" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE=$rmw_config
elif [[ "$rmw" == "rmw_cyclonedds_cpp" && ! -z "$rmw_config" ]]; then
    export CYCLONEDDS_URI=$rmw_config
fi

# ===========================
# SHM Cleanup function
# ===========================
function cleanup_shm() {
    echo "[WATCHDOG] Cleaning FastDDS SHM..."
    rm -rf /dev/shm/fastrtps_*
    rm -rf /dev/shm/sem.fastrtps_*
    echo "[WATCHDOG] SHM cleaned."
}

# ===========================
# Bringup function
# ===========================
function bringup() {
    echo "[WATCHDOG] Bringing up ROS2..."
    echo "[WATCHDOG] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
    
    # Source ROS2 and project environment
    source /opt/ros/humble/setup.bash
    source $WORKING_DIR/install/setup.bash
    # source /home/hfut-nuc/next_navigator/env.zsh
    # source /opt/intel/oneapi/setvars.sh
    source /opt/MVS/bin/set_env_path.sh

    cleanup_shm   # 启动前再清一次 SHM（保险）

    # USB 相机权限设置
    # USB_LINE=$(lsusb | grep "Hikrobot MV-CS016-10UC" | head -1)
    # if [ ! -z "$USB_LINE" ]; then
    #     echo "找到Hikrobot相机设备: $USB_LINE"
    #     BUS_NUM=$(echo "$USB_LINE" | sed -E 's/Bus ([0-9]+) Device ([0-9]+):.*/\1/')
    #     DEV_NUM=$(echo "$USB_LINE" | sed -E 's/Bus ([0-9]+) Device ([0-9]+):.*/\2/')
    #     BUS_NUM=$(printf "%03d" $BUS_NUM)
    #     DEV_NUM=$(printf "%03d" $DEV_NUM)
    #     USB_DEVICE="/dev/bus/usb/$BUS_NUM/$DEV_NUM"
    #     echo "设置USB设备权限: $USB_DEVICE"
    #     chmod 666 "$USB_DEVICE"
    # else
    #     echo "警告: 未找到Hikrobot MV-CS016-10UC相机设备"
    # fi

    nohup ros2 launch $LAUNCH_FILE > "$OUTPUT_FILE" 2>&1 &
    echo "[WATCHDOG] ROS2 launched, logging to $OUTPUT_FILE"
}

# ===========================
# Restart function
# ===========================
function restart() {
    echo "[WATCHDOG] Restarting ROS2 nodes..."

    # 杀干净 ROS2 / DDS 相关进程
    pkill -f ros2
    pkill -f component_container
    pkill -f rmw
    sleep 2
    pkill -9 -f ros2

    cleanup_shm

    # 重启 ROS2 daemon
    ros2 daemon stop
    sleep 1
    ros2 daemon start

    bringup
}

# ===========================
# Initial bringup
# ===========================
bringup
sleep $TIMEOUT
sleep $TIMEOUT  # 给节点稳定时间

# ===========================
# Heartbeat monitoring loop
# ===========================
while true; do
    for node in "${NODE_NAMES[@]}"; do
        topic="$NAMESPACE/$node/heartbeat"
        echo "[WATCHDOG] Checking $node heartbeat..."
        
        if ros2 topic list 2>/dev/null | grep -q $topic 2>/dev/null; then
            data_value=$(timeout 10 ros2 topic echo $topic --once | grep -o "data: [0-9]*" | awk '{print $2}' 2>/dev/null)
            if [ ! -z "$data_value" ]; then
                echo "    $node is OK! Heartbeat Count: $data_value"
            else
                echo "    Heartbeat lost for $topic, restarting all nodes..."
                restart
                break
            fi
        else
            echo "    Heartbeat topic $topic does not exist, restarting all nodes..."
            restart
            break
        fi
    done
    sleep $TIMEOUT
done