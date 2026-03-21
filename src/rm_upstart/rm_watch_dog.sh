#!/bin/bash
# watch_dog.sh

TIMEOUT=10  # 设定超时时间为10秒
NAMESPACE="" # 命名空间 例如 "/infantry_3" 注意要有"/"
# NODE_NAMES=("armor_detector" "armor_solver" "serial_driver" "camera_driver")  # 列出所有需要监控的节点名称，注意是用空格分隔
NODE_NAMES=("armor_detector")  # 列出所有需要监控的节点名称，注意是用空格分隔
USER="$(whoami)" #用户名
HOME_DIR=$(eval echo ~$USER)
WORKING_DIR="$HOME_DIR/hfut_rm_auto_aim_ws/" # 代码目录 
# LAUNCH_FILE="rm_bringup bringup.launch.py" # launch 文件
LAUNCH_FILE="rm_bringup bringup_pipeline.launch.py" # launch 文件
OUTPUT_FILE="$WORKING_DIR/screen.output" # 终端输出记录文件

rmw="rmw_fastrtps_cpp" #RMW
export RMW_IMPLEMENTATION="$rmw" # RMW实现

export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"

source /opt/ros/humble/setup.bash
source $WORKING_DIR/install/setup.bash

rmw_config=""
if [[ "$rmw" == "rmw_fastrtps_cpp" ]]
then
  if [[ ! -z $rmw_config ]]
  then
    export FASTRTPS_DEFAULT_PROFILES_FILE=$rmw_config
  fi
elif [[ "$rmw" == "rmw_cyclonedds_cpp" ]]
then
  if [[ ! -z $rmw_config ]]
  then
    export CYCLONEDDS_URI=$rmw_config
  fi
fi

function bringup() {
    source /opt/ros/humble/setup.bash
    source $WORKING_DIR/install/setup.bash
    source /home/hfut-nuc/next_navigator/env.zsh
    source /opt/intel/oneapi/setvars.sh
    source /opt/MVS/bin/set_env_path.sh
    
    # 设置串口权限
    #sudo chmod 666 /dev/ttyMIAO
    
    # 检测并设置Hikrobot相机USB设备权限
    USB_LINE=$(lsusb | grep "Hikrobot MV-CS016-10UC" | head -1)
    if [ ! -z "$USB_LINE" ]; then
        echo "找到Hikrobot相机设备: $USB_LINE"
        BUS_NUM=$(echo "$USB_LINE" | sed -E 's/Bus ([0-9]+) Device ([0-9]+):.*/\1/')
        DEV_NUM=$(echo "$USB_LINE" | sed -E 's/Bus ([0-9]+) Device ([0-9]+):.*/\2/')
        
        # 格式化总线号和设备号为3位数字（如004, 002）
        BUS_NUM=$(printf "%03d" $BUS_NUM)
        DEV_NUM=$(printf "%03d" $DEV_NUM)
        
        USB_DEVICE="/dev/bus/usb/$BUS_NUM/$DEV_NUM"
        echo "设置USB设备权限: $USB_DEVICE"
        chmod 666 "$USB_DEVICE"
    else
        echo "警告: 未找到Hikrobot MV-CS016-10UC相机设备"
    fi

    nohup ros2 launch $LAUNCH_FILE > "$OUTPUT_FILE" 2>&1 &
}

function restart() {
    PID=$(ps -eo pid,cmd | grep "[/]opt/ros/humble/bin/ros2 launch rm_bringup bringup_pipeline.launch.py" | awk '{print $1}') && echo $PID
    #pkill -f ros  # 杀掉所有ROS2进程
    kill $PID
    sleep 2
    kill -9 $PID
    #ros2 daemon stop
    ros2 daemon start
    bringup
}

bringup
sleep $TIMEOUT
sleep $TIMEOUT

# 监控每个节点的心跳
while true; do
    for node in "${NODE_NAMES[@]}"; do
        topic="$NAMESPACE/$node/heartbeat"
        echo "- Check $node"
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
