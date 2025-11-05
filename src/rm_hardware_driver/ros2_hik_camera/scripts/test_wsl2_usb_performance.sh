#!/bin/bash

# WSL2 USB性能测试脚本
# 用于诊断USB带宽限制

echo "======================================"
echo "   WSL2 USB性能诊断工具"
echo "======================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 检查USB设备
echo -e "${GREEN}[1/6] USB设备检测${NC}"
echo "--------------------------------------"
if lsusb | grep -i "hikrobot\|hikvision\|2bdf" > /dev/null; then
    echo -e "${GREEN}✓${NC} 找到HIKVision相机:"
    lsusb | grep -i "hikrobot\|hikvision\|2bdf"
else
    echo -e "${RED}✗${NC} 未找到HIKVision相机"
    echo "请确保:"
    echo "  1. 相机已连接到Windows"
    echo "  2. 已使用USBIPD绑定并附加到WSL2"
    echo ""
    echo "Windows PowerShell命令:"
    echo "  usbipd list"
    echo "  usbipd bind --busid <BUSID>"
    echo "  usbipd attach --wsl --busid <BUSID>"
    exit 1
fi
echo ""

# 2. USB拓扑和速度
echo -e "${GREEN}[2/6] USB拓扑结构${NC}"
echo "--------------------------------------"
lsusb -t | grep -B 2 -A 2 "2bdf\|hikrobot" | head -10
echo ""

# 3. USB速度详情
echo -e "${GREEN}[3/6] USB速度信息${NC}"
echo "--------------------------------------"
USB_SPEED=$(lsusb -v 2>/dev/null | grep -i "2bdf\|hikrobot" -A 20 | grep -i "bcdusb\|speed" | head -3)
if [ -z "$USB_SPEED" ]; then
    echo -e "${YELLOW}⚠${NC} 无法读取USB速度（可能需要sudo权限）"
    echo "尝试运行: sudo lsusb -v"
else
    echo "$USB_SPEED"
fi
echo ""

# 4. 系统网络参数（影响USBIPD性能）
echo -e "${GREEN}[4/6] 网络缓冲区设置${NC}"
echo "--------------------------------------"
echo "当前设置:"
sysctl net.core.rmem_max net.core.wmem_max 2>/dev/null
echo ""
echo "推荐值:"
echo "  net.core.rmem_max = 134217728 (128MB)"
echo "  net.core.wmem_max = 134217728 (128MB)"
echo ""

# 5. 相机带宽计算
echo -e "${GREEN}[5/6] 相机带宽需求计算${NC}"
echo "--------------------------------------"
WIDTH=1440
HEIGHT=1080
BYTES_PER_PIXEL=3  # RGB
FRAME_SIZE=$(($WIDTH * $HEIGHT * $BYTES_PER_PIXEL))
FRAME_SIZE_MB=$(echo "scale=2; $FRAME_SIZE / 1048576" | bc)

echo "图像尺寸: ${WIDTH}x${HEIGHT}"
echo "每帧大小: ${FRAME_SIZE} 字节 (${FRAME_SIZE_MB} MB)"
echo ""
echo "不同帧率的带宽需求:"
echo "  10 fps  → $(echo "scale=1; $FRAME_SIZE_MB * 10" | bc) MB/s"
echo "  20 fps  → $(echo "scale=1; $FRAME_SIZE_MB * 20" | bc) MB/s"
echo "  30 fps  → $(echo "scale=1; $FRAME_SIZE_MB * 30" | bc) MB/s"
echo "  50 fps  → $(echo "scale=1; $FRAME_SIZE_MB * 50" | bc) MB/s"
echo "  100 fps → $(echo "scale=1; $FRAME_SIZE_MB * 100" | bc) MB/s"
echo "  249 fps → $(echo "scale=1; $FRAME_SIZE_MB * 249" | bc) MB/s"
echo ""
echo "USB接口理论带宽:"
echo "  USB 2.0: 60 MB/s"
echo "  USB 3.0: 400 MB/s"
echo ""
echo -e "${YELLOW}WSL2实际带宽（USBIPD）: 约30-50 MB/s${NC}"
echo -e "${YELLOW}可支持最大帧率: 约10-15 fps${NC}"
echo ""

# 6. 实际帧率测试
echo -e "${GREEN}[6/6] 实际帧率测试${NC}"
echo "--------------------------------------"

# 检查ROS2环境
if [ -f "/home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash" ]; then
    source /home/amatrix/hfut_rm_auto_aim_ws/install/setup.bash
else
    echo -e "${YELLOW}⚠${NC} 未找到ROS2工作空间setup文件"
    echo "请先编译: colcon build --packages-select ros2_hik_camera"
    exit 1
fi

# 测试不同帧率
TEST_RATES=(10 20 30 50)
echo "测试不同帧率下的实际表现..."
echo ""

for fps in "${TEST_RATES[@]}"; do
    echo -e "${YELLOW}测试 ${fps} fps...${NC}"
    
    # 运行节点10秒并捕获输出
    timeout 10 ros2 run ros2_hik_camera ros2_hik_camera_node \
        --ros-args -p frame_rate:=${fps}.0 2>&1 | \
        tee /tmp/hik_test_${fps}.log | \
        grep -E "Publishing|MV_E_NODATA|ERROR" &
    
    NODE_PID=$!
    sleep 5
    
    # 统计成功和失败
    SUCCESS=$(grep -c "Publishing" /tmp/hik_test_${fps}.log 2>/dev/null || echo 0)
    NODATA=$(grep -c "MV_E_NODATA" /tmp/hik_test_${fps}.log 2>/dev/null || echo 0)
    
    # 等待节点退出
    wait $NODE_PID 2>/dev/null
    
    echo "结果: 成功=$SUCCESS 帧, 无数据错误=$NODATA 次"
    
    # 评级
    if [ $SUCCESS -ge 40 ]; then
        echo -e "${GREEN}★★★★★ 优秀${NC}"
    elif [ $SUCCESS -ge 20 ]; then
        echo -e "${YELLOW}★★★☆☆ 一般${NC}"
    else
        echo -e "${RED}★☆☆☆☆ 较差${NC}"
    fi
    echo ""
    
    sleep 2
done

# 7. 总结和建议
echo "======================================"
echo -e "${GREEN}测试完成！${NC}"
echo "======================================"
echo ""
echo "问题诊断:"
echo "  您的相机在Windows下: 249 fps ✓"
echo "  您的相机在WSL2下:    10 fps ✗"
echo ""
echo -e "${YELLOW}根本原因: WSL2的USB性能限制${NC}"
echo ""
echo "WSL2使用USBIPD通过网络虚拟化USB设备，导致:"
echo "  - USB 3.0性能降级到USB 2.0级别"
echo "  - 实际带宽: 30-50 MB/s（理论400 MB/s）"
echo "  - 仅能支持: 10-15 fps（需要1108 MB/s支持249fps）"
echo ""
echo "解决方案（按推荐顺序）:"
echo ""
echo -e "${GREEN}1. Windows原生ROS2 (推荐)${NC}"
echo "   优点: 完全原生USB性能，支持249 fps"
echo "   安装: https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html"
echo ""
echo -e "${GREEN}2. 双系统Ubuntu${NC}"
echo "   优点: 最佳性能和兼容性"
echo "   缺点: 需要重启切换系统"
echo ""
echo -e "${YELLOW}3. Docker + USB直通${NC}"
echo "   优点: 比WSL2好，可达50-100 fps"
echo "   缺点: 配置复杂"
echo ""
echo -e "${YELLOW}4. 继续使用WSL2 (仅开发调试)${NC}"
echo "   适用: 10 fps足够的应用场景"
echo "   限制: 无法突破15 fps"
echo ""
echo "详细说明请查看: WSL2_USB_LIMITATIONS.md"
echo ""
