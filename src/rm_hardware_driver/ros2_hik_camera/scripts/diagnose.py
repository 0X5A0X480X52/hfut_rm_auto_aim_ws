#!/usr/bin/env python3
"""
HIKVision相机诊断工具
用于检测和诊断相机连接问题
"""

import subprocess
import sys
import os

def run_command(cmd):
    """运行命令并返回输出"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "Command timeout"
    except Exception as e:
        return -1, "", str(e)

def check_usb_devices():
    """检查USB设备"""
    print("=" * 60)
    print("1. 检查USB设备")
    print("=" * 60)
    
    ret, out, err = run_command("lsusb")
    if ret == 0:
        print(out)
        # 查找HIKVision设备 (通常vendor ID是2bdf)
        if "2bdf" in out.lower() or "hikvision" in out.lower():
            print("✓ 找到HIKVision USB设备")
            return True
        else:
            print("⚠ 未找到HIKVision USB设备")
            print("提示: HIKVision设备的Vendor ID通常是 2bdf")
            return False
    else:
        print(f"✗ lsusb命令失败: {err}")
        return False

def check_mvs_sdk():
    """检查MVS SDK安装"""
    print("\n" + "=" * 60)
    print("2. 检查HIKVision MVS SDK")
    print("=" * 60)
    
    paths_to_check = [
        "/opt/MVS/lib/64/libMvCameraControl.so",
        "/opt/MVS/lib/32/libMvCameraControl.so"
    ]
    
    found = False
    for path in paths_to_check:
        if os.path.exists(path):
            print(f"✓ 找到SDK库: {path}")
            # 检查库依赖
            ret, out, err = run_command(f"ldd {path} | grep 'not found'")
            if out:
                print(f"⚠ 库依赖缺失:\n{out}")
            found = True
            break
    
    if not found:
        print("✗ 未找到MVS SDK库文件")
        print("请运行: sudo MvCamCtrlSDK_Runtime-*/setup.sh")
        return False
    
    return True

def check_permissions():
    """检查USB权限"""
    print("\n" + "=" * 60)
    print("3. 检查USB权限")
    print("=" * 60)
    
    ret, out, err = run_command("groups")
    if ret == 0:
        groups = out.strip().split()
        if "video" in groups or "plugdev" in groups:
            print(f"✓ 用户在合适的组中: {', '.join(groups)}")
            return True
        else:
            print(f"⚠ 当前用户组: {', '.join(groups)}")
            print("建议添加用户到video和plugdev组:")
            print(f"  sudo usermod -aG video,plugdev $USER")
            print("  然后重新登录")
            return False
    return False

def check_library_path():
    """检查LD_LIBRARY_PATH"""
    print("\n" + "=" * 60)
    print("4. 检查LD_LIBRARY_PATH")
    print("=" * 60)
    
    ld_path = os.environ.get("LD_LIBRARY_PATH", "")
    if "/opt/MVS/lib" in ld_path:
        print(f"✓ LD_LIBRARY_PATH 包含MVS SDK路径")
        print(f"  {ld_path}")
        return True
    else:
        print(f"⚠ LD_LIBRARY_PATH 不包含MVS SDK路径")
        print(f"  当前: {ld_path if ld_path else '(空)'}")
        print("\n建议添加到 ~/.bashrc:")
        print('  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/MVS/lib/64')
        return False

def check_ros2_package():
    """检查ROS2包"""
    print("\n" + "=" * 60)
    print("5. 检查ROS2包")
    print("=" * 60)
    
    # Source ROS2 environment
    source_cmd = "source ~/hfut_rm_auto_aim_ws/install/setup.bash && "
    
    ret, out, err = run_command(source_cmd + "ros2 pkg list | grep ros2_hik_camera")
    if ret == 0 and "ros2_hik_camera" in out:
        print("✓ ros2_hik_camera 包已安装")
        
        # Check executable
        ret, out, err = run_command(source_cmd + "ros2 pkg executables ros2_hik_camera")
        if "ros2_hik_camera_node" in out:
            print("✓ ros2_hik_camera_node 可执行文件存在")
            return True
    else:
        print("✗ ros2_hik_camera 包未找到")
        print("请运行: cd ~/hfut_rm_auto_aim_ws && colcon build --packages-select ros2_hik_camera")
        return False
    
    return False

def print_recommendations():
    """打印建议"""
    print("\n" + "=" * 60)
    print("建议的解决步骤")
    print("=" * 60)
    print("""
1. 确保相机已正确连接到USB端口

2. 确保SDK环境变量已设置:
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/MVS/lib/64

3. 检查相机是否被其他程序占用:
   - 关闭HIKVision MVS软件
   - 检查是否有其他节点在运行

4. 尝试使用HIKVision官方工具测试:
   - 运行 MVS 软件确认相机可见

5. 如果仍然有问题，尝试:
   - 重新插拔相机USB
   - 重启计算机
   - 检查USB线缆质量

6. 对于 MV_E_NODATA (0x80000007) 错误:
   - 确认触发模式已关闭
   - 确认相机已开始采集
   - 增加超时时间
   - 检查帧率设置是否合理
    """)

def main():
    print("\n")
    print("╔" + "=" * 58 + "╗")
    print("║" + " " * 12 + "HIKVision 相机诊断工具" + " " * 24 + "║")
    print("╚" + "=" * 58 + "╝")
    print()
    
    results = []
    
    results.append(("USB设备", check_usb_devices()))
    results.append(("MVS SDK", check_mvs_sdk()))
    results.append(("USB权限", check_permissions()))
    results.append(("库路径", check_library_path()))
    results.append(("ROS2包", check_ros2_package()))
    
    print("\n" + "=" * 60)
    print("诊断结果汇总")
    print("=" * 60)
    
    for name, result in results:
        status = "✓ 通过" if result else "✗ 失败"
        print(f"{name:15s}: {status}")
    
    all_passed = all(r[1] for r in results)
    
    if all_passed:
        print("\n✓ 所有检查都通过！可以尝试运行相机节点。")
        print("\n运行命令:")
        print("  source ~/hfut_rm_auto_aim_ws/install/setup.bash")
        print("  ros2 run ros2_hik_camera ros2_hik_camera_node")
    else:
        print("\n⚠ 有一些问题需要解决")
        print_recommendations()
    
    print()

if __name__ == "__main__":
    main()
