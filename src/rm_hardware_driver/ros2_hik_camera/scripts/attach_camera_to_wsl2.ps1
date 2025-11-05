# Windows PowerShell脚本 - USB设备管理工具
# 用于将HIKVision相机附加到WSL2

Write-Host "======================================" -ForegroundColor Green
Write-Host "   HIKVision相机 WSL2 绑定工具" -ForegroundColor Green
Write-Host "======================================" -ForegroundColor Green
Write-Host ""

# 检查是否以管理员权限运行
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)

if (-not $isAdmin) {
    Write-Host "错误: 需要管理员权限运行此脚本" -ForegroundColor Red
    Write-Host "请右键点击PowerShell，选择'以管理员身份运行'" -ForegroundColor Yellow
    Read-Host "按Enter键退出"
    exit
}

# 检查USBIPD是否安装
Write-Host "[1/5] 检查USBIPD安装状态..." -ForegroundColor Cyan
$usbipd = Get-Command usbipd -ErrorAction SilentlyContinue

if (-not $usbipd) {
    Write-Host "✗ USBIPD未安装" -ForegroundColor Red
    Write-Host ""
    Write-Host "安装方法:" -ForegroundColor Yellow
    Write-Host "  1. 使用winget安装 (推荐):" -ForegroundColor White
    Write-Host "     winget install usbipd" -ForegroundColor Gray
    Write-Host ""
    Write-Host "  2. 手动下载安装:" -ForegroundColor White
    Write-Host "     https://github.com/dorssel/usbipd-win/releases" -ForegroundColor Gray
    Write-Host ""
    Read-Host "按Enter键退出"
    exit
} else {
    Write-Host "✓ USBIPD已安装" -ForegroundColor Green
}
Write-Host ""

# 列出所有USB设备
Write-Host "[2/5] 扫描USB设备..." -ForegroundColor Cyan
Write-Host "--------------------------------------" -ForegroundColor Gray
$devices = usbipd list
Write-Host $devices
Write-Host ""

# 查找HIKVision相机
Write-Host "[3/5] 查找HIKVision相机..." -ForegroundColor Cyan
$hikDevice = $devices | Select-String -Pattern "hikrobot|hikvision|2bdf:0001" -CaseSensitive:$false

if (-not $hikDevice) {
    Write-Host "✗ 未找到HIKVision相机" -ForegroundColor Red
    Write-Host ""
    Write-Host "请检查:" -ForegroundColor Yellow
    Write-Host "  1. 相机是否已连接到电脑" -ForegroundColor White
    Write-Host "  2. 相机驱动是否已安装" -ForegroundColor White
    Write-Host "  3. 设备管理器中是否能看到相机" -ForegroundColor White
    Write-Host ""
    Read-Host "按Enter键退出"
    exit
}

# 解析BUSID
$busid = ""
if ($hikDevice -match "(\d+-\d+)") {
    $busid = $Matches[1]
    Write-Host "✓ 找到HIKVision相机: BUSID = $busid" -ForegroundColor Green
} else {
    Write-Host "✗ 无法解析BUSID" -ForegroundColor Red
    Read-Host "按Enter键退出"
    exit
}
Write-Host ""

# 检查绑定状态
Write-Host "[4/5] 检查绑定状态..." -ForegroundColor Cyan
$deviceInfo = usbipd list | Select-String -Pattern $busid

if ($deviceInfo -match "Shared") {
    Write-Host "✓ 设备已绑定" -ForegroundColor Green
} else {
    Write-Host "○ 设备未绑定，正在绑定..." -ForegroundColor Yellow
    try {
        usbipd bind --busid $busid
        Write-Host "✓ 绑定成功" -ForegroundColor Green
    } catch {
        Write-Host "✗ 绑定失败: $_" -ForegroundColor Red
        Read-Host "按Enter键退出"
        exit
    }
}
Write-Host ""

# 附加到WSL2
Write-Host "[5/5] 附加设备到WSL2..." -ForegroundColor Cyan

# 检查WSL状态
$wslStatus = wsl --list --running
if ($wslStatus -match "Ubuntu") {
    Write-Host "✓ WSL2正在运行" -ForegroundColor Green
} else {
    Write-Host "⚠ WSL2未运行，正在启动..." -ForegroundColor Yellow
    wsl -d Ubuntu /bin/true
    Start-Sleep -Seconds 2
}

# 附加设备
Write-Host "正在附加设备 $busid 到WSL2..." -ForegroundColor White
try {
    usbipd attach --wsl --busid $busid
    Write-Host "✓ 附加成功！" -ForegroundColor Green
} catch {
    Write-Host "✗ 附加失败: $_" -ForegroundColor Red
    Write-Host ""
    Write-Host "可能的原因:" -ForegroundColor Yellow
    Write-Host "  1. 设备正在被其他程序使用" -ForegroundColor White
    Write-Host "  2. WSL2内核不支持USB" -ForegroundColor White
    Write-Host "  3. 需要更新WSL版本: wsl --update" -ForegroundColor White
    Read-Host "按Enter键退出"
    exit
}

Write-Host ""
Write-Host "======================================" -ForegroundColor Green
Write-Host "   配置完成！" -ForegroundColor Green
Write-Host "======================================" -ForegroundColor Green
Write-Host ""
Write-Host "下一步操作:" -ForegroundColor Cyan
Write-Host ""
Write-Host "1. 在WSL2中验证设备:" -ForegroundColor White
Write-Host "   wsl lsusb" -ForegroundColor Gray
Write-Host ""
Write-Host "2. 运行性能测试:" -ForegroundColor White
Write-Host "   wsl ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera/scripts/test_wsl2_usb_performance.sh" -ForegroundColor Gray
Write-Host ""
Write-Host "3. 启动ROS2相机节点:" -ForegroundColor White
Write-Host "   wsl" -ForegroundColor Gray
Write-Host "   cd ~/hfut_rm_auto_aim_ws" -ForegroundColor Gray
Write-Host "   source install/setup.bash" -ForegroundColor Gray
Write-Host "   ros2 run ros2_hik_camera ros2_hik_camera_node" -ForegroundColor Gray
Write-Host ""
Write-Host "⚠ 重要提示:" -ForegroundColor Yellow
Write-Host "WSL2的USB性能有限，最大约支持10-15 fps" -ForegroundColor Red
Write-Host "如需完整性能(249 fps)，建议使用:" -ForegroundColor Yellow
Write-Host "  - Windows原生ROS2" -ForegroundColor White
Write-Host "  - Ubuntu双系统" -ForegroundColor White
Write-Host ""
Write-Host "详细说明: ~/hfut_rm_auto_aim_ws/src/rm_hardware_driver/ros2_hik_camera/WSL2_USB_LIMITATIONS.md" -ForegroundColor Gray
Write-Host ""

# 保持设备附加
Write-Host "如需断开设备，请运行:" -ForegroundColor Cyan
Write-Host "  usbipd detach --busid $busid" -ForegroundColor Gray
Write-Host ""
Read-Host "按Enter键退出"
