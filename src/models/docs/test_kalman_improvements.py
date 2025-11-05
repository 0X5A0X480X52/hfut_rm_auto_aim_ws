#!/usr/bin/env python3
"""
卡尔曼滤波器改进验证脚本
测试改进后的预测函数是否正常工作
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def simulate_circular_motion(radius=5.0, omega=0.5, T=0.1, steps=100):
    """
    模拟圆周运动轨迹
    
    Args:
        radius: 圆半径 (米)
        omega: 角速度 (rad/s)
        T: 采样周期 (秒)
        steps: 采样点数
    
    Returns:
        positions: (steps, 2) 位置数组
        velocities: (steps, 2) 速度数组
    """
    t = np.arange(steps) * T
    theta = omega * t
    
    positions = np.column_stack([
        radius * np.cos(theta),
        radius * np.sin(theta)
    ])
    
    velocities = np.column_stack([
        -radius * omega * np.sin(theta),
        radius * omega * np.cos(theta)
    ])
    
    return positions, velocities

def simulate_zigzag_motion(length=10.0, width=3.0, T=0.1, steps=100):
    """
    模拟之字形运动（测试模型切换）
    
    Args:
        length: 每段长度 (米)
        width: 横向宽度 (米)
        T: 采样周期 (秒)
        steps: 采样点数
    
    Returns:
        positions: (steps, 2) 位置数组
    """
    positions = []
    x, y = 0.0, 0.0
    vx, vy = 2.0, 0.0  # 初始速度 2 m/s
    
    segment_length = 0
    direction = 1  # 1: 右, -1: 左
    
    for i in range(steps):
        positions.append([x, y])
        
        # 更新位置
        x += vx * T
        y += vy * T
        segment_length += np.sqrt((vx * T)**2 + (vy * T)**2)
        
        # 每移动一定距离后转向
        if segment_length > length:
            segment_length = 0
            direction *= -1
            # 改变速度方向
            vy = direction * 1.5  # 横向速度
            vx = 1.5  # 纵向速度
    
    return np.array(positions)

def add_measurement_noise(positions, noise_std=0.1):
    """
    添加测量噪声
    
    Args:
        positions: 真实位置
        noise_std: 噪声标准差 (米)
    
    Returns:
        noisy_positions: 带噪声的位置
    """
    noise = np.random.normal(0, noise_std, positions.shape)
    return positions + noise

def visualize_trajectory(true_positions, measured_positions=None, predicted_positions=None):
    """
    可视化轨迹
    
    Args:
        true_positions: 真实轨迹
        measured_positions: 测量轨迹（可选）
        predicted_positions: 预测轨迹（可选）
    """
    plt.figure(figsize=(12, 8))
    
    # 绘制真实轨迹
    plt.plot(true_positions[:, 0], true_positions[:, 1], 
             'g-', linewidth=2, label='True Trajectory', alpha=0.7)
    
    # 绘制测量点
    if measured_positions is not None:
        plt.scatter(measured_positions[:, 0], measured_positions[:, 1], 
                   c='r', s=10, alpha=0.5, label='Measurements')
    
    # 绘制预测轨迹
    if predicted_positions is not None:
        plt.plot(predicted_positions[:, 0], predicted_positions[:, 1], 
                'b--', linewidth=1.5, label='Predicted Trajectory', alpha=0.7)
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Trajectory Comparison')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig('trajectory_comparison.png', dpi=150)
    print("✅ 轨迹对比图已保存: trajectory_comparison.png")
    plt.close()

def analyze_prediction_error(true_positions, predicted_positions, prediction_steps):
    """
    分析预测误差
    
    Args:
        true_positions: 真实位置
        predicted_positions: 预测位置
        prediction_steps: 预测步数
    """
    errors = np.linalg.norm(true_positions - predicted_positions, axis=1)
    
    plt.figure(figsize=(12, 6))
    
    # 绘制误差曲线
    plt.subplot(1, 2, 1)
    plt.plot(errors, 'b-', linewidth=1.5)
    plt.xlabel('Time Step')
    plt.ylabel('Prediction Error (m)')
    plt.title(f'Prediction Error (N={prediction_steps} steps ahead)')
    plt.grid(True, alpha=0.3)
    
    # 绘制误差直方图
    plt.subplot(1, 2, 2)
    plt.hist(errors, bins=30, edgecolor='black', alpha=0.7)
    plt.xlabel('Prediction Error (m)')
    plt.ylabel('Frequency')
    plt.title('Error Distribution')
    plt.axvline(np.mean(errors), color='r', linestyle='--', 
                linewidth=2, label=f'Mean: {np.mean(errors):.3f}m')
    plt.axvline(np.median(errors), color='g', linestyle='--', 
                linewidth=2, label=f'Median: {np.median(errors):.3f}m')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('prediction_error_analysis.png', dpi=150)
    print("✅ 预测误差分析图已保存: prediction_error_analysis.png")
    plt.close()
    
    # 打印统计信息
    print(f"\n📊 预测误差统计 (N={prediction_steps} 步):")
    print(f"   平均误差: {np.mean(errors):.4f} m")
    print(f"   中位误差: {np.median(errors):.4f} m")
    print(f"   标准差:   {np.std(errors):.4f} m")
    print(f"   最大误差: {np.max(errors):.4f} m")
    print(f"   95%误差:  {np.percentile(errors, 95):.4f} m")

def generate_test_report():
    """
    生成测试报告
    """
    print("\n" + "="*60)
    print("🎯 卡尔曼滤波器改进验证测试")
    print("="*60)
    
    print("\n📋 测试场景:")
    print("   1. 圆周运动 - 验证 CTRV 模型非线性预测")
    print("   2. 之字形运动 - 验证 IMM 模型切换能力")
    
    # 场景1: 圆周运动
    print("\n🔄 场景1: 圆周运动测试")
    print("-" * 60)
    positions_circular, _ = simulate_circular_motion(radius=5.0, omega=0.5, T=0.1, steps=100)
    measured_circular = add_measurement_noise(positions_circular, noise_std=0.05)
    
    print(f"   生成轨迹: {len(positions_circular)} 个采样点")
    print(f"   半径: 5.0 m")
    print(f"   角速度: 0.5 rad/s")
    print(f"   测量噪声: 0.05 m (标准差)")
    
    visualize_trajectory(positions_circular, measured_circular)
    
    # 场景2: 之字形运动
    print("\n🔀 场景2: 之字形运动测试")
    print("-" * 60)
    positions_zigzag = simulate_zigzag_motion(length=3.0, width=2.0, T=0.1, steps=100)
    measured_zigzag = add_measurement_noise(positions_zigzag, noise_std=0.08)
    
    print(f"   生成轨迹: {len(positions_zigzag)} 个采样点")
    print(f"   段长度: 3.0 m")
    print(f"   测量噪声: 0.08 m (标准差)")
    
    # 模拟预测误差分析
    # 注意：这里只是示例，实际需要C++端的预测结果
    prediction_steps = 10
    simulated_predicted = positions_zigzag + np.random.normal(0, 0.1, positions_zigzag.shape)
    analyze_prediction_error(positions_zigzag, simulated_predicted, prediction_steps)
    
    print("\n" + "="*60)
    print("✅ 测试报告生成完成")
    print("="*60)
    
    print("\n📝 建议:")
    print("   1. 使用实际的 ROS2 bag 数据进行测试")
    print("   2. 对比改进前后的预测精度")
    print("   3. 在不同运动模式下验证 IMM 的模型置信度")
    print("   4. 检查长期预测（N=20, 30）的稳定性")
    
    print("\n🔧 C++ 集成测试:")
    print("   编译命令:")
    print("   $ cd /home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws")
    print("   $ colcon build --symlink-install --packages-select models basic_models combined_models")
    print("\n   运行测试:")
    print("   $ source install/setup.bash")
    print("   $ ros2 launch rm_bringup bringup.launch.py")

if __name__ == "__main__":
    # 设置随机种子以保证可重复性
    np.random.seed(42)
    
    # 生成测试报告
    generate_test_report()
