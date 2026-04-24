"""
输出平滑器功能测试

测试 OutputSmoother 对抖动输出的消抖效果:
1. OneEuro 滤波器基本功能
2. 结构参数收敛估计器
3. 集成到 AdaptiveArmorTracker 的完整流程
"""

import numpy as np
import sys
import os

# 确保能导入
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from max_entropy_tracker.utils.one_euro_filter import (
    OneEuroFilter, OneEuroFilter3D, OneEuroFilterAngle
)
from max_entropy_tracker.utils.structural_estimator import (
    ScalarBayesianEstimator, StructuralParameterEstimator
)
from max_entropy_tracker.utils.output_smoother import (
    OutputSmoother, SmootherConfig, SmoothedOutput
)


def test_one_euro_filter_basic():
    """测试 OneEuro 标量滤波器消除噪声"""
    print("=" * 60)
    print("Test 1: OneEuro Filter - 标量消抖")
    
    f = OneEuroFilter(freq=30.0, min_cutoff=1.0, beta=0.007)
    
    np.random.seed(42)
    n = 100
    # 真值: 缓慢变化的正弦
    t = np.linspace(0, 2 * np.pi, n)
    true_vals = np.sin(t) * 0.5
    # 添加噪声 (模拟检测抖动)
    noisy_vals = true_vals + np.random.normal(0, 0.1, n)
    
    filtered = []
    for i in range(n):
        filtered.append(f.filter(noisy_vals[i], timestamp=t[i]))
    filtered = np.array(filtered)
    
    # 计算误差
    raw_rmse = np.sqrt(np.mean((noisy_vals[20:] - true_vals[20:]) ** 2))
    filt_rmse = np.sqrt(np.mean((filtered[20:] - true_vals[20:]) ** 2))
    
    print(f"  原始噪声 RMSE: {raw_rmse:.4f}")
    print(f"  滤波后 RMSE:   {filt_rmse:.4f}")
    print(f"  RMSE 降低:     {(1 - filt_rmse / raw_rmse) * 100:.1f}%")
    
    assert filt_rmse < raw_rmse, "滤波后误差应小于原始噪声"
    print("  ✓ PASS\n")


def test_one_euro_angle():
    """测试角度滤波器的环绕处理"""
    print("=" * 60)
    print("Test 2: OneEuro Angle Filter - 角度环绕处理")
    
    f = OneEuroFilterAngle(freq=30.0, min_cutoff=1.0, beta=0.001)
    
    # 角度从 π-0.1 到 π+0.1 (穿越 ±π 边界)
    angles = np.linspace(np.pi - 0.3, np.pi + 0.3, 30)
    # 归一化到 [-π, π]
    angles = np.array([a if a <= np.pi else a - 2 * np.pi for a in angles])
    # 加噪声
    np.random.seed(123)
    noisy = angles + np.random.normal(0, 0.05, len(angles))
    
    filtered = []
    for i, a in enumerate(noisy):
        filtered.append(f.filter(a, timestamp=i / 30.0))
    filtered = np.array(filtered)
    
    # 检查输出不会出现跳变（使用角度差计算，考虑环绕）
    angle_diffs = []
    for i in range(1, len(filtered)):
        d = abs(np.arctan2(np.sin(filtered[i] - filtered[i-1]), 
                           np.cos(filtered[i] - filtered[i-1])))
        angle_diffs.append(d)
    max_jump = max(angle_diffs)
    print(f"  最大帧间跳变: {np.degrees(max_jump):.2f}°")
    assert max_jump < np.pi / 2, "滤波输出不应出现大角度跳变"
    print("  ✓ PASS\n")


def test_structural_estimator_convergence():
    """测试结构参数估计器收敛行为"""
    print("=" * 60)
    print("Test 3: Structural Parameter Estimator - 参数收敛")
    
    est = StructuralParameterEstimator(
        base_process_noise=1e-5,
        decay_process_noise=0.01,
        decay_rate=0.95,
    )
    
    # 真值
    true_r1, true_r2, true_dza = 0.18, 0.22, 0.05
    
    # 初始值 (有偏差)
    est.initialize(r1=0.15, r2=0.20, dza=0.0)
    
    np.random.seed(42)
    n_steps = 100
    r1_history = []
    r2_history = []
    dza_history = []
    
    for i in range(n_steps):
        est.predict()
        
        # 模拟双观测更新 (每2帧一次，增加更新频率)
        if i % 2 == 0:
            # 有噪声的观测
            r1_obs = true_r1 + np.random.normal(0, 0.02)
            r2_obs = true_r2 + np.random.normal(0, 0.02)
            dza_obs = true_dza + np.random.normal(0, 0.01)
            
            # 模拟 UKF 协方差中的对应方差
            fake_P = np.eye(11) * 0.005  # 较小的协方差=更可信的观测
            est.update_from_ukf(
                r1_ukf=r1_obs, r2_ukf=r2_obs, dza_ukf=dza_obs,
                P=fake_P,
                r1_idx=8, r2_idx=9, dza_idx=10,
                is_dual_obs=True,
            )
        
        r1_history.append(est.r1_est.mu)
        r2_history.append(est.r2_est.mu)
        dza_history.append(est.dza_est.mu)
    
    r1_final = est.r1_est.mu
    r2_final = est.r2_est.mu
    dza_final = est.dza_est.mu
    
    print(f"  初始偏差: r1={abs(0.15 - true_r1):.3f}, r2={abs(0.20 - true_r2):.3f}, dza={abs(0.0 - true_dza):.3f}")
    print(f"  收敛结果: r1={r1_final:.4f} (真值{true_r1}), r2={r2_final:.4f} (真值{true_r2}), dza={dza_final:.4f} (真值{true_dza})")
    print(f"  最终偏差: r1={abs(r1_final - true_r1):.4f}, r2={abs(r2_final - true_r2):.4f}, dza={abs(dza_final - true_dza):.4f}")
    print(f"  方差: r1={est.r1_est.variance:.6f}, r2={est.r2_est.variance:.6f}, dza={est.dza_est.variance:.6f}")
    print(f"  收敛状态: {est.is_converged()}")
    
    assert abs(r1_final - true_r1) < 0.03, f"r1 应收敛到真值附近: got {r1_final}"
    assert abs(r2_final - true_r2) < 0.03, f"r2 应收敛到真值附近: got {r2_final}"
    assert abs(dza_final - true_dza) < 0.03, f"dza 应收敛到真值附近: got {dza_final}"
    print("  ✓ PASS\n")


def test_output_smoother_jitter_reduction():
    """测试 OutputSmoother 对输出抖动的综合消除效果"""
    print("=" * 60)
    print("Test 4: OutputSmoother - 综合消抖效果")
    
    config = SmootherConfig(
        pos_min_cutoff=1.5,
        pos_beta=0.01,
        yaw_min_cutoff=1.0,
        yaw_beta=0.005,
        vel_min_cutoff=2.0,
        vel_beta=0.01,
    )
    smoother = OutputSmoother(config)
    smoother.initialize(r1=0.15, r2=0.20, dza=0.0)
    
    np.random.seed(42)
    n = 80
    dt = 1.0 / 30.0
    
    # 真值: 匀速运动目标
    true_positions = np.zeros((n, 3))
    true_yaws = np.zeros(n)
    for i in range(n):
        t = i * dt
        true_positions[i] = [2.0 + 0.3 * t, 1.0, 0.5]  # x方向匀速
        true_yaws[i] = 0.5 + 0.1 * t  # yaw缓慢变化
    
    # 模拟抖动
    noisy_pos = true_positions + np.random.normal(0, 0.03, (n, 3))
    noisy_yaw = true_yaws + np.random.normal(0, 0.05, n)
    
    raw_jitter_pos = []
    smooth_jitter_pos = []
    raw_jitter_yaw = []
    smooth_jitter_yaw = []
    
    for i in range(n):
        timestamp = i * dt
        velocity = np.array([0.3, 0.0, 0.0])  # 近似速度
        vel_noisy = velocity + np.random.normal(0, 0.05, 3)
        
        result = smoother.smooth(
            center_pos=noisy_pos[i],
            yaw=noisy_yaw[i],
            velocity=vel_noisy,
            yaw_velocity=0.1,
            r1=0.18, r2=0.22, dza=0.05,
            is_dual_obs=(i % 5 == 0),
            timestamp=timestamp,
        )
        
        if i > 0:
            # 帧间跳变 (抖动量)
            raw_jitter_pos.append(np.linalg.norm(noisy_pos[i] - noisy_pos[i-1]))
            smooth_jitter_pos.append(np.linalg.norm(result.center_position - prev_smooth_pos))
            raw_jitter_yaw.append(abs(noisy_yaw[i] - noisy_yaw[i-1]))
            smooth_jitter_yaw.append(abs(result.yaw - prev_smooth_yaw))
        
        prev_smooth_pos = result.center_position.copy()
        prev_smooth_yaw = result.yaw
    
    avg_raw_jitter_pos = np.mean(raw_jitter_pos)
    avg_smooth_jitter_pos = np.mean(smooth_jitter_pos)
    avg_raw_jitter_yaw = np.mean(raw_jitter_yaw)
    avg_smooth_jitter_yaw = np.mean(smooth_jitter_yaw)
    
    print(f"  位置平均帧间抖动: 原始={avg_raw_jitter_pos:.4f}, 平滑后={avg_smooth_jitter_pos:.4f}")
    print(f"  Yaw平均帧间抖动:  原始={np.degrees(avg_raw_jitter_yaw):.2f}°, 平滑后={np.degrees(avg_smooth_jitter_yaw):.2f}°")
    print(f"  位置抖动降低: {(1 - avg_smooth_jitter_pos / avg_raw_jitter_pos) * 100:.1f}%")
    print(f"  Yaw抖动降低:  {(1 - avg_smooth_jitter_yaw / avg_raw_jitter_yaw) * 100:.1f}%")
    
    assert avg_smooth_jitter_pos < avg_raw_jitter_pos, "平滑后位置抖动应减小"
    assert avg_smooth_jitter_yaw < avg_raw_jitter_yaw, "平滑后Yaw抖动应减小"
    print("  ✓ PASS\n")


def test_one_euro_speed_response():
    """测试 OneEuro 滤波器对突变运动的跟随能力"""
    print("=" * 60)
    print("Test 5: OneEuro Filter - 速度响应能力")
    
    f_low_beta = OneEuroFilter(freq=30.0, min_cutoff=1.0, beta=0.0)   # 纯低通
    f_adaptive = OneEuroFilter(freq=30.0, min_cutoff=1.0, beta=0.05)  # 自适应
    
    n = 90
    dt = 1.0 / 30.0
    true_vals = np.zeros(n)
    # 前30帧静止，中间30帧快速移动，后30帧静止于新位置
    for i in range(n):
        if i < 30:
            true_vals[i] = 0.0
        elif i < 60:
            true_vals[i] = (i - 30) * 0.1  # 运动
        else:
            true_vals[i] = 3.0
    
    np.random.seed(42)
    noisy = true_vals + np.random.normal(0, 0.05, n)
    
    filt_low = []
    filt_adaptive = []
    for i in range(n):
        filt_low.append(f_low_beta.filter(noisy[i], timestamp=i * dt))
        filt_adaptive.append(f_adaptive.filter(noisy[i], timestamp=i * dt))
    
    # 在快速移动阶段(帧40-50)，自适应版本应跟随更好
    true_mid = true_vals[40:50]
    low_mid = np.array(filt_low[40:50])
    adaptive_mid = np.array(filt_adaptive[40:50])
    
    lag_low = np.mean(np.abs(low_mid - true_mid))
    lag_adaptive = np.mean(np.abs(adaptive_mid - true_mid))
    
    print(f"  快速运动阶段延迟: 纯低通={lag_low:.4f}, 自适应={lag_adaptive:.4f}")
    assert lag_adaptive < lag_low, "自适应滤波器在快速运动时延迟应更小"
    
    # 在静止阶段(后20帧)，两者平滑效果类似
    true_static = true_vals[70:]
    low_static = np.array(filt_low[70:])
    adaptive_static = np.array(filt_adaptive[70:])
    
    jitter_low = np.std(low_static - true_static)
    jitter_adaptive = np.std(adaptive_static - true_static)
    
    print(f"  静态阶段抖动:     纯低通={jitter_low:.4f}, 自适应={jitter_adaptive:.4f}")
    print("  ✓ PASS\n")


def test_smoother_disabled_passthrough():
    """测试禁用平滑时的直通行为"""
    print("=" * 60)
    print("Test 6: OutputSmoother - 禁用时直通透传")
    
    config = SmootherConfig(
        enable_position_smooth=False,
        enable_yaw_smooth=False,
        enable_velocity_smooth=False,
        enable_structural_convergence=False,
    )
    smoother = OutputSmoother(config)
    smoother.initialize(r1=0.15, r2=0.20, dza=0.0)
    
    pos = np.array([1.0, 2.0, 3.0])
    vel = np.array([0.1, 0.2, 0.3])
    
    result = smoother.smooth(
        center_pos=pos,
        yaw=1.5,
        velocity=vel,
        yaw_velocity=0.1,
        r1=0.18, r2=0.22, dza=0.05,
        timestamp=0.0,
    )
    
    np.testing.assert_array_almost_equal(result.center_position, pos)
    assert abs(result.yaw - 1.5) < 1e-10
    np.testing.assert_array_almost_equal(result.velocity, vel)
    assert abs(result.r1 - 0.18) < 1e-10
    assert abs(result.r2 - 0.22) < 1e-10
    assert abs(result.dza - 0.05) < 1e-10
    
    print("  禁用时输出 == 输入: ✓")
    print("  ✓ PASS\n")


def main():
    print("\n" + "=" * 60)
    print("        输出平滑器 (Output Smoother) 功能测试")
    print("=" * 60 + "\n")
    
    test_one_euro_filter_basic()
    test_one_euro_angle()
    test_structural_estimator_convergence()
    test_output_smoother_jitter_reduction()
    test_one_euro_speed_response()
    test_smoother_disabled_passthrough()
    
    print("=" * 60)
    print("  所有测试通过 ✓")
    print("=" * 60)


if __name__ == '__main__':
    main()
