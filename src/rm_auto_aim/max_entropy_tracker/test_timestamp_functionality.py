#!/usr/bin/env python3
"""
测试基于时间戳的预测与更新功能

验证tracker是否正确使用消息时间戳而非系统时间
"""

import numpy as np
import time
from max_entropy_tracker.core.config import UnifiedConfig
from max_entropy_tracker.core.observation import ObservationData
from max_entropy_tracker.trackers.adaptive_armor_tracker import AdaptiveArmorTracker


def test_timestamp_based_update():
    """测试基于时间戳的更新"""
    print("=" * 60)
    print("测试1: 基于时间戳的预测和更新")
    print("=" * 60)
    
    # 创建tracker
    config = UnifiedConfig.create_default()
    tracker = AdaptiveArmorTracker(config=config, dt=0.01)
    
    # 初始观测（t=0）
    obs1 = ObservationData(
        x=0.3, y=0.0, z=0.0,
        yaw=0.0,
        timestamp=0.0
    )
    tracker.initialize([obs1], r1=0.15, r2=0.20, dza=0.0)
    print(f"初始化: t={obs1.timestamp:.3f}s")
    print(f"  内部时间: {tracker._current_time:.3f}s")
    
    # 模拟不稳定频率的观测
    # 正常间隔: 0.02s
    # 异常间隔: 0.05s, 0.01s
    timestamps = [0.02, 0.04, 0.09, 0.10, 0.13]  # 不规则间隔
    
    for i, t in enumerate(timestamps):
        # 创建观测
        obs = ObservationData(
            x=0.3 + 0.01 * i,
            y=0.0,
            z=0.0,
            yaw=0.1 * i,
            timestamp=t
        )
        
        # 更新（内部会自动predict到观测时间）
        success = tracker.update([obs])
        
        # 验证时间同步
        dt_expected = t - tracker._dt_history[-1] if len(tracker._dt_history) > 1 else 0
        dt_actual = tracker._dt_history[-1]
        
        print(f"\n观测 {i+1}: t={t:.3f}s")
        print(f"  dt={dt_actual:.4f}s (从上次更新)")
        print(f"  内部时间: {tracker._current_time:.3f}s")
        print(f"  更新成功: {success}")
        print(f"  位置: ({tracker.get_center_position()[0]:.3f}, "
              f"{tracker.get_center_position()[1]:.3f}, "
              f"{tracker.get_center_position()[2]:.3f})")
    
    # 统计dt
    stats = tracker.get_dt_statistics()
    print("\n" + "=" * 60)
    print("dt统计:")
    print(f"  平均: {stats['mean']:.4f}s")
    print(f"  标准差: {stats['std']:.4f}s")
    print(f"  最小: {stats['min']:.4f}s")
    print(f"  最大: {stats['max']:.4f}s")
    print(f"  样本数: {stats['count']}")
    print("=" * 60)


def test_out_of_order_timestamps():
    """测试乱序时间戳的处理"""
    print("\n" + "=" * 60)
    print("测试2: 乱序时间戳保护")
    print("=" * 60)
    
    config = UnifiedConfig.create_default()
    tracker = AdaptiveArmorTracker(config=config, dt=0.01)
    
    # 初始化
    obs1 = ObservationData(x=0.3, y=0.0, z=0.0, yaw=0.0, timestamp=1.0)
    tracker.initialize([obs1], r1=0.15, r2=0.20, dza=0.0)
    print(f"初始化: t={obs1.timestamp:.3f}s")
    
    # 正常更新
    obs2 = ObservationData(x=0.3, y=0.0, z=0.0, yaw=0.1, timestamp=1.05)
    success = tracker.update([obs2])
    print(f"\n正常更新: t={obs2.timestamp:.3f}s, 成功={success}")
    print(f"  内部时间: {tracker._current_time:.3f}s")
    
    # 尝试使用更早的时间戳（应该被保护）
    obs3 = ObservationData(x=0.3, y=0.0, z=0.0, yaw=0.2, timestamp=1.02)
    print(f"\n乱序观测: t={obs3.timestamp:.3f}s < 当前时间 {tracker._current_time:.3f}s")
    success = tracker.update([obs3])
    print(f"  更新成功: {success}")
    print(f"  内部时间保持: {tracker._current_time:.3f}s (应该不变或小幅调整)")
    
    print("=" * 60)


def test_large_time_gap():
    """测试大时间间隔的处理"""
    print("\n" + "=" * 60)
    print("测试3: 大时间间隔保护")
    print("=" * 60)
    
    config = UnifiedConfig.create_default()
    tracker = AdaptiveArmorTracker(config=config, dt=0.01)
    
    # 初始化
    obs1 = ObservationData(x=0.3, y=0.0, z=0.0, yaw=0.0, timestamp=1.0)
    tracker.initialize([obs1], r1=0.15, r2=0.20, dza=0.0)
    print(f"初始化: t={obs1.timestamp:.3f}s")
    
    # 正常更新
    obs2 = ObservationData(x=0.3, y=0.0, z=0.0, yaw=0.1, timestamp=1.02)
    tracker.update([obs2])
    print(f"正常更新: t={obs2.timestamp:.3f}s")
    
    # 大时间间隔（超过max_dt=0.5s）
    obs3 = ObservationData(x=0.3, y=0.0, z=0.0, yaw=0.2, timestamp=2.0)
    print(f"\n大间隔观测: t={obs3.timestamp:.3f}s, Δt={obs3.timestamp - tracker._current_time:.3f}s")
    success = tracker.update([obs3])
    
    # 检查实际使用的dt
    actual_dt = tracker._dt_history[-1]
    print(f"  实际dt: {actual_dt:.3f}s (应该被限制在max_dt={tracker._max_dt:.1f}s)")
    print(f"  更新成功: {success}")
    
    print("=" * 60)


def test_dual_observation_timestamps():
    """测试双观测时间戳处理"""
    print("\n" + "=" * 60)
    print("测试4: 双观测时间戳")
    print("=" * 60)
    
    config = UnifiedConfig.create_default()
    tracker = AdaptiveArmorTracker(config=config, dt=0.01)
    
    # 初始化
    obs1 = ObservationData(x=0.3, y=0.0, z=0.0, yaw=0.0, timestamp=1.0)
    tracker.initialize([obs1], r1=0.15, r2=0.20, dza=0.0)
    print(f"初始化: t={obs1.timestamp:.3f}s")
    
    # 双观测，时间戳略有不同（模拟真实情况）
    obs_a = ObservationData(x=0.2, y=0.0, z=0.0, yaw=0.0, timestamp=1.050)
    obs_b = ObservationData(x=-0.2, y=0.0, z=0.0, yaw=np.pi, timestamp=1.052)
    
    print(f"\n双观测:")
    print(f"  观测A: t={obs_a.timestamp:.3f}s")
    print(f"  观测B: t={obs_b.timestamp:.3f}s")
    print(f"  时间差: {abs(obs_b.timestamp - obs_a.timestamp) * 1000:.1f}ms")
    
    success = tracker.update([obs_a, obs_b])
    print(f"\n更新成功: {success}")
    print(f"  内部时间: {tracker._current_time:.3f}s (应该使用max时间戳)")
    print(f"  预期: {max(obs_a.timestamp, obs_b.timestamp):.3f}s")
    
    print("=" * 60)


if __name__ == "__main__":
    print("\n基于时间戳的预测与更新 - 功能测试\n")
    
    test_timestamp_based_update()
    test_out_of_order_timestamps()
    test_large_time_gap()
    test_dual_observation_timestamps()
    
    print("\n所有测试完成！\n")
