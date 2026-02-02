"""
Demo2 模块验证测试

验证 DualRadiusSpinUKF 和 AdaptiveArmorTracker 的基本功能
"""

import numpy as np
import sys
sys.path.insert(0, '.')

from src.max_entropy_ukf_dual_radius_demo2 import (
    DualRadiusSpinUKF, 
    AdaptiveArmorTracker, 
    UnifiedConfig, 
    ObservationData
)
from src.max_entropy_ukf_dual_radius_demo2.filters.dual_radius_spin_ukf import StateIndex


def test_ukf_basic():
    """测试UKF基本功能"""
    print("\n" + "="*60)
    print("测试 DualRadiusSpinUKF 基本功能")
    print("="*60)
    
    config = UnifiedConfig.create_default()
    ukf = DualRadiusSpinUKF(config=config, dt=0.05)
    
    # 测试初始化
    obs = ObservationData(x=1.0, y=2.0, z=0.5, yaw=0.0)
    ukf.initialize([obs], r1=0.15, r2=0.20, dza=0.05)
    
    print(f"✓ 初始化成功")
    print(f"  - 状态维度: {ukf.state_dim}")
    print(f"  - 中心位置: {ukf.get_center_position()}")
    print(f"  - Yaw: {np.degrees(ukf.get_yaw()):.1f}°")
    print(f"  - k={ukf.get_k()}, delta={np.degrees(ukf.get_delta()):.1f}°")
    print(f"  - r1={ukf.get_radii()[0]:.3f}, r2={ukf.get_radii()[1]:.3f}")
    
    # 测试预测
    ukf.predict()
    print(f"✓ 预测成功")
    
    # 测试单观测更新
    obs_update = ObservationData(x=1.05, y=2.05, z=0.52, yaw=0.1)
    success = ukf.update([obs_update], r_types=['r1'], armor_layers=['lower'])
    print(f"✓ 单观测更新: {'成功' if success else '失败'}")
    
    # 测试双观测更新
    obs1 = ObservationData(x=1.1, y=2.0, z=0.55, yaw=0.1)
    obs2 = ObservationData(x=1.0, y=2.1, z=0.45, yaw=np.pi/2 + 0.1)
    success = ukf.update([obs1, obs2], r_types=['r1', 'r2'], armor_layers=['upper', 'lower'])
    print(f"✓ 双观测更新: {'成功' if success else '失败'}")
    
    # 测试状态字典
    state = ukf.get_state_dict()
    print(f"✓ 状态字典包含 {len(state)} 个键")
    
    return True


def test_tracker_basic():
    """测试Tracker基本功能"""
    print("\n" + "="*60)
    print("测试 AdaptiveArmorTracker 基本功能")
    print("="*60)
    
    config = UnifiedConfig.create_default()
    tracker = AdaptiveArmorTracker(config=config, dt=0.05)
    
    # 测试初始化
    obs = ObservationData(x=1.0, y=2.0, z=0.5, yaw=0.0)
    tracker.initialize([obs], r1=0.15, r2=0.20, dza=0.05)
    
    print(f"✓ 初始化成功")
    print(f"  - 跟踪状态: {tracker.state.name}")
    print(f"  - Panel ID: {tracker.get_panel_id()}")
    print(f"  - 中心位置: {tracker.get_center_position()}")
    
    # 测试预测
    tracker.predict()
    print(f"✓ 预测成功")
    
    # 测试单观测更新
    obs_update = ObservationData(x=1.05, y=2.05, z=0.52, yaw=0.1)
    success = tracker.update([obs_update])
    print(f"✓ 单观测更新: {'成功' if success else '失败'}")
    print(f"  - 高度标签: {tracker.get_height_label().name}")
    print(f"  - 高度置信度: {tracker.get_height_confidence():.2f}")
    
    # 测试多帧跟踪
    for i in range(10):
        tracker.predict()
        yaw = 0.1 + i * 0.05
        obs = ObservationData(x=1.0 + 0.01*i, y=2.0 + 0.01*i, z=0.5, yaw=yaw)
        tracker.update([obs])
    
    print(f"✓ 多帧跟踪完成 (10帧)")
    print(f"  - 帧计数: {tracker.frame_count}")
    print(f"  - 最终位置: {tracker.get_center_position()}")
    print(f"  - 最终Yaw: {np.degrees(tracker.get_yaw()):.1f}°")
    
    # 测试统计
    panel_stats = tracker.get_panel_statistics()
    height_stats = tracker.get_height_statistics()
    print(f"✓ Panel统计: {panel_stats.get('total', 0)} 次分配")
    print(f"✓ 高度统计: unknown率 = {height_stats.get('unknown_rate', 0):.1%}")
    
    return True


def test_state_index_enum():
    """测试StateIndex枚举"""
    print("\n" + "="*60)
    print("测试 StateIndex 枚举")
    print("="*60)
    
    print(f"  X = {StateIndex.X}")
    print(f"  VX = {StateIndex.VX}")
    print(f"  DELTA = {StateIndex.DELTA}")
    print(f"  R1 = {StateIndex.R1}")
    print(f"  R2 = {StateIndex.R2}")
    print(f"  DZA = {StateIndex.DZA}")
    
    # 验证可以用作数组索引
    x = np.zeros(11)
    x[StateIndex.X] = 1.0
    x[StateIndex.R1] = 0.15
    print(f"✓ StateIndex可用作数组索引")
    
    return True


def test_observation_data():
    """测试ObservationData"""
    print("\n" + "="*60)
    print("测试 ObservationData")
    print("="*60)
    
    # 基本创建
    obs = ObservationData(x=1.0, y=2.0, z=0.5, yaw=np.pi/4)
    print(f"✓ 基本创建: pos={obs.position}")
    
    # 从数组创建
    obs2 = ObservationData.from_array(np.array([1.0, 2.0, 0.5]), yaw=0.0)
    print(f"✓ 从数组创建: pos={obs2.position}")
    
    # 4D表示
    arr4d = obs.as_4d
    print(f"✓ 4D表示: {arr4d}")
    
    # 从4D创建
    obs3 = ObservationData.from_4d(arr4d)
    print(f"✓ 从4D创建: pos={obs3.position}, yaw={obs3.yaw:.2f}")
    
    return True


def main():
    """主测试函数"""
    print("\n" + "#"*60)
    print("  max_entropy_ukf_dual_radius_demo2 模块验证测试")
    print("#"*60)
    
    all_passed = True
    
    try:
        all_passed &= test_state_index_enum()
        all_passed &= test_observation_data()
        all_passed &= test_ukf_basic()
        all_passed &= test_tracker_basic()
    except Exception as e:
        print(f"\n❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        all_passed = False
    
    print("\n" + "="*60)
    if all_passed:
        print("✅ 所有测试通过!")
    else:
        print("❌ 部分测试失败")
    print("="*60)
    
    return all_passed


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
