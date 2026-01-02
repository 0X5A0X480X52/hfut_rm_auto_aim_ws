#!/usr/bin/env python3
"""
生成测试MPC审计数据，用于测试可视化功能
"""

import numpy as np
import pandas as pd
from pathlib import Path

def generate_test_data():
    """生成测试数据"""
    
    # 参数
    n_frames = 50
    dt = 0.05  # 50ms
    horizon = 18
    
    # 创建主CSV数据
    frames = []
    
    for i in range(n_frames):
        t = i * dt
        
        # 云台跟踪正弦目标
        target_theta = 0.5 * np.sin(2 * np.pi * 0.2 * t)
        current_theta = target_theta + 0.1 * np.sin(2 * np.pi * 2 * t)  # 加入小幅振荡
        current_omega = 0.1 * 2 * np.pi * 2 * np.cos(2 * np.pi * 2 * t)
        current_alpha = 0.0
        
        # MPC目标轨迹
        target_traj = [target_theta + 0.02 * j for j in range(horizon)]
        
        # 控制序列
        U_seq = [0.1 * np.random.randn() for _ in range(horizon)]
        u_opt = U_seq[0]
        
        # 预测轨迹
        pred_traj = []
        state = [current_theta, current_omega, current_alpha]
        for j in range(horizon + 1):
            pred_traj.append(state.copy())
            if j < horizon:
                # 简单的状态更新
                state[2] += U_seq[j] * dt  # alpha += jerk * dt
                state[1] += state[2] * dt   # omega += alpha * dt
                state[0] += state[1] * dt   # theta += omega * dt
        
        # 目标信息（模拟2个目标在视野中移动）
        target_x = -2.5 + 0.5 * np.cos(2 * np.pi * 0.1 * t)
        target_y = 0.3 * np.sin(2 * np.pi * 0.15 * t)
        target_z = -0.1
        target_dist = np.sqrt(target_x**2 + target_y**2 + target_z**2)
        
        # 构建帧数据
        frame_data = {
            'timestamp': 1000000000 + t,
            'frame_id': i,
            'relative_time': t,
            'current_theta': current_theta,
            'current_omega': current_omega,
            'current_alpha': current_alpha,
        }
        
        # 添加目标轨迹
        for j in range(horizon):
            frame_data[f'target_theta_{j}'] = target_traj[j]
        
        # 控制
        frame_data['u_optimal'] = u_opt
        for j in range(horizon):
            frame_data[f'U_seq_{j}'] = U_seq[j]
        
        # 预测轨迹
        for j in range(horizon + 1):
            frame_data[f'pred_theta_{j}'] = pred_traj[j][0]
            frame_data[f'pred_omega_{j}'] = pred_traj[j][1]
            frame_data[f'pred_alpha_{j}'] = pred_traj[j][2]
        
        # 求解信息
        frame_data['solve_time_ms'] = 5 + 3 * np.random.rand()
        frame_data['solver_status'] = 'solved'
        
        # 目标信息
        frame_data['target_track_id'] = 1
        frame_data['target_x'] = target_x
        frame_data['target_y'] = target_y
        frame_data['target_z'] = target_z
        frame_data['target_distance'] = target_dist
        frame_data['target_confidence'] = 0.95
        frame_data['target_yaw_from_origin'] = np.arctan2(target_y, target_x)
        
        # 误差
        frame_data['yaw_error'] = current_theta - target_theta
        frame_data['pitch_error'] = 0.01 * np.random.randn()
        frame_data['fire_advice'] = abs(frame_data['yaw_error']) < 0.05
        
        frames.append(frame_data)
    
    df_main = pd.DataFrame(frames)
    
    # 创建候选目标CSV数据
    candidates = []
    
    for i in range(n_frames):
        t = i * dt
        
        # 生成3个候选目标
        for cand_idx in range(3):
            track_id = cand_idx + 1
            
            # 每个目标在不同位置运动
            phase = cand_idx * 2.0
            cx = -2.5 + 0.5 * np.cos(2 * np.pi * 0.1 * t + phase)
            cy = 0.3 * np.sin(2 * np.pi * 0.15 * t + phase) + cand_idx * 0.4 - 0.4
            cz = -0.1
            cdist = np.sqrt(cx**2 + cy**2 + cz**2)
            
            # armor yaw (假设装甲板朝向略微旋转)
            armor_yaw = np.arctan2(cy, cx) + np.pi + 0.3 * np.sin(2 * np.pi * 0.3 * t + phase)
            view_yaw = np.arctan2(cy, cx)
            facing_diff = abs(abs((armor_yaw - view_yaw + np.pi) % (2*np.pi) - np.pi) - np.pi)
            
            # 过滤判断
            is_occluded = facing_diff > 1.05
            if is_occluded:
                filter_reason = f"occluded(facing_diff={np.degrees(facing_diff):.1f}°>60.0°)"
            else:
                filter_reason = "valid"
            
            cand_data = {
                'frame_id': i,
                'candidate_idx': cand_idx,
                'track_id': track_id,
                'armor_id': str(cand_idx + 1),
                'armor_type': 'small' if cand_idx < 2 else 'large',
                'target_x': cx,
                'target_y': cy,
                'target_z': cz,
                'distance': cdist,
                'confidence': 0.9 + 0.1 * np.random.rand(),
                'armor_yaw': armor_yaw,
                'view_yaw': view_yaw,
                'facing_diff': facing_diff,
                'is_occluded': is_occluded,
                'filter_reason': filter_reason,
            }
            
            # yaw轨迹
            for j in range(horizon):
                cand_data[f'target_theta_{j}'] = view_yaw + 0.01 * j
            
            # 位置预测轨迹（简单线性预测）
            vx = 0.2 * np.sin(2 * np.pi * 0.15 * t + phase)
            vy = 0.1 * np.cos(2 * np.pi * 0.1 * t + phase)
            vz = 0.0
            
            for j in range(horizon):
                pred_t = j * dt
                cand_data[f'pos_x_{j}'] = cx + vx * pred_t
                cand_data[f'pos_y_{j}'] = cy + vy * pred_t
                cand_data[f'pos_z_{j}'] = cz + vz * pred_t
            
            candidates.append(cand_data)
    
    df_cand = pd.DataFrame(candidates)
    
    return df_main, df_cand


def main():
    # 生成数据
    print("Generating test MPC audit data...")
    df_main, df_cand = generate_test_data()
    
    # 保存
    output_dir = Path('/tmp')
    main_path = output_dir / 'mpc_audit_test.csv'
    cand_path = output_dir / 'mpc_audit_test_candidates.csv'
    
    df_main.to_csv(main_path, index=False)
    df_cand.to_csv(cand_path, index=False)
    
    print(f"Saved main CSV: {main_path}")
    print(f"  {len(df_main)} frames")
    print(f"Saved candidates CSV: {cand_path}")
    print(f"  {len(df_cand)} candidate entries")
    
    print("\nTo visualize:")
    print(f"  python3 visualize_audit_v2.py {main_path}")


if __name__ == '__main__':
    main()
