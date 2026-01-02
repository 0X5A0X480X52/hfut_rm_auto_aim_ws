#!/usr/bin/env python3
"""
MPC审计日志记录器

功能:
- 记录MPC每帧的输入输出数据到CSV文件
- 支持缓冲写入避免影响实时控制性能
- 提供统计摘要功能
"""

import csv
import time
from pathlib import Path
from typing import Optional, Dict, Any
import numpy as np


class MPCAuditLogger:
    """MPC审计日志记录器"""
    
    def __init__(self, log_path: str, buffer_size: int = 100, horizon: int = 18):
        """
        初始化审计日志记录器
        
        Args:
            log_path: CSV日志文件路径
            buffer_size: 缓冲区大小（帧数），达到此数量后批量写入文件
            horizon: 预测步数 N
        """
        self.log_path = Path(log_path)
        self.buffer_size = buffer_size
        self.horizon = horizon
        self.frame_buffer = []
        self.candidate_buffer = []
        self.frame_id = 0
        self.start_time = time.time()
        
        # 创建日志目录
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        
        # 初始化CSV文件并写入表头
        self._init_csv_file()
        self._init_candidates_csv()
        
        # 统计信息
        self.stats = {
            'total_frames': 0,
            'solve_success': 0,
            'solve_failed': 0,
            'total_solve_time': 0.0,
            'max_solve_time': 0.0,
            'min_solve_time': float('inf'),
        }
    
    def _init_csv_file(self):
        """初始化CSV文件并写入表头"""
        with open(self.log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            
            N = self.horizon
            # CSV表头
            header = [
                'timestamp',
                'frame_id',
                'relative_time',
                
                # 当前云台状态
                'current_theta',
                'current_omega',
                'current_alpha',
                
                # 目标轨迹 (N步)
                *[f'target_theta_{i}' for i in range(N)],
                
                # MPC输出
                'u_optimal',
                *[f'U_seq_{i}' for i in range(N)],
                
                # 预测轨迹 (N+1步)
                *[f'pred_theta_{i}' for i in range(N+1)],
                *[f'pred_omega_{i}' for i in range(N+1)],
                *[f'pred_alpha_{i}' for i in range(N+1)],
                
                # 求解信息
                'solve_time_ms',
                'solver_status',
                
                # 目标信息
                'target_track_id',
                'target_x',
                'target_y',
                'target_z',
                'target_distance',
                'target_confidence',
                'target_yaw_from_origin',
                # 云台坐标系（转换后）
                'target_x_gimbal', 'target_y_gimbal', 'target_z_gimbal',
                'target_yaw_gimbal',
                'gimbal_yaw',
                
                # 开火判断
                'yaw_error',
                'pitch_error',
                'fire_advice',
            ]
            
            writer.writerow(header)
    
    def log_frame(
        self,
        current_state: np.ndarray,
        target_trajectory: np.ndarray,
        u_optimal: float,
        U_sequence: np.ndarray,
        predicted_trajectory: np.ndarray,
        solve_time_ms: float,
        solver_status: str,
        target_info: Optional[Dict[str, Any]] = None,
        yaw_error: float = 0.0,
        pitch_error: float = 0.0,
        fire_advice: bool = False,
        candidates: Optional[list] = None,
    ):
        """
        记录一帧MPC数据
        
        Args:
            current_state: 当前云台状态 [theta, omega, alpha]
            target_trajectory: 目标yaw轨迹 (N,) 或 (N,3)
            u_optimal: 最优控制输入（jerk）
            U_sequence: 完整控制序列 (N,)
            predicted_trajectory: 预测状态轨迹 (N+1, 3)
            solve_time_ms: 求解耗时（毫秒）
            solver_status: 求解状态 ("solved", "failed", etc.)
            target_info: 目标信息字典
            yaw_error: yaw误差
            pitch_error: pitch误差
            fire_advice: 是否建议开火
        """
        timestamp = time.time()
        relative_time = timestamp - self.start_time
        
        # 构建数据行
        row = [
            timestamp,
            self.frame_id,
            relative_time,
            
            # 当前状态
            current_state[0],  # theta
            current_state[1],  # omega
            current_state[2],  # alpha
        ]
        
        # 目标轨迹（仅yaw角）
        if target_trajectory.ndim == 1:
            target_yaws = target_trajectory
        else:
            target_yaws = target_trajectory[:, 0]
        
        # 填充目标轨迹（固定18步）
        for i in range(18):
            if i < len(target_yaws):
                row.append(float(target_yaws[i]))
            else:
                row.append(np.nan)
        
        # MPC输出
        row.append(float(u_optimal))
        
        # 控制序列（固定18步）
        for i in range(18):
            if i < len(U_sequence):
                row.append(float(U_sequence[i]))
            else:
                row.append(np.nan)
        
        # 预测轨迹（固定19步 = N+1）
        for state_idx in range(3):  # theta, omega, alpha
            for i in range(19):
                if i < len(predicted_trajectory):
                    row.append(float(predicted_trajectory[i, state_idx]))
                else:
                    row.append(np.nan)
        
        # 求解信息
        row.append(float(solve_time_ms))
        row.append(solver_status)
        
        # 目标信息
        if target_info:
            row.extend([
                target_info.get('track_id', -1),
                target_info.get('position', [0, 0, 0])[0],
                target_info.get('position', [0, 0, 0])[1],
                target_info.get('position', [0, 0, 0])[2],
                target_info.get('distance', 0.0),
                target_info.get('confidence', 0.0),
                target_info.get('yaw_from_origin', 0.0),
                # gimbal frame fields
                target_info.get('position_gimbal', [float('nan'), float('nan'), float('nan')])[0],
                target_info.get('position_gimbal', [float('nan'), float('nan'), float('nan')])[1],
                target_info.get('position_gimbal', [float('nan'), float('nan'), float('nan')])[2],
                target_info.get('yaw_from_origin_gimbal', float('nan')),
                float(current_state[0]),  # gimbal_yaw used for transform
            ])
        else:
            row.extend([-1, 0, 0, 0, 0, 0, 0, float('nan'), float('nan'), float('nan'), float('nan'), float('nan')])
        
        # 开火判断
        row.extend([
            float(yaw_error),
            float(pitch_error),
            int(fire_advice),
        ])
        
        # 添加到缓冲区
        self.frame_buffer.append(row)
        self.frame_id += 1
        
        # 更新统计
        self._update_stats(solve_time_ms, solver_status)
        
        # 缓冲区满时批量写入
        if len(self.frame_buffer) >= self.buffer_size:
            self.flush()
        
        # 记录候选目标（如果有）
        if candidates:
            self.log_candidates(self.frame_id - 1, candidates)
    
    def _update_stats(self, solve_time_ms: float, solver_status: str):
        """更新统计信息"""
        self.stats['total_frames'] += 1
        
        if 'solve' in solver_status.lower():
            self.stats['solve_success'] += 1
        else:
            self.stats['solve_failed'] += 1
        
        self.stats['total_solve_time'] += solve_time_ms
        self.stats['max_solve_time'] = max(self.stats['max_solve_time'], solve_time_ms)
        self.stats['min_solve_time'] = min(self.stats['min_solve_time'], solve_time_ms)
    
    def flush(self):
        """将缓冲区数据写入文件"""
        if not self.frame_buffer:
            return
        
        with open(self.log_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(self.frame_buffer)
        
        self.frame_buffer.clear()
    
    def close(self):
        """关闭日志记录器，写入剩余数据"""
        self.flush()
        self.flush_candidates()
        self._write_summary()
    
    def _write_summary(self):
        """写入统计摘要文件"""
        summary_path = self.log_path.parent / f"{self.log_path.stem}_summary.txt"
        
        avg_solve_time = (
            self.stats['total_solve_time'] / self.stats['total_frames']
            if self.stats['total_frames'] > 0 else 0
        )
        
        success_rate = (
            100.0 * self.stats['solve_success'] / self.stats['total_frames']
            if self.stats['total_frames'] > 0 else 0
        )
        
        with open(summary_path, 'w') as f:
            f.write("MPC Audit Summary\n")
            f.write("=" * 50 + "\n\n")
            f.write(f"Total Frames: {self.stats['total_frames']}\n")
            f.write(f"Solve Success: {self.stats['solve_success']}\n")
            f.write(f"Solve Failed: {self.stats['solve_failed']}\n")
            f.write(f"Success Rate: {success_rate:.2f}%\n\n")
            f.write(f"Average Solve Time: {avg_solve_time:.3f} ms\n")
            f.write(f"Max Solve Time: {self.stats['max_solve_time']:.3f} ms\n")
            f.write(f"Min Solve Time: {self.stats['min_solve_time']:.3f} ms\n\n")
            f.write(f"Log File: {self.log_path}\n")
            f.write(f"Total Duration: {time.time() - self.start_time:.2f} seconds\n")
        
        # 写入候选目标摘要（若有）
        cand_summary = self.log_path.parent / f"{self.log_path.stem}_candidates.csv"
        if cand_summary.exists():
            with open(self.log_path.parent / f"{self.log_path.stem}_cand_summary.txt", 'w') as cf:
                cf.write("Candidates CSV present: \n")
                cf.write(f"Path: {cand_summary}\n")
    
    def get_stats(self) -> Dict[str, Any]:
        """获取当前统计信息"""
        return self.stats.copy()

    def _init_candidates_csv(self):
        """初始化候选目标CSV文件并写入表头"""
        cand_path = self.log_path.parent / f"{self.log_path.stem}_candidates.csv"
        self.candidates_path = cand_path
        with open(cand_path, 'w', newline='') as f:
            writer = csv.writer(f)
            N = self.horizon
            header = [
                'frame_id',
                'candidate_idx',
                'track_id',
                'armor_id',
                'armor_type',
                'target_x', 'target_y', 'target_z',
                'distance',
                'confidence',
                # 遮挡过滤信息
                'armor_yaw',
                'view_yaw',
                'view_yaw_gimbal',
                'facing_diff',
                'is_occluded',
                'filter_reason',
                # yaw trajectory (absolute)
                *[f'target_theta_{i}' for i in range(N)],
                # yaw trajectory in gimbal frame
                *[f'target_theta_gimbal_{i}' for i in range(N)],
                # position trajectory (N steps) absolute
                *[f'pos_x_{i}' for i in range(N)],
                *[f'pos_y_{i}' for i in range(N)],
                *[f'pos_z_{i}' for i in range(N)],
                # position trajectory in gimbal frame
                *[f'pos_x_gimbal_{i}' for i in range(N)],
                *[f'pos_y_gimbal_{i}' for i in range(N)],
                *[f'pos_z_gimbal_{i}' for i in range(N)],
            ]
            writer.writerow(header)

    def log_candidates(self, frame_id: int, candidates: list):
        """记录候选目标列表（每个候选一行）"""
        rows = []
        N = self.horizon
        for idx, cand in enumerate(candidates):
            row = [
                frame_id,
                idx,
                cand.get('track_id', -1),
                cand.get('armor_id', 'unknown'),
                cand.get('armor_type', 'unknown'),
                cand.get('position', [float('nan')] * 3)[0],
                cand.get('position', [float('nan')] * 3)[1],
                cand.get('position', [float('nan')] * 3)[2],
                cand.get('distance', float('nan')),
                cand.get('confidence', float('nan')),
                # 遮挡过滤信息
                cand.get('armor_yaw', float('nan')),
                cand.get('view_yaw', float('nan')),
                cand.get('view_yaw_gimbal', float('nan')),
                cand.get('facing_diff', float('nan')),
                cand.get('is_occluded', False),
                cand.get('filter_reason', 'unknown'),
            ]
            # yaw trajectory (absolute)
            yaw_traj = cand.get('yaw_trajectory', [float('nan')] * N)
            for i in range(N):
                row.append(yaw_traj[i] if i < len(yaw_traj) else float('nan'))
            # yaw trajectory (gimbal frame)
            yaw_traj_g = cand.get('yaw_trajectory_gimbal', [float('nan')] * N)
            for i in range(N):
                row.append(yaw_traj_g[i] if i < len(yaw_traj_g) else float('nan'))
            # pos trajectory (absolute)
            pos_traj = cand.get('pos_trajectory', None)
            if pos_traj is None:
                # fill nan
                for i in range(N):
                    row.append(float('nan'))
                for i in range(N):
                    row.append(float('nan'))
                for i in range(N):
                    row.append(float('nan'))
            else:
                for i in range(N):
                    row.append(pos_traj[i][0])
                for i in range(N):
                    row.append(pos_traj[i][1])
                for i in range(N):
                    row.append(pos_traj[i][2])
            # pos trajectory (gimbal frame)
            pos_traj_g = cand.get('pos_trajectory_gimbal', None)
            if pos_traj_g is None:
                for i in range(N):
                    row.append(float('nan'))
                for i in range(N):
                    row.append(float('nan'))
                for i in range(N):
                    row.append(float('nan'))
            else:
                for i in range(N):
                    row.append(pos_traj_g[i][0])
                for i in range(N):
                    row.append(pos_traj_g[i][1])
                for i in range(N):
                    row.append(pos_traj_g[i][2])
            rows.append(row)
            
            # 缓存
            self.candidate_buffer.append(row)
            if len(self.candidate_buffer) >= self.buffer_size:
                self.flush_candidates()

    def flush_candidates(self):
        """将候选目标缓冲区写入CSV"""
        if not self.candidate_buffer:
            return
        with open(self.candidates_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerows(self.candidate_buffer)
        self.candidate_buffer.clear()
