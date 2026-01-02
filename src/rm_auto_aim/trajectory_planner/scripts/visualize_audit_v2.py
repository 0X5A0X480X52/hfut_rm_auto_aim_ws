#!/usr/bin/env python3
"""
MPC审计日志可视化工具 V2 - 增强多目标可视化

新功能:
- 每个候选目标用不同颜色区分
- 显示每个目标的预测未来位置
- 用箭头显示目标yaw姿态
- 标签显示遮挡过滤条件和检测值
"""

import sys
import argparse
import numpy as np
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import Circle, FancyArrowPatch, Arrow
import matplotlib.cm as cm
import math


class MPCAuditVisualizerV2:
    """MPC审计日志可视化器 - 增强版"""
    
    def __init__(self, csv_path: str):
        """
        初始化可视化器
        
        Args:
            csv_path: CSV审计日志文件路径
        """
        self.csv_path = csv_path
        self.df = None
        self.cand_df = None
        self.current_frame = 0
        
        # 加载数据
        self._load_data()
        
        # 颜色映射：track_id -> color
        self.track_colors = {}
        self._assign_track_colors()
        
        # 创建图形界面
        self.fig = None
        self.ax_main = None
        self.ax_yaw_time = None
        self.ax_control = None
        self.ax_error = None
        self.ax_solve_time = None
        self.slider = None
        
        self._setup_figure()
    
    def _load_data(self):
        """加载CSV数据"""
        print(f"Loading audit log from: {self.csv_path}")
        self.df = pd.read_csv(self.csv_path)
        print(f"Loaded {len(self.df)} frames")
        
        # 加载候选目标CSV
        cand_path = Path(self.csv_path).parent / f"{Path(self.csv_path).stem}_candidates.csv"
        if cand_path.exists():
            self.cand_df = pd.read_csv(cand_path)
            print(f"Loaded candidate CSV with {len(self.cand_df)} rows, columns: {list(self.cand_df.columns[:20])}")
        else:
            self.cand_df = None
            print("No candidate CSV found")
        
        # 统计信息
        print(f"\nStatistics:")
        print(f"  Total frames: {len(self.df)}")
        print(f"  Duration: {self.df['relative_time'].max():.2f} seconds")
        print(f"  Average solve time: {self.df['solve_time_ms'].mean():.3f} ms")
        print(f"  Max solve time: {self.df['solve_time_ms'].max():.3f} ms")
        print(f"  Solver status distribution:")
        print(self.df['solver_status'].value_counts())
    
    def _assign_track_colors(self):
        """为每个track_id分配颜色"""
        if self.cand_df is None:
            return
        
        unique_tracks = self.cand_df['track_id'].unique()
        # 使用tab10或Set3 colormap
        cmap = cm.get_cmap('tab10' if len(unique_tracks) <= 10 else 'tab20')
        
        for i, tid in enumerate(unique_tracks):
            if tid < 0:  # 无效track_id
                self.track_colors[tid] = 'gray'
            else:
                self.track_colors[tid] = cmap(i % 20)
        
        print(f"Assigned colors to {len(unique_tracks)} tracks")
    
    def _setup_figure(self):
        """设置图形布局"""
        self.fig = plt.figure(figsize=(18, 11))
        self.fig.suptitle('MPC Audit Visualization (Enhanced)', fontsize=16, fontweight='bold')
        
        # 创建子图布局
        gs = self.fig.add_gridspec(3, 2, height_ratios=[2, 1, 1], hspace=0.35, wspace=0.3)
        
        # 主图: 2D yaw平面图（俯视图）
        self.ax_main = self.fig.add_subplot(gs[0, :])
        self.ax_main.set_title('2D Bird\'s Eye View - Multi-Target Tracking', fontsize=12, fontweight='bold')
        self.ax_main.set_xlabel('X (m)')
        self.ax_main.set_ylabel('Y (m)')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_aspect('equal')
        
        # 子图1: yaw-time曲线
        self.ax_yaw_time = self.fig.add_subplot(gs[1, 0])
        self.ax_yaw_time.set_title('Yaw vs Time', fontsize=10)
        self.ax_yaw_time.set_xlabel('Time (s)')
        self.ax_yaw_time.set_ylabel('Yaw (rad)')
        self.ax_yaw_time.grid(True, alpha=0.3)
        
        # 子图2: 控制输入序列
        self.ax_control = self.fig.add_subplot(gs[1, 1])
        self.ax_control.set_title('Control Sequence (Jerk)', fontsize=10)
        self.ax_control.set_xlabel('Step')
        self.ax_control.set_ylabel('Jerk (rad/s³)')
        self.ax_control.grid(True, alpha=0.3)
        
        # 子图3: 误差历史
        self.ax_error = self.fig.add_subplot(gs[2, 0])
        self.ax_error.set_title('Tracking Error History', fontsize=10)
        self.ax_error.set_xlabel('Time (s)')
        self.ax_error.set_ylabel('Error (rad)')
        self.ax_error.grid(True, alpha=0.3)
        
        # 子图4: 求解时间历史
        self.ax_solve_time = self.fig.add_subplot(gs[2, 1])
        self.ax_solve_time.set_title('Solve Time History', fontsize=10)
        self.ax_solve_time.set_xlabel('Time (s)')
        self.ax_solve_time.set_ylabel('Solve Time (ms)')
        self.ax_solve_time.grid(True, alpha=0.3)
        
        # 添加滑动条
        ax_slider = self.fig.add_axes([0.15, 0.02, 0.7, 0.02])
        self.slider = Slider(
            ax_slider, 'Frame', 
            0, len(self.df) - 1, 
            valinit=0, 
            valstep=1,
            valfmt='%d'
        )
        self.slider.on_changed(self._update_frame)
    
    def _update_frame(self, val):
        """更新显示帧"""
        self.current_frame = int(self.slider.val)
        self._draw_frame()
    
    def _draw_frame(self):
        """绘制当前帧"""
        frame = self.df.iloc[self.current_frame]
        
        # 清空所有子图
        self.ax_main.clear()
        self.ax_yaw_time.clear()
        self.ax_control.clear()
        self.ax_error.clear()
        self.ax_solve_time.clear()
        
        # === 主图: 2D鸟瞰图 ===
        self.ax_main.set_title(f'Frame {self.current_frame} | Time: {frame["relative_time"]:.3f}s | Solver: {frame["solver_status"]}', 
                               fontsize=12, fontweight='bold')
        self.ax_main.set_xlabel('X (m)')
        self.ax_main.set_ylabel('Y (m)')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_aspect('equal')
        
        # 绘制机器人原点
        self.ax_main.plot(0, 0, 'ro', markersize=10, label='Robot', zorder=10)
        self.ax_main.add_patch(Circle((0, 0), 0.3, fill=False, color='red', linewidth=2))
        
        # 绘制当前云台yaw方向
        current_theta = frame['current_theta']
        gimbal_len = 1.2
        gimbal_x = gimbal_len * np.cos(current_theta)
        gimbal_y = gimbal_len * np.sin(current_theta)
        self.ax_main.arrow(0, 0, gimbal_x, gimbal_y, 
                          head_width=0.15, head_length=0.15, 
                          fc='red', ec='red', linewidth=2.5, label='Gimbal Yaw', zorder=9)
        
        # 绘制MPC预测yaw轨迹（从预测状态中提取）
        pred_theta_cols = [c for c in self.df.columns if c.startswith('pred_theta_')]
        if pred_theta_cols:
            pred_thetas = [frame[c] for c in pred_theta_cols if not np.isnan(frame[c])]
            if pred_thetas:
                # 假设预测从云台当前状态开始，绘制预测方向（仅显示终点）
                final_theta = pred_thetas[-1]
                pred_x = gimbal_len * 0.8 * np.cos(final_theta)
                pred_y = gimbal_len * 0.8 * np.sin(final_theta)
                self.ax_main.plot([0, pred_x], [0, pred_y], 'b--', linewidth=1.5, alpha=0.6, label='MPC Predicted Yaw')
        
        # 绘制选中目标（如果可用）
        selected_track_id = frame['target_track_id'] if not np.isnan(frame['target_track_id']) else None
        # 选中目标（优先使用云台坐标系字段）
        if selected_track_id is not None:
            if 'target_x_gimbal' in self.df.columns and not np.isnan(frame['target_x_gimbal']):
                target_x = frame['target_x_gimbal']
                target_y = frame['target_y_gimbal']
            elif not np.isnan(frame['target_x']):
                target_x = frame['target_x']
                target_y = frame['target_y']
            else:
                target_x, target_y = None, None

            if target_x is not None:
                self.ax_main.plot(target_x, target_y, 'g*', markersize=15, label='Selected Target', zorder=8)
                self.ax_main.add_patch(Circle((target_x, target_y), 0.08, 
                                             fill=True, color='green', alpha=0.4))
                
                # 绘制目标参考轨迹（yaw方向），优先使用gimbal帧的yaw轨迹
                target_theta_cols_g = [c for c in self.df.columns if c.startswith('target_theta_gimbal_')]
                if target_theta_cols_g:
                    target_thetas = [frame[c] for c in target_theta_cols_g if not np.isnan(frame[c])]
                else:
                    target_theta_cols = [c for c in self.df.columns if c.startswith('target_theta_')]
                    target_thetas = [frame[c] for c in target_theta_cols if not np.isnan(frame[c])]

                if target_thetas:
                    final_target = target_thetas[-1]
                    tref_x = gimbal_len * 1.0 * np.cos(final_target)
                    tref_y = gimbal_len * 1.0 * np.sin(final_target)
                    self.ax_main.plot([0, tref_x], [0, tref_y], 'm-.', linewidth=1.5, alpha=0.7, label='Target Yaw Ref')
        
        # 绘制候选目标（来自 candidates CSV）
        if self.cand_df is not None:
            cand_rows = self.cand_df[self.cand_df['frame_id'] == self.current_frame]
            
            for idx, crow in cand_rows.iterrows():
                track_id = crow['track_id']
                cx = crow['target_x']
                cy = crow['target_y']
                
                if np.isnan(cx) or np.isnan(cy):
                    continue
                
                # 获取颜色
                color = self.track_colors.get(track_id, 'gray')
                
                # 判断是否是被选中的目标
                is_selected = (track_id == selected_track_id)
                
                # 绘制候选目标位置
                marker = 'o' if not is_selected else 'o'
                markersize = 10 if not is_selected else 12
                self.ax_main.plot(cx, cy, marker=marker, color=color, 
                                markersize=markersize, alpha=0.8, zorder=7)
                
                # 绘制目标方向箭头
                # 优先使用armor_yaw（装甲板自身姿态），若无则优先使用view_yaw_gimbal，再fallback到view_yaw
                arrow_yaw = None
                if 'armor_yaw' in crow and not np.isnan(crow['armor_yaw']) and abs(crow['armor_yaw']) > 0.01:
                    arrow_yaw = crow['armor_yaw']
                elif 'view_yaw_gimbal' in crow and not np.isnan(crow['view_yaw_gimbal']):
                    arrow_yaw = crow['view_yaw_gimbal']
                elif 'view_yaw' in crow and not np.isnan(crow['view_yaw']):
                    arrow_yaw = crow['view_yaw']

                if arrow_yaw is not None:
                    arrow_len = 0.5
                    ax_end = cx + arrow_len * np.cos(arrow_yaw)
                    ay_end = cy + arrow_len * np.sin(arrow_yaw)
                    self.ax_main.arrow(cx, cy, ax_end - cx, ay_end - cy,
                                     head_width=0.1, head_length=0.1,
                                     fc=color, ec=color, linewidth=1.8, alpha=0.7, zorder=6)
                
                # 绘制预测未来位置（优先使用gimbal帧列 pos_x_gimbal_*）
                px_g_cols = [c for c in crow.index if c.startswith('pos_x_gimbal_')]
                py_g_cols = [c for c in crow.index if c.startswith('pos_y_gimbal_')]
                if px_g_cols and py_g_cols:
                    vals_x = [crow[c] for c in px_g_cols if not np.isnan(crow[c])]
                    vals_y = [crow[c] for c in py_g_cols if not np.isnan(crow[c])]
                else:
                    px_cols = [c for c in crow.index if c.startswith('pos_x_')]
                    py_cols = [c for c in crow.index if c.startswith('pos_y_')]
                    vals_x = [crow[c] for c in px_cols if not np.isnan(crow[c])]
                    vals_y = [crow[c] for c in py_cols if not np.isnan(crow[c])]

                if len(vals_x) > 0 and len(vals_y) > 0:
                    # 绘制未来位置轨迹
                    self.ax_main.plot(vals_x, vals_y, '-', color=color, 
                                    linewidth=1.5, alpha=0.5, zorder=5)
                    # 绘制未来位置点
                    self.ax_main.plot(vals_x, vals_y, 'o', color=color, 
                                    markersize=4, alpha=0.6, zorder=5)
                
                # 添加标签：显示遮挡过滤信息
                label_parts = [f'ID:{int(track_id)}']
                
                # 显示置信度
                if 'confidence' in crow and not np.isnan(crow['confidence']):
                    label_parts.append(f'c={crow["confidence"]:.2f}')
                
                # 显示距离
                if 'distance' in crow and not np.isnan(crow['distance']):
                    label_parts.append(f'd={crow["distance"]:.2f}m')
                
                # 显示过滤原因
                if 'filter_reason' in crow:
                    reason = str(crow['filter_reason'])
                    if reason == 'valid':
                        label_parts.append('✓')
                    else:
                        label_parts.append(f'✗ {reason}')
                
                # 显示facing_diff（如果是遮挡过滤）
                if 'facing_diff' in crow and not np.isnan(crow['facing_diff']):
                    facing_diff_deg = np.degrees(crow['facing_diff'])
                    label_parts.append(f'facing:{facing_diff_deg:.1f}°')
                
                label_text = '\n'.join(label_parts)
                
                # 根据位置调整标签位置
                text_x = cx + 0.2
                text_y = cy + 0.2
                self.ax_main.text(text_x, text_y, label_text,
                                fontsize=7, ha='left', va='bottom',
                                bbox=dict(boxstyle='round,pad=0.3', 
                                        facecolor=color, alpha=0.3, edgecolor=color),
                                zorder=8)
        
        # 设置合理的视图范围
        all_x = [0]
        all_y = [0]
        if not np.isnan(frame['target_x']):
            all_x.append(frame['target_x'])
            all_y.append(frame['target_y'])
        if self.cand_df is not None:
            cand_rows = self.cand_df[self.cand_df['frame_id'] == self.current_frame]
            for _, crow in cand_rows.iterrows():
                if not np.isnan(crow['target_x']):
                    all_x.append(crow['target_x'])
                    all_y.append(crow['target_y'])
        
        if len(all_x) > 1:
            margin = 1.0
            self.ax_main.set_xlim(min(all_x) - margin, max(all_x) + margin)
            self.ax_main.set_ylim(min(all_y) - margin, max(all_y) + margin)
        else:
            self.ax_main.set_xlim(-3, 3)
            self.ax_main.set_ylim(-3, 3)
        
        self.ax_main.legend(loc='upper right', fontsize=8)
        
        # === Yaw-Time 图 ===
        time_data = self.df['relative_time'].to_numpy()
        current_theta_data = self.df['current_theta'].to_numpy()
        
        # 当前yaw
        self.ax_yaw_time.plot(time_data, current_theta_data, 'b-', label='Current Yaw', linewidth=1.5)
        
        # 目标yaw（第一步）
        if 'target_theta_0' in self.df.columns:
            target_theta_data = self.df['target_theta_0'].to_numpy()
            self.ax_yaw_time.plot(time_data, target_theta_data, 'm--', label='Target Yaw (step 0)', linewidth=1.5, alpha=0.7)
        
        # 当前帧标记
        self.ax_yaw_time.axvline(frame['relative_time'], color='red', linestyle=':', linewidth=1.5, alpha=0.7)
        self.ax_yaw_time.legend(fontsize=8)
        self.ax_yaw_time.set_xlabel('Time (s)')
        self.ax_yaw_time.set_ylabel('Yaw (rad)')
        self.ax_yaw_time.grid(True, alpha=0.3)
        
        # === Control Sequence 图 ===
        U_cols = [c for c in self.df.columns if c.startswith('U_seq_')]
        U_vals = [frame[c] for c in U_cols if not np.isnan(frame[c])]
        if U_vals:
            self.ax_control.bar(range(len(U_vals)), U_vals, color='orange', alpha=0.7)
            self.ax_control.set_xlabel('Step')
            self.ax_control.set_ylabel('Jerk (rad/s³)')
            self.ax_control.grid(True, alpha=0.3)
        
        # === Error History 图 ===
        yaw_error_data = self.df['yaw_error'].to_numpy()
        pitch_error_data = self.df['pitch_error'].to_numpy()
        
        self.ax_error.plot(time_data, np.abs(yaw_error_data), 'r-', label='|Yaw Error|', linewidth=1.5)
        self.ax_error.plot(time_data, np.abs(pitch_error_data), 'g-', label='|Pitch Error|', linewidth=1.5, alpha=0.7)
        self.ax_error.axvline(frame['relative_time'], color='red', linestyle=':', linewidth=1.5, alpha=0.7)
        self.ax_error.legend(fontsize=8)
        self.ax_error.set_xlabel('Time (s)')
        self.ax_error.set_ylabel('Error (rad)')
        self.ax_error.grid(True, alpha=0.3)
        
        # === Solve Time History 图 ===
        solve_time_data = self.df['solve_time_ms'].to_numpy()
        self.ax_solve_time.plot(time_data, solve_time_data, 'purple', linewidth=1.5)
        self.ax_solve_time.axvline(frame['relative_time'], color='red', linestyle=':', linewidth=1.5, alpha=0.7)
        self.ax_solve_time.axhline(self.df['solve_time_ms'].mean(), color='blue', linestyle='--', linewidth=1, alpha=0.5, label='Mean')
        self.ax_solve_time.legend(fontsize=8)
        self.ax_solve_time.set_xlabel('Time (s)')
        self.ax_solve_time.set_ylabel('Solve Time (ms)')
        self.ax_solve_time.grid(True, alpha=0.3)
        
        self.fig.canvas.draw_idle()
    
    def show(self):
        """显示可视化界面"""
        self._draw_frame()
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize MPC Audit Logs (Enhanced)')
    parser.add_argument('csv_path', type=str, help='Path to the main MPC audit CSV file')
    args = parser.parse_args()
    
    # 创建可视化器
    viz = MPCAuditVisualizerV2(args.csv_path)
    viz.show()


if __name__ == '__main__':
    main()
