#!/usr/bin/env python3
"""
MPC审计日志可视化工具

功能:
- 读取MPC审计CSV日志文件
- 绘制交互式可视化面板：
  * 2D yaw平面图（俯视图，显示目标位置、预测轨迹）
  * yaw-time曲线（目标vs实际vs预测）
  * 控制输入序列
  * 误差历史曲线
  * 时间滑动条切换帧
"""

import sys
import argparse
import numpy as np
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from matplotlib.patches import Circle, FancyArrowPatch
import math


class MPCAuditVisualizer:
    """MPC审计日志可视化器"""
    
    def __init__(self, csv_path: str):
        """
        初始化可视化器
        
        Args:
            csv_path: CSV审计日志文件路径
        """
        self.csv_path = csv_path
        self.df = None
        self.current_frame = 0
        
        # 加载数据
        self._load_data()
        
        # 创建图形界面
        self.fig = None
        self.ax_main = None
        self.ax_yaw_time = None
        self.ax_control = None
        self.ax_error = None
        self.slider = None
        
        self._setup_figure()
    
    def _load_data(self):
        """加载CSV数据"""
        print(f"Loading audit log from: {self.csv_path}")
        self.df = pd.read_csv(self.csv_path)
        print(f"Loaded {len(self.df)} frames")
        print(f"Columns: {list(self.df.columns[:10])}...")  # 显示前10列
        # 尝试加载候选目标CSV（如果存在）
        cand_path = Path(self.csv_path).parent / f"{Path(self.csv_path).stem}_candidates.csv"
        if cand_path.exists():
            self.cand_df = pd.read_csv(cand_path)
            print(f"Loaded candidate CSV with {len(self.cand_df)} rows")
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
    
    def _setup_figure(self):
        """设置图形布局"""
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.suptitle('MPC Audit Visualization', fontsize=16, fontweight='bold')
        
        # 创建子图布局
        gs = self.fig.add_gridspec(3, 2, height_ratios=[2, 1, 1], hspace=0.3, wspace=0.3)
        
        # 主图: 2D yaw平面图（俯视图）
        self.ax_main = self.fig.add_subplot(gs[0, :])
        self.ax_main.set_title('2D Bird\'s Eye View (Yaw Plane)', fontsize=12, fontweight='bold')
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
        
        # 重新设置标题和标签
        self.ax_main.set_title(f'Frame {self.current_frame} - Time: {frame["relative_time"]:.3f}s', 
                               fontsize=12, fontweight='bold')
        self.ax_main.set_xlabel('X (m)')
        self.ax_main.set_ylabel('Y (m)')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.set_aspect('equal')
        
        # === 主图: 2D鸟瞰图 ===
        # 绘制机器人原点
        self.ax_main.plot(0, 0, 'ro', markersize=10, label='Robot')
        self.ax_main.add_patch(Circle((0, 0), 0.3, fill=False, color='red', linewidth=2))
        
        # 绘制当前云台方向（注意：这里yaw=0应该指向+X方向）
        current_theta = frame['current_theta']
        gimbal_x = 1.5 * np.cos(current_theta)
        gimbal_y = 1.5 * np.sin(current_theta)
        self.ax_main.arrow(0, 0, gimbal_x, gimbal_y, 
                          head_width=0.2, head_length=0.2, 
                          fc='red', ec='red', linewidth=2, label='Current Yaw')
        
        # 绘制目标位置（如果可用）
        if not np.isnan(frame['target_x']):
            target_x = frame['target_x']
            target_y = frame['target_y']
            self.ax_main.plot(target_x, target_y, 'go', markersize=8, label='Selected Target')
            self.ax_main.add_patch(Circle((target_x, target_y), 0.05, 
                                         fill=True, color='green', alpha=0.3))
            
            # 绘制目标线
            self.ax_main.plot([0, target_x], [0, target_y], 'g--', alpha=0.5, linewidth=1)
            
            # 显示目标信息
            target_dist = frame['target_distance']
            target_conf = frame['target_confidence']
            self.ax_main.text(target_x, target_y + 0.3, 
                            f'd={target_dist:.2f}m\nc={target_conf:.2f}',
                            ha='center', fontsize=8)
        
        # 绘制候选目标（来自 candidates CSV）
        if self.cand_df is not None:
            cand_rows = self.cand_df[self.cand_df['frame_id'] == self.current_frame]
            for idx, crow in cand_rows.iterrows():
                cx = crow['target_x']
                cy = crow['target_y']
                # 候选目标位置标记
                self.ax_main.plot(cx, cy, 'kx', markersize=8, alpha=0.6, label='Candidate' if idx == 0 else '')
                
                # 绘制candidate预测位置轨迹（如果存在）
                if 'pos_x_0' in self.cand_df.columns:
                    px_cols = [c for c in self.cand_df.columns if c.startswith('pos_x_')]
                    py_cols = [c for c in self.cand_df.columns if c.startswith('pos_y_')]
                    vals_x = [crow[c] for c in px_cols if not np.isnan(crow[c])]
                    vals_y = [crow[c] for c in py_cols if not np.isnan(crow[c])]
                    if vals_x and vals_y:
                        self.ax_main.plot(vals_x, vals_y, 'k-', alpha=0.3, linewidth=1.5, 
                                        label='Cand. Pos Traj' if idx == 0 else '')
        
        # 绘制MPC预测轨迹（云台状态预测，映射到实际位置）
        pred_thetas = []
        pred_df_cols = [c for c in frame.index if c.startswith('pred_theta_')]
        for col_name in sorted(pred_df_cols, key=lambda x: int(x.split('_')[-1])):
            if not np.isnan(frame[col_name]):
                pred_thetas.append(frame[col_name])
        
        if len(pred_thetas) > 1:
            # 将预测的yaw角转换为实际位置（假设目标在某距离上）
            # 使用当前选中目标的距离
            if not np.isnan(frame['target_distance']):
                radius = frame['target_distance']
            else:
                radius = 3.0  # 默认3米
            pred_x = radius * np.cos(np.array(pred_thetas))
            pred_y = radius * np.sin(np.array(pred_thetas))
            self.ax_main.plot(pred_x, pred_y, 'b.-', alpha=0.7, 
                            linewidth=2, markersize=5, label='MPC Predicted Yaw Traj')
        
        # 绘制目标yaw轨迹（输入给MPC的参考轨迹）
        target_yaws = []
        target_cols = [c for c in frame.index if c.startswith('target_theta_')]
        for col_name in sorted(target_cols, key=lambda x: int(x.split('_')[-1])):
            if not np.isnan(frame[col_name]):
                target_yaws.append(frame[col_name])
        
        if len(target_yaws) > 1:
            # 使用目标实际距离
            if not np.isnan(frame['target_distance']):
                radius = frame['target_distance']
            else:
                radius = 3.0
            target_x_traj = radius * np.cos(np.array(target_yaws))
            target_y_traj = radius * np.sin(np.array(target_yaws))
            self.ax_main.plot(target_x_traj, target_y_traj, 'm.-', alpha=0.7, 
                            linewidth=2, markersize=5, label='Target Yaw Reference')
        
        self.ax_main.legend(loc='upper right', fontsize=8)
        self.ax_main.set_xlim(-6, 6)
        self.ax_main.set_ylim(-6, 6)
        
        # === yaw-time曲线 ===
        window_size = 100  # 显示前后50帧
        start_idx = max(0, self.current_frame - window_size // 2)
        end_idx = min(len(self.df), self.current_frame + window_size // 2)
        
        window_df = self.df.iloc[start_idx:end_idx]
        
        # 转换为 numpy arrays 以避免 pandas 多维切片限制
        time_arr = window_df['relative_time'].to_numpy()
        curr_theta_arr = window_df['current_theta'].to_numpy()
        self.ax_yaw_time.plot(time_arr, 
                             curr_theta_arr, 
                             'b-', label='Current Yaw', linewidth=2)
        
        # 绘制目标yaw（如果存在）
        if 'target_theta_0' in window_df.columns:
            target_theta_arr = window_df['target_theta_0'].to_numpy()
            self.ax_yaw_time.plot(time_arr, 
                                 target_theta_arr, 
                                 'm--', label='Target Yaw', alpha=0.7)
        
        # 当前帧标记
        self.ax_yaw_time.axvline(frame['relative_time'], 
                                color='red', linestyle='--', alpha=0.5)
        
        self.ax_yaw_time.set_xlabel('Time (s)')
        self.ax_yaw_time.set_ylabel('Yaw (rad)')
        self.ax_yaw_time.legend(fontsize=8)
        self.ax_yaw_time.grid(True, alpha=0.3)
        
        # === 控制序列 ===
        U_seq = []
        for i in range(18):
            col_name = f'U_seq_{i}'
            if col_name in frame and not np.isnan(frame[col_name]):
                U_seq.append(frame[col_name])
        
        if len(U_seq) > 0:
            self.ax_control.bar(range(len(U_seq)), U_seq, color='steelblue', alpha=0.7)
            self.ax_control.axhline(0, color='black', linewidth=0.5)
            self.ax_control.set_xlabel('Step')
            self.ax_control.set_ylabel('Jerk (rad/s³)')
            
            # 显示u_optimal
            u_opt = frame['u_optimal']
            self.ax_control.bar(0, u_opt, color='red', alpha=0.8, label=f'u_opt={u_opt:.2f}')
            self.ax_control.legend(fontsize=8)
        
        self.ax_control.grid(True, alpha=0.3)
        
        # === 误差历史 ===
        # 误差历史（转换为 numpy arrays）
        time_arr_err = window_df['relative_time'].to_numpy()
        yaw_err_arr = np.abs(window_df['yaw_error'].to_numpy())
        self.ax_error.plot(time_arr_err, 
                          yaw_err_arr, 
                          'r-', label='|Yaw Error|', linewidth=2)
        
        if 'pitch_error' in window_df.columns:
            pitch_err_arr = np.abs(window_df['pitch_error'].to_numpy())
            self.ax_error.plot(time_arr_err, 
                              pitch_err_arr, 
                              'b-', label='|Pitch Error|', alpha=0.7)
        
        self.ax_error.axvline(frame['relative_time'], 
                             color='green', linestyle='--', alpha=0.5)
        
        self.ax_error.set_xlabel('Time (s)')
        self.ax_error.set_ylabel('Error (rad)')
        self.ax_error.legend(fontsize=8)
        self.ax_error.grid(True, alpha=0.3)
        
        # === 求解时间历史 ===
        # 求解时间历史（转换为 numpy arrays）
        time_arr_s = window_df['relative_time'].to_numpy()
        solve_time_arr = window_df['solve_time_ms'].to_numpy()
        self.ax_solve_time.plot(time_arr_s, 
                               solve_time_arr, 
                               'g-', linewidth=2)
        self.ax_solve_time.axvline(frame['relative_time'], 
                                  color='red', linestyle='--', alpha=0.5)
        self.ax_solve_time.axhline(10, color='orange', linestyle='--', 
                                  alpha=0.5, label='10ms target')
        
        self.ax_solve_time.set_xlabel('Time (s)')
        self.ax_solve_time.set_ylabel('Solve Time (ms)')
        self.ax_solve_time.legend(fontsize=8)
        self.ax_solve_time.grid(True, alpha=0.3)
        
        # 添加帧信息文本
        info_text = f"Solver: {frame['solver_status']}\n"
        info_text += f"Solve Time: {frame['solve_time_ms']:.3f}ms\n"
        info_text += f"Yaw Error: {np.rad2deg(frame['yaw_error']):.2f}°\n"
        info_text += f"Fire Advice: {bool(frame['fire_advice'])}"
        
        self.fig.text(0.02, 0.15, info_text, 
                     fontsize=9, family='monospace',
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.draw()
    
    def show(self):
        """显示可视化界面"""
        self._draw_frame()
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize MPC audit logs')
    parser.add_argument('csv_file', help='Path to the MPC audit CSV file')
    
    args = parser.parse_args()
    
    try:
        visualizer = MPCAuditVisualizer(args.csv_file)
        visualizer.show()
    except FileNotFoundError:
        print(f"Error: File not found: {args.csv_file}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
