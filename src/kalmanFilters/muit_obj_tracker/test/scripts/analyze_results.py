#!/usr/bin/env python3
"""
跟踪结果分析和可视化脚本

用于分析不同滤波器的跟踪效果，生成对比报告和可视化图表。
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import argparse
import os
from collections import defaultdict

# 设置中文字体（如果可用）
try:
    matplotlib.rcParams['font.family'] = ['DejaVu Sans', 'sans-serif']
except:
    pass


def load_tracking_results(filename):
    """加载跟踪结果"""
    return pd.read_csv(filename)


def load_ground_truth(filename):
    """加载真值数据"""
    return pd.read_csv(filename)


def load_detections(filename):
    """加载检测数据"""
    return pd.read_csv(filename)


def load_trajectory_info(filename):
    """加载轨迹信息"""
    return pd.read_csv(filename)


def analyze_tracking_results(tracking_df, detection_df, ground_truth_df=None):
    """分析跟踪结果"""
    results = {}
    
    # 基本统计
    num_frames = tracking_df['frame_id'].nunique()
    num_tracks = tracking_df['track_id'].nunique()
    total_track_outputs = len(tracking_df)
    
    results['num_frames'] = num_frames
    results['num_tracks'] = num_tracks
    results['total_track_outputs'] = total_track_outputs
    results['avg_tracks_per_frame'] = total_track_outputs / num_frames if num_frames > 0 else 0
    
    # 检测统计
    total_detections = len(detection_df)
    results['total_detections'] = total_detections
    results['avg_detections_per_frame'] = total_detections / num_frames if num_frames > 0 else 0
    
    # ID 切换分析
    track_ids_per_frame = tracking_df.groupby('frame_id')['track_id'].apply(list)
    id_switches = 0
    prev_ids = set()
    for frame_id in sorted(track_ids_per_frame.index):
        current_ids = set(track_ids_per_frame[frame_id])
        # 简单的 ID 切换计数：新出现的 ID 数量（排除第一帧）
        if len(prev_ids) > 0:
            new_ids = current_ids - prev_ids
            lost_ids = prev_ids - current_ids
            # 如果有目标丢失后出现新ID，可能是ID切换
            id_switches += min(len(new_ids), len(lost_ids))
        prev_ids = current_ids
    results['id_switches'] = id_switches
    
    # 轨迹连续性分析
    track_lengths = tracking_df.groupby('track_id').size()
    results['avg_track_length'] = track_lengths.mean()
    results['max_track_length'] = track_lengths.max()
    results['min_track_length'] = track_lengths.min()
    
    return results


def compare_filters(results_dict):
    """对比不同滤波器的结果"""
    comparison = pd.DataFrame(results_dict).T
    return comparison


def visualize_tracking(tracking_df, detection_df, trajectory_df, filter_name, output_dir):
    """可视化跟踪结果"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 1. 轨迹对比
    ax1 = axes[0, 0]
    
    # 绘制真实机器人轨迹
    ax1.plot(trajectory_df['robot_x'], trajectory_df['robot_y'], 
             'b-', label='Robot trajectory', alpha=0.5, linewidth=2)
    
    # 绘制跟踪轨迹（每个track_id一种颜色）
    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    for i, track_id in enumerate(tracking_df['track_id'].unique()):
        track_data = tracking_df[tracking_df['track_id'] == track_id]
        color = colors[i % len(colors)]
        ax1.plot(track_data['state_x'], track_data['state_y'], 
                'o-', color=color, markersize=2, alpha=0.7, 
                label=f'Track {track_id}' if i < 5 else '')
    
    ax1.set_xlabel('X (pixels)')
    ax1.set_ylabel('Y (pixels)')
    ax1.set_title(f'{filter_name} - Tracking Trajectories')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # 2. 每帧跟踪数量
    ax2 = axes[0, 1]
    
    det_per_frame = detection_df.groupby('frame_id').size()
    track_per_frame = tracking_df.groupby('frame_id').size()
    
    frames = range(max(det_per_frame.index.max(), track_per_frame.index.max()) + 1)
    det_counts = [det_per_frame.get(f, 0) for f in frames]
    track_counts = [track_per_frame.get(f, 0) for f in frames]
    
    ax2.plot(frames, det_counts, 'g-', label='Detections', alpha=0.7)
    ax2.plot(frames, track_counts, 'r-', label='Tracks', alpha=0.7)
    ax2.set_xlabel('Frame')
    ax2.set_ylabel('Count')
    ax2.set_title(f'{filter_name} - Detection vs Track Count')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 3. 轨迹长度分布
    ax3 = axes[1, 0]
    
    track_lengths = tracking_df.groupby('track_id').size()
    ax3.hist(track_lengths, bins=20, edgecolor='black', alpha=0.7)
    ax3.axvline(track_lengths.mean(), color='r', linestyle='--', 
                label=f'Mean: {track_lengths.mean():.1f}')
    ax3.set_xlabel('Track Length (frames)')
    ax3.set_ylabel('Count')
    ax3.set_title(f'{filter_name} - Track Length Distribution')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 4. 估计位置与检测位置对比
    ax4 = axes[1, 1]
    
    # 随机选择一些帧进行对比
    sample_frames = np.random.choice(tracking_df['frame_id'].unique(), 
                                     min(50, len(tracking_df['frame_id'].unique())), 
                                     replace=False)
    
    for frame_id in sorted(sample_frames):
        track_data = tracking_df[tracking_df['frame_id'] == frame_id]
        det_data = detection_df[detection_df['frame_id'] == frame_id]
        
        # 绘制检测
        for _, det in det_data.iterrows():
            ax4.scatter(det['x'] + det['width']/2, det['y'] + det['height']/2, 
                       c='green', s=20, alpha=0.5, marker='o')
        
        # 绘制估计
        for _, track in track_data.iterrows():
            ax4.scatter(track['state_x'], track['state_y'], 
                       c='red', s=20, alpha=0.5, marker='x')
    
    ax4.scatter([], [], c='green', marker='o', label='Detection')
    ax4.scatter([], [], c='red', marker='x', label='Estimated')
    ax4.set_xlabel('X (pixels)')
    ax4.set_ylabel('Y (pixels)')
    ax4.set_title(f'{filter_name} - Detection vs Estimation (sampled)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.set_aspect('equal')
    
    plt.tight_layout()
    
    output_file = os.path.join(output_dir, f'analysis_{filter_name.lower().replace(" ", "_")}.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Visualization saved to: {output_file}")
    
    plt.close()


def generate_comparison_report(results_dict, output_dir):
    """生成对比报告"""
    comparison = compare_filters(results_dict)
    
    # 保存为 CSV
    output_file = os.path.join(output_dir, 'filter_comparison.csv')
    comparison.to_csv(output_file)
    print(f"Comparison report saved to: {output_file}")
    
    # 生成对比图
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    filters = list(results_dict.keys())
    x = np.arange(len(filters))
    width = 0.35
    
    # 1. 跟踪数量对比
    ax1 = axes[0, 0]
    ax1.bar(x, [results_dict[f]['num_tracks'] for f in filters], width, label='Total Tracks')
    ax1.set_ylabel('Count')
    ax1.set_title('Total Tracks Generated')
    ax1.set_xticks(x)
    ax1.set_xticklabels(filters, rotation=45, ha='right')
    ax1.grid(True, alpha=0.3)
    
    # 2. ID 切换对比
    ax2 = axes[0, 1]
    ax2.bar(x, [results_dict[f]['id_switches'] for f in filters], width, color='orange')
    ax2.set_ylabel('Count')
    ax2.set_title('ID Switches')
    ax2.set_xticks(x)
    ax2.set_xticklabels(filters, rotation=45, ha='right')
    ax2.grid(True, alpha=0.3)
    
    # 3. 平均轨迹长度对比
    ax3 = axes[1, 0]
    ax3.bar(x, [results_dict[f]['avg_track_length'] for f in filters], width, color='green')
    ax3.set_ylabel('Frames')
    ax3.set_title('Average Track Length')
    ax3.set_xticks(x)
    ax3.set_xticklabels(filters, rotation=45, ha='right')
    ax3.grid(True, alpha=0.3)
    
    # 4. 每帧平均跟踪数对比
    ax4 = axes[1, 1]
    ax4.bar(x, [results_dict[f]['avg_tracks_per_frame'] for f in filters], width, color='purple')
    ax4.axhline(y=results_dict[filters[0]]['avg_detections_per_frame'], 
                color='r', linestyle='--', label='Avg Detections')
    ax4.set_ylabel('Count')
    ax4.set_title('Average Tracks per Frame')
    ax4.set_xticks(x)
    ax4.set_xticklabels(filters, rotation=45, ha='right')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    output_file = os.path.join(output_dir, 'filter_comparison.png')
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"Comparison visualization saved to: {output_file}")
    
    plt.close()
    
    # 打印文本报告
    print("\n" + "="*60)
    print("Filter Comparison Report")
    print("="*60)
    print(comparison.to_string())
    print("="*60)


def main():
    parser = argparse.ArgumentParser(description='Analyze tracking results')
    parser.add_argument('--data-dir', type=str, default='data', help='Data directory')
    parser.add_argument('--detection-file', type=str, default='complex_detections.csv')
    parser.add_argument('--gt-file', type=str, default='complex_ground_truth.csv')
    parser.add_argument('--trajectory-file', type=str, default='complex_trajectory_info.csv')
    parser.add_argument('--results-pattern', type=str, default='results_*.csv',
                       help='Pattern for result files')
    
    args = parser.parse_args()
    
    data_dir = args.data_dir
    
    # 加载基础数据
    detection_df = load_detections(os.path.join(data_dir, args.detection_file))
    trajectory_df = load_trajectory_info(os.path.join(data_dir, args.trajectory_file))
    
    print(f"Loaded {len(detection_df)} detections")
    print(f"Loaded {len(trajectory_df)} trajectory frames")
    
    # 查找所有结果文件
    import glob
    result_files = glob.glob(os.path.join(data_dir, args.results_pattern))
    
    if not result_files:
        print(f"No result files found matching pattern: {args.results_pattern}")
        return
    
    print(f"Found {len(result_files)} result files")
    
    # 分析每个滤波器的结果
    results_dict = {}
    for result_file in result_files:
        # 提取滤波器名称
        basename = os.path.basename(result_file)
        filter_name = basename.replace('results_', '').replace('.csv', '').upper()
        
        print(f"\nAnalyzing {filter_name}...")
        
        tracking_df = load_tracking_results(result_file)
        results = analyze_tracking_results(tracking_df, detection_df)
        results_dict[filter_name] = results
        
        # 生成可视化
        visualize_tracking(tracking_df, detection_df, trajectory_df, filter_name, data_dir)
    
    # 生成对比报告
    if len(results_dict) > 1:
        generate_comparison_report(results_dict, data_dir)


if __name__ == '__main__':
    main()
