"""
AdaptiveArmorTracker 敏感性分析测试（demo2版本）

基于 demo1 的 test_tracker_v3_sensitivity.py，使用重构后的接口测试场景：
1. 原地自旋
2. 仅机动（不自旋）
3. 自旋+机动（混合运动）
4. 基于可见性的动态观测（自旋+机动，模拟真实场景）

应与demo1版本产生一致的输出。
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Dict
import sys
import os
from dataclasses import dataclass

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(__file__))))

from src.max_entropy_ukf_dual_radius_demo2.trackers.adaptive_armor_tracker import AdaptiveArmorTracker
from src.max_entropy_ukf_dual_radius_demo2.core.config import UnifiedConfig
from src.max_entropy_ukf_dual_radius_demo2.core.observation import ObservationData
from src.max_entropy_ukf_dual_radius_demo2.utils.angle_utils import normalize_angle, delta_angle_diff
from src.data_generator.simulator import TrajectoryGenerator
from src.core.se3_utils import se3_to_rt


@dataclass
class TrajectoryParams:
    start_pos: np.ndarray
    velocity: np.ndarray
    omega: float
    r1: float
    r2: float
    dzc: float
    dza: float
    duration: float
    dt: float
    single_observation: bool = False  # 是否使用单观测（仅观测panel 0）


@dataclass
class PerformanceMetrics:
    position_rmse: float
    position_errors: np.ndarray
    yaw_rmse: float
    yaw_errors: np.ndarray
    r1_weighted_error: float
    r2_weighted_error: float
    dzc_weighted_error: float
    dza_weighted_error: float
    r1_convergence_frame: int
    r2_convergence_frame: int
    r1_errors: np.ndarray
    r2_errors: np.ndarray
    dzc_errors: np.ndarray
    dza_errors: np.ndarray
    feature_rmse: float
    feature_errors: np.ndarray


def angle_difference(a1: float, a2: float) -> float:
    """计算两个角度之间的最短差值"""
    diff = a1 - a2
    return np.arctan2(np.sin(diff), np.cos(diff))


def generate_trajectory(params: TrajectoryParams) -> Dict:
    n_steps = int(params.duration / params.dt)
    timestamps = np.arange(n_steps) * params.dt
    center_positions = np.zeros((n_steps, 3))
    center_yaws = np.zeros(n_steps)
    pos = params.start_pos.copy()
    yaw = 0.0
    
    for i in range(n_steps):
        center_positions[i] = pos
        center_yaws[i] = yaw
        pos += params.velocity * params.dt
        yaw += params.omega * params.dt
        yaw = normalize_angle(yaw)
    
    armor_observations = []
    armor_yaws = []
    armor_layers = []
    num_visible_per_frame = []
    
    for i in range(n_steps):
        center_pos = center_positions[i]
        center_yaw = center_yaws[i]
        panel_angles = [0, np.pi/2, np.pi, 3*np.pi/2]
        visible_panels = [0] if params.single_observation else [0, 1]
        
        frame_observations = []
        frame_yaws = []
        frame_layers = []
        
        for panel_idx in visible_panels:
            panel_angle = panel_angles[panel_idx]
            armor_yaw_outward = center_yaw + panel_angle
            r = params.r1 if panel_idx % 2 == 0 else params.r2
            position_angle = armor_yaw_outward
            dx = r * np.cos(position_angle)
            dy = r * np.sin(position_angle)
            
            # panel 0,2 (偶数，r1) → +dza → z更高 → upper层
            # panel 1,3 (奇数，r2) → -dza → z更低 → lower层
            layer_offset = params.dza if panel_idx % 2 == 0 else -params.dza
            armor_pos = center_pos + np.array([dx, dy, params.dzc + layer_offset])
            
            armor_yaw = armor_yaw_outward + np.pi
            armor_yaw = normalize_angle(armor_yaw)
            
            noise = np.random.randn(3) * 0.001
            armor_pos_noisy = armor_pos + noise
            
            frame_observations.append(armor_pos_noisy)
            frame_yaws.append(armor_yaw)
            layer = 'upper' if panel_idx % 2 == 0 else 'lower'
            frame_layers.append(layer)
        
        armor_observations.append(frame_observations)
        armor_yaws.append(frame_yaws)
        armor_layers.append(frame_layers)
        num_visible_per_frame.append(len(visible_panels))
    
    return {
        'timestamps': timestamps,
        'center_positions': center_positions,
        'center_yaws': center_yaws,
        'armor_observations': armor_observations,
        'armor_yaws': armor_yaws,
        'armor_layers': armor_layers,
        'num_visible_per_frame': num_visible_per_frame,
        'true_params': {'r1': params.r1, 'r2': params.r2, 'dzc': params.dzc, 'dza': params.dza}
    }


def compute_yaw_error_symmetric(yaw_est: float, yaw_true: float) -> float:
    yaw_options = [yaw_true, yaw_true + np.pi, yaw_true - np.pi]
    errors = [abs(angle_difference(yaw_est, yaw_opt)) for yaw_opt in yaw_options]
    return min(errors)


def compute_signed_yaw_error(yaw_est: float, yaw_true: float) -> float:
    """在考虑 180° 对称性的前提下，返回带符号的 yaw 差值（rad）"""
    yaw_options = [yaw_true, yaw_true + np.pi, yaw_true - np.pi]
    diffs = [angle_difference(yaw_est, yaw_opt) for yaw_opt in yaw_options]
    abs_vals = [abs(d) for d in diffs]
    idx = int(np.argmin(abs_vals))
    return diffs[idx]


def compute_weighted_parameter_error(estimates: np.ndarray, true_value: float, covariances: np.ndarray) -> float:
    errors = np.abs(estimates - true_value)
    epsilon = 1e-6
    weights_uncertainty = 1.0 / (covariances + epsilon)
    n = len(estimates)
    weights_time = np.linspace(0.5, 1.5, n)
    weights = weights_uncertainty * weights_time
    weights /= weights.sum()
    weighted_error = np.sum(errors * weights)
    return weighted_error


def find_convergence_frame(errors: np.ndarray, threshold: float = 0.05) -> int:
    window_size = 10
    for i in range(len(errors) - window_size):
        window = errors[i:i+window_size]
        if np.all(window < threshold):
            return i
    return -1


def run_tracker_test(trajectory: Dict, config: UnifiedConfig, dt: float, log_prefix: str = "test") -> Tuple[PerformanceMetrics, Dict, AdaptiveArmorTracker]:
    import json
    
    # 创建日志目录
    log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'outputs', 'debug_logs')
    os.makedirs(log_dir, exist_ok=True)
    
    # 保存真实轨迹数据
    ground_truth_log = os.path.join(log_dir, f'{log_prefix}_demo2_ground_truth.jsonl')
    observations_log = os.path.join(log_dir, f'{log_prefix}_demo2_observations.jsonl')
    tracker_states_log = os.path.join(log_dir, f'{log_prefix}_demo2_tracker_states.jsonl')
    ukf_states_log = os.path.join(log_dir, f'{log_prefix}_demo2_ukf_states.jsonl')
    
    tracker = AdaptiveArmorTracker(config=config, dt=dt)
    true_params = trajectory['true_params']
    
    # 保存真实轨迹
    n_frames = len(trajectory['center_positions'])
    with open(ground_truth_log, 'w') as f:
        for i in range(n_frames):
            truth = {
                'frame': i,
                'time': i * dt,
                'center_pos': trajectory['center_positions'][i].tolist(),
                'center_yaw': float(trajectory['center_yaws'][i]),
                'true_params': {k: float(v) for k, v in trajectory['true_params'].items()}
            }
            f.write(json.dumps(truth) + '\n')
    
    print(f"已保存真实轨迹: {ground_truth_log}")
    
    est_positions = []
    est_yaws = []
    est_r1 = []
    est_r2 = []
    est_dzc = []
    est_dza = []
    cov_r1 = []
    cov_r2 = []
    cov_dzc = []
    cov_dza = []
    
    # 收集true labels（用于调试统计）
    true_labels = []
    
    for i in range(n_frames):
        observations = trajectory['armor_observations'][i]
        yaws = trajectory['armor_yaws'][i]
        layers = trajectory['armor_layers'][i]
        
        # 记录观测数据
        with open(observations_log, 'a') as f:
            obs_data = {
                'frame': i,
                'time': i * dt,
                'n_observations': len(observations),
                'observations': [obs.tolist() for obs in observations],
                'armor_yaws': [float(y) for y in yaws],
                'armor_layers': layers
            }
            f.write(json.dumps(obs_data) + '\n')
        
        # 转换为ObservationData列表
        obs_list = [ObservationData(x=obs[0], y=obs[1], z=obs[2], yaw=yaw) 
                   for obs, yaw in zip(observations, yaws)]
        
        if i == 0:
            # 初始化：如果有双观测，使用平均z来消除upper/lower的dza偏移
            if len(observations) == 2:
                avg_z = (observations[0][2] + observations[1][2]) / 2.0
                init_obs_array = observations[0].copy()
                init_obs_array[2] = avg_z
                init_obs_list = [ObservationData.from_array(init_obs_array, yaws[0])]
            else:
                init_obs_list = obs_list[:1]
            
            tracker.initialize(init_obs_list, r1=0.19, r2=0.24, dza=0.01)
        else:
            tracker.predict()
            tracker.update(obs_list)
        
        # 记录tracker状态
        with open(tracker_states_log, 'a') as f:
            tracker_state = {
                'frame': i,
                'time': i * dt,
                'tracker_state': tracker.state.name if hasattr(tracker, 'state') else None,
                'height_label': tracker.height_identifier.last_label.name if (hasattr(tracker, 'height_identifier') and tracker.height_identifier.last_label is not None) else None
            }
            f.write(json.dumps(tracker_state) + '\n')
        
        # 记录UKF状态
        from src.max_entropy_ukf_dual_radius_demo2.filters.dual_radius_spin_ukf import StateIndex
        if tracker.ukf.x is not None:
            with open(ukf_states_log, 'a') as f:
                state_names = [name for name in StateIndex.__members__.keys()]
                ukf_state = {
                    'frame': i,
                    'time': i * dt,
                    'state_vector': tracker.ukf.x.tolist(),
                    'state_names': state_names,
                    'covariance_diag': np.diag(tracker.ukf.P).tolist()
                }
                f.write(json.dumps(ukf_state) + '\n')
        
        est_positions.append(tracker.get_center_position())
        est_yaws.append(tracker.get_yaw())
        r1, r2 = tracker.get_radii()
        est_r1.append(r1)
        est_r2.append(r2)
        
        if tracker.ukf.x is not None:
            # demo2的DualRadiusSpinUKF: dza=10（无dzc状态）
            est_dzc.append(0.0)  # dzc已删除，记录0
            est_dza.append(tracker.ukf.x[StateIndex.DZA])
            
            if tracker.ukf.P is not None:
                cov_r1.append(tracker.ukf.P[StateIndex.R1, StateIndex.R1])
                cov_r2.append(tracker.ukf.P[StateIndex.R2, StateIndex.R2])
                cov_dzc.append(tracker.ukf.P[StateIndex.DZA, StateIndex.DZA])  # 占位
                cov_dza.append(tracker.ukf.P[StateIndex.DZA, StateIndex.DZA])
    
    est_positions = np.array(est_positions)
    est_yaws = np.array(est_yaws)
    est_r1 = np.array(est_r1)
    est_r2 = np.array(est_r2)
    est_dzc = np.array(est_dzc)
    est_dza = np.array(est_dza)
    cov_r1 = np.array(cov_r1)
    cov_r2 = np.array(cov_r2)
    cov_dzc = np.array(cov_dzc)
    cov_dza = np.array(cov_dza)
    
    true_positions = trajectory['center_positions'].copy()
    # UKF估计的z是装甲板平均高度(center_z + dzc)
    dzc = true_params.get('dzc', 0.0)
    true_positions[:, 2] = true_positions[:, 2] + dzc
    
    position_errors = np.linalg.norm(est_positions - true_positions, axis=1)
    position_rmse = np.sqrt(np.mean(position_errors**2))
    
    true_yaws = trajectory['center_yaws']
    yaw_errors = np.array([compute_yaw_error_symmetric(est_yaws[i], true_yaws[i]) for i in range(n_frames)])
    yaw_rmse = np.sqrt(np.mean(yaw_errors**2))
    
    signed_yaw_errors = np.array([compute_signed_yaw_error(est_yaws[i], true_yaws[i]) for i in range(n_frames)])
    
    r1_errors = np.abs(est_r1 - true_params['r1']) / true_params['r1']
    r2_errors = np.abs(est_r2 - true_params['r2']) / true_params['r2']
    
    signed_r1_errors = (est_r1 - true_params['r1']) / true_params['r1']
    signed_r2_errors = (est_r2 - true_params['r2']) / true_params['r2']
    
    dzc_errors = np.abs(est_dzc - true_params['dzc'])
    dza_errors = np.abs(est_dza - true_params['dza'])
    
    signed_dzc = (est_dzc - true_params['dzc'])
    signed_dza = (est_dza - true_params['dza'])
    
    r1_weighted_error = compute_weighted_parameter_error(est_r1, true_params['r1'], cov_r1)
    r2_weighted_error = compute_weighted_parameter_error(est_r2, true_params['r2'], cov_r2)
    dzc_weighted_error = compute_weighted_parameter_error(est_dzc, true_params['dzc'], cov_dzc)
    dza_weighted_error = compute_weighted_parameter_error(est_dza, true_params['dza'], cov_dza)
    
    r1_convergence = find_convergence_frame(r1_errors, threshold=0.05)
    r2_convergence = find_convergence_frame(r2_errors, threshold=0.05)
    
    # 计算特征点重投影误差
    panel_angles = [0, np.pi/2, np.pi, 3*np.pi/2]
    visible_panels = [0, 1]
    n = n_frames
    true_armor_positions = np.zeros((n, len(visible_panels), 3))
    
    for i in range(n):
        cp = true_positions[i]
        cy = true_yaws[i]
        for j, pid in enumerate(visible_panels):
            pa = panel_angles[pid]
            armor_yaw_out = cy + pa
            r_true = true_params['r1'] if pid % 2 == 0 else true_params['r2']
            dx = r_true * np.cos(armor_yaw_out)
            dy = r_true * np.sin(armor_yaw_out)
            dz = true_params['dza'] if pid % 2 == 0 else -true_params['dza']
            true_armor_positions[i, j] = cp + np.array([dx, dy, true_params['dzc'] + dz])
    
    recon_positions = np.zeros_like(true_armor_positions)
    for i in range(n):
        ep = est_positions[i]
        ey = est_yaws[i]
        for j, pid in enumerate(visible_panels):
            pa = panel_angles[pid]
            armor_yaw_out = ey + pa
            r_est = est_r1[i] if pid % 2 == 0 else est_r2[i]
            dx = r_est * np.cos(armor_yaw_out)
            dy = r_est * np.sin(armor_yaw_out)
            dz = est_dza[i] if pid % 2 == 0 else -est_dza[i]
            recon_positions[i, j] = ep + np.array([dx, dy, est_dzc[i] + dz])
    
    per_panel_err = np.linalg.norm(recon_positions - true_armor_positions, axis=2)
    per_frame_err = np.sqrt(np.mean(per_panel_err**2, axis=1))
    feature_rmse = np.sqrt(np.mean(per_frame_err**2))
    
    metrics = PerformanceMetrics(
        position_rmse=position_rmse,
        position_errors=position_errors,
        yaw_rmse=yaw_rmse,
        yaw_errors=yaw_errors,
        r1_weighted_error=r1_weighted_error,
        r2_weighted_error=r2_weighted_error,
        dzc_weighted_error=dzc_weighted_error,
        dza_weighted_error=dza_weighted_error,
        r1_convergence_frame=r1_convergence,
        r2_convergence_frame=r2_convergence,
        r1_errors=r1_errors,
        r2_errors=r2_errors,
        dzc_errors=dzc_errors,
        dza_errors=dza_errors,
        feature_rmse=feature_rmse,
        feature_errors=per_frame_err
    )
    
    results = {
        'est_positions': est_positions,
        'est_yaws': est_yaws,
        'est_r1': est_r1,
        'est_r2': est_r2,
        'est_dzc': est_dzc,
        'est_dza': est_dza,
        'cov_r1': cov_r1,
        'cov_r2': cov_r2,
        'cov_dzc': cov_dzc,
        'cov_dza': cov_dza,
        'true_armor_positions': true_armor_positions,
        'recon_armor_positions': recon_positions,
        'feature_errors': per_frame_err,
        'true_positions': true_positions,
        'true_yaws': true_yaws,
        'true_params': true_params,
        'true_labels': true_labels,
        'signed_yaw_errors': signed_yaw_errors,
        'signed_r1_errors': signed_r1_errors,
        'signed_r2_errors': signed_r2_errors,
        'signed_dzc': signed_dzc,
        'signed_dza': signed_dza,
        'signed_position_errors': est_positions - true_positions
    }
    
    return metrics, results, tracker


def unwrap_angles(angles):
    """展开角度序列，消除±π跳变"""
    return np.unwrap(angles)


def plot_results(results_dict: Dict[str, Tuple[PerformanceMetrics, Dict, AdaptiveArmorTracker]], save_path: str = None):
    fig = plt.figure(figsize=(16, 12))
    
    ax1 = plt.subplot(3, 3, 1)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(metrics.position_errors)) * 0.05
        ax1.plot(timestamps, metrics.position_errors * 100, label=name, alpha=0.7)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position Error (cm)')
    ax1.set_title('Position Tracking Error')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2 = plt.subplot(3, 3, 2)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(metrics.yaw_errors)) * 0.05
        ax2.plot(timestamps, np.rad2deg(metrics.yaw_errors), label=name, alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Yaw Error (deg)')
    ax2.set_title('Yaw Tracking Error (180° Symmetric)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    ax3 = plt.subplot(3, 3, 4)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(results['est_r1'])) * 0.05
        ax3.plot(timestamps, results['est_r1'], label=name, alpha=0.7)
        ax3.axhline(results['true_params']['r1'], color='k', linestyle='--', alpha=0.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('R1 (m)')
    ax3.set_title('R1 Parameter Estimation')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    ax4 = plt.subplot(3, 3, 5)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(results['est_r2'])) * 0.05
        ax4.plot(timestamps, results['est_r2'], label=name, alpha=0.7)
        ax4.axhline(results['true_params']['r2'], color='k', linestyle='--', alpha=0.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('R2 (m)')
    ax4.set_title('R2 Parameter Estimation')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    ax5 = plt.subplot(3, 3, 7)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(metrics.r1_errors)) * 0.05
        ax5.plot(timestamps, metrics.r1_errors * 100, label=name, alpha=0.7)
    ax5.axhline(5, color='r', linestyle='--', alpha=0.5, label='5% threshold')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('R1 Relative Error (%)')
    ax5.set_title('R1 Estimation Error')
    ax5.legend()
    ax5.grid(True, alpha=0.3)
    ax5.set_ylim([0, 50])
    
    ax6 = plt.subplot(3, 3, 8)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(metrics.r2_errors)) * 0.05
        ax6.plot(timestamps, metrics.r2_errors * 100, label=name, alpha=0.7)
    ax6.axhline(5, color='r', linestyle='--', alpha=0.5, label='5% threshold')
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('R2 Relative Error (%)')
    ax6.set_title('R2 Estimation Error')
    ax6.legend()
    ax6.grid(True, alpha=0.3)
    ax6.set_ylim([0, 50])
    
    ax7 = plt.subplot(3, 3, 3)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(results['est_dzc'])) * 0.05
        ax7.plot(timestamps, results['est_dzc'] * 100, label=name, alpha=0.7)
        ax7.axhline(results['true_params']['dzc'] * 100, color='k', linestyle='--', alpha=0.5)
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('dzc (cm)')
    ax7.set_title('dzc Parameter Estimation')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    ax8 = plt.subplot(3, 3, 6)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(results['est_dza'])) * 0.05
        ax8.plot(timestamps, results['est_dza'] * 100, label=name, alpha=0.7)
        ax8.axhline(results['true_params']['dza'] * 100, color='k', linestyle='--', alpha=0.5)
    ax8.set_xlabel('Time (s)')
    ax8.set_ylabel('dza (cm)')
    ax8.set_title('dza Parameter Estimation')
    ax8.legend()
    ax8.grid(True, alpha=0.3)
    
    ax9 = plt.subplot(3, 3, 9)
    scenario_names = list(results_dict.keys())
    pos_rmse_list = [results_dict[name][0].position_rmse * 100 for name in scenario_names]
    x_pos = np.arange(len(scenario_names))
    ax9.bar(x_pos, pos_rmse_list, alpha=0.7)
    ax9.set_xticks(x_pos)
    ax9.set_xticklabels(scenario_names, rotation=15, ha='right')
    ax9.set_ylabel('Position RMSE (cm)')
    ax9.set_title('Position RMSE Comparison')
    ax9.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches='tight')
        print(f"\n图表已保存到: {save_path}")
    plt.show()
    
    # Signed-error figure
    fig2 = plt.figure(figsize=(14, 8))
    axs = [fig2.add_subplot(2, 3, i + 1) for i in range(6)]
    
    # 1. Signed position components (cm)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(metrics.position_errors)) * 0.05
        signed_pos = results.get('signed_position_errors', None)
        if signed_pos is not None:
            axs[0].plot(timestamps, signed_pos[:, 0] * 100, label=name + ' x', linestyle='-')
            axs[0].plot(timestamps, signed_pos[:, 1] * 100, label=name + ' y', linestyle='--')
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Signed Position Error (cm)')
    axs[0].set_title('Signed Position Components')
    axs[0].legend()
    axs[0].grid(True, alpha=0.3)
    
    # 2. Signed yaw (deg)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(metrics.yaw_errors)) * 0.05
        signed_yaw = results.get('signed_yaw_errors', None)
        if signed_yaw is not None:
            signed_yaw_unwrapped = unwrap_angles(signed_yaw)
            axs[1].plot(timestamps, np.rad2deg(signed_yaw_unwrapped), label=name)
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Signed Yaw Error (deg)')
    axs[1].set_title('Signed Yaw Error (unwrapped)')
    axs[1].legend()
    axs[1].grid(True, alpha=0.3)
    
    # 3. Signed R1 relative error (%)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(metrics.r1_errors)) * 0.05
        signed_r1 = results.get('signed_r1_errors', None)
        if signed_r1 is not None:
            axs[2].plot(timestamps, signed_r1 * 100, label=name)
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Signed R1 Rel Error (%)')
    axs[2].set_title('Signed R1 Relative Error')
    axs[2].legend()
    axs[2].grid(True, alpha=0.3)
    
    # 4. Signed R2 relative error (%)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(metrics.r2_errors)) * 0.05
        signed_r2 = results.get('signed_r2_errors', None)
        if signed_r2 is not None:
            axs[3].plot(timestamps, signed_r2 * 100, label=name)
    axs[3].set_xlabel('Time (s)')
    axs[3].set_ylabel('Signed R2 Rel Error (%)')
    axs[3].set_title('Signed R2 Relative Error')
    axs[3].legend()
    axs[3].grid(True, alpha=0.3)
    
    # 5. Signed dzc (cm)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(results['est_dzc'])) * 0.05
        signed_dzc = results.get('signed_dzc', None)
        if signed_dzc is not None:
            axs[4].plot(timestamps, signed_dzc * 100, label=name)
    axs[4].set_xlabel('Time (s)')
    axs[4].set_ylabel('Signed dzc (cm)')
    axs[4].set_title('Signed dzc Error')
    axs[4].legend()
    axs[4].grid(True, alpha=0.3)
    
    # 6. Signed dza (cm)
    for name, (metrics, results, tracker) in results_dict.items():
        timestamps = np.arange(len(results['est_dza'])) * 0.05
        signed_dza = results.get('signed_dza', None)
        if signed_dza is not None:
            axs[5].plot(timestamps, signed_dza * 100, label=name)
    axs[5].set_xlabel('Time (s)')
    axs[5].set_ylabel('Signed dza (cm)')
    axs[5].set_title('Signed dza Error')
    axs[5].legend()
    axs[5].grid(True, alpha=0.3)
    
    plt.tight_layout()
    if save_path:
        base, ext = os.path.splitext(save_path)
        signed_path = base + '_signed' + ext
        plt.savefig(signed_path, dpi=300, bbox_inches='tight')
        print(f"\nSigned-error 图表已保存到: {signed_path}")
    plt.show()


def print_metrics_summary(results_dict: Dict[str, Tuple[PerformanceMetrics, Dict, AdaptiveArmorTracker]]):
    print("\n" + "="*80)
    print("性能指标汇总 (Demo2)")
    print("="*80)
    
    for name, (metrics, results, tracker) in results_dict.items():
        print(f"\n【{name}】")
        print(f"  位置跟踪:")
        print(f"    RMSE: {metrics.position_rmse*100:.2f} cm")
        print(f"    最大误差: {np.max(metrics.position_errors)*100:.2f} cm")
        print(f"  姿态跟踪:")
        print(f"    Yaw RMSE: {np.rad2deg(metrics.yaw_rmse):.2f}°")
        print(f"    最大Yaw误差: {np.rad2deg(np.max(metrics.yaw_errors)):.2f}°")
        print(f"  参数估计 (加权误差):")
        print(f"    R1: {metrics.r1_weighted_error*100:.2f} cm (真值: {results['true_params']['r1']*100:.2f} cm)")
        print(f"    R2: {metrics.r2_weighted_error*100:.2f} cm (真值: {results['true_params']['r2']*100:.2f} cm)")
        print(f"    dzc: {metrics.dzc_weighted_error*100:.2f} cm (真值: {results['true_params']['dzc']*100:.2f} cm)")
        print(f"    dza: {metrics.dza_weighted_error*100:.2f} cm (真值: {results['true_params']['dza']*100:.2f} cm)")
        print(f"  收敛性:")
        if metrics.r1_convergence_frame >= 0:
            print(f"    R1收敛帧数: {metrics.r1_convergence_frame} ({metrics.r1_convergence_frame*0.05:.2f}s)")
        else:
            print(f"    R1收敛帧数: 未收敛")
        if metrics.r2_convergence_frame >= 0:
            print(f"    R2收敛帧数: {metrics.r2_convergence_frame} ({metrics.r2_convergence_frame*0.05:.2f}s)")
        else:
            print(f"    R2收敛帧数: 未收敛")
        
        final_r1 = np.mean(results['est_r1'][-10:])
        final_r2 = np.mean(results['est_r2'][-10:])
        final_dzc = np.mean(results['est_dzc'][-10:])
        final_dza = np.mean(results['est_dza'][-10:])
        print(f"  最终估计 (后10帧平均):")
        print(f"    R1: {final_r1*100:.2f} cm (误差: {abs(final_r1-results['true_params']['r1'])*100:.2f} cm)")
        print(f"    R2: {final_r2*100:.2f} cm (误差: {abs(final_r2-results['true_params']['r2'])*100:.2f} cm)")
        print(f"    dzc: {final_dzc*100:.2f} cm (误差: {abs(final_dzc-results['true_params']['dzc'])*100:.2f} cm)")
        print(f"    dza: {final_dza*100:.2f} cm (误差: {abs(final_dza-results['true_params']['dza'])*100:.2f} cm)")


def create_tracker_config() -> UnifiedConfig:
    """创建tracker配置（对应TrackerV3Config）"""
    config = UnifiedConfig.create_optimized()
    
    # 最佳配置（3/4场景达标，yaw全部优秀）
    config.ukf.single_obs_update_weight_pos = 1.0  
    config.ukf.obs_noise_pos = 0.047  
    config.ukf.obs_noise_yaw = 0.05  
    config.ukf.enable_innovation_gating = True  
    config.ukf.innovation_gate_chi2_threshold = 15.0  
    config.motion.ca_process_noise_acc = 0.1  
    config.ukf.dual_obs_noise_pos = 0.0003  
    config.ukf.dual_obs_noise_yaw = 0.01  
    
    return config


def generate_trajectory_with_visibility(
    n_frames: int = 500,
    dt: float = 0.05,
    velocity: float = 1.0,
    spin_rate: float = 1.5,
    r1: float = 0.20,
    r2: float = 0.25,
    dzc: float = 0.05,
    dza: float = 0.02,
    noise_pos: float = 0.001,
    observer_position: np.ndarray = None,
    visibility_angle: float = np.pi/2
) -> Dict:
    """生成包含可见性判断的轨迹数据"""
    duration = n_frames * dt
    poses, timestamps = TrajectoryGenerator.generate_2d_spinning_trajectory(
        n_poses=n_frames,
        duration=duration,
        max_velocity=velocity,
        max_spin_rate=spin_rate
    )
    
    # 验证实际速度
    positions_check = []
    for T in poses:
        R_, t_ = se3_to_rt(T)
        positions_check.append(t_[:2])
    positions_check = np.array(positions_check)
    velocities_check = np.linalg.norm(np.diff(positions_check, axis=0), axis=1) / dt
    print(f"轨迹速度验证 (velocity参数={velocity}):")
    print(f"  实际平均速度: {np.mean(velocities_check):.2f} m/s")
    print(f"  实际最大速度: {np.max(velocities_check):.2f} m/s")
    print(f"  平均每帧位移: {np.mean(velocities_check)*dt*100:.2f} cm")
    
    if observer_position is None:
        R0, t0 = se3_to_rt(poses[0])
        observer_position = np.array([t0[0] - 3.0, t0[1] + 3.0, 0.5])
    
    panel_angles = [0, np.pi/2, np.pi, 3*np.pi/2]
    center_positions = np.zeros((n_frames, 3))
    center_yaws = np.zeros(n_frames)
    armor_observations = []
    armor_yaws = []
    armor_layers = []
    num_visible_per_frame = []
    
    observer_offset = np.array([-3.0, 3.0, 0.0])
    
    for frame_idx, T in enumerate(poses):
        R, t = se3_to_rt(T)
        yaw = np.arctan2(R[1, 0], R[0, 0])
        center = np.array([t[0], t[1], 0.5])
        
        center_positions[frame_idx] = center
        center_yaws[frame_idx] = yaw
        
        current_observer = center + observer_offset
        
        frame_observations = []
        frame_yaws = []
        frame_layers = []
        
        for panel_idx in range(4):
            panel_angle = panel_angles[panel_idx]
            armor_yaw_outward = yaw + panel_angle
            
            r = r1 if panel_idx % 2 == 0 else r2
            layer_offset = dza if panel_idx % 2 == 0 else -dza
            layer = 'upper' if panel_idx % 2 == 0 else 'lower'
            
            position_angle = armor_yaw_outward
            dx = r * np.cos(position_angle)
            dy = r * np.sin(position_angle)
            armor_pos = center + np.array([dx, dy, dzc + layer_offset])
            
            armor_yaw = armor_yaw_outward + np.pi
            armor_yaw = normalize_angle(armor_yaw)
            
            # 可见性判断
            to_observer = current_observer[:2] - armor_pos[:2]
            distance_to_observer = np.linalg.norm(to_observer)
            
            if distance_to_observer < 1e-6:
                continue
            
            direction_to_observer = np.arctan2(to_observer[1], to_observer[0])
            panel_normal = armor_yaw_outward
            angle_diff = abs(angle_difference(direction_to_observer, panel_normal))
            
            if angle_diff <= visibility_angle:
                noise = np.random.randn(3) * noise_pos
                armor_pos_noisy = armor_pos + noise
                
                frame_observations.append(armor_pos_noisy)
                frame_yaws.append(armor_yaw)
                frame_layers.append(layer)
        
        armor_observations.append(frame_observations)
        armor_yaws.append(frame_yaws)
        armor_layers.append(frame_layers)
        num_visible_per_frame.append(len(frame_observations))
    
    return {
        'timestamps': timestamps,
        'center_positions': center_positions,
        'center_yaws': center_yaws,
        'armor_observations': armor_observations,
        'armor_yaws': armor_yaws,
        'armor_layers': armor_layers,
        'num_visible_per_frame': num_visible_per_frame,
        'true_params': {'r1': r1, 'r2': r2, 'dzc': dzc, 'dza': dza}
    }


def test_scenario_1_spin_only():
    print("\n" + "="*80)
    print("测试场景1：原地自旋（单观测）")
    print("="*80)
    
    params = TrajectoryParams(
        start_pos=np.array([0.8, 0.0, 0.5]),
        velocity=np.zeros(3),
        omega=18.0 * np.pi / 180 / 0.05,  # 18°/帧
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        duration=10.0,
        dt=0.05,
        single_observation=True  # 使用单观测
    )
    
    trajectory = generate_trajectory(params)
    
    # 验证速度
    positions = trajectory['center_positions']
    if len(positions) > 1:
        velocities = np.linalg.norm(np.diff(positions, axis=0), axis=1) / params.dt
        print(f"轨迹速度验证 (velocity参数={np.linalg.norm(params.velocity)}):")
        print(f"  实际平均速度: {np.mean(velocities):.2f} m/s")
        print(f"  实际最大速度: {np.max(velocities):.2f} m/s")
        print(f"  平均每帧位移: {np.mean(velocities)*params.dt*100:.2f} cm")
    
    config = create_tracker_config()
    metrics, results, tracker = run_tracker_test(trajectory, config, 0.05, log_prefix='scenario1_spin')
    return metrics, results, tracker


def test_scenario_2_maneuver_only():
    print("\n" + "="*80)
    print("测试场景2：仅机动（不自旋，基于可见性）")
    print("="*80)
    
    trajectory = generate_trajectory_with_visibility(
        n_frames=200,
        dt=0.05,
        velocity=0.15,
        spin_rate=0.0,
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        noise_pos=0.001,
        observer_position=None,
        visibility_angle=np.pi/3
    )
    
    config = create_tracker_config()
    metrics, results, tracker = run_tracker_test(trajectory, config, 0.05, log_prefix='scenario2_maneuver')
    return metrics, results, tracker


def test_scenario_3_mixed_motion():
    print("\n" + "="*80)
    print("测试场景3：自旋+机动（混合运动，基于可见性）")
    print("="*80)
    
    trajectory = generate_trajectory_with_visibility(
        n_frames=200,
        dt=0.05,
        velocity=0.15,
        spin_rate=2.0,
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        noise_pos=0.001,
        observer_position=None,
        visibility_angle=np.pi/3
    )
    
    config = create_tracker_config()
    metrics, results, tracker = run_tracker_test(trajectory, config, 0.05, log_prefix='scenario3_mixed')
    return metrics, results, tracker


def test_scenario_4_visibility_based():
    print("\n" + "="*80)
    print("测试场景4：基于可见性的动态观测（自旋+机动）")
    print("="*80)
    
    trajectory = generate_trajectory_with_visibility(
        n_frames=1000,
        dt=0.05,
        velocity=0.15,
        spin_rate=np.pi,
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        noise_pos=0.001,
        observer_position=None,
        visibility_angle=np.pi/3
    )
    
    # 打印可见性统计
    num_visible = trajectory['num_visible_per_frame']
    print(f"可见性统计:")
    print(f"  平均可见装甲板数: {np.mean(num_visible):.2f}")
    print(f"  最小可见装甲板数: {np.min(num_visible)}")
    print(f"  最大可见装甲板数: {np.max(num_visible)}")
    print(f"  可见分布: {np.bincount(num_visible)}")
    
    config = create_tracker_config()
    metrics, results, tracker = run_tracker_test(trajectory, config, 0.05, log_prefix='scenario4_visibility')
    return metrics, results, tracker


def main():
    print("="*80)
    print("AdaptiveArmorTracker 敏感性分析测试 (Demo2)")
    print("="*80)
    
    results_dict = {}
    
    metrics1, results1, tracker1 = test_scenario_1_spin_only()
    results_dict['Spin Only'] = (metrics1, results1, tracker1)
    
    metrics2, results2, tracker2 = test_scenario_2_maneuver_only()
    results_dict['Maneuver Only'] = (metrics2, results2, tracker2)
    
    metrics3, results3, tracker3 = test_scenario_3_mixed_motion()
    results_dict['Mixed Motion'] = (metrics3, results3, tracker3)
    
    metrics4, results4, tracker4 = test_scenario_4_visibility_based()
    results_dict['Visibility-Based'] = (metrics4, results4, tracker4)
    
    print_metrics_summary(results_dict)
    
    # 保存图表
    output_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # 保存JSON结果
    import json
    for scenario_name, (metrics, results, tracker) in results_dict.items():
        filename_map = {
            'Spin Only': 'scenario1_spin_only_results_demo2.json',
            'Maneuver Only': 'scenario2_maneuver_only_results_demo2.json',
            'Mixed Motion': 'scenario3_spin_and_movement_results_demo2.json'
        }
        if scenario_name in filename_map:
            json_path = os.path.join(output_dir, filename_map[scenario_name])
            json_data = []
            for i in range(len(metrics.position_errors)):
                json_data.append({
                    'frame': i,
                    'time': i * 0.05,
                    'position_error': float(metrics.position_errors[i]),
                    'yaw_error': float(metrics.yaw_errors[i]),
                    'r1_error': float(results['signed_r1_errors'][i]),
                    'r2_error': float(results['signed_r2_errors'][i]),
                    'r1_est': float(results['est_r1'][i]),
                    'r2_est': float(results['est_r2'][i])
                })
            with open(json_path, 'w') as f:
                json.dump(json_data, f, indent=2)
            print(f"已保存JSON结果: {json_path}")
    
    save_path = os.path.join(output_dir, 'tracker_demo2_sensitivity_analysis.png')
    plot_results(results_dict, save_path=save_path)
    
    print("\n测试完成!")


if __name__ == '__main__':
    np.random.seed(42)
    main()
