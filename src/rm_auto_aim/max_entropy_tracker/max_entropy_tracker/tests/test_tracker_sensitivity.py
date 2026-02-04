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
# from src.max_entropy_ukf_dual_radius_demo2.trackers.simple_armor_tracker import SimpleArmorTracker
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
    observation_direction: np.ndarray # 观测方向向量
    observation_opening_angle: float # 观测锥体开角（弧度）
    single_observation: bool = False  # 是否使用单观测（仅观测panel 0）
    noise_level: float = 0.05  # 观测噪声水平

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


def generate_ca_trajectory(params: TrajectoryParams, jerk_std: float = 0.5, yaw_jerk_std: float = 0.1, seed: int = 42) -> Dict:
    """
    使用CA (Constant Acceleration) 模型生成机动性更强的轨迹
    
    状态模型：
    - 位置运动: jerk (加加速度) 为零均值高斯噪声
      - acceleration_{k+1} = acceleration_k + jerk * dt
      - velocity_{k+1} = velocity_k + acceleration_k * dt
      - position_{k+1} = position_k + velocity_k * dt + 0.5 * acceleration_k * dt^2
    
    - 角度运动: yaw_jerk 为零均值高斯噪声
      - yaw_acceleration_{k+1} = yaw_acceleration_k + yaw_jerk * dt
      - yaw_rate_{k+1} = yaw_rate_k + yaw_acceleration_k * dt
      - yaw_{k+1} = yaw_k + yaw_rate_k * dt + 0.5 * yaw_acceleration_k * dt^2
    
    Args:
        params: 轨迹参数（velocity作为初始速度，omega作为初始角速度）
        jerk_std: 加加速度标准差 (m/s^3)
        yaw_jerk_std: 角加加速度标准差 (rad/s^3)
        seed: 随机种子，确保可重复性
    """
    np.random.seed(seed)
    
    n_steps = int(params.duration / params.dt)
    timestamps = np.arange(n_steps) * params.dt
    
    # 初始化状态
    position = params.start_pos.copy()
    velocity = params.velocity.copy()
    acceleration = np.zeros(3)
    
    yaw = 0.0
    yaw_rate = params.omega
    yaw_acceleration = 0.0
    
    # 存储轨迹
    center_positions = np.zeros((n_steps, 3))
    center_yaws = np.zeros(n_steps)
    
    for i in range(n_steps):
        # 记录当前状态
        center_positions[i] = position
        center_yaws[i] = yaw
        
        # 生成加加速度噪声（零均值高斯噪声）
        jerk = np.random.randn(3) * jerk_std
        yaw_jerk = np.random.randn() * yaw_jerk_std
        
        # 更新加速度
        acceleration += jerk * params.dt
        yaw_acceleration += yaw_jerk * params.dt
        
        # 更新速度
        velocity += acceleration * params.dt
        yaw_rate += yaw_acceleration * params.dt
        
        # 更新位置（使用CA模型的运动方程）
        position += velocity * params.dt + 0.5 * acceleration * params.dt**2
        yaw += yaw_rate * params.dt + 0.5 * yaw_acceleration * params.dt**2
        yaw = normalize_angle(yaw)
    
    # 生成观测数据
    armor_observations = []
    armor_yaws = []
    armor_layers = []
    num_visible_per_frame = []
    
    panel_angles = [0, np.pi/2, np.pi, 3*np.pi/2]
    # 始终遍历所有 4 个 panel，根据可见性筛选后再限制数量
    all_panels = [0, 1, 2, 3]
    
    for i in range(n_steps):
        center_pos = center_positions[i]
        center_yaw = center_yaws[i]
        
        frame_observations = []
        frame_yaws = []
        frame_layers = []
        
        for panel_idx in all_panels:
            panel_angle = panel_angles[panel_idx]
            # armor_yaw定义为：中心指向装甲板的径向方向（外法向）
            armor_yaw = center_yaw + panel_angle
            armor_yaw = normalize_angle(armor_yaw)
            
            # 判断观测方向是否在可见锥体内,由于 observation_direction 是固定的，需要计算装甲板方向与观测方向的夹角
            # 由于 observation_direction 指向目标,而 armor_yaw 指向装甲板,其夹角为钝角,需取补角
            armor_dir = np.array([np.cos(armor_yaw), np.sin(armor_yaw), 0.0])
            # 计算从装甲板指向观察者的单位向量（更直观且避免取补角）
            obs_dir = params.observation_direction
            obs_dir_norm = obs_dir / np.linalg.norm(obs_dir)
            dir_to_observer = -obs_dir_norm
            cos_angle = np.dot(armor_dir, dir_to_observer)
            # 确保装甲板面朝向观察者（机器人不透明）
            if cos_angle <= 0:
                continue
            angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
            # 使用绝对夹角与 observation_opening_angle 比较（无需除以 2）
            if angle > params.observation_opening_angle:
                continue  # 不在可见范围内，跳过该装甲板观测
            
            r = params.r1 if panel_idx % 2 == 0 else params.r2
            # 装甲板位置 = 中心 + 半径 * 径向单位向量
            dx = r * np.cos(armor_yaw)
            dy = r * np.sin(armor_yaw)
            
            # panel 0,2 (偶数，r1) → +dza → z更高 → upper层
            # panel 1,3 (奇数，r2) → -dza → z更低 → lower层
            layer_offset = params.dza if panel_idx % 2 == 0 else -params.dza
            armor_pos = center_pos + np.array([dx, dy, params.dzc + layer_offset])
            
            noise = np.random.randn(3) * params.noise_level
            armor_pos_noisy = armor_pos + noise
            
            frame_observations.append(armor_pos_noisy)
            frame_yaws.append(armor_yaw)
            layer = 'upper' if panel_idx % 2 == 0 else 'lower'
            frame_layers.append(layer)
        
        # 单观测模式：只保留第一个可见装甲板
        if params.single_observation and len(frame_observations) > 1:
            frame_observations = frame_observations[:1]
            frame_yaws = frame_yaws[:1]
            frame_layers = frame_layers[:1]
        
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
        'true_params': {'r1': params.r1, 'r2': params.r2, 'dzc': params.dzc, 'dza': params.dza}
    }


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
        # 始终遍历所有 4 个 panel，根据可见性筛选后再限制数量
        all_panels = [0, 1, 2, 3]
        
        frame_observations = []
        frame_yaws = []
        frame_layers = []
        
        for panel_idx in all_panels:
            panel_angle = panel_angles[panel_idx]
            # armor_yaw定义为：中心指向装甲板的径向方向（外法向）
            armor_yaw = center_yaw + panel_angle
            armor_yaw = normalize_angle(armor_yaw)
            
            # 判断观测方向是否在可见锥体内,由于 observation_direction 是固定的，需要计算装甲板方向与观测方向的夹角
            # 由于 observation_direction 指向目标,而 armor_yaw 指向装甲板,其夹角为钝角,需取补角
            armor_dir = np.array([np.cos(armor_yaw), np.sin(armor_yaw), 0.0])
            # 计算从装甲板指向观察者的单位向量（更直观且避免取补角）
            obs_dir = params.observation_direction
            obs_dir_norm = obs_dir / np.linalg.norm(obs_dir)
            dir_to_observer = -obs_dir_norm
            cos_angle = np.dot(armor_dir, dir_to_observer)
            # 确保装甲板面朝向观察者（机器人不透明）
            if cos_angle <= 0:
                continue
            angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
            # 使用绝对夹角与 observation_opening_angle 比较（无需除以 2）
            if angle > params.observation_opening_angle:
                continue  # 不在可见范围内，跳过该装甲板观测
            
            r = params.r1 if panel_idx % 2 == 0 else params.r2
            # 装甲板位置 = 中心 + 半径 * 径向单位向量
            dx = r * np.cos(armor_yaw)
            dy = r * np.sin(armor_yaw)
            
            # panel 0,2 (偶数，r1) → +dza → z更高 → upper层
            # panel 1,3 (奇数，r2) → -dza → z更低 → lower层
            layer_offset = params.dza if panel_idx % 2 == 0 else -params.dza
            armor_pos = center_pos + np.array([dx, dy, params.dzc + layer_offset])
            
            noise = np.random.randn(3) * params.noise_level
            armor_pos_noisy = armor_pos + noise
            
            frame_observations.append(armor_pos_noisy)
            frame_yaws.append(armor_yaw)
            layer = 'upper' if panel_idx % 2 == 0 else 'lower'
            frame_layers.append(layer)
        
        # 单观测模式：只保留第一个可见装甲板
        if params.single_observation and len(frame_observations) > 1:
            frame_observations = frame_observations[:1]
            frame_yaws = frame_yaws[:1]
            frame_layers = frame_layers[:1]
        
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


def run_tracker_test(trajectory: Dict, config: UnifiedConfig, dt: float, log_prefix: str = "test", tracker_type: str = 'adaptive') -> Tuple[PerformanceMetrics, Dict]:
    import json
    
    # 创建日志目录
    log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'outputs', 'debug_logs')
    os.makedirs(log_dir, exist_ok=True)
    
    # 保存真实轨迹数据
    ground_truth_log = os.path.join(log_dir, f'{log_prefix}_demo2_ground_truth.jsonl')
    observations_log = os.path.join(log_dir, f'{log_prefix}_demo2_observations.jsonl')
    tracker_states_log = os.path.join(log_dir, f'{log_prefix}_demo2_tracker_states.jsonl')
    ukf_states_log = os.path.join(log_dir, f'{log_prefix}_demo2_ukf_states.jsonl')
    
    # 清空旧日志文件（避免追加模式下的数据混乱）
    for log_file in [ground_truth_log, observations_log, tracker_states_log, ukf_states_log]:
        if os.path.exists(log_file):
            os.remove(log_file)
    
    # 根据类型创建tracker
    if tracker_type == 'simple':
        # Simple版本禁用innovation gating（或放宽阈值）避免首帧漂移导致恶性循环
        config.ukf.enable_innovation_gating = False  # 暂时禁用
        # tracker = SimpleArmorTracker(config=config, dt=dt)
        tracker_name = 'SimpleArmorTracker'
    else:
        tracker = AdaptiveArmorTracker(config=config, dt=dt)
        tracker_name = 'AdaptiveArmorTracker'
    
    print(f"\n使用 {tracker_name}")
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
    
    IS_INIT = False
    
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
        
        if IS_INIT == False and len(observations) == 0:
            # 无观测，跳过
            est_positions.append(tracker.get_center_position())
            est_yaws.append(tracker.get_yaw())
            r1, r2 = tracker.get_radii()
            est_r1.append(r1)
            est_r2.append(r2)
            est_dzc.append(0.0)
            est_dza.append(0.0)
            cov_r1.append(np.nan)
            cov_r2.append(np.nan)
            cov_dzc.append(np.nan)
            cov_dza.append(np.nan)
            continue
        elif IS_INIT == False:
            # 初始化：如果有双观测，使用平均z来消除upper/lower的dza偏移
            if len(observations) == 2:
                avg_z = (observations[0][2] + observations[1][2]) / 2.0
                init_obs_array = observations[0].copy()
                init_obs_array[2] = avg_z
                init_obs_list = [ObservationData.from_array(init_obs_array, yaws[0])]
            else:
                init_obs_list = obs_list[:1]
            
            # 初始化值设置为稍微靠近真值，帮助 UKF 更快收敛（但保留一定偏差以检验滤波器收敛能力）
            tracker.initialize(init_obs_list, r1=0.17, r2=0.29, dza=0.005)
            IS_INIT = True
        else:
            # 使用轨迹时间戳与tracker同步预测时间（如果轨迹包含timestamps）
            timestamps = trajectory.get('timestamps', None)
            if timestamps is not None:
                target_time = float(timestamps[i])
                tracker.predict(target_time)
            else:
                tracker.predict(i * dt)
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
        
        # 记录UKF状态（使用动态布局以支持不同过程模型）
        if tracker.ukf.x is not None:
            with open(ukf_states_log, 'a') as f:
                pm_info = tracker.ukf.process_model.get_state_info()
                # state_info.layout is a dict name->index, convert to ordered names by index
                ordered = sorted(pm_info['layout'].items(), key=lambda x: x[1])
                state_names = [name for name, _ in ordered]
                ukf_state = {
                    'frame': i,
                    'time': i * dt,
                    'state_vector': tracker.ukf.x.tolist(),
                    'state_names': state_names,
                    'covariance_diag': np.diag(tracker.ukf.P).tolist(),
                    'k': tracker.ukf.k,  # 记录离散模态k
                    'mode_switches': tracker.ukf.mode_switches  # 记录模态切换次数
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
            # 使用动态索引访问状态，避免静态枚举与过程模型布局不一致的问题
            sidx = tracker.ukf.state_idx
            est_dza.append(tracker.ukf.x[sidx.DZA])
            
            if tracker.ukf.P is not None:
                cov_r1.append(tracker.ukf.P[sidx.R1, sidx.R1])
                cov_r2.append(tracker.ukf.P[sidx.R2, sidx.R2])
                cov_dzc.append(tracker.ukf.P[sidx.DZA, sidx.DZA])  # 占位
                cov_dza.append(tracker.ukf.P[sidx.DZA, sidx.DZA])
    
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
    
    return metrics, results


def unwrap_angles(angles):
    """展开角度序列，消除±π跳变"""
    return np.unwrap(angles)


def plot_results(results_dict: Dict[str, Tuple[PerformanceMetrics, Dict]], save_path: str = None):
    fig = plt.figure(figsize=(16, 12))
    
    ax1 = plt.subplot(3, 3, 1)
    for name, (metrics, results) in results_dict.items():
        timestamps = np.arange(len(metrics.position_errors)) * 0.05
        ax1.plot(timestamps, metrics.position_errors * 100, label=name, alpha=0.7)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position Error (cm)')
    ax1.set_title('Position Tracking Error')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2 = plt.subplot(3, 3, 2)
    for name, (metrics, results) in results_dict.items():
        timestamps = np.arange(len(metrics.yaw_errors)) * 0.05
        ax2.plot(timestamps, np.rad2deg(metrics.yaw_errors), label=name, alpha=0.7)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Yaw Error (deg)')
    ax2.set_title('Yaw Tracking Error (180° Symmetric)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    ax3 = plt.subplot(3, 3, 4)
    for name, (metrics, results) in results_dict.items():
        timestamps = np.arange(len(results['est_r1'])) * 0.05
        ax3.plot(timestamps, results['est_r1'], label=name, alpha=0.7)
        ax3.axhline(results['true_params']['r1'], color='k', linestyle='--', alpha=0.5)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('R1 (m)')
    ax3.set_title('R1 Parameter Estimation')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    ax4 = plt.subplot(3, 3, 5)
    for name, (metrics, results) in results_dict.items():
        timestamps = np.arange(len(results['est_r2'])) * 0.05
        ax4.plot(timestamps, results['est_r2'], label=name, alpha=0.7)
        ax4.axhline(results['true_params']['r2'], color='k', linestyle='--', alpha=0.5)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('R2 (m)')
    ax4.set_title('R2 Parameter Estimation')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    ax5 = plt.subplot(3, 3, 7)
    for name, (metrics, results) in results_dict.items():
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
    for name, (metrics, results) in results_dict.items():
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
    for name, (metrics, results) in results_dict.items():
        timestamps = np.arange(len(results['est_dzc'])) * 0.05
        ax7.plot(timestamps, results['est_dzc'] * 100, label=name, alpha=0.7)
        ax7.axhline(results['true_params']['dzc'] * 100, color='k', linestyle='--', alpha=0.5)
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('dzc (cm)')
    ax7.set_title('dzc Parameter Estimation')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    ax8 = plt.subplot(3, 3, 6)
    for name, (metrics, results) in results_dict.items():
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
    for name, (metrics, results) in results_dict.items():
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
    for name, (metrics, results) in results_dict.items():
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
    for name, (metrics, results) in results_dict.items():
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
    for name, (metrics, results) in results_dict.items():
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
    for name, (metrics, results) in results_dict.items():
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
    for name, (metrics, results) in results_dict.items():
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


def print_metrics_summary(results_dict: Dict[str, Tuple[PerformanceMetrics, Dict]]):
    print("\n" + "="*80)
    print("性能指标汇总 (Demo2)")
    print("="*80)
    
    for name, (metrics, results) in results_dict.items():
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
    """创建tracker配置（对应TrackerV3Config）。

    调整目标：降低 r1/r2/dza 偏差到 5% 内，尽量把位置误差控制在 10-15cm，yaw 在 5° 内。
    主要思路：降低单观测对位置的过强权重，合理设置观测噪声与结构过程噪声，让 UKF 能更快收敛到正确的 dza/r 值。
    """
    config = UnifiedConfig.create_optimized()
    
    # 调参：减少单观测对位置更新的权重（单观测信息量小，容易引入偏差）
    # 增大alpha以改善 Sigma 点数值稳定性（避免 cholesky 非正定）
    config.ukf.alpha = 0.5
    config.ukf.single_obs_update_weight_pos = 0.2
    # 观测噪声设为 1cm 级别，yaw 约 1.1°（以提高对 dza 的敏感性）
    config.ukf.obs_noise_pos = 0.01
    config.ukf.obs_noise_yaw = 0.02
    # 启用创新门控，使用较常见阈值（2 DOF 的 95% chi2 ≈ 5.99，但保持保守一些）
    config.ukf.enable_innovation_gating = True
    config.ukf.innovation_gate_chi2_threshold = 9.49
    # 适度增加运动模型的过程噪声，允许结构参数（dza/dzc）有更大适应性
    config.motion.ca_process_noise_acc = 0.5
    config.motion.process_noise_dz = 0.01
    # 双观测时更可信但不应过度自信
    config.ukf.dual_obs_noise_pos = 0.003
    config.ukf.dual_obs_noise_yaw = 0.008
    
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

def test_scenario_0_no_motion(tracker_type: str = 'adaptive'):
    print("\n" + "="*80)
    print(f"测试场景0：原地静止（单观测） - {tracker_type}")
    print("="*80)
    
    params = TrajectoryParams(
        start_pos=np.array([0.8, 0.0, 0.5]),
        velocity=np.zeros(3),
        omega=0.0,
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        duration=10.0,
        dt=0.05,
        single_observation=True,  # 修正：使用单观测
        noise_level=0.01,
        observation_direction=np.array([1.0, 0.0, 0.0]),
        observation_opening_angle=np.pi / 3
    )
    
    trajectory = generate_trajectory(params)
    config = create_tracker_config()
    metrics, results = run_tracker_test(trajectory, config, 0.05, log_prefix=f'scenario0_no_motion_{tracker_type}', tracker_type=tracker_type)
    return metrics, results

def test_scenario_1_spin_only(tracker_type: str = 'adaptive'):
    print("\n" + "="*80)
    print(f"测试场景1：原地自旋（单观测） - {tracker_type}")
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
        single_observation=True,  # 修正：使用单观测
        noise_level=0.01,
        observation_direction=np.array([1.0, 0.0, 0.0]),
        observation_opening_angle=np.pi / 3
    )
    
    trajectory = generate_trajectory(params)
    config = create_tracker_config()
    metrics, results = run_tracker_test(trajectory, config, 0.05, log_prefix=f'scenario1_spin_{tracker_type}', tracker_type=tracker_type)
    return metrics, results


def test_scenario_2_maneuver_only(tracker_type: str = 'adaptive'):
    print("\n" + "="*80)
    print(f"测试场景2：仅机动（不自旋，双观测） - {tracker_type}")
    print("="*80)
    
    params = TrajectoryParams(
        start_pos=np.array([0.0, 0.0, 0.5]),
        velocity=np.array([0.15, 0.10, 0.0]),
        omega=0.0,
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        duration=10.0,
        dt=0.05,
        single_observation=False,  # 使用双观测
        noise_level=0.01,
        observation_direction=np.array([1.0, 1.0, 0.0]),
        observation_opening_angle=np.pi / 3
    )
    
    trajectory = generate_trajectory(params)
    config = create_tracker_config()
    metrics, results = run_tracker_test(trajectory, config, 0.05, log_prefix=f'scenario2_maneuver_{tracker_type}', tracker_type=tracker_type)
    return metrics, results


def test_scenario_3_mixed_motion(tracker_type: str = 'adaptive'):
    print("\n" + "="*80)
    print(f"测试场景3：自旋+机动（混合运动，双观测） - {tracker_type}")
    print("="*80)
    
    params = TrajectoryParams(
        start_pos=np.array([0.0, 0.0, 0.5]),
        velocity=np.array([0.10, 0.08, 0.0]),
        omega=10.0 * np.pi / 180 / 0.05,  # 10°/帧
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        duration=10.0,
        dt=0.05,
        single_observation=False,  # 使用双观测
        noise_level=0.01,
        observation_direction=np.array([1.0, 1.0, 0.0]),
        observation_opening_angle=np.pi / 3
    )
    
    trajectory = generate_trajectory(params)
    config = create_tracker_config()
    metrics, results = run_tracker_test(trajectory, config, 0.05, log_prefix=f'scenario3_mixed_{tracker_type}', tracker_type=tracker_type)
    return metrics, results


def test_scenario_4_visibility_based():
    print("\n" + "="*80)
    print("测试场景4：基于可见性的动态观测（自旋+机动）")
    print("="*80)


def test_scenario_5_ca_agile_maneuver(tracker_type: str = 'adaptive'):
    """测试场景5：CA模型生成的高机动性轨迹（单观测）"""
    print("\n" + "="*80)
    print(f"测试场景5：CA模型高机动性轨迹（单观测） - {tracker_type}")
    print("="*80)
    
    params = TrajectoryParams(
        start_pos=np.array([0.0, 0.0, 0.5]),
        velocity=np.array([0.15, 0.10, 0.0]),  # 初始速度
        omega=15.0 * np.pi / 180 / 0.05,  # 15°/帧 初始角速度
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        duration=10.0,
        dt=0.05,
        single_observation=True,  # 使用单观测
        noise_level=0.01,
        observation_direction=np.array([1.0, 0.0, 0.0]),
        observation_opening_angle=np.pi / 3
    )
    
    # 使用CA模型生成轨迹（jerk驱动）
    trajectory = generate_ca_trajectory(params, jerk_std=0.8, yaw_jerk_std=0.15, seed=42)
    
    # 计算轨迹统计信息
    positions = trajectory['center_positions']
    yaws = trajectory['center_yaws']
    velocities = np.linalg.norm(np.diff(positions, axis=0), axis=1) / params.dt
    yaw_rates = np.abs(np.diff(yaws)) / params.dt
    
    print(f"\nCA轨迹统计:")
    print(f"  平均速度: {np.mean(velocities):.3f} m/s")
    print(f"  最大速度: {np.max(velocities):.3f} m/s")
    print(f"  速度标准差: {np.std(velocities):.3f} m/s")
    print(f"  平均角速度: {np.rad2deg(np.mean(yaw_rates)):.2f}°/s")
    print(f"  最大角速度: {np.rad2deg(np.max(yaw_rates)):.2f}°/s")
    print(f"  总位移: {np.linalg.norm(positions[-1] - positions[0]):.3f} m")
    print(f"  总旋转: {np.rad2deg(abs(yaws[-1] - yaws[0])):.2f}°")
    
    config = create_tracker_config()
    metrics, results = run_tracker_test(trajectory, config, 0.05, log_prefix=f'scenario5_ca_agile_{tracker_type}', tracker_type=tracker_type)
    return metrics, results


def test_scenario_6_ca_extreme_maneuver(tracker_type: str = 'adaptive'):
    """测试场景6：CA模型生成的极限机动性轨迹（双观测）"""
    print("\n" + "="*80)
    print(f"测试场景6：CA模型极限机动性轨迹（双观测） - {tracker_type}")
    print("="*80)
    
    params = TrajectoryParams(
        start_pos=np.array([0.0, 0.0, 0.5]),
        velocity=np.array([0.20, 0.15, 0.0]),  # 更高的初始速度
        omega=20.0 * np.pi / 180 / 0.05,  # 20°/帧 更快的初始旋转
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        duration=15.0,  # 更长的测试时间
        dt=0.05,
        single_observation=False,  # 双观测
        noise_level=0.01,
        observation_direction=np.array([1.0, 1.0, 0.0]),
        observation_opening_angle=np.pi / 3
    )
    
    # 使用更强的jerk生成极限机动轨迹
    trajectory = generate_ca_trajectory(params, jerk_std=1.2, yaw_jerk_std=0.25, seed=123)
    
    # 计算轨迹统计信息
    positions = trajectory['center_positions']
    yaws = trajectory['center_yaws']
    velocities = np.linalg.norm(np.diff(positions, axis=0), axis=1) / params.dt
    accelerations = np.diff(velocities) / params.dt
    yaw_rates = np.abs(np.diff(yaws)) / params.dt
    
    print(f"\nCA极限轨迹统计:")
    print(f"  平均速度: {np.mean(velocities):.3f} m/s")
    print(f"  最大速度: {np.max(velocities):.3f} m/s")
    print(f"  速度标准差: {np.std(velocities):.3f} m/s")
    print(f"  平均加速度: {np.mean(np.abs(accelerations)):.3f} m/s²")
    print(f"  最大加速度: {np.max(np.abs(accelerations)):.3f} m/s²")
    print(f"  平均角速度: {np.rad2deg(np.mean(yaw_rates)):.2f}°/s")
    print(f"  最大角速度: {np.rad2deg(np.max(yaw_rates)):.2f}°/s")
    print(f"  总位移: {np.linalg.norm(positions[-1] - positions[0]):.3f} m")
    print(f"  总旋转: {np.rad2deg(abs(yaws[-1] - yaws[0])):.2f}°")
    
    config = create_tracker_config()
    metrics, results = run_tracker_test(trajectory, config, 0.05, log_prefix=f'scenario6_ca_extreme_{tracker_type}', tracker_type=tracker_type)
    return metrics, results


def test_scenario_7_ca_spinning_maneuver(tracker_type: str = 'adaptive'):
    """测试场景7：CA模型生成的自旋机动混合轨迹（双观测）"""
    print("\n" + "="*80)
    print(f"测试场景7：CA模型自旋机动混合轨迹（双观测） - {tracker_type}")
    print("="*80)
    
    params = TrajectoryParams(
        start_pos=np.array([0.5, 0.5, 0.5]),
        velocity=np.array([0.12, 0.12, 0.0]),
        omega=18.0 * np.pi / 180 / 0.05,  # 18°/帧 快速旋转
        r1=0.20,
        r2=0.25,
        dzc=0.05,
        dza=0.02,
        duration=12.0,
        dt=0.05,
        single_observation=False,
        noise_level=0.01,
        observation_direction=np.array([1.0, 1.0, 0.0]),
        observation_opening_angle=np.pi / 3
    )
    
    # 中等jerk强度，模拟真实机器人的机动能力
    trajectory = generate_ca_trajectory(params, jerk_std=0.6, yaw_jerk_std=0.18, seed=456)
    
    # 计算轨迹统计信息
    positions = trajectory['center_positions']
    yaws = trajectory['center_yaws']
    velocities = np.linalg.norm(np.diff(positions, axis=0), axis=1) / params.dt
    yaw_rates = np.abs(np.diff(yaws)) / params.dt
    
    print(f"\nCA自旋机动混合轨迹统计:")
    print(f"  平均速度: {np.mean(velocities):.3f} m/s")
    print(f"  最大速度: {np.max(velocities):.3f} m/s")
    print(f"  速度标准差: {np.std(velocities):.3f} m/s")
    print(f"  平均角速度: {np.rad2deg(np.mean(yaw_rates)):.2f}°/s")
    print(f"  最大角速度: {np.rad2deg(np.max(yaw_rates)):.2f}°/s")
    print(f"  总位移: {np.linalg.norm(positions[-1] - positions[0]):.3f} m")
    print(f"  总旋转: {np.rad2deg(abs(yaws[-1] - yaws[0])):.2f}°")
    
    config = create_tracker_config()
    metrics, results = run_tracker_test(trajectory, config, 0.05, log_prefix=f'scenario7_ca_spinning_{tracker_type}', tracker_type=tracker_type)
    return metrics, results


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
    print("AdaptiveArmorTracker 敏感性测试 (Demo2) - 包含CA模型高机动性轨迹")
    print("="*80)
    
    results_dict = {}
    
    # 测试场景0: 原地静止
    print("\n" + "="*80)
    print("场景0: 原地静止")
    print("="*80)
    metrics0_adaptive, results0_adaptive = test_scenario_0_no_motion()
    results_dict['Stationary-Adaptive'] = (metrics0_adaptive, results0_adaptive)
    
    # 测试场景1: 原地自旋
    print("\n" + "="*80)
    print("场景1: 原地自旋")
    print("="*80)
    metrics1_adaptive, results1_adaptive = test_scenario_1_spin_only()
    results_dict['Spin-Adaptive'] = (metrics1_adaptive, results1_adaptive)
    
    # 测试场景2: 仅机动
    print("\n" + "="*80)
    print("场景2: 仅机动")
    print("="*80)
    metrics2_adaptive, results2_adaptive = test_scenario_2_maneuver_only()
    results_dict['Maneuver-Adaptive'] = (metrics2_adaptive, results2_adaptive)
    
    # 测试场景3: 混合运动
    print("\n" + "="*80)
    print("场景3: 混合运动")
    print("="*80)
    metrics3_adaptive, results3_adaptive = test_scenario_3_mixed_motion()
    results_dict['Mixed-Adaptive'] = (metrics3_adaptive, results3_adaptive)
    
    # 新增：测试场景5: CA模型高机动性轨迹
    print("\n" + "="*80)
    print("场景5: CA模型高机动性轨迹")
    print("="*80)
    metrics5_adaptive, results5_adaptive = test_scenario_5_ca_agile_maneuver()
    results_dict['CA-Agile'] = (metrics5_adaptive, results5_adaptive)
    
    # 新增：测试场景6: CA模型极限机动性轨迹
    print("\n" + "="*80)
    print("场景6: CA模型极限机动性轨迹")
    print("="*80)
    metrics6_adaptive, results6_adaptive = test_scenario_6_ca_extreme_maneuver()
    results_dict['CA-Extreme'] = (metrics6_adaptive, results6_adaptive)
    
    # 新增：测试场景7: CA模型自旋机动混合轨迹
    print("\n" + "="*80)
    print("场景7: CA模型自旋机动混合轨迹")
    print("="*80)
    metrics7_adaptive, results7_adaptive = test_scenario_7_ca_spinning_maneuver()
    results_dict['CA-Spinning'] = (metrics7_adaptive, results7_adaptive)
    
    print_metrics_summary(results_dict)
    
    # 保存图表
    output_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), 'outputs')
    os.makedirs(output_dir, exist_ok=True)
    
    # 保存JSON结果
    import json
    for scenario_name, (metrics, results) in results_dict.items():
        # 简化文件名映射
        safe_name = scenario_name.replace(' ', '_').replace('-', '_').lower()
        json_path = os.path.join(output_dir, f'{safe_name}_demo2_comparison.json')
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
    
    save_path = os.path.join(output_dir, 'tracker_demo2_adaptive_vs_simple_comparison.png')
    plot_results(results_dict, save_path=save_path)
    
    print("\n测试完成!")


if __name__ == '__main__':
    np.random.seed(42)
    main()
