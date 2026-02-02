"""
双半径自旋UKF实现

专门用于4面装甲板机器人的位姿估计
使用Yaw分解方法: yaw = k*π + delta

状态向量 (11维):
[x, vx, y, vy, z, vz, delta, delta_rate, r1, r2, dza]

物理模型:
- z: 装甲板平面的平均高度
- dza: 装甲板完整高度差（upper层+dza, lower层-dza）
- r1: 偶数panel(0,2)使用的半径
- r2: 奇数panel(1,3)使用的半径
"""

import numpy as np
from enum import IntEnum
from typing import List, Optional, Dict, Any, Tuple
import logging

from .base_ukf import BaseUKF
from ..core.config import UnifiedConfig
from ..core.observation import ObservationData
from ..utils.angle_utils import (
    normalize_angle, decompose_yaw, compose_yaw, 
    select_best_k, delta_angle_diff
)
from ..utils.constraints import apply_state_constraints

logger = logging.getLogger(__name__)


class StateIndex(IntEnum):
    """
    状态索引枚举
    
    11维状态向量布局
    """
    X = 0           # X位置
    VX = 1          # X速度
    Y = 2           # Y位置
    VY = 3          # Y速度
    Z = 4           # Z位置（装甲板平均高度）
    VZ = 5          # Z速度
    DELTA = 6       # 连续角度 ∈ [-π/2, π/2]
    DELTA_RATE = 7  # 角速度
    R1 = 8          # 半径1（偶数panel）
    R2 = 9          # 半径2（奇数panel）
    DZA = 10        # 装甲板高度差


class DualRadiusSpinUKF(BaseUKF):
    """
    双半径自旋UKF
    
    核心特性:
    1. Yaw分解: yaw = k*π + delta
    2. 状态空间: 在连续delta空间更新 ([-π/2, π/2])
    3. 离散模态: k∈{0,1} 通过后验概率选择
    4. 双观测更新: 利用几何约束估计结构参数
    5. 单观测参数冻结: 单观测时冻结r1, r2, dza防止漂移
    """
    
    # 固定的装甲板配置（4面）
    N_PANELS = 4
    PANEL_ANGLE_STEP = np.pi / 2  # 90°
    
    def __init__(self, config: UnifiedConfig, dt: float = 0.1):
        """
        初始化双半径自旋UKF
        
        Args:
            config: 统一配置对象
            dt: 默认时间步长
        """
        super().__init__(config, dt)
        
        # 离散模态 k
        self.k: int = 0  # k ∈ {0, 1}
        self.last_k: Optional[int] = None
        self.mode_switches: int = 0
        
        # 初始化状态和协方差
        self._x = np.zeros(self.state_dim)
        self._P = np.eye(self.state_dim) * 100.0
        
        # 构建过程噪声
        self._Q = self._build_process_noise()
        
        # 初始化Sigma点生成器
        self._init_sigma_generator()
        
        logger.info("DualRadiusSpinUKF initialized")
    
    # ==================== 属性实现 ====================
    
    @property
    def state_dim(self) -> int:
        return 11
    
    @property
    def obs_dim(self) -> int:
        return 4  # [x, y, z, yaw]
    
    # ==================== 初始化 ====================
    
    def initialize(
        self,
        observations: List[ObservationData],
        r1: float = 0.15,
        r2: float = 0.20,
        dza: float = 0.0,
        **kwargs
    ) -> None:
        """
        初始化滤波器状态
        
        Args:
            observations: 初始观测列表（至少1个）
            r1: 初始半径1
            r2: 初始半径2
            dza: 初始装甲板高度差
        """
        if len(observations) == 0:
            raise ValueError("At least one observation required for initialization")
        
        obs = observations[0]
        
        # 从装甲板位置反推中心位置
        # armor_yaw指向中心，因此：center = armor + r * [cos(armor_yaw), sin(armor_yaw)]
        # 初始化时假设使用r1
        center_x = obs.x + r1 * np.cos(obs.yaw)
        center_y = obs.y + r1 * np.sin(obs.yaw)
        center_z = obs.z
        center_yaw = obs.yaw  # 首帧假设panel_idx=0
        
        # 分解yaw
        self.k, delta = decompose_yaw(center_yaw)
        self.last_k = self.k
        
        # 设置状态
        self._x[StateIndex.X] = center_x
        self._x[StateIndex.VX] = 0.0
        self._x[StateIndex.Y] = center_y
        self._x[StateIndex.VY] = 0.0
        self._x[StateIndex.Z] = center_z
        self._x[StateIndex.VZ] = 0.0
        self._x[StateIndex.DELTA] = delta
        self._x[StateIndex.DELTA_RATE] = 0.0
        self._x[StateIndex.R1] = r1
        self._x[StateIndex.R2] = r2
        self._x[StateIndex.DZA] = dza
        
        # 初始化协方差
        self._P = np.eye(self.state_dim)
        self._P[:6, :6] *= 0.1      # 位置和速度
        self._P[6:8, 6:8] *= 0.3    # delta和delta_rate
        self._P[8:10, 8:10] *= 0.01 # r1, r2
        self._P[10, 10] *= 0.05     # dza
        
        self._initialized = True
        
        logger.info(
            f"DualRadiusSpinUKF initialized at ({center_x:.2f}, {center_y:.2f}, {center_z:.2f}), "
            f"yaw={np.degrees(center_yaw):.1f}° (k={self.k}, delta={np.degrees(delta):.1f}°)"
        )
    
    # ==================== 过程模型 ====================
    
    def _build_process_noise(self) -> np.ndarray:
        """构建过程噪声矩阵 (11x11)"""
        Q = np.zeros((self.state_dim, self.state_dim))
        
        # 位置-速度噪声块 (简化的CA模型)
        q_pos = self.config.motion.ca_process_noise_acc
        dt = self.dt
        for i in range(3):
            idx = i * 2
            # [pos, vel] 块的噪声
            Q[idx, idx] = (dt**4 / 4) * q_pos**2
            Q[idx, idx+1] = (dt**3 / 2) * q_pos**2
            Q[idx+1, idx] = (dt**3 / 2) * q_pos**2
            Q[idx+1, idx+1] = (dt**2) * q_pos**2
        
        # Delta和delta_rate噪声
        q_spin = self.config.spin.spin_process_noise_delta_rate
        Q[StateIndex.DELTA, StateIndex.DELTA] = (dt**2) * q_spin**2
        Q[StateIndex.DELTA, StateIndex.DELTA_RATE] = dt * q_spin**2
        Q[StateIndex.DELTA_RATE, StateIndex.DELTA] = dt * q_spin**2
        Q[StateIndex.DELTA_RATE, StateIndex.DELTA_RATE] = q_spin**2
        
        # 结构参数噪声
        Q[StateIndex.R1, StateIndex.R1] = self.config.motion.process_noise_r**2
        Q[StateIndex.R2, StateIndex.R2] = self.config.motion.process_noise_r**2
        Q[StateIndex.DZA, StateIndex.DZA] = (self.config.motion.process_noise_dz * 0.1)**2
        
        return Q
    
    def _process_model(self, x: np.ndarray, dt: float) -> np.ndarray:
        """
        过程模型
        
        运动方程:
        - 平移: pos += vel * dt
        - 自旋: delta += delta_rate * dt
        - 结构参数: 保持不变（随机游走在噪声中体现）
        """
        x_next = x.copy()
        
        # 平移更新
        for i in range(3):
            pos_idx = i * 2
            vel_idx = i * 2 + 1
            x_next[pos_idx] = x[pos_idx] + x[vel_idx] * dt
        
        # Delta更新（不做归一化，保持连续性）
        x_next[StateIndex.DELTA] = x[StateIndex.DELTA] + x[StateIndex.DELTA_RATE] * dt
        
        return x_next
    
    # ==================== 观测模型 ====================
    
    def _observation_model(
        self,
        x: np.ndarray,
        r_type: str = 'r1',
        armor_layer: str = 'lower',
        panel_angle: float = 0.0,  # 添加panel角度参数
        **kwargs
    ) -> np.ndarray:
        """
        观测模型: 从中心状态预测装甲板位置
        
        Args:
            x: 状态向量
            r_type: 半径类型 ('r1' 或 'r2')
            armor_layer: 装甲板层级 ('upper' 或 'lower')
            panel_angle: Panel相对中心的角度（0, π/2, π, 3π/2）
            
        Returns:
            预测观测 [x_obs, y_obs, z_obs, center_yaw]  # 注意：返回center_yaw
        """
        x_c = x[StateIndex.X]
        y_c = x[StateIndex.Y]
        z_mean = x[StateIndex.Z]
        delta = x[StateIndex.DELTA]
        dza = x[StateIndex.DZA]
        
        # 获取半径
        if r_type == 'r1':
            radius = x[StateIndex.R1]
        else:
            radius = x[StateIndex.R2]
        
        # 完整yaw = k*π + delta
        center_yaw = compose_yaw(self.k, delta)
        
        # 装甲板朝向（outward）= center_yaw + panel_angle
        armor_yaw_outward = normalize_angle(center_yaw + panel_angle)
        
        # 装甲板位置 = 中心 + R(outward) * [r, 0]
        x_obs = x_c + radius * np.cos(armor_yaw_outward)
        y_obs = y_c + radius * np.sin(armor_yaw_outward)
        
        # 层级高度偏移
        if armor_layer == 'upper':
            layer_offset = dza
        else:
            layer_offset = -dza
        z_obs = z_mean + layer_offset
        
        # 返回center_yaw供更新使用
        return np.array([x_obs, y_obs, z_obs, center_yaw])
    
    def _observation_model_geometry(self, x: np.ndarray) -> np.ndarray:
        """
        几何观测模型（用于双观测更新）
        
        Returns:
            [x_center, y_center, z, r1, r2, dza]
        """
        return np.array([
            x[StateIndex.X],
            x[StateIndex.Y],
            x[StateIndex.Z],
            x[StateIndex.R1],
            x[StateIndex.R2],
            x[StateIndex.DZA]
        ])
    
    # ==================== 预测步骤 ====================
    
    def predict(self, dt: Optional[float] = None) -> None:
        """预测步骤"""
        if not self._initialized:
            logger.warning("UKF not initialized, skipping predict")
            return
        
        dt = dt if dt is not None else self.dt
        
        # 生成Sigma点
        sigma_points = self.generate_sigma_points(self._x, self._P)
        
        # 传播Sigma点
        sigma_points_pred = np.array([
            self._process_model(sp, dt) for sp in sigma_points
        ])
        
        # 获取权重
        Wm, Wc = self.get_sigma_weights()
        
        # 计算预测均值
        self._x = np.sum(Wm[:, np.newaxis] * sigma_points_pred, axis=0)
        
        # 计算预测协方差（delta使用特殊角度差）
        P_pred = np.zeros((self.state_dim, self.state_dim))
        for i in range(len(sigma_points_pred)):
            diff = sigma_points_pred[i] - self._x
            # delta差使用特殊处理
            diff[StateIndex.DELTA] = delta_angle_diff(
                sigma_points_pred[i, StateIndex.DELTA],
                self._x[StateIndex.DELTA]
            )
            P_pred += Wc[i] * np.outer(diff, diff)
        
        # 添加过程噪声
        P_pred += self._Q
        
        self._P = P_pred
        
        # 应用约束
        self._apply_constraints()
        
        # 确保协方差正定
        self.ensure_covariance_valid()
    
    # ==================== 更新步骤 ====================
    
    def update(
        self,
        observations: List[ObservationData],
        r_types: Optional[List[str]] = None,
        armor_layers: Optional[List[str]] = None,
        height_confidence: float = 1.0,
        position_confidence: float = 1.0,
        panel_angle: float = 0.0,  # 添加panel_angle参数
        **kwargs
    ) -> bool:
        """
        更新步骤
        
        根据观测数量自动选择单观测或双观测更新策略
        
        Args:
            observations: 观测列表
            r_types: 半径类型列表 (['r1', 'r2', ...])
            armor_layers: 层级列表 (['upper', 'lower', ...'])
            height_confidence: 高度识别置信度
            position_confidence: 位置更新置信度
            panel_angle: Panel相对中心的角度（仅单观测时使用）
            
        Returns:
            是否更新成功
        """
        if not self._initialized:
            logger.warning("UKF not initialized, skipping update")
            return False
        
        if len(observations) == 0:
            logger.warning("Empty observations list")
            return False
        
        logger.debug(f"Update called: {len(observations)} observations, panel_angle={np.rad2deg(panel_angle):.1f}°")
        
        if len(observations) == 1:
            # 单观测更新
            r_type = r_types[0] if r_types else 'r1'
            layer = armor_layers[0] if armor_layers else None
            result = self._update_single(
                observations[0], r_type, layer,
                height_confidence, position_confidence, panel_angle
            )
            logger.debug(f"_update_single returned: {result}")
            return result
        else:
            # 双观测更新
            return self._update_dual(
                observations[0], observations[1],
                r_types[0] if r_types else 'r1',
                r_types[1] if r_types and len(r_types) > 1 else 'r2',
                armor_layers[0] if armor_layers else None,
                armor_layers[1] if armor_layers and len(armor_layers) > 1 else None,
                height_confidence
            )
    
    def _update_single(
        self,
        obs: ObservationData,
        r_type: str,
        armor_layer: Optional[str],
        height_confidence: float,
        position_confidence: float,
        panel_angle: float = 0.0  # 添加panel_angle参数
    ) -> bool:
        """
        单观测更新
        
        关键策略:
        - 强制冻结r1, r2, dza参数（单观测无法可靠估计）
        - 低置信度时降低位置更新权重
        
        Args:
            panel_angle: Panel相对中心的角度（0, π/2, π, 3π/2）
        """
        # 模态选择
        # 注意：obs.yaw是装甲板yaw，需要减去panel_angle得到center_yaw
        obs_yaw = obs.yaw
        center_yaw_obs = normalize_angle(obs_yaw - panel_angle)
        current_delta = self._x[StateIndex.DELTA]
        best_k = select_best_k(current_delta, center_yaw_obs, self.k)
        
        if self.last_k is not None and best_k != self.last_k:
            self.mode_switches += 1
        self.k = best_k
        self.last_k = self.k
        
        # 观测向量（位置使用装甲板位置，yaw使用center_yaw）
        # 从armor_yaw转换到center_yaw
        center_yaw_obs = normalize_angle(obs.yaw - panel_angle)
        z_obs = np.array([obs.x, obs.y, obs.z, center_yaw_obs])
        
        # 自动推断层级
        if armor_layer is None:
            armor_layer = self._infer_armor_layer(obs.z, r_type)
        
        # 观测噪声矩阵
        R = np.diag([
            self.config.ukf.obs_noise_pos**2,
            self.config.ukf.obs_noise_pos**2,
            self.config.ukf.obs_noise_pos**2,
            self.config.ukf.obs_noise_yaw**2,
        ])
        
        # 低置信度时放大噪声
        if height_confidence < 0.3:
            R[2, 2] *= 100.0
        if position_confidence < 0.8:
            noise_scale = 100.0 / max(position_confidence, 0.1)
            R[0, 0] *= noise_scale
            R[1, 1] *= noise_scale
        
        # 生成Sigma点
        sigma_points = self.generate_sigma_points(self._x, self._P)
        Wm, Wc = self.get_sigma_weights()
        
        # 预测观测（传递panel_angle）
        z_pred_points = np.array([
            self._observation_model(sp, r_type, armor_layer, panel_angle)
            for sp in sigma_points
        ])
        
        z_pred = np.sum(Wm[:, np.newaxis] * z_pred_points, axis=0)
        
        # 观测残差（yaw维度使用角度差）
        innov = z_obs - z_pred
        innov[3] = normalize_angle(innov[3])
        
        # Innovation gating
        if self.config.ukf.enable_innovation_gating:
            if not self._check_innovation_gate(innov, z_pred_points, z_pred, R, Wc):
                return False
        
        # 计算协方差
        diff_z = z_pred_points - z_pred
        for i in range(len(diff_z)):
            diff_z[i, 3] = normalize_angle(diff_z[i, 3])
        
        Pzz = np.sum(
            Wc[:, np.newaxis, np.newaxis] * diff_z[:, :, np.newaxis] * diff_z[:, np.newaxis, :],
            axis=0
        ) + R
        
        diff_x = sigma_points - self._x
        Pxz = np.sum(
            Wc[:, np.newaxis, np.newaxis] * diff_x[:, :, np.newaxis] * diff_z[:, np.newaxis, :],
            axis=0
        )
        
        # Kalman增益
        K = self.compute_kalman_gain(Pxz, Pzz)
        if K is None:
            return False
        
        # 关键：强制冻结参数
        K[StateIndex.R1, :] = 0.0
        K[StateIndex.R2, :] = 0.0
        K[StateIndex.DZA, :] = 0.0
        
        # 低置信度时降低位置更新权重（仅位置，不影响Yaw）
        # R矩阵放大已在447-451行完成，这里额外降低K矩阵作为双重保护
        if position_confidence < 0.9:
            weight = self.config.ukf.single_obs_update_weight_pos
            for idx in [StateIndex.X, StateIndex.Y, StateIndex.VX, 
                       StateIndex.VY, StateIndex.Z, StateIndex.VZ]:
                K[idx, :] *= weight
        
        # 应用更新
        self.apply_kalman_update(K, innov, Pzz)
        
        # 处理模态切换
        self._handle_mode_switch()
        
        # 应用约束
        self._apply_constraints()
        
        return True
    
    def _update_dual(
        self,
        obs1: ObservationData,
        obs2: ObservationData,
        r_type_1: str,
        r_type_2: str,
        layer_1: Optional[str],
        layer_2: Optional[str],
        height_confidence: float
    ) -> bool:
        """
        双观测更新（解析几何方法）
        
        利用两个装甲板的位置和yaw，通过射线交点计算机器人中心
        """
        # 自动推断层级（基于z坐标差）
        z_diff = abs(obs1.z - obs2.z)
        if z_diff > 0.01:  # 1cm阈值
            if obs1.z > obs2.z:
                layer_1, layer_2 = 'upper', 'lower'
            else:
                layer_1, layer_2 = 'lower', 'upper'
        
        # 计算机器人中心（射线交点）
        cos_yaw1, sin_yaw1 = np.cos(obs1.yaw), np.sin(obs1.yaw)
        cos_yaw2, sin_yaw2 = np.cos(obs2.yaw), np.sin(obs2.yaw)
        
        A = np.array([
            [cos_yaw1, -cos_yaw2],
            [sin_yaw1, -sin_yaw2]
        ])
        b = np.array([obs2.x - obs1.x, obs2.y - obs1.y])
        
        try:
            t_params = np.linalg.solve(A, b)
            t1 = t_params[0]
        except np.linalg.LinAlgError:
            logger.warning("Dual observation: parallel rays, skipping")
            return False
        
        if t1 < 0 or t_params[1] < 0:
            logger.warning(f"Dual observation: invalid intersection (t1={t1:.3f})")
            return False
        
        x_center = obs1.x + t1 * cos_yaw1
        y_center = obs1.y + t1 * sin_yaw1
        
        # 计算半径
        r_to_1 = np.sqrt((obs1.x - x_center)**2 + (obs1.y - y_center)**2)
        r_to_2 = np.sqrt((obs2.x - x_center)**2 + (obs2.y - y_center)**2)
        
        # 根据r_type分配半径
        if r_type_1 == 'r1':
            r1_est = r_to_1
            r2_est = r_to_2 if r_type_2 == 'r2' else self._x[StateIndex.R2]
        else:
            r2_est = r_to_1
            r1_est = r_to_2 if r_type_2 == 'r1' else self._x[StateIndex.R1]
        
        # 计算高度参数
        is_different_layers = (layer_1 != layer_2)
        if is_different_layers:
            dza_est = abs(obs1.z - obs2.z) / 2.0
            z_est = (obs1.z + obs2.z) / 2.0
            dza_noise_factor = 1.5
            z_noise_factor = 1.0
        else:
            dza_est = self._x[StateIndex.DZA]
            z_mean_obs = (obs1.z + obs2.z) / 2.0
            if layer_1 == 'upper':
                z_est = z_mean_obs - dza_est
            else:
                z_est = z_mean_obs + dza_est
            dza_noise_factor = 100.0
            z_noise_factor = 5.0
        
        # 低置信度时进一步放大dza噪声
        if height_confidence < 0.3:
            dza_noise_factor *= 100.0
        
        # 几何观测向量（6维：不包含Yaw）
        z_geometry = np.array([x_center, y_center, z_est, r1_est, r2_est, dza_est])
        
        # 几何观测噪声（6维）
        pos_noise = self.config.ukf.dual_obs_noise_pos
        R_geometry = np.diag([
            pos_noise**2,
            pos_noise**2,
            (pos_noise * z_noise_factor)**2,
            pos_noise**2,
            pos_noise**2,
            (pos_noise * dza_noise_factor)**2,
        ])
        
        # UKF更新
        sigma_points = self.generate_sigma_points(self._x, self._P)
        Wm, Wc = self.get_sigma_weights()
        
        z_pred_points = np.array([
            self._observation_model_geometry(sp) for sp in sigma_points
        ])
        z_pred = np.sum(Wm[:, np.newaxis] * z_pred_points, axis=0)
        
        innov = z_geometry - z_pred
        
        diff_z = z_pred_points - z_pred
        Pzz = np.sum(
            Wc[:, np.newaxis, np.newaxis] * diff_z[:, :, np.newaxis] * diff_z[:, np.newaxis, :],
            axis=0
        ) + R_geometry
        
        diff_x = sigma_points - self._x
        Pxz = np.sum(
            Wc[:, np.newaxis, np.newaxis] * diff_x[:, :, np.newaxis] * diff_z[:, np.newaxis, :],
            axis=0
        )
        
        K = self.compute_kalman_gain(Pxz, Pzz)
        if K is None:
            return False
        
        # 低置信度时冻结dza
        if height_confidence < 0.3:
            K[StateIndex.DZA, :] = 0.0
        
        self.apply_kalman_update(K, innov, Pzz)
        self._apply_constraints()
        
        logger.info(
            f"Dual observation update: center=({self._x[StateIndex.X]:.3f}, "
            f"{self._x[StateIndex.Y]:.3f}), r1={self._x[StateIndex.R1]:.3f}, "
            f"r2={self._x[StateIndex.R2]:.3f}"
        )
        
        return True
    
    # ==================== 辅助方法 ====================
    
    def _infer_armor_layer(self, z_obs: float, r_type: str) -> str:
        """基于最大熵原则推断装甲板层级"""
        if not self.is_dza_converged():
            return 'lower'  # 默认
        
        z_mean = self._x[StateIndex.Z]
        dza = self._x[StateIndex.DZA]
        
        z_upper = z_mean + dza
        z_lower = z_mean - dza
        
        if abs(z_obs - z_upper) < abs(z_obs - z_lower):
            return 'upper'
        else:
            return 'lower'
    
    def is_dza_converged(
        self,
        variance_threshold: float = 0.01,
        min_value_threshold: float = 0.005
    ) -> bool:
        """判断dza是否已收敛"""
        if not self._initialized:
            return False
        
        dza_variance = self._P[StateIndex.DZA, StateIndex.DZA]
        dza_value = abs(self._x[StateIndex.DZA])
        
        return (dza_variance < variance_threshold) and (dza_value > min_value_threshold)
    
    def _handle_mode_switch(self):
        """处理delta越界导致的模态切换"""
        delta = self._x[StateIndex.DELTA]
        
        if delta > np.pi / 2:
            self._x[StateIndex.DELTA] = delta - np.pi
            self.k = 1 - self.k
            if self.last_k is not None and self.k != self.last_k:
                self.mode_switches += 1
            self.last_k = self.k
        elif delta < -np.pi / 2:
            self._x[StateIndex.DELTA] = delta + np.pi
            self.k = 1 - self.k
            if self.last_k is not None and self.k != self.last_k:
                self.mode_switches += 1
            self.last_k = self.k
    
    def _apply_constraints(self):
        """应用状态约束"""
        self._x = apply_state_constraints(
            self._x,
            r1_idx=StateIndex.R1,
            r2_idx=StateIndex.R2,
            dza_idx=StateIndex.DZA,
            min_radius=self.config.constraints.min_radius,
            max_radius=self.config.constraints.max_radius,
            min_dz=0.0,  # dza必须非负
            max_dz=self.config.constraints.max_dz
        )
    
    def _check_innovation_gate(
        self,
        innov: np.ndarray,
        z_pred_points: np.ndarray,
        z_pred: np.ndarray,
        R: np.ndarray,
        Wc: np.ndarray
    ) -> bool:
        """
        Chi-square检验
        
        策略：分别检查位置和yaw，只要有一个通过就接受更新
        这样可以避免位置误差大时拒绝yaw的正确更新
        """
        diff_z = z_pred_points - z_pred
        # 关键：对yaw维度进行角度归一化处理
        for i in range(len(diff_z)):
            diff_z[i, 3] = normalize_angle(diff_z[i, 3])
        
        Pzz = np.sum(
            Wc[:, np.newaxis, np.newaxis] * diff_z[:, :, np.newaxis] * diff_z[:, np.newaxis, :],
            axis=0
        ) + R
        
        try:
            # 策略：分别检查yaw和位置的chi2
            # 只要有一个通过就接受更新，避免相互干扰
            
            # yaw维度的chi2（1维）
            innov_yaw = innov[3]
            Pzz_yaw = Pzz[3, 3]
            chi2_yaw = (innov_yaw ** 2) / Pzz_yaw
            
            # 位置维度的chi2（3维）
            innov_pos = innov[:3]
            Pzz_pos = Pzz[:3, :3]
            Pzz_pos_inv = np.linalg.inv(Pzz_pos)
            chi2_pos = innov_pos.T @ Pzz_pos_inv @ innov_pos
            
            threshold = self.config.ukf.innovation_gate_chi2_threshold
            
            # yaw通过：1维，使用原始阈值
            yaw_pass = chi2_yaw <= threshold
            
            # 位置通过：3维，使用3倍阈值（因为自由度更高）
            pos_pass = chi2_pos <= threshold * 3.0
            
            if yaw_pass or pos_pass:
                return True
            
            # 都不通过才拒绝
            logger.warning(f"Innovation gating: rejected (chi2_yaw={chi2_yaw:.2f}, chi2_pos={chi2_pos:.2f})")
            return False
            
        except np.linalg.LinAlgError:
            # 矩阵求逆失败，接受更新
            pass
        
        return True
    
    # ==================== 状态获取接口 ====================
    
    def get_yaw(self) -> float:
        """获取完整yaw角"""
        return compose_yaw(self.k, self._x[StateIndex.DELTA])
    
    def get_delta(self) -> float:
        """获取delta角"""
        return self._x[StateIndex.DELTA]
    
    def get_k(self) -> int:
        """获取离散模态k"""
        return self.k
    
    def get_center_position(self) -> np.ndarray:
        """获取中心位置 [x, y, z]"""
        return np.array([
            self._x[StateIndex.X],
            self._x[StateIndex.Y],
            self._x[StateIndex.Z]
        ])
    
    def get_radii(self) -> Tuple[float, float]:
        """获取半径 (r1, r2)"""
        return self._x[StateIndex.R1], self._x[StateIndex.R2]
    
    def get_dza(self) -> float:
        """获取装甲板高度差"""
        return self._x[StateIndex.DZA]
    
    def get_state_dict(self) -> Dict[str, Any]:
        """获取状态字典"""
        return {
            'x': self._x[StateIndex.X],
            'y': self._x[StateIndex.Y],
            'z': self._x[StateIndex.Z],
            'vx': self._x[StateIndex.VX],
            'vy': self._x[StateIndex.VY],
            'vz': self._x[StateIndex.VZ],
            'yaw': self.get_yaw(),
            'delta': self._x[StateIndex.DELTA],
            'delta_rate': self._x[StateIndex.DELTA_RATE],
            'k': self.k,
            'r1': self._x[StateIndex.R1],
            'r2': self._x[StateIndex.R2],
            'dza': self._x[StateIndex.DZA],
            'mode_switches': self.mode_switches,
        }
