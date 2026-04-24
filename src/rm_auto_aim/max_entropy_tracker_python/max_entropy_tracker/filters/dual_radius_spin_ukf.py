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
from .process_models import (
    CompositeProcessModel,
    TranslationModelType,
    RotationModelType,
    TranslationConfig,
    RotationConfig,
    StructuralConfig,
    CVTranslation,
    CATranslation,
    SingerTranslation,
    CVRotation,
    CARotation,
    StructuralModel,
    create_default_process_model,
    StateLayout
)
from ..core.config import UnifiedConfig, TranslationModel
from ..core.observation import ObservationData
from ..utils.angle_utils import (
    normalize_angle, decompose_yaw, compose_yaw, 
    select_best_k, select_best_k_from_center_yaw, delta_angle_diff
)
from ..utils.constraints import apply_state_constraints

logger = logging.getLogger(__name__)


class StateIndex(IntEnum):
    """
    状态索引枚举 (用于向后兼容)
    
    11维状态向量布局 (CV平移模型)
    注意: 实际索引由 process_model.layout 动态决定
    此枚举仅作为默认CV模型的参考
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


class DynamicStateIndex:
    """
    动态状态索引访问器
    
    基于 StateLayout 提供与 StateIndex IntEnum 相同的访问接口
    支持不同过程模型的动态状态布局
    """
    
    def __init__(self, layout: StateLayout):
        self._layout = layout
    
    def __getattr__(self, name: str) -> int:
        if name.startswith('_'):
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
        return self._layout.get(name)
    
    def __repr__(self) -> str:
        return f"DynamicStateIndex({self._layout})"


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
    
    def __init__(
        self, 
        config: UnifiedConfig, 
        dt: float | None = None,
        process_model: Optional[CompositeProcessModel] = None
    ):
        """
        初始化双半径自旋UKF
        
        Args:
            config: 统一配置对象
            dt: 默认时间步长。如果为None，将使用 `config.dt`。
            process_model: 可选的自定义过程模型。如果为None，则根据config自动创建
        """
        # 决定使用的 dt
        eff_dt = dt if dt is not None else getattr(config, 'dt', 0.05)
        # 先创建过程模型（state_dim依赖它）
        self._motion_model = process_model or self._create_process_model(config, eff_dt)
        
        # 动态状态索引访问器（兼容原有代码）
        self._state_idx = DynamicStateIndex(self._motion_model.layout)
        
        super().__init__(config, eff_dt)
        
        # 离散模态 k
        self.k: int = 0  # k ∈ {0, 1}
        self.last_k: Optional[int] = None
        self.mode_switches: int = 0
        
        # 初始化状态和协方差（使用过程模型的维度）
        self._x = np.zeros(self.state_dim)
        self._P = np.eye(self.state_dim) * 100.0
        
        # 构建过程噪声（委托给过程模型）
        self._Q = self._build_process_noise()
        
        # 初始化Sigma点生成器
        self._init_sigma_generator()
        
        # 诊断数据记录器（用于分析单观测问题）
        self.diagnostics = {
            'enabled': False,
            'predict_displacement': [],
            'update_displacement': [],
            'innovation': [],
            'kalman_gain_pos': [],
            'position_confidence': [],
            'dt_actual': [],
            'state_before_predict': [],
            'state_after_predict': [],
            'state_after_update': []
        }
        
        logger.info(
            f"DualRadiusSpinUKF initialized with {type(self._process_model).__name__}, "
            f"state_dim={self.state_dim}"
        )
    
    def _create_process_model(self, config: UnifiedConfig, dt: float) -> CompositeProcessModel:
        """
        根据配置创建过程模型
        
        Args:
            config: 统一配置
            dt: 时间步长
            
        Returns:
            组合过程模型
        """
        # 从config.motion中获取平移模型类型
        trans_type_map = {
            TranslationModel.CV: TranslationModelType.CV,
            TranslationModel.CA: TranslationModelType.CA,
            TranslationModel.Singer: TranslationModelType.SINGER,
        }
        trans_type = trans_type_map.get(config.motion.translation_model, TranslationModelType.CV)
        
        # 创建配置
        trans_config = TranslationConfig(
            cv_process_noise_vel=config.motion.cv_process_noise_vel,
            ca_process_noise_acc=config.motion.ca_process_noise_acc,
            singer_alpha=config.motion.singer_alpha,
            singer_sigma=config.motion.singer_sigma,
        )
        
        rot_config = RotationConfig(
            cv_process_noise_rate=config.spin.spin_process_noise_delta_rate,
            ca_process_noise_acc=config.spin.spin_process_noise_delta_acc,
        )
        
        struct_config = StructuralConfig(
            process_noise_r=config.motion.process_noise_r,
            process_noise_dz=config.motion.process_noise_dz,
        )
        
        # 创建组合模型（旋转模型默认使用CV）
        return create_default_process_model(
            translation_type=trans_type,
            rotation_type=RotationModelType.CV,
            translation_config=trans_config,
            rotation_config=rot_config,
            structural_config=struct_config,
            n_dims=3
        )
    
    # ==================== 属性实现 ====================
    
    @property
    def process_model(self) -> CompositeProcessModel:
        """获取过程模型"""
        return self._motion_model
    
    @property
    def state_idx(self) -> DynamicStateIndex:
        """获取动态状态索引访问器"""
        return self._state_idx
    
    @property
    def state_dim(self) -> int:
        """状态维度（由过程模型决定）"""
        return self._motion_model.state_dim
    
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
        
        # 接口说明：接受 armor 观测（装甲板位置与yaw，yaw为中心->装甲板的径向方向）
        # trackers 传入 panel_id 以指示使用 r1 或 r2 进行反推中心位置
        obs = observations[0]

        # 通过panel_id选择半径以反推中心位置
        panel_id = kwargs.get('panel_id', None)
        if panel_id is None:
            # 如果未提供panel_id，默认使用r1（与早期实现兼容）
            use_r = r1
            panel_angle = 0.0
        else:
            use_r = r1 if (panel_id % 2 == 0) else r2
            panel_angle = panel_id * (np.pi / 2)

        # armor_yaw定义为径向外法向（中心指向装甲板）
        # center = armor - r * unit(armor_yaw)
        center_x = obs.x - use_r * np.cos(obs.yaw)
        center_y = obs.y - use_r * np.sin(obs.yaw)
        center_z = obs.z
        # 由装甲板yaw减去panel_angle得到center_yaw
        center_yaw = normalize_angle(obs.yaw - panel_angle)
        
        # 分解yaw
        self.k, delta = decompose_yaw(center_yaw)
        self.last_k = self.k
        
        # 设置状态（使用动态索引）
        idx = self._state_idx
        self._x[idx.X] = center_x
        self._x[idx.VX] = 0.0
        self._x[idx.Y] = center_y
        self._x[idx.VY] = 0.0
        self._x[idx.Z] = center_z
        self._x[idx.VZ] = 0.0
        self._x[idx.DELTA] = delta
        self._x[idx.DELTA_RATE] = 0.0
        self._x[idx.R1] = r1
        self._x[idx.R2] = r2
        self._x[idx.DZA] = dza
        
        # 初始化协方差（使用过程模型提供的初始协方差）
        self._P = self._motion_model.get_initial_covariance()
        
        self._initialized = True
        
        logger.info(
            f"DualRadiusSpinUKF initialized at ({center_x:.2f}, {center_y:.2f}, {center_z:.2f}), "
            f"yaw={np.degrees(center_yaw):.1f}° (k={self.k}, delta={np.degrees(delta):.1f}°)"
        )
    
    # ==================== 过程模型 ====================
    
    def _build_process_noise(self) -> np.ndarray:
        """
        构建过程噪声矩阵
        
        委托给组合过程模型
        """
        return self._motion_model.build_Q(self.dt)
    
    def _process_model(self, x: np.ndarray, dt: float) -> np.ndarray:
        """
        状态转移方程 (抽象方法实现)
        
        委托给组合过程模型
        
        Args:
            x: 状态向量
            dt: 时间步长
            
        Returns:
            预测状态向量
        """
        return self._motion_model.predict(x, dt)
    
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
        观测模型: 从中心状态预测装甲板位置和yaw
        
        坐标系说明：
        - 使用相机坐标系
        - armor_yaw定义为径向方向（中心指向装甲板）
        - center_yaw是旋转中心的朝向
        - armor_yaw = center_yaw + panel_angle
        
        Args:
            x: 状态向量
            r_type: 半径类型 ('r1' 或 'r2')
            armor_layer: 装甲板层级 ('upper' 或 'lower')
            panel_angle: Panel相对中心的角度（0, π/2, π, 3π/2）
            
        Returns:
            预测观测 [x_armor, y_armor, z_armor, center_yaw]
            注意：返回center_yaw而非armor_yaw，因为更新时会将obs.yaw转换为center_yaw
        """
        # 使用动态状态索引
        idx = self._state_idx
        
        x_c = x[idx.X]
        y_c = x[idx.Y]
        z_mean = x[idx.Z]
        delta = x[idx.DELTA]
        dza = x[idx.DZA]
        
        # 获取半径
        if r_type == 'r1':
            radius = x[idx.R1]
        else:
            radius = x[idx.R2]
        
        # 完整yaw = k*π + delta
        center_yaw = compose_yaw(self.k, delta)
        
        # 装甲板yaw（径向方向：中心指向装甲板）
        armor_yaw = normalize_angle(center_yaw + panel_angle)
        
        # 装甲板位置 = 中心 + 半径 * 径向单位向量
        x_obs = x_c + radius * np.cos(armor_yaw)
        y_obs = y_c + radius * np.sin(armor_yaw)
        
        # 层级高度偏移
        if armor_layer == 'upper':
            layer_offset = dza
        else:
            layer_offset = -dza
        z_obs = z_mean + layer_offset
        
        # 返回center_yaw供更新使用（更新时会将obs.yaw转换为center_yaw）
        return np.array([x_obs, y_obs, z_obs, center_yaw])
    
    def _observation_model_geometry(self, x: np.ndarray) -> np.ndarray:
        """
        几何观测模型（用于双观测更新）
        
        Returns:
            [x_center, y_center, z, r1, r2, dza]
        """
        idx = self._state_idx
        return np.array([
            x[idx.X],
            x[idx.Y],
            x[idx.Z],
            x[idx.R1],
            x[idx.R2],
            x[idx.DZA]
        ])
    
    # ==================== 预测步骤 ====================
    
    def predict(self, dt: Optional[float] = None) -> None:
        """预测步骤"""
        if not self._initialized:
            logger.warning("UKF not initialized, skipping predict")
            return
        
        dt = dt if dt is not None else self.dt
        
        # 诊断: 记录预测前状态
        if self.diagnostics['enabled']:
            self.diagnostics['state_before_predict'].append(self._x.copy())
            self.diagnostics['dt_actual'].append(dt)
        
        # 更新过程噪声（如果dt变化了）
        self._Q = self._motion_model.build_Q(dt)
        
        # 生成Sigma点
        sigma_points = self.generate_sigma_points(self._x, self._P)
        
        # 传播Sigma点（使用过程模型）
        sigma_points_pred = np.array([
            self._motion_model.predict(sp, dt) for sp in sigma_points
        ])
        
        # 获取权重
        Wm, Wc = self.get_sigma_weights()
        
        # 计算预测均值
        self._x = np.sum(Wm[:, np.newaxis] * sigma_points_pred, axis=0)
        
        # 计算预测协方差（delta使用特殊角度差）
        idx = self._state_idx
        P_pred = np.zeros((self.state_dim, self.state_dim))
        for i in range(len(sigma_points_pred)):
            diff = sigma_points_pred[i] - self._x
            # delta差使用特殊处理
            diff[idx.DELTA] = delta_angle_diff(
                sigma_points_pred[i, idx.DELTA],
                self._x[idx.DELTA]
            )
            P_pred += Wc[i] * np.outer(diff, diff)
        
        # 添加过程噪声
        P_pred += self._Q
        
        self._P = P_pred
        
        # 应用约束
        self._apply_constraints()
        
        # 确保协方差正定
        self.ensure_covariance_valid()
        
        # 诊断: 记录预测后状态和位移
        if self.diagnostics['enabled']:
            idx = self._state_idx
            self.diagnostics['state_after_predict'].append(self._x.copy())
            pos_before = self.diagnostics['state_before_predict'][-1][[idx.X, idx.Y]]
            pos_after = self._x[[idx.X, idx.Y]]
            pred_disp = np.linalg.norm(pos_after - pos_before)
            self.diagnostics['predict_displacement'].append(pred_disp)
    
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
        idx = self._state_idx  # 动态状态索引
        
        # 模态选择
        # 注意：obs.yaw是装甲板yaw，需要减去panel_angle得到center_yaw
        obs_yaw = obs.yaw
        center_yaw_obs = normalize_angle(obs_yaw - panel_angle)
        current_delta = self._x[idx.DELTA]
        
        # 修复: 使用select_best_k_from_center_yaw，正确处理center_yaw作为观测
        best_k = select_best_k_from_center_yaw(current_delta, center_yaw_obs, self.k)
        
        if self.last_k is not None and best_k != self.last_k:
            self.mode_switches += 1
            logger.debug(f"Mode switch: k={self.last_k} → k={best_k}, delta={current_delta:.3f}, center_yaw_obs={center_yaw_obs:.3f}")
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
        
        # 诊断: 记录原始卡尔曼增益（位置部分）
        if self.diagnostics['enabled']:
            K_pos_avg = (np.linalg.norm(K[idx.X, :]) + np.linalg.norm(K[idx.Y, :])) / 2.0
            self.diagnostics['kalman_gain_pos'].append(K_pos_avg)
            self.diagnostics['innovation'].append(innov.copy())
            self.diagnostics['position_confidence'].append(position_confidence)
        
        # 关键：强制冻结参数（使用动态索引）
        K[idx.R1, :] = 0.0
        K[idx.R2, :] = 0.0
        K[idx.DZA, :] = 0.0
        
        # 注意：已移除K矩阵二次降权机制
        # 原因：R矩阵动态调整（基于position_confidence）已经足够控制观测权重
        # 二次降权会导致过度不信任单观测，造成位置飘移
        # 详见诊断报告：单观测飘移诊断报告.md
        
        # 应用更新
        self.apply_kalman_update(K, innov, Pzz)
        
        # 处理模态切换
        self._handle_mode_switch()
        
        # 应用约束
        self._apply_constraints()
        
        # 诊断: 记录更新后状态和位移
        if self.diagnostics['enabled']:
            self.diagnostics['state_after_update'].append(self._x.copy())
            pos_pred = self.diagnostics['state_after_predict'][-1][[idx.X, idx.Y]]
            pos_update = self._x[[idx.X, idx.Y]]
            update_disp = np.linalg.norm(pos_update - pos_pred)
            self.diagnostics['update_displacement'].append(update_disp)
        
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
        # 注意：obs.yaw为armor_yaw（径向方向：中心→装甲板）
        # 需要反向：装甲板→中心，即加π
        yaw1_to_center = normalize_angle(obs1.yaw + np.pi)
        yaw2_to_center = normalize_angle(obs2.yaw + np.pi)
        
        cos_yaw1, sin_yaw1 = np.cos(yaw1_to_center), np.sin(yaw1_to_center)
        cos_yaw2, sin_yaw2 = np.cos(yaw2_to_center), np.sin(yaw2_to_center)
        
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
        idx = self._state_idx
        if r_type_1 == 'r1':
            r1_est = r_to_1
            r2_est = r_to_2 if r_type_2 == 'r2' else self._x[idx.R2]
        else:
            r2_est = r_to_1
            r1_est = r_to_2 if r_type_2 == 'r1' else self._x[idx.R1]
        
        # 计算高度参数
        is_different_layers = (layer_1 != layer_2)
        if is_different_layers:
            dza_est = abs(obs1.z - obs2.z) / 2.0
            z_est = (obs1.z + obs2.z) / 2.0
            dza_noise_factor = 1.5
            z_noise_factor = 1.0
        else:
            dza_est = self._x[idx.DZA]
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
            K[idx.DZA, :] = 0.0
        
        self.apply_kalman_update(K, innov, Pzz)
        self._apply_constraints()
        
        logger.info(
            f"Dual observation update: center=({self._x[idx.X]:.3f}, "
            f"{self._x[idx.Y]:.3f}), r1={self._x[idx.R1]:.3f}, "
            f"r2={self._x[idx.R2]:.3f}"
        )
        
        return True
    
    # ==================== 辅助方法 ====================
    
    def _infer_armor_layer(self, z_obs: float, r_type: str) -> str:
        """基于最大熵原则推断装甲板层级"""
        if not self.is_dza_converged():
            return 'lower'  # 默认
        
        idx = self._state_idx
        z_mean = self._x[idx.Z]
        dza = self._x[idx.DZA]
        
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
        
        idx = self._state_idx
        dza_variance = self._P[idx.DZA, idx.DZA]
        dza_value = abs(self._x[idx.DZA])
        
        return (dza_variance < variance_threshold) and (dza_value > min_value_threshold)
    
    def _handle_mode_switch(self):
        """处理delta越界导致的模态切换"""
        idx = self._state_idx
        delta = self._x[idx.DELTA]
        
        if delta > np.pi / 2:
            self._x[idx.DELTA] = delta - np.pi
            self.k = 1 - self.k
            if self.last_k is not None and self.k != self.last_k:
                self.mode_switches += 1
            self.last_k = self.k
        elif delta < -np.pi / 2:
            self._x[idx.DELTA] = delta + np.pi
            self.k = 1 - self.k
            if self.last_k is not None and self.k != self.last_k:
                self.mode_switches += 1
            self.last_k = self.k
    
    def _apply_constraints(self):
        """应用状态约束"""
        idx = self._state_idx
        self._x = apply_state_constraints(
            self._x,
            r1_idx=idx.R1,
            r2_idx=idx.R2,
            dza_idx=idx.DZA,
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
        idx = self._state_idx
        return compose_yaw(self.k, self._x[idx.DELTA])
    
    def get_delta(self) -> float:
        """获取delta角"""
        idx = self._state_idx
        return self._x[idx.DELTA]
    
    def get_k(self) -> int:
        """获取离散模态k"""
        return self.k
    
    def get_center_position(self) -> np.ndarray:
        """获取中心位置 [x, y, z]"""
        idx = self._state_idx
        return np.array([
            self._x[idx.X],
            self._x[idx.Y],
            self._x[idx.Z]
        ])
    
    def get_radii(self) -> Tuple[float, float]:
        """获取半径 (r1, r2)"""
        idx = self._state_idx
        return self._x[idx.R1], self._x[idx.R2]
    
    def get_dza(self) -> float:
        """获取装甲板高度差"""
        idx = self._state_idx
        return self._x[idx.DZA]
    
    def get_state_dict(self) -> Dict[str, Any]:
        """获取状态字典"""
        idx = self._state_idx
        return {
            'x': self._x[idx.X],
            'y': self._x[idx.Y],
            'z': self._x[idx.Z],
            'vx': self._x[idx.VX],
            'vy': self._x[idx.VY],
            'vz': self._x[idx.VZ],
            'yaw': self.get_yaw(),
            'delta': self._x[idx.DELTA],
            'delta_rate': self._x[idx.DELTA_RATE],
            'k': self.k,
            'r1': self._x[idx.R1],
            'r2': self._x[idx.R2],
            'dza': self._x[idx.DZA],
            'mode_switches': self.mode_switches,
        }
    
    # ==================== 诊断接口 ====================
    
    def enable_diagnostics(self, enabled: bool = True):
        """启用/禁用诊断模式"""
        self.diagnostics['enabled'] = enabled
        if enabled:
            # 清空历史数据
            for key in self.diagnostics:
                if isinstance(self.diagnostics[key], list):
                    self.diagnostics[key].clear()
            logger.info("Diagnostics enabled")
        else:
            logger.info("Diagnostics disabled")
    
    def get_diagnostics(self) -> Dict[str, Any]:
        """获取诊断数据"""
        return self.diagnostics.copy()
