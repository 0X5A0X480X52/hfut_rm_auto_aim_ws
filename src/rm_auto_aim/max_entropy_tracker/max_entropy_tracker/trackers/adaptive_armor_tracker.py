"""
自适应装甲板跟踪器

组合使用DualRadiusSpinUKF、PanelAssociator、HeightIdentifier等模块
实现完整的装甲板跟踪功能
"""

import numpy as np
from typing import List, Optional, Dict, Any, Tuple
import logging

from .base_tracker import BaseTracker, TrackerState
from ..filters.dual_radius_spin_ukf import DualRadiusSpinUKF, StateIndex
from ..core.config import UnifiedConfig
from ..core.observation import ObservationData
from ..association.panel_associator import PanelAssociator
from ..association.height_identifier import HeightIdentifier, HeightLabel
from ..association.oscillation_detector import OscillationDetector

logger = logging.getLogger(__name__)


class AdaptiveArmorTracker(BaseTracker):
    """
    自适应装甲板跟踪器
    
    核心功能:
    1. 装甲板yaw到中心yaw的转换（通过PanelAssociator）
    2. 自动高度层级识别（通过HeightIdentifier）
    3. 支持单观测和双观测更新
    4. 震荡检测和参数重启（通过OscillationDetector）
    
    装甲板布局（4面固定结构）:
    - Panel 0: offset=0°,   r1, lower
    - Panel 1: offset=90°,  r2, upper
    - Panel 2: offset=180°, r1, lower
    - Panel 3: offset=270°, r2, upper
    """
    
    def __init__(
        self,
        config: UnifiedConfig,
        dt: float | None = None,
        enable_oscillation_detection: bool = False
    ):
        """
        初始化跟踪器
        
        Args:
            config: 统一配置
            dt: 时间步长。如果为None，将使用 config.dt
            enable_oscillation_detection: 是否启用震荡检测
        """
        # 使用配置中的基础 dt 作为默认
        eff_dt = dt if dt is not None else getattr(config, 'dt', 0.05)
        super().__init__(eff_dt)
        
        self.config = config
        
        # 创建UKF滤波器
        self.ukf = DualRadiusSpinUKF(config=config, dt=eff_dt)
        
        # 创建数据关联模块
        self.panel_associator = PanelAssociator()
        self.height_identifier = HeightIdentifier()
        self.oscillation_detector = OscillationDetector(enabled=enable_oscillation_detection)
        
        # 跟踪状态
        self._current_panel_id: int = 0
        self._reference_center_yaw: Optional[float] = None
        
        # 高度识别状态
        self._height_label = HeightLabel.UNKNOWN
        self._height_confidence: float = 0.0
        
        # 调试记录
        self._debug_panel_history: List[Tuple] = []
        self._debug_height_history: List[Tuple] = []
        self._debug_dual_obs_history: List[Tuple] = []
        
        logger.info("AdaptiveArmorTracker initialized")
    
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
        初始化跟踪器
        
        Args:
            observations: 初始观测列表（至少1个）
            r1: 初始半径1
            r2: 初始半径2
            dza: 初始装甲板高度差
        """
        if len(observations) == 0:
            raise ValueError("At least one observation required")
        
        obs = observations[0]
        
        # 首帧Panel关联
        panel_id, center_yaw, _ = self.panel_associator.associate_panel(
            armor_yaw=obs.yaw,
            center_yaw_pred=None
        )
        
        self._current_panel_id = panel_id
        self._reference_center_yaw = center_yaw
        
        # 从装甲板位置反推中心位置
        # armor_pos = center + r * unit(armor_yaw)  (armor_yaw为径向方向：中心指向装甲板)
        # => center = armor - r * unit(armor_yaw)
        r = r1 if panel_id % 2 == 0 else r2
        center_x = obs.x - r * np.cos(obs.yaw)
        center_y = obs.y - r * np.sin(obs.yaw)
        
        # 初始化UKF：传入原始装甲板观测以及panel_id，由UKF在内部反推中心
        self.ukf.initialize([obs], r1=r1, r2=r2, dza=dza, panel_id=panel_id)
        
        # 初始化时间（如果观测带有时间戳）
        if obs.timestamp is not None:
            self._current_time = obs.timestamp
            self._last_update_time = obs.timestamp
            logger.info(f"Time initialized to {obs.timestamp:.3f}s")
        
        self._transition_to(TrackerState.TRACKING)  # 修复：初始化后应该是TRACKING状态
        self._increment_frame()
        
        logger.info(f"Tracker initialized at panel {panel_id}, center_yaw={np.degrees(center_yaw):.1f}°")
    
    # ==================== 预测步骤 ====================
    
    def predict(self, target_time: Optional[float] = None) -> None:
        """
        预测步骤
        
        Args:
            target_time: 目标时间戳。如果为None，使用默认dt
        """
        if not self.is_initialized:
            return
        
        # 计算实际dt
        dt = self._compute_dt(target_time)
        
        # 记录dt（诊断用）
        self._dt_history.append(dt)
        
        # 使用实际dt预测
        self.ukf.predict(dt)
        
        # 更新内部时间
        if target_time is not None:
            self._current_time = target_time
        elif self._current_time is not None:
            self._current_time += dt
        # 如果两者都为None，则不更新时间（使用fallback dt）
        
        # 震荡检测
        r1, r2 = self.ukf.get_radii()
        if self.oscillation_detector.update(r1, r2):
            self._reset_parameters()
        
        # 更新参考yaw
        self._reference_center_yaw = self.ukf.get_yaw()
    
    # ==================== 更新步骤 ====================
    
    def update(
        self,
        observations: List[ObservationData],
        **kwargs
    ) -> bool:
        """
        更新步骤
        
        根据观测数量自动选择单观测或双观测更新
        自动使用观测的timestamp计算dt并先进行predict
        
        Args:
            observations: 观测列表
            
        Returns:
            是否更新成功
        """
        if not self.is_initialized or len(observations) == 0:
            self._handle_observation_loss(
                self.config.tracker.tracking_thres,
                self.config.tracker.lost_thres
            )
            return False
        
        # === 新增：基于观测时间戳自动predict ===
        # 使用所有观测的最大时间戳作为目标预测时间（以处理双观测时间略有不同的情况）
        timestamps = [o.timestamp for o in observations if o.timestamp is not None]
        obs_time = max(timestamps) if timestamps else None
        if len(timestamps) > 1 and max(timestamps) - min(timestamps) > 0.01:
            # 如果同帧观测的时间差超过10ms，记录警告
            logger.debug(f"Observation timestamps differ: min={min(timestamps):.3f}, max={max(timestamps):.3f}")
        if obs_time is not None and self._current_time is not None:
            # 计算到观测时间的dt
            dt = obs_time - self._current_time
            # 允许微小负偏差以容忍时间同步误差
            if dt > self._min_dt:
                # 先预测到观测时间点
                logger.debug(f"Auto-predict to observation time: dt={dt:.4f}s")
                self.predict(obs_time)
            elif dt < -1e-3:
                # 仅在显著倒退时报警
                logger.warning(f"Observation time {obs_time:.3f}s < current time {self._current_time:.3f}s")
        
        self._handle_observation_received(self.config.tracker.tracking_thres)
        
        if len(observations) == 1:
            success = self._update_single(observations[0])
        else:
            success = self._update_dual(observations[0], observations[1])
        
        # 更新时间（使用选定的最大时间戳）
        if obs_time is not None:
            self._update_time(obs_time)
        
        self._increment_frame()
        
        # 更新参考yaw
        if success:
            self._reference_center_yaw = self.ukf.get_yaw()
        
        return success
    
    def _update_single(
        self, 
        obs: ObservationData,
        override_position_confidence: Optional[float] = None
    ) -> bool:
        """
        单观测更新
        
        说明：obs.yaw为装甲板yaw（径向方向：中心指向装甲板）
        
        流程:
        1. Panel关联（armor_yaw→panel_id→center_yaw）
        2. 高度层级识别
        3. 计算置信度
        4. 调用UKF更新
        
        Args:
            obs: 观测数据
            override_position_confidence: 如果提供，将覆盖自动计算的position_confidence
        """
        # Panel关联（根据armor_yaw确定panel_id，并转换为center_yaw）
        panel_id, center_yaw, matching_error = self.panel_associator.associate_panel(
            armor_yaw=obs.yaw,  # 输入：装甲板yaw（径向方向）
            center_yaw_pred=self._reference_center_yaw,
            z_obs=obs.z,
            center_z=self.ukf.x[StateIndex.Z]
        )
        
        self._current_panel_id = panel_id
        r_type = self.panel_associator.get_r_type(panel_id)
        
        # 高度层级识别
        height_label, height_confidence = self.height_identifier.identify_single(
            z_obs=obs.z,
            panel_id=panel_id,
            z_mean=self.ukf.x[StateIndex.Z],
            dza=self.ukf.x[StateIndex.DZA],
            dza_converged=self.ukf.is_dza_converged()
        )
        
        self._height_label = height_label
        self._height_confidence = height_confidence
        
        # 转换层级标签
        if height_label == HeightLabel.UPPER:
            armor_layer = 'upper'
        elif height_label == HeightLabel.LOWER:
            armor_layer = 'lower'
        else:
            armor_layer = None
        
        # 计算位置置信度（可以被override覆盖）
        if override_position_confidence is not None:
            position_confidence = override_position_confidence
        else:
            position_confidence = self._compute_position_confidence(
                armor_layer, height_confidence, r_type
            )
        
        # 注意：传递原始装甲板yaw给UKF，而不是center_yaw
        # UKF内部会进行模态选择和delta转换
        # 计算panel_angle供观测模型使用
        panel_angle = panel_id * (np.pi / 2)  # Panel 0,1,2,3 → 0, π/2, π, 3π/2
        
        # 调用UKF更新
        success = self.ukf.update(
            observations=[obs],  # 传递原始观测，包含装甲板yaw
            r_types=[r_type],
            armor_layers=[armor_layer],
            height_confidence=height_confidence,
            position_confidence=position_confidence,
            panel_angle=panel_angle  # 传递panel_angle
        )
        
        # 调试记录
        self._debug_panel_history.append((
            self._frame_count, obs.yaw, panel_id, r_type,
            self._reference_center_yaw, matching_error
        ))
        self._debug_height_history.append((
            self._frame_count, panel_id, armor_layer, height_confidence, obs.z
        ))
        
        return success
    
    def _update_dual(
        self,
        obs1: ObservationData,
        obs2: ObservationData
    ) -> bool:
        """
        双观测更新
        
        策略（与demo1一致）：
        1. 先用第一个观测进行单观测更新（包含Yaw更新，使用高position_confidence）
        2. 再用双观测进行几何更新（仅位置和参数，不更新Yaw）
        
        关键：第一次单观测使用高position_confidence (0.9)，因为：
        - 双观测场景下位置信息更可靠
        - 后续双观测更新会进一步修正位置
        - 不应因参数未收敛而过度抑制位置更新
        """
        # 先用第一个观测做单观测更新（会更新Yaw和位置）
        # 在双观测场景中，使用高position_confidence=1.0，完全信任位置观测
        # 因为：1) 双观测提供了强几何约束  2) 后续双观测会进一步修正
        success_single = self._update_single(obs1, override_position_confidence=1.0)
        
        if not success_single:
            return False
        
        # 再进行双观测几何更新
        # Panel关联
        panel_id_1, _, _ = self.panel_associator.associate_panel(
            armor_yaw=obs1.yaw,
            center_yaw_pred=self._reference_center_yaw,
            z_obs=obs1.z,
            center_z=self.ukf.x[StateIndex.Z]
        )
        
        panel_id_2, _, _ = self.panel_associator.associate_panel(
            armor_yaw=obs2.yaw,
            center_yaw_pred=self._reference_center_yaw,
            z_obs=obs2.z,
            center_z=self.ukf.x[StateIndex.Z]
        )
        
        r_type_1 = self.panel_associator.get_r_type(panel_id_1)
        r_type_2 = self.panel_associator.get_r_type(panel_id_2)
        
        # 双观测高度识别
        layer_1, layer_2, height_confidence = self.height_identifier.identify_dual(
            z1=obs1.z,
            z2=obs2.z
        )
        
        self._height_confidence = height_confidence
        
        # 数据问题提示：如果面板不是相邻面（panel id不相邻），仅记录warning并继续双观测以便诊断
        diff = abs(panel_id_1 - panel_id_2)
        if diff not in (1, 3):  # 非相邻（例如0-2或1-3）
            logger.warning(
                f"Dual observation: non-adjacent panels detected (panel1={panel_id_1}, panel2={panel_id_2}). "
                "This likely indicates a data-generation or association issue."
            )
            self._debug_dual_obs_history.append((
                self._frame_count, obs1.z, obs2.z, layer_1, layer_2, height_confidence, 'non-adjacent-warning'
            ))
        
        # 不再回退为单观测更新；如果是对角板将由UKF或下游诊断捕获错误信息
        # 这里仅记录额外信息以便之后分析
        # (已在上方根据panel id记录non-adjacent-warning)
        
        # 调用UKF双观测更新（仅更新位置和参数，不更新Yaw）
        success = self.ukf.update(
            observations=[obs1, obs2],
            r_types=[r_type_1, r_type_2],
            armor_layers=[layer_1, layer_2],
            height_confidence=height_confidence
        )
        
        # 调试记录
        self._debug_dual_obs_history.append((
            self._frame_count, obs1.z, obs2.z, layer_1, layer_2, height_confidence
        ))
        
        return success
    
    def _compute_position_confidence(
        self,
        armor_layer: Optional[str],
        height_confidence: float,
        r_type: str
    ) -> float:
        """
        计算位置更新置信度
        
        考虑因素:
        1. 层级识别是否成功
        2. 对应半径的不确定性
        """
        # 层级未知 → 极低置信度
        if armor_layer is None:
            return 0.1
        
        # 获取半径不确定性
        r_idx = StateIndex.R1 if r_type == 'r1' else StateIndex.R2
        r_variance = self.ukf.P[r_idx, r_idx]
        r_std = np.sqrt(r_variance)
        
        # 参数不确定性映射
        if r_std < 0.01:
            param_confidence = 1.0
        elif r_std < 0.05:
            param_confidence = 1.0 - (r_std - 0.01) * (0.4 / 0.04)
        elif r_std < 0.1:
            param_confidence = 0.6 - (r_std - 0.05) * (0.3 / 0.05)
        else:
            param_confidence = 0.3
        
        # 综合置信度
        position_confidence = np.sqrt(height_confidence * param_confidence)
        
        return position_confidence
    
    def _reset_parameters(self):
        """重启参数估计"""
        r1_reset, r2_reset = self.oscillation_detector.get_reset_values()
        
        self.ukf.x[StateIndex.R1] = r1_reset
        self.ukf.x[StateIndex.R2] = r2_reset
        
        logger.info(f"Parameters reset: r1={r1_reset:.3f}, r2={r2_reset:.3f}")
    
    # ==================== 状态获取接口 ====================
    
    def get_center_position(self) -> np.ndarray:
        """获取中心位置 [x, y, z]"""
        return self.ukf.get_center_position()
    
    def get_yaw(self) -> float:
        """获取yaw角"""
        return self.ukf.get_yaw()
    
    def get_radii(self) -> Tuple[float, float]:
        """获取半径估计 (r1, r2)"""
        return self.ukf.get_radii()
    
    def get_dza(self) -> float:
        """获取装甲板高度差"""
        return self.ukf.get_dza()
    
    def get_k(self) -> int:
        """获取离散模态k"""
        return self.ukf.get_k()
    
    def get_delta(self) -> float:
        """获取连续delta角"""
        return self.ukf.get_delta()
    
    def get_panel_id(self) -> int:
        """获取当前panel索引"""
        return self._current_panel_id
    
    def get_height_label(self) -> HeightLabel:
        """获取当前高度标签"""
        return self._height_label
    
    def get_height_confidence(self) -> float:
        """获取高度识别置信度"""
        return self._height_confidence
    
    def get_state(self) -> Dict[str, Any]:
        """获取完整状态"""
        ukf_state = self.ukf.get_state_dict()
        
        return {
            **ukf_state,
            'tracker_state': self._state.name,
            'frame_count': self._frame_count,
            'current_panel_id': self._current_panel_id,
            'height_label': self._height_label.name,
            'height_confidence': self._height_confidence,
            'reference_center_yaw': self._reference_center_yaw,
        }
    
    # ==================== 调试接口 ====================
    
    def get_panel_statistics(self) -> Dict:
        """获取Panel分配统计"""
        if not self._debug_panel_history:
            return {}
        
        panel_counts = {0: 0, 1: 0, 2: 0, 3: 0}
        errors = []
        
        for _, _, panel_id, _, _, error in self._debug_panel_history:
            panel_counts[panel_id] += 1
            if error > 0:
                errors.append(error)
        
        return {
            'total': len(self._debug_panel_history),
            'panel_distribution': panel_counts,
            'avg_matching_error': np.mean(errors) if errors else 0,
            'max_matching_error': np.max(errors) if errors else 0,
            'z_corrections': self.panel_associator.z_correction_count,
        }
    
    def get_height_statistics(self) -> Dict:
        """获取高度识别统计"""
        if not self._debug_height_history:
            return {}
        
        total = len(self._debug_height_history)
        unknown_count = sum(1 for _, _, layer, _, _ in self._debug_height_history if layer is None)
        
        confidences = [conf for _, _, _, conf, _ in self._debug_height_history]
        
        return {
            'total': total,
            'unknown_count': unknown_count,
            'unknown_rate': unknown_count / total if total > 0 else 0,
            'avg_confidence': np.mean(confidences) if confidences else 0,
            'dual_obs_count': len(self._debug_dual_obs_history),
        }
    
    def clear_debug_history(self):
        """清空调试记录"""
        self._debug_panel_history.clear()
        self._debug_height_history.clear()
        self._debug_dual_obs_history.clear()
