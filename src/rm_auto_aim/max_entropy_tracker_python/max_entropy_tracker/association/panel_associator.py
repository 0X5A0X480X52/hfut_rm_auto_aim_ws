"""
装甲板Panel关联器

将观测到的装甲板yaw角关联到具体的panel_id (0,1,2,3)

坐标系说明：
- 使用相机坐标系
- armor_yaw定义为径向方向（中心指向装甲板）
- center_yaw是旋转中心的朝向
- armor_yaw = center_yaw + panel_angle

使用组合代价函数: cost = yaw_error + w_pos * position_error
当位置信息可用时，可以避免角度差较小但位置差较大引发的错误匹配。
"""

import math
import numpy as np
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class PanelAssociator:
    """
    装甲板Panel关联器
    
    核心功能:
    1. 根据观测yaw和预测center_yaw确定panel_id
    2. 支持位置辅助的组合代价函数（yaw + 位置误差）
    3. 支持Z坐标辅助判断（当yaw匹配误差大时）
    4. 将armor_yaw转换为center_yaw
    
    装甲板布局（4面固定结构）:
    - Panel 0: offset=0°,   r1, lower
    - Panel 1: offset=90°,  r2, upper
    - Panel 2: offset=180°, r1, lower
    - Panel 3: offset=270°, r2, upper
    """
    
    N_PANELS = 4
    PANEL_ANGLE_STEP = np.pi / 2  # 90°
    # 位置权重（rad/m）: 1m位置误差 ≈ 2rad yaw误差
    DEFAULT_POS_WEIGHT = 2.0
    
    def __init__(self):
        """初始化关联器"""
        self._last_confident_panel: Optional[int] = None
        self._z_correction_count: int = 0
    
    @staticmethod
    def _predict_position_error(
        center_yaw_pred: float, panel_id: int,
        cx: float, cy: float, r1_val: float, r2_val: float,
        obs_x_val: float, obs_y_val: float
    ) -> float:
        """计算特定panel的预测装甲板位置到观测位置的欧氏距离"""
        radius = r1_val if panel_id % 2 == 0 else r2_val
        armor_yaw = center_yaw_pred + panel_id * PanelAssociator.PANEL_ANGLE_STEP
        armor_yaw = math.atan2(math.sin(armor_yaw), math.cos(armor_yaw))
        pred_x = cx + radius * math.cos(armor_yaw)
        pred_y = cy + radius * math.sin(armor_yaw)
        return math.hypot(obs_x_val - pred_x, obs_y_val - pred_y)
    
    def associate_panel(
        self,
        armor_yaw: float,
        center_yaw_pred: Optional[float],
        z_obs: Optional[float] = None,
        center_z: Optional[float] = None,
        obs_x: Optional[float] = None,
        obs_y: Optional[float] = None,
        center_x: Optional[float] = None,
        center_y: Optional[float] = None,
        r1: Optional[float] = None,
        r2: Optional[float] = None
    ) -> Tuple[int, float, float]:
        """
        将观测armor_yaw关联到panel_id
        
        Args:
            armor_yaw: 观测到的装甲板yaw角（径向方向：中心指向装甲板）
            center_yaw_pred: 预测的中心yaw（如果为None则为首帧）
            z_obs: 可选，装甲板z坐标（用于辅助判断）
            center_z: 可选，预测的中心z坐标
            obs_x: 可选，观测装甲板x位置
            obs_y: 可选，观测装甲板y位置
            center_x: 可选，预测旋转中心x位置
            center_y: 可选，预测旋转中心y位置
            r1: 可选，偶数panel预测半径
            r2: 可选，奇数panel预测半径
            
        Returns:
            (panel_id, center_yaw, matching_error)
        """
        if center_yaw_pred is None:
            # 首帧：根据armor_yaw找最接近的90°倍数
            armor_yaw_normalized = np.arctan2(np.sin(armor_yaw), np.cos(armor_yaw))
            armor_yaw_positive = (armor_yaw_normalized + 2 * np.pi) % (2 * np.pi)
            panel_id = int(np.round(armor_yaw_positive / self.PANEL_ANGLE_STEP)) % 4
            center_yaw = armor_yaw_normalized - panel_id * self.PANEL_ANGLE_STEP
            center_yaw = np.arctan2(np.sin(center_yaw), np.cos(center_yaw))
            return panel_id, center_yaw, 0.0
        
        cyp = center_yaw_pred
        has_pos = all(v is not None for v in (obs_x, obs_y, center_x, center_y, r1, r2))
        
        best_panel_id = 0
        second_best_panel_id = 0
        best_cost = float('inf')
        second_best_cost = float('inf')
        best_yaw_error = float('inf')
        
        for pid in range(4):
            expected_armor_yaw = cyp + pid * self.PANEL_ANGLE_STEP
            diff = armor_yaw - expected_armor_yaw
            diff = np.arctan2(np.sin(diff), np.cos(diff))
            yaw_err = abs(diff)
            
            cost = yaw_err  # 默认：仅yaw代价
            if has_pos:
                pos_err = self._predict_position_error(
                    cyp, pid, center_x, center_y, r1, r2, obs_x, obs_y)
                cost = yaw_err + self.DEFAULT_POS_WEIGHT * pos_err
            
            if cost < best_cost:
                second_best_cost = best_cost
                second_best_panel_id = best_panel_id
                best_cost = cost
                best_panel_id = pid
                best_yaw_error = yaw_err
            elif cost < second_best_cost:
                second_best_cost = cost
                second_best_panel_id = pid
        
        panel_id = best_panel_id
        
        # Z坐标辅助判断（当yaw匹配误差大且位置不可用时）
        if z_obs is not None and center_z is not None and best_yaw_error > np.deg2rad(30.0):
            panel_id = self._z_assisted_association(
                armor_yaw, cyp, z_obs, center_z,
                best_panel_id, best_yaw_error
            )
        else:
            # 歧义检测
            is_ambiguous = (
                best_yaw_error > np.deg2rad(20) and
                abs(best_cost - second_best_cost) < 0.1
            )
            
            if is_ambiguous:
                if has_pos:
                    # 使用位置距离作为歧义消解
                    pos_best = self._predict_position_error(
                        cyp, best_panel_id, center_x, center_y,
                        r1, r2, obs_x, obs_y)
                    pos_second = self._predict_position_error(
                        cyp, second_best_panel_id, center_x, center_y,
                        r1, r2, obs_x, obs_y)
                    if pos_second < pos_best:
                        panel_id = second_best_panel_id
                elif self._last_confident_panel is not None:
                    panel_id = self._last_confident_panel
            
            # 记录可靠的panel选择
            if best_yaw_error < np.deg2rad(15):
                self._last_confident_panel = panel_id
        
        # 计算center_yaw
        center_yaw = armor_yaw - panel_id * self.PANEL_ANGLE_STEP
        center_yaw = np.arctan2(np.sin(center_yaw), np.cos(center_yaw))
        
        return panel_id, center_yaw, best_yaw_error
    
    def _z_assisted_association(
        self,
        armor_yaw: float,
        center_yaw_pred: float,
        z_obs: float,
        center_z: float,
        yaw_best_panel: int,
        yaw_best_error: float
    ) -> int:
        """
        使用Z坐标辅助判断panel_id
        
        Upper层（z > center_z）→ 奇数panel (1,3) → r2
        Lower层（z < center_z）→ 偶数panel (0,2) → r1
        """
        is_upper = z_obs > center_z
        preferred_parity = 1 if is_upper else 0
        
        # 在preferred_parity的panel中选择yaw误差更小的
        candidate_panels = [i for i in range(4) if i % 2 == preferred_parity]
        
        best_parity_panel = candidate_panels[0]
        best_parity_error = float('inf')
        
        for p_id in candidate_panels:
            expected_armor_yaw = center_yaw_pred + p_id * self.PANEL_ANGLE_STEP
            diff = armor_yaw - expected_armor_yaw
            diff = np.arctan2(np.sin(diff), np.cos(diff))
            error = abs(diff)
            
            if error < best_parity_error:
                best_parity_error = error
                best_parity_panel = p_id
        
        # 只有当z辅助选择明显更好时才切换
        if (yaw_best_panel % 2 != preferred_parity and
            best_parity_error < yaw_best_error * 0.8):
            self._z_correction_count += 1
            return best_parity_panel
        
        return yaw_best_panel
    
    def get_r_type(self, panel_id: int) -> str:
        """
        根据panel_id获取半径类型
        
        偶数panel → r1
        奇数panel → r2
        """
        return 'r1' if panel_id % 2 == 0 else 'r2'
    
    def get_default_layer(self, panel_id: int) -> str:
        """
        根据panel_id获取默认层级
        
        偶数panel → lower
        奇数panel → upper
        """
        return 'lower' if panel_id % 2 == 0 else 'upper'
    
    @property
    def z_correction_count(self) -> int:
        """Z坐标修正次数"""
        return self._z_correction_count
