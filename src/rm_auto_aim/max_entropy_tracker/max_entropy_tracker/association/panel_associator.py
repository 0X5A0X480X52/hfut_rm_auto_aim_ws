"""
装甲板Panel关联器

将观测到的装甲板yaw角关联到具体的panel_id (0,1,2,3)

坐标系说明：
- 使用相机坐标系
- armor_yaw定义为径向方向（中心指向装甲板）
- center_yaw是旋转中心的朝向
- armor_yaw = center_yaw + panel_angle
"""

import numpy as np
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class PanelAssociator:
    """
    装甲板Panel关联器
    
    核心功能:
    1. 根据观测yaw和预测center_yaw确定panel_id
    2. 支持Z坐标辅助判断（当yaw匹配误差大时）
    3. 将armor_yaw转换为center_yaw
    
    装甲板布局（4面固定结构）:
    - Panel 0: offset=0°,   r1, lower
    - Panel 1: offset=90°,  r2, upper
    - Panel 2: offset=180°, r1, lower
    - Panel 3: offset=270°, r2, upper
    """
    
    N_PANELS = 4
    PANEL_ANGLE_STEP = np.pi / 2  # 90°
    
    def __init__(self):
        """初始化关联器"""
        self._last_confident_panel: Optional[int] = None
        self._z_correction_count: int = 0
    
    def associate_panel(
        self,
        armor_yaw: float,
        center_yaw_pred: Optional[float],
        z_obs: Optional[float] = None,
        center_z: Optional[float] = None
    ) -> Tuple[int, float, float]:
        """
        将观测armor_yaw关联到panel_id
        
        Args:
            armor_yaw: 观测到的装甲板yaw角（径向方向：中心指向装甲板）
            center_yaw_pred: 预测的中心yaw（如果为None则为首帧）
            z_obs: 可选，装甲板z坐标（用于辅助判断）
            center_z: 可选，预测的中心z坐标
            
        Returns:
            (panel_id, center_yaw, matching_error)
        """
        if center_yaw_pred is None:
            # 首帧：根据armor_yaw找最接近的90°倍数
            # 为了避免180°歧义，先归一化到[-π, π]，然后映射到4个区间
            armor_yaw_normalized = np.arctan2(np.sin(armor_yaw), np.cos(armor_yaw))  # [-π, π]
            
            # 映射到[0, 2π)以便计算panel_id
            armor_yaw_positive = (armor_yaw_normalized + 2 * np.pi) % (2 * np.pi)
            
            # 找最接近的90°倍数
            panel_id = int(np.round(armor_yaw_positive / self.PANEL_ANGLE_STEP)) % 4
            
            # 计算center_yaw（armor_yaw减去panel偏移）
            center_yaw = armor_yaw_normalized - panel_id * self.PANEL_ANGLE_STEP
            center_yaw = np.arctan2(np.sin(center_yaw), np.cos(center_yaw))  # 归一化到[-π, π]
            
            return panel_id, center_yaw, 0.0
        
        # 计算各panel的匹配误差
        best_panel_id = 0
        best_error = float('inf')
        second_best_panel_id = 0
        second_best_error = float('inf')
        
        for panel_id in range(4):
            expected_armor_yaw = center_yaw_pred + panel_id * self.PANEL_ANGLE_STEP
            diff = armor_yaw - expected_armor_yaw
            diff = np.arctan2(np.sin(diff), np.cos(diff))
            error = abs(diff)
            
            if error < best_error:
                second_best_panel_id = best_panel_id
                second_best_error = best_error
                best_error = error
                best_panel_id = panel_id
            elif error < second_best_error:
                second_best_panel_id = panel_id
                second_best_error = error
        
        panel_id = best_panel_id
        
        # Z坐标辅助判断（当yaw匹配误差大时）
        if z_obs is not None and center_z is not None and best_error > np.deg2rad(30.0):
            panel_id = self._z_assisted_association(
                armor_yaw, center_yaw_pred, z_obs, center_z,
                best_panel_id, best_error
            )
        else:
            # 歧义检测：如果最佳和次佳误差太接近，使用上次可靠的panel
            is_ambiguous = (
                best_error > np.deg2rad(20) and
                abs(best_error - second_best_error) < np.deg2rad(10)
            )
            
            if is_ambiguous and self._last_confident_panel is not None:
                panel_id = self._last_confident_panel
            
            # 记录可靠的panel选择
            if best_error < np.deg2rad(15):
                self._last_confident_panel = panel_id
        
        # 计算center_yaw
        center_yaw = armor_yaw - panel_id * self.PANEL_ANGLE_STEP
        center_yaw = np.arctan2(np.sin(center_yaw), np.cos(center_yaw))
        
        return panel_id, center_yaw, best_error
    
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
