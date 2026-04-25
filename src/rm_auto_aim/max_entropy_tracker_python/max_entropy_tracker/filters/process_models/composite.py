"""
组合过程模型

将多个过程模型组件组合成一个完整的过程模型
自动管理状态向量布局和索引映射
"""

import numpy as np
from typing import List, Optional, Dict, Any, Tuple
from dataclasses import dataclass
import logging

from .base import ProcessModelComponent, StateLayout, ComponentStateSpec
from .translation import (
    TranslationModelType, TranslationConfig,
    CVTranslation, CATranslation, SingerTranslation,
    create_translation_model
)
from .rotation import (
    RotationModelType, RotationConfig,
    CVRotation, CARotation,
    create_rotation_model
)
from .structural import StructuralConfig, StructuralModel

logger = logging.getLogger(__name__)


class CompositeProcessModel:
    """
    组合过程模型
    
    将多个过程模型组件组合成一个完整的过程模型
    
    功能:
    1. 动态拼接子模型的状态向量
    2. 提供统一的 StateLayout 进行索引访问
    3. 组合各子模型的 predict() 和 build_Q()
    4. 兼容原有 StateIndex 枚举的使用方式
    
    使用示例:
        # 创建组合模型
        model = CompositeProcessModel([
            CVTranslation(),    # 平移: [X, VX, Y, VY, Z, VZ]
            CVRotation(),       # 旋转: [DELTA, DELTA_RATE]
            StructuralModel()   # 结构: [R1, R2, DZA]
        ])
        
        # 访问状态索引
        x_idx = model.layout.X
        delta_idx = model.layout.DELTA
        
        # 预测
        x_next = model.predict(x, dt)
        
        # 获取过程噪声
        Q = model.build_Q(dt)
    """
    
    def __init__(self, components: List[ProcessModelComponent]):
        """
        初始化组合模型
        
        Args:
            components: 过程模型组件列表
        """
        self._components = components
        self._layout = StateLayout()
        self._component_offsets: List[int] = []
        
        # 构建状态布局
        self._build_layout()
        
        logger.info(
            f"CompositeProcessModel created with {len(components)} components, "
            f"total state_dim={self.state_dim}"
        )
    
    def _build_layout(self) -> None:
        """构建状态向量布局"""
        current_offset = 0
        
        for component in self._components:
            # 设置组件偏移量
            component.state_offset = current_offset
            self._component_offsets.append(current_offset)
            
            # 注册组件状态到布局
            spec = component.get_state_spec()
            for name in spec.names:
                self._layout.register(name, current_offset)
                current_offset += 1
        
        # 冻结布局
        self._layout.freeze()
    
    @property
    def layout(self) -> StateLayout:
        """获取状态布局（支持属性式访问）"""
        return self._layout
    
    @property
    def state_dim(self) -> int:
        """总状态维度"""
        return self._layout.dim
    
    @property
    def components(self) -> List[ProcessModelComponent]:
        """获取所有组件"""
        return self._components
    
    def get_component_by_state(self, state_name: str) -> Optional[ProcessModelComponent]:
        """根据状态名称获取对应的组件"""
        for component in self._components:
            if state_name in component.state_names:
                return component
        return None
    
    def predict(self, x: np.ndarray, dt: float) -> np.ndarray:
        """
        组合预测
        
        依次调用每个组件的predict方法
        
        Args:
            x: 完整状态向量
            dt: 时间步长
            
        Returns:
            预测后的状态向量
        """
        x_next = x.copy()
        
        for component in self._components:
            # 提取组件状态
            x_comp = component.extract_component_state(x)
            
            # 组件预测
            x_comp_next = component.predict(x_comp, dt, full_state=x)
            
            # 注入预测结果
            x_next = component.inject_component_state(x_next, x_comp_next)
        
        return x_next
    
    def build_Q(self, dt: float) -> np.ndarray:
        """
        构建完整过程噪声矩阵
        
        将各组件的Q矩阵块组合成完整矩阵
        
        Args:
            dt: 时间步长
            
        Returns:
            完整过程噪声矩阵 (state_dim x state_dim)
        """
        Q = np.zeros((self.state_dim, self.state_dim))
        
        for component in self._components:
            offset = component.state_offset
            dim = component.state_dim
            
            Q_block = component.build_Q(dt)
            Q[offset:offset+dim, offset:offset+dim] = Q_block
        
        return Q
    
    def get_initial_state(self) -> np.ndarray:
        """获取初始状态向量"""
        x = np.zeros(self.state_dim)
        
        for component in self._components:
            offset = component.state_offset
            dim = component.state_dim
            x[offset:offset+dim] = component.get_initial_state()
        
        return x
    
    def get_initial_covariance(self) -> np.ndarray:
        """获取初始协方差矩阵"""
        P = np.zeros((self.state_dim, self.state_dim))
        
        for component in self._components:
            offset = component.state_offset
            dim = component.state_dim
            P[offset:offset+dim, offset:offset+dim] = component.get_initial_covariance()
        
        return P
    
    def get_state_info(self) -> Dict[str, Any]:
        """获取状态信息（用于调试）"""
        return {
            'state_dim': self.state_dim,
            'layout': self._layout.to_dict(),
            'components': [
                {
                    'type': type(comp).__name__,
                    'offset': comp.state_offset,
                    'dim': comp.state_dim,
                    'states': comp.state_names
                }
                for comp in self._components
            ]
        }
    
    def __repr__(self) -> str:
        comp_names = [type(c).__name__ for c in self._components]
        return f"CompositeProcessModel({comp_names}, dim={self.state_dim})"


def create_default_process_model(
    translation_type: TranslationModelType = TranslationModelType.CV,
    rotation_type: RotationModelType = RotationModelType.CV,
    translation_config: Optional[TranslationConfig] = None,
    rotation_config: Optional[RotationConfig] = None,
    structural_config: Optional[StructuralConfig] = None,
    n_dims: int = 3
) -> CompositeProcessModel:
    """
    创建默认的组合过程模型
    
    组合: 平移 + 旋转 + 结构参数
    
    Args:
        translation_type: 平移模型类型 (CV/CA/Singer)
        rotation_type: 旋转模型类型 (CV/CA)
        translation_config: 平移模型配置
        rotation_config: 旋转模型配置
        structural_config: 结构参数配置
        n_dims: 空间维度 (2D或3D)
        
    Returns:
        组合过程模型
    """
    components = [
        create_translation_model(translation_type, translation_config, n_dims),
        create_rotation_model(rotation_type, rotation_config),
        StructuralModel(structural_config)
    ]
    
    return CompositeProcessModel(components)


# ==================== 兼容层 ====================
# 提供与原有 StateIndex 枚举相同的接口

class StateIndexAdapter:
    """
    状态索引适配器
    
    提供与原有 StateIndex IntEnum 相同的接口
    用于平滑过渡到新的 StateLayout 系统
    
    使用示例:
        StateIndex = StateIndexAdapter(process_model.layout)
        idx = StateIndex.X  # 与原来的 StateIndex.X 用法相同
    """
    
    def __init__(self, layout: StateLayout):
        self._layout = layout
    
    def __getattr__(self, name: str) -> int:
        return self._layout.get(name)
    
    def __repr__(self) -> str:
        return f"StateIndexAdapter({self._layout})"
