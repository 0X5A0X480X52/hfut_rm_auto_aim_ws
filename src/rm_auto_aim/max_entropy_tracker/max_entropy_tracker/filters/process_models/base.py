"""
过程模型抽象基类

定义过程模型组件的统一接口和状态布局管理
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, field
from enum import IntEnum
import numpy as np


class StateLayout:
    """
    状态向量布局管理器
    
    提供动态的状态索引访问，保持与原有 StateIndex 枚举相同的使用方式
    通过 @property 封装，对外行为一致
    
    使用示例:
        layout = StateLayout()
        layout.register('X', 0)
        layout.register('VX', 1)
        
        # 通过属性访问
        idx = layout.X  # 返回 0
        
        # 或通过方法访问
        idx = layout.get('X')  # 返回 0
    """
    
    def __init__(self):
        self._indices: Dict[str, int] = {}
        self._names: Dict[int, str] = {}
        self._dim: int = 0
        self._frozen: bool = False
    
    def register(self, name: str, index: int) -> None:
        """
        注册状态变量
        
        Args:
            name: 状态变量名称 (如 'X', 'VX', 'DELTA')
            index: 状态向量中的索引
        """
        if self._frozen:
            raise RuntimeError("StateLayout is frozen, cannot register new states")
        
        if name in self._indices:
            raise ValueError(f"State '{name}' already registered at index {self._indices[name]}")
        
        self._indices[name] = index
        self._names[index] = name
        self._dim = max(self._dim, index + 1)
    
    def register_block(self, names: List[str], start_index: int) -> int:
        """
        注册一组连续的状态变量
        
        Args:
            names: 状态变量名称列表
            start_index: 起始索引
            
        Returns:
            下一个可用索引
        """
        for i, name in enumerate(names):
            self.register(name, start_index + i)
        return start_index + len(names)
    
    def freeze(self) -> None:
        """冻结布局，防止后续修改"""
        self._frozen = True
    
    def get(self, name: str) -> int:
        """
        获取状态索引
        
        Args:
            name: 状态变量名称
            
        Returns:
            状态向量中的索引
        """
        if name not in self._indices:
            raise KeyError(f"Unknown state: '{name}'. Available: {list(self._indices.keys())}")
        return self._indices[name]
    
    def get_name(self, index: int) -> str:
        """获取索引对应的状态名称"""
        if index not in self._names:
            raise KeyError(f"Unknown index: {index}")
        return self._names[index]
    
    def has(self, name: str) -> bool:
        """检查是否包含指定状态"""
        return name in self._indices
    
    @property
    def dim(self) -> int:
        """状态维度"""
        return self._dim
    
    @property
    def names(self) -> List[str]:
        """所有状态名称"""
        return list(self._indices.keys())
    
    def get_slice(self, names: List[str]) -> List[int]:
        """获取多个状态的索引列表"""
        return [self.get(name) for name in names]
    
    def __getattr__(self, name: str) -> int:
        """
        支持属性式访问: layout.X
        
        注意：仅在常规属性查找失败后调用
        """
        if name.startswith('_'):
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
        
        try:
            return self.get(name)
        except KeyError:
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
    
    def __repr__(self) -> str:
        items = [f"{name}={idx}" for name, idx in sorted(self._indices.items(), key=lambda x: x[1])]
        return f"StateLayout({', '.join(items)})"
    
    def to_dict(self) -> Dict[str, int]:
        """转换为字典"""
        return self._indices.copy()
    
    @classmethod
    def from_dict(cls, indices: Dict[str, int]) -> 'StateLayout':
        """从字典创建"""
        layout = cls()
        for name, index in indices.items():
            layout.register(name, index)
        return layout


@dataclass
class ComponentStateSpec:
    """
    组件状态规格
    
    描述一个过程模型组件的状态变量
    """
    names: List[str]  # 状态变量名称列表
    dim: int  # 状态维度
    
    @classmethod
    def from_names(cls, names: List[str]) -> 'ComponentStateSpec':
        return cls(names=names, dim=len(names))


class ProcessModelComponent(ABC):
    """
    过程模型组件抽象基类
    
    每个组件负责状态向量的一部分，包括：
    - 定义该部分的状态变量
    - 实现状态转移方程
    - 构建过程噪声矩阵块
    
    子类需要实现:
    - get_state_spec(): 返回状态规格
    - predict(): 状态预测
    - build_Q(): 构建过程噪声矩阵块
    """
    
    def __init__(self, config: Optional[Any] = None):
        """
        初始化组件
        
        Args:
            config: 配置对象（可选）
        """
        self._config = config
        self._state_offset: int = 0  # 在组合状态向量中的偏移量
    
    @property
    def config(self) -> Any:
        return self._config
    
    @property
    def state_offset(self) -> int:
        """组件状态在完整状态向量中的起始偏移"""
        return self._state_offset
    
    @state_offset.setter
    def state_offset(self, value: int) -> None:
        self._state_offset = value
    
    @abstractmethod
    def get_state_spec(self) -> ComponentStateSpec:
        """
        获取状态规格
        
        Returns:
            ComponentStateSpec: 包含状态名称和维度
        """
        pass
    
    @property
    def state_dim(self) -> int:
        """组件状态维度"""
        return self.get_state_spec().dim
    
    @property
    def state_names(self) -> List[str]:
        """组件状态名称列表"""
        return self.get_state_spec().names
    
    @abstractmethod
    def predict(
        self, 
        x_component: np.ndarray, 
        dt: float,
        full_state: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        状态预测
        
        Args:
            x_component: 组件对应的状态子向量
            dt: 时间步长
            full_state: 完整状态向量（某些模型可能需要访问其他状态）
            
        Returns:
            预测后的组件状态子向量
        """
        pass
    
    @abstractmethod
    def build_Q(self, dt: float) -> np.ndarray:
        """
        构建过程噪声矩阵块
        
        Args:
            dt: 时间步长
            
        Returns:
            该组件对应的过程噪声矩阵块 (state_dim x state_dim)
        """
        pass
    
    def get_initial_state(self) -> np.ndarray:
        """
        获取初始状态（可选实现）
        
        Returns:
            初始状态向量
        """
        return np.zeros(self.state_dim)
    
    def get_initial_covariance(self) -> np.ndarray:
        """
        获取初始协方差矩阵块（可选实现）
        
        Returns:
            初始协方差矩阵块
        """
        return np.eye(self.state_dim)
    
    def extract_component_state(self, full_state: np.ndarray) -> np.ndarray:
        """从完整状态向量中提取组件状态"""
        return full_state[self._state_offset:self._state_offset + self.state_dim]
    
    def inject_component_state(
        self, 
        full_state: np.ndarray, 
        component_state: np.ndarray
    ) -> np.ndarray:
        """将组件状态注入完整状态向量"""
        full_state = full_state.copy()
        full_state[self._state_offset:self._state_offset + self.state_dim] = component_state
        return full_state
