"""
可扩展过程模型组件

支持CV/CA/Singer等运动模型的组合使用
"""

from .base import ProcessModelComponent, StateLayout, ComponentStateSpec
from .translation import (
    TranslationModelType,
    TranslationConfig,
    CVTranslation,
    CATranslation,
    SingerTranslation,
    create_translation_model
)
from .rotation import (
    RotationModelType,
    RotationConfig,
    CVRotation,
    CARotation,
    create_rotation_model
)
from .structural import (
    StructuralConfig,
    DualRadiusModel,
    HeightModel,
    StructuralModel
)
from .composite import CompositeProcessModel, create_default_process_model

__all__ = [
    # 基类
    'ProcessModelComponent',
    'StateLayout',
    'ComponentStateSpec',
    # 平移模型
    'TranslationModelType',
    'TranslationConfig',
    'CVTranslation',
    'CATranslation', 
    'SingerTranslation',
    'create_translation_model',
    # 旋转模型
    'RotationModelType',
    'RotationConfig',
    'CVRotation',
    'CARotation',
    'create_rotation_model',
    # 结构参数模型
    'StructuralConfig',
    'DualRadiusModel',
    'HeightModel',
    'StructuralModel',
    # 组合模型
    'CompositeProcessModel',
    'create_default_process_model',
]
