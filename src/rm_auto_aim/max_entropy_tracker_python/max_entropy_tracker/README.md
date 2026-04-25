# Max Entropy UKF Dual Radius Demo V2

高度解耦的模块化重构版本，基于 `max_entropy_ukf_dual_radius_demo` 中的 `decomposed_ukf_v2.py` 和 `tracker_v3.py` 实现。

## 设计原则

1. **纯抽象基类**: `BaseUKF` 和 `BaseTracker` 不包含具体实现，仅定义接口
2. **独立状态索引**: 每个UKF子类使用 `IntEnum` 独立定义状态索引
3. **列表输入接口**: `update()` 方法接受观测列表，支持单观测和多观测
4. **模块化数据关联**: Panel关联、高度识别、震荡检测独立为模块
5. **Sigma点生成独立**: 作为独立工具类，可复用于不同UKF变体

## 目录结构

```
max_entropy_ukf_dual_radius_demo2/
├── __init__.py              # 顶层导出
├── README.md                # 本文档
│
├── core/                    # 核心模块
│   ├── config.py           # 统一配置类 (UnifiedConfig)
│   └── observation.py      # 观测数据结构 (ObservationData)
│
├── utils/                   # 工具模块
│   ├── angle_utils.py      # 角度处理 (normalize, decompose, compose)
│   ├── sigma_points.py     # Sigma点生成器
│   └── constraints.py      # 状态约束
│
├── filters/                 # 滤波器模块
│   ├── base_ukf.py         # UKF抽象基类
│   └── dual_radius_spin_ukf.py  # 双半径自旋UKF实现
│
├── association/             # 数据关联模块
│   ├── panel_associator.py     # Panel ID关联
│   ├── height_identifier.py    # 高度层级识别
│   └── oscillation_detector.py # 参数震荡检测
│
├── trackers/                # 跟踪器模块
│   ├── base_tracker.py     # Tracker抽象基类
│   └── adaptive_armor_tracker.py  # 自适应装甲板跟踪器
│
└── tests/                   # 测试模块
    └── test_basic.py       # 基本功能测试
```

## 核心类说明

### DualRadiusSpinUKF

11维状态向量的UKF实现，专门用于4面装甲板机器人位姿估计。

**状态向量** (通过 `StateIndex` 枚举索引):
| 索引 | 名称 | 说明 |
|-----|------|------|
| 0 | X | X位置 |
| 1 | VX | X速度 |
| 2 | Y | Y位置 |
| 3 | VY | Y速度 |
| 4 | Z | Z位置（装甲板平均高度） |
| 5 | VZ | Z速度 |
| 6 | DELTA | 连续角度 ∈ [-π/2, π/2] |
| 7 | DELTA_RATE | 角速度 |
| 8 | R1 | 半径1（偶数panel） |
| 9 | R2 | 半径2（奇数panel） |
| 10 | DZA | 装甲板高度差 |

**核心特性**:
- Yaw分解: `yaw = k*π + delta`，k∈{0,1}
- 单观测参数冻结: 防止r1/r2/dza漂移
- 双观测几何更新: 通过射线交点估计中心和半径

### AdaptiveArmorTracker

组合使用多个模块的完整跟踪器实现。

**组件**:
- `DualRadiusSpinUKF`: 状态估计
- `PanelAssociator`: yaw→panel_id转换
- `HeightIdentifier`: 上/下层级识别
- `OscillationDetector`: 参数震荡检测

## 使用示例

```python
from src.max_entropy_ukf_dual_radius_demo2 import (
    AdaptiveArmorTracker, UnifiedConfig, ObservationData
)

# 创建配置和跟踪器
config = UnifiedConfig.create_default()
tracker = AdaptiveArmorTracker(config=config, dt=0.05)

# 初始化
obs_init = ObservationData(x=1.0, y=2.0, z=0.5, yaw=0.0)
tracker.initialize([obs_init], r1=0.15, r2=0.20)

# 跟踪循环
for frame in frames:
    tracker.predict()
    
    # 构造观测
    observations = [ObservationData(x=x, y=y, z=z, yaw=yaw) for ...]
    
    # 更新（自动处理单观测/双观测）
    tracker.update(observations)
    
    # 获取结果
    center = tracker.get_center_position()
    yaw = tracker.get_yaw()
    r1, r2 = tracker.get_radii()
```

## 扩展指南

### 实现新的UKF

```python
from src.max_entropy_ukf_dual_radius_demo2.filters.base_ukf import BaseUKF
from enum import IntEnum

class MyStateIndex(IntEnum):
    X = 0
    Y = 1
    # ... 自定义状态索引

class MyUKF(BaseUKF):
    @property
    def state_dim(self) -> int:
        return len(MyStateIndex)
    
    def _process_model(self, x, dt):
        # 实现状态转移方程
        pass
    
    def _observation_model(self, x, **kwargs):
        # 实现观测方程
        pass
    
    # ... 实现其他抽象方法
```

### 实现新的Tracker

```python
from src.max_entropy_ukf_dual_radius_demo2.trackers.base_tracker import BaseTracker

class MyTracker(BaseTracker):
    def initialize(self, observations, **kwargs):
        # 初始化逻辑
        pass
    
    def predict(self):
        # 预测逻辑
        pass
    
    def update(self, observations, **kwargs):
        # 更新逻辑
        pass
    
    # ... 实现其他抽象方法
```

## 与原实现的对应关系

| demo1 文件 | demo2 等价实现 |
|-----------|---------------|
| `filters/decomposed_ukf_v2.py` | `filters/dual_radius_spin_ukf.py` |
| `trackers/tracker_v3.py` | `trackers/adaptive_armor_tracker.py` |
| `filters/base_ukf.py` (部分实现) | `filters/base_ukf.py` (纯抽象) |
| - | `trackers/base_tracker.py` (新增) |
| (内嵌) | `association/panel_associator.py` (拆分) |
| (内嵌) | `association/height_identifier.py` (拆分) |
| (内嵌) | `association/oscillation_detector.py` (拆分) |
| (内嵌) | `utils/sigma_points.py` (拆分) |
