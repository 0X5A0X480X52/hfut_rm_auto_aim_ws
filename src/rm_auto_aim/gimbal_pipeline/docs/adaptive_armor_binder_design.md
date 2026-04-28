# AdaptiveArmorBinder 统一装甲板绑定机制设计

## 背景

`gimbal_pipeline` 中有两个 Tracker 各自独立实现了装甲板 ID 绑定机制：

- **AdaptiveArmorTracker**（4板）：Jump Binding，通过面板邻接性和 z 跳变幅度检测来识别和绑定 panel_id
- **OutpostArmorTracker**（3板）：Z-Jump Audit + Periodic Evidence，通过假设代价比较和周期跳变模板匹配来绑定 panel_id

两个实现在状态机、EMA 学习、信心度评分等核心逻辑上高度相似，但散落在各自代码中，难以维护和统一配置。

## 目标

在 `association/` 目录下设计统一的 `AdaptiveArmorBinder` 类，满足：

1. 通过配置参数支持任意面板数目（N ≥ 2）
2. 通过 `z_offsets[]` 配置各面板 ID 间的高度差
3. 支持两种识别策略（PROXIMITY / COST），分别对应两种不同绑定场景
4. 提取共同的状态机、EMA 学习、信心度评分，消除重复代码
5. 可选的周期证据系统（适用于旋转目标）

## 核心设计

### 类图

```
AdaptiveArmorBinder
├── Strategy: PROXIMITY | COST
├── Config
│   ├── n_panels, z_offsets[]          ← 面板数及高度差
│   ├── confirm_frames, cooldown_frames ← 状态机参数
│   ├── PROXIMITY 门限参数
│   ├── COST 门限参数
│   └── Periodic evidence 参数
├── 状态机
│   ├── BindingState: LOCKED | TRANSITION_CANDIDATE
│   └── attempt_transition(candidate_panel, gate_passed, confirm_required)
├── bind_proximity()  ← AdaptiveArmorTracker 风格
├── bind_cost()       ← OutpostArmorTracker 风格
├── update_periodic_evidence()  ← 周期模板匹配
├── apply_periodic_prior()      ← 周期先验注入
├── compute_proximity_confidence()
├── compute_cost_confidence()
└── compute_same_panel_score()
```

### 策略对比

| 维度 | PROXIMITY | COST |
|------|-----------|------|
| **适用场景** | 4 板标准布局 | 3 板前哨站布局 |
| **触发条件** | 面板邻接 + z跳变幅度匹配EMA | 假设代价比较 + switch_score |
| **门限层数** | 5 层 | 3 层 |
| **切换得分** | 成本裕度 + yaw误差 + jump_score | 候选概率 + 裕度 + same_panel_score |
| **冷却机制** | 有（cooldown_frames） | 无 |
| **高度解算** | 跳变方向推导（dz_gate） | z_offsets 查询 |

### 状态机转移

```
                    ┌─────────────────────────┐
                    │       LOCKED            │
                    │  (bound_panel 稳定)      │
                    └─────┬───────┬───────────┘
                          │       │
          candidate == bound    candidate != bound
          (同一面板)             + gate_passed
                          │       │
                          ▼       ▼
                    stay LOCKED   ┌──────────────────────────────┐
                                  │  TRANSITION_CANDIDATE        │
                                  │  (记录候选, confirm_count=1) │
                                  └──┬──────────┬───────────────┘
                                     │          │
                        连续相同候选        候选变化/门槛不通过
                        count++              │
                                     │          │
                                     ▼          ▼
                              count >= N    重启候选/LOCKED
                                     │
                                     ▼
                              ┌──────────┐
                              │ 确认切换  │
                              │ 更新bound │
                              │ +冷却     │
                              └──────────┘
```

### 信心度评分

**PROXIMITY 模式**：
```
base = 0.50 * margin_score + 0.30 * yaw_score + 0.20 * jump_score
confidence = floor + (1 - floor) * base
```
- margin_score: 成本裕度归一化（相对于 2×margin_min）
- yaw_score: yaw误差归一化（相对于 yaw_gate）
- jump_score: 门槛通过即 1.0
- floor: 最低信心度（防止过低）

**COST 模式**：
```
v = (0.35*prob + 0.30*margin_norm + 0.20*period_conf + 0.15*same_score) / sum(weights)
confidence = floor + (1 - floor) * v
```

### 周期证据系统

适用于旋转目标（如前哨站），通过匹配 N 板旋转模板来辅助识别：

- EMA 学习 dz_small 和 dz_large 两个跳变量级
- 根据旋转方向（CW/CCW）生成周期模板
- 对每个 phase 计算归一化跳变序列与模板的匹配度
- 匹配结果作为先验权重注入假设代价

## 接口速查

### PROXIMITY 模式
```cpp
bool bind_proximity(double obs_z, double obs_yaw, int candidate_panel,
                    const AssociationDiagnostics &diag,
                    HeightLabel candidate_label, double candidate_h_conf,
                    BindResult *result);
```

### COST 模式
```cpp
bool bind_cost(const std::vector<HypothesisScore> &hypotheses,
               double predicted_center_z, double center_yaw_est,
               BindResult *result);
```

### 周期证据
```cpp
void update_periodic_evidence(double z_jump, double yaw_rate_est,
                              bool allow_model_update);
void apply_periodic_prior(std::vector<HypothesisScore> &hyps,
                          double z_jump) const;
```

### 状态管理
```cpp
void reset(int init_panel_id, HeightLabel init_label,
           std::optional<double> obs_z, std::optional<double> obs_time);
DebugSnapshot debug_snapshot() const;
```

## 文件位置

| 文件 | 说明 |
|------|------|
| `include/max_entropy_tracker/association/adaptive_armor_binder.hpp` | 类定义 |
| `src/max_entropy_tracker/association/adaptive_armor_binder.cpp` | 实现 |

## 配置示例

### 4 板机器人（PROXIMITY）
```yaml
armor_binder:
  n_panels: 4
  z_offsets: [0.06, 0.0, 0.06, 0.0]  # panel 0=LOWER, 1=UPPER, 2=LOWER, 3=UPPER
  strategy: 0                          # PROXIMITY
  confirm_frames: 3
  cooldown_frames: 20
  z_jump_min: 0.025
  yaw_err_gate: 0.17
  cost_margin_min: 0.05
  dz_match_tolerance: 0.015
  dz_gate: 0.01
  dz_ema_alpha: 0.20
  confidence_floor: 0.30
```

### 3 板前哨站（COST）
```yaml
armor_binder:
  n_panels: 3
  z_offsets: [0.102, 0.00, -0.102]   # panel 0=HIGH, 1=MIDDLE, 2=LOW
  strategy: 1                          # COST
  confirm_frames: 3
  min_candidate_prob: 0.40
  min_candidate_margin: 0.12
  switch_strong_score: 0.60
  same_panel_yaw_gate: 0.35
  same_panel_z_gate: 0.08
  same_panel_xy_gate: 0.18
  confidence_floor: 0.15
  periodic_enable: true
  periodic_window: 12
  periodic_weight: 0.60
  periodic_min_spin_rate: 0.8
  periodic_update_min_jump: 0.015
```

## 集成方案

改造步骤：

1. **AdaptiveArmorTracker**：将 `update_jump_binding()`、`reset_jump_binding()`、`compute_jump_binding_confidence()`、`update_jump_statistics()` 等内部逻辑替换为对 `AdaptiveArmorBinder::bind_proximity()` 的调用
2. **OutpostArmorTracker**：将 `update_binding_state_machine()`、`update_periodic_evidence()`、`apply_periodic_jump_prior()` 等内部逻辑替换为对 `AdaptiveArmorBinder::bind_cost()` 的调用
3. **配置迁移**：将两处的绑定相关配置参数合并到统一的 `armor_binder` 段
