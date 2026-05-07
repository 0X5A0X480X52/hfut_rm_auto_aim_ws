# Fire Advice 正面入射与最小激发速度门控细化设计（主 C++链路）

## 1. 背景与问题定义

当前主 C++链路中，`fire_advice` 的概率模块已具备时间窗命中概率评估与滞回输出能力，但对“入射方向物理可打性”的约束仍不完整，主要表现为：

1. 可能在非正面入射（背打/掠射）场景仍给出开火建议。
2. 即使正面入射，若法向分速度不足以触发装甲板，也可能给出开火建议。

目标是补齐该物理约束，使开火建议满足：

1. 仅允许正面入射时进入可开火候选。
2. 法向有效速度低于最小激发速度时，明确禁止开火建议。

---

## 2. 几何语义与符号约定

### 2.1 装甲板法向

沿用现有语义：

- 装甲板法向 `n` 定义为“机器人中心 -> 装甲板方向”的单位向量。
- 该方向视为装甲板正方向（外法线）。

### 2.2 子弹速度方向

- 子弹速度向量记为 `v_b`。
- `v_b` 的方向为子弹前进方向（子弹正方向）。

### 2.3 正面入射判定

采用点积判定：

- `dot(v_b, n) < 0`：子弹从装甲板正面射入（钝角）。
- `dot(v_b, n) >= 0`：非正面入射，不允许开火建议。

工程上引入容差 `eps_front`，避免数值抖动：

- `front_ok = dot(v_b, n) < -eps_front`

建议 `eps_front = 1e-4 ~ 1e-3`。

### 2.4 法向有效速度

定义：

- `v_n = max(0, -dot(v_b, n))`

含义：

- 只统计“朝向装甲板内部”的法向速度分量。
- 掠射或反向时 `v_n` 变小或为 0。

---

## 3. 目标策略：两级门控 + 可选加权

为同时保证物理正确性与可调性，采用三段式机制：

1. 硬门控 A（正面门控）：非正面入射直接拒绝。
2. 硬门控 B（激发门控）：`v_n < v_activate_min` 直接拒绝。
3. 软加权（可选）：对通过硬门控的样本按 `v_n` 做权重缩放。

### 3.1 硬门控（必须）

- 若 `require_front_face=true` 且 `!front_ok`：
  - `p_hit(tau) = 0`
- 若 `v_n < v_activate_min`：
  - `p_hit(tau) = 0`

此机制保证“不满足物理入射条件时绝不开火”。

### 3.2 软加权（可选）

在通过硬门控后，可启用法向速度权重：

- `w_n = clamp(v_n / v_ref, w_min, 1.0)`
- `p_hit'(tau) = p_hit(tau) * w_n`

说明：

- 软加权用于区分“刚过阈值”和“正冲击”场景的命中质量。
- 若仅需严格物理约束，可关闭软加权，仅保留硬门控。

---

## 4. 参数设计

## 4.1 新增参数（建议）

命名建议（ROS 参数空间）：

- `controller.fire.probability.normal_velocity_gate.enable` (bool, default=true)
- `controller.fire.probability.normal_velocity_gate.require_front_face` (bool, default=true)
- `controller.fire.probability.normal_velocity_gate.v_activate_min` (double, m/s)
- `controller.fire.probability.normal_velocity_gate.front_epsilon` (double, default=1e-4)

## 4.2 复用参数（已预留）

现有字段：

- `controller.fire.probability.normal_velocity_weight.enable`
- `controller.fire.probability.normal_velocity_weight.v_ref`
- `controller.fire.probability.normal_velocity_weight.w_min`

主链路需接线到 `ProbabilityConfig` 并在 `ProbabilityEngine` 中使用。

## 4.3 推荐初值

- `normal_velocity_gate.enable = true`
- `require_front_face = true`
- `v_activate_min = 8.0`（需结合实弹标定）
- `front_epsilon = 1e-4`
- `normal_velocity_weight.enable = false`（先硬门控稳定后再开）

---

## 5. 代码改造点（主 C++链路）

## 5.1 数据结构扩展

文件：

- `include/gimbal_controller/fire_advice/types.hpp`

建议新增：

1. `NormalVelocityGateConfig`（硬门控配置）
2. `TauDebugSample` 调试字段：
   - `bool front_ok`
   - `double normal_velocity`
   - `double normal_weight`
   - `bool normal_gate_pass`

## 5.2 参数声明与读取

文件：

- `src/gimbal_pipeline_node.cpp`

改造内容：

1. 声明 `normal_velocity_gate.*` 参数。
2. 读取并写入 `prob_cfg`。
3. 将已存在的 `normal_velocity_weight.*` 参数接入 `prob_cfg`（当前未接线）。

## 5.3 概率引擎逻辑

文件：

- `src/gimbal_controller/fire_advice/probability_engine.cpp`

在每个 `tau` 样本处（计算 `p_hit` 后、融合前）执行：

1. 计算 `front_ok`。
2. 计算 `v_n`。
3. 执行硬门控（可将 `p_hit` 直接清零）。
4. 可选执行 `normal_velocity_weight`。
5. 写入调试样本。

关键要求：

- 门控应作用于每个 `tau` 样本，而不是仅作用于最终 `best_tau`，避免时间窗融合绕过约束。

---

## 6. 伪代码

```cpp
for each tau_sample {
  // 已有：e_u/e_v/sigma_u/sigma_v -> p_hit
  double p_hit = hitProbabilityIndependent(...);

  // 1) 计算子弹速度（world）
  Eigen::Vector3d bullet_vel_world = ...;  // 基于当前弹道模型

  // 2) 装甲板法向（impact时刻）
  Eigen::Vector3d n = n_impact.normalized();

  // 3) 正面入射与法向速度
  double dot_vn = bullet_vel_world.dot(n);
  bool front_ok = (dot_vn < -front_epsilon);
  double v_n = std::max(0.0, -dot_vn);

  // 4) 硬门控
  bool gate_pass = true;
  if (normal_gate_enable) {
    if (require_front_face && !front_ok) gate_pass = false;
    if (v_n < v_activate_min) gate_pass = false;
  }
  if (!gate_pass) {
    p_hit = 0.0;
  }

  // 5) 软加权（可选）
  double normal_weight = 1.0;
  if (gate_pass && normal_weight_enable) {
    normal_weight = clamp(v_n / v_ref, w_min, 1.0);
    p_hit *= normal_weight;
  }

  // 6) debug
  sample.front_ok = front_ok;
  sample.normal_velocity = v_n;
  sample.normal_weight = normal_weight;
  sample.normal_gate_pass = gate_pass;
}
```

---

## 7. 与现有模块关系

1. `FireAdviceEngine` 的候选 facing 过滤继续保留（粗筛）。
2. `ProbabilityEngine` 增加入射物理约束（细筛）。
3. `fire_gate`（低通/积分+滞回）无需改动，仅消费门控后 `p_window`。

这样可形成“几何朝向 + 物理入射 + 概率稳定输出”的层级约束。

---

## 8. 验收用例

## 8.1 正面且速度足够

- 条件：`dot(v_b, n) < 0`，`v_n >= v_activate_min`
- 期望：`p_hit` 正常，可能开火。

## 8.2 正面但速度不足

- 条件：`dot(v_b, n) < 0`，`v_n < v_activate_min`
- 期望：`p_hit=0`，`fire_advice=false`。

## 8.3 非正面入射

- 条件：`dot(v_b, n) >= 0`
- 期望：`p_hit=0`，`fire_advice=false`。

## 8.4 阈值边缘抖动

- 条件：`v_n` 在阈值附近波动。
- 期望：在 `fire_gate` 滞回作用下输出无高频抖动。

---

## 9. 风险与后续

1. 当前子弹速度模型若未考虑阻力，`v_n` 可能偏乐观。
2. `v_activate_min` 对不同枪口、弹种、距离敏感，需实弹标定。
3. 后续可扩展：
   - 阈值随距离/弹速动态映射；
   - 对 `v_n` 使用置信区间门控（结合 sigma-point）；
   - 以实测数据拟合“激发概率曲线”替代硬阈值。

---

## 10. 里程碑建议

1. M1：参数接线 + 调试字段 + 硬门控上线。
2. M2：软加权上线 + 参数整定。
3. M3：实弹回放标定 `v_activate_min` 与 `v_ref/w_min`。
4. M4：速度模型升级（阻力或查表）并复核门限。

该方案可在不破坏现有架构的前提下，显著减少“穿透式/擦边式”误开火建议，并将 `fire_advice` 与真实激发物理约束对齐。
