下面给你一套**可以直接落地到你现有自瞄 + MPC 的完整方案**，从建模 → 预测 → 命中概率 → 权重 → QP → 工程细节，全部串起来。目标是：

> **在保持 QP 实时性的前提下，实现“自动选板 + 选时机 + 提前量补偿”的连续优化控制**

---

# 一、总体架构（比赛可用）

```text
每个控制周期（Δt ≈ 5~20ms）：

[1] 状态估计（Tracker / KalmanNet）
        ↓
[2] 未来预测（θ_k, ω_k, d_k, k=0..N）
        ↓
[3] 命中概率建模 r_k
        ↓
[4] 权重构造 w_k（冻结）
        ↓
[5] MPC(QP) 求解
        ↓
[6] 执行 u_0（只执行第一步）
        ↓
循环滚动
```

---

# 二、状态与预测模型

## 1️⃣ 状态定义（简化版）

```cpp
x = [yaw, pitch, yaw_rate, pitch_rate]
```

目标侧（由 tracker 给）：

```cpp
target: θ, ω, d
```

---

## 2️⃣ 预测模型（关键）

对未来 N 步：

```cpp
θ_k = θ_0 + ω * k * dt
ω_k ≈ ω（或用KalmanNet预测）
d_k ≈ d（短时可近似）
```

👉 得到：

```cpp
for k in 0..N:
    θ_k, ω_k, d_k
```

---

# 三、命中概率模型（核心）

我们构造：

[
r_k = f(\theta_k, \omega_k, d_k, T_b, S)
]

---

## 1️⃣ 子弹飞行时间

```cpp
T_b = d_k / v_bullet
```

---

## 2️⃣ 击中时刻角度

```cpp
θ_hit = θ_k + ω_k * T_b
```

---

## 3️⃣ 最优击打角（含提前量）

```cpp
θ_center = θ_ideal + ω_k * T_delay
```

👉 T_delay = 系统延迟 + 发射延迟

---

## 4️⃣ 角度误差（必须wrap）

```cpp
dθ = normalize_angle(θ_hit - θ_center)
```

---

## 5️⃣ 有效窗口 σ（关键）

```cpp
σ_theta = S / d_k
σ_eff = sqrt(σ_theta^2 + σ_sys^2)
σ = clamp(σ_eff, σ_min, σ_max)
```

👉 推荐：

```text
σ_min ≈ 0.03~0.08 rad
σ_sys ≈ 0.02~0.05 rad
```

---

## 6️⃣ 角度命中概率

```cpp
p_angle = exp(-0.5 * dθ^2 / σ^2)
```

---

## 7️⃣ 角速度惩罚

```cpp
p_omega = exp(-k_omega * abs(ω_k))
```

---

## ⭐ 最终命中概率

```cpp
r_k = p_angle * p_omega
```

---

# 四、多装甲板建模（重要）

假设 N 块板：

```cpp
θ_k_i = θ_k + i * 2π/N
```

---

## ⭐ softmax融合（推荐）

```cpp
r_k = sum_i exp(- dθ_i^2 / σ^2)
```

或：

```cpp
r_k = max_i(...)
```

👉 推荐 sum（更平滑，不跳板）

---

# 五、权重构造（保证QP）

```cpp
w_k = 1 + α * r_k
```

---

## 平滑（必须）

```cpp
w_k = 0.7 * w_k_prev + 0.3 * w_k_new
```

---

## 限幅

```cpp
w_k = clamp(w_k, 1, w_max)
```

推荐：

```text
α ≈ 2~5
w_max ≈ 5~10
```

---

# 六、MPC（QP形式）

---

## 1️⃣ 代价函数

[
J =
\sum_{k=0}^{N}
w_k \cdot |x_k - x_k^{ref}|^2

* \sum u_k^T R u_k
* \sum \rho |\Delta u_k|^2
  ]

---

## 2️⃣ 关键点

* (w_k)：已知常数 ✔ → 仍是QP
* 加 (\Delta u)：防抖 ✔

---

## 3️⃣ 参考轨迹

```cpp
x_ref_k = [θ_k, pitch_ref, 0, 0]
```

---

# 七、关键增强：选“最佳击打时刻”

---

## 1️⃣ 找峰值

```cpp
k* = argmax(r_k)
```

---

## 2️⃣ 强化该时刻

```cpp
w_k* += w_boost
```

或：

加入终端项：

[
J += (x_{k*} - x_{ref})^T Q_f (x_{k*} - x_{ref})
]

---

👉 效果：

* 不只是“偏向”
* 而是“主动对齐最佳时机”

---

# 八、完整伪代码（核心）

```cpp
for each control cycle:

    // 1. 预测
    for k in 0..N:
        θ_k = θ0 + ω * k * dt
        d_k = d0

        T_b = d_k / v_bullet
        θ_hit = θ_k + ω * T_b

        θ_center = θ_ideal + ω * T_delay

        dθ = normalize(θ_hit - θ_center)

        σ = compute_sigma(d_k, ω)

        p_angle = exp(-0.5 * dθ*dθ / (σ*σ))
        p_omega = exp(-kω * abs(ω))

        r_k = p_angle * p_omega

    // 2. 多板融合（可选）
    r_k = fuse_armor_boards(...)

    // 3. 权重
    w_k = 1 + α * r_k
    w_k = smooth(w_k)

    // 4. 找最佳时刻
    k_star = argmax(r_k)
    w_k_star += boost

    // 5. MPC(QP)
    solve:
        min Σ w_k * error^2 + control + Δu

    // 6. 输出
    apply u_0
```

---

# 九、工程关键参数（直接给你）

```text
N = 10~20
dt = 0.01~0.02

α = 3~5
k_omega = 0.3~1.0

σ_min = 0.05
σ_sys = 0.02~0.05

ρ (Δu) = 中等偏大（关键）
```

---

# 九点五、落地实现说明（当前代码）

本仓库实现采用配置项 `controller.mpc.weighting.*` 控制加权，仅对 Q (状态跟踪) 生效。
以下功能保持未实现以降低复杂度：

- 多装甲板融合 `fuse_armor_boards(...)`
- 最佳时刻强化 `k*` / `w_k* += boost`

参考参数映射：

```text
controller.mpc.weighting.enable
controller.mpc.weighting.alpha
controller.mpc.weighting.k_omega
controller.mpc.weighting.sigma_min
controller.mpc.weighting.sigma_max
controller.mpc.weighting.sigma_sys
controller.mpc.weighting.target_size
controller.mpc.weighting.delay_s
controller.mpc.weighting.max_w
controller.mpc.weighting.smooth_alpha
controller.mpc.weighting.min_distance
controller.mpc.weighting.sigma_beta
controller.mpc.weighting.gamma
```

---

# 十、最终效果（你会得到什么）

这套系统会自动表现出：

---

## ✅ 自动提前量（lead）

不用手写补偿

---

## ✅ 自动选板

softmax完成

---

## ✅ 自动选时机

k*机制完成

---

## ✅ 高速稳定

σ收缩 + ω惩罚

---

## ✅ 不抖

Δu + 权重平滑

---

# 十一、一句话总结

> 这是一个 **“命中概率场 + 动态加权MPC + 时机选择” 的完整闭环系统**，本质是在用概率引导控制，实现连续版的“选板 + 等时机 + 精确打击”。
