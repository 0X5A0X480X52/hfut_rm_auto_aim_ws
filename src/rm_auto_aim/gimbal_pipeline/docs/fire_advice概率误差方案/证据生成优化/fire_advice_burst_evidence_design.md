# Fire Advice：基于 Burst 统计证据的开火决策扩展设计

## 一、设计背景

当前 `fire_advice` 已具备：

```text
未来窗口命中概率估计
→ sigma 点传播
→ softmax/max 融合
→ EMA/积分门控
→ fire_state 输出
```

其核心流程为：

```text
p_window
→ score_
→ fire_state
```

其中 `score_` 为：

```text
lowpass / integrator
```

形式的概率累计量。

现有实现默认假设：

```text
开火行为近似连续、细粒度、可精确控制
```

但实际下位机表现为：

```text
拨盘启动存在延迟；
一旦越过临界，可能立即连续打出 1~5 发。
```

因此：

```text
当前 fire_state 的语义：
“这一帧是否应该开火”
```

与实际系统：

```text
“现在是否值得提交一次 burst”
```

并不一致。

因此，本设计不修改现有概率估计、sigma 点传播、弹道建模、法向速度计算等实现，仅在现有 `p_window → gate` 后新增：

```text
Burst 概率语义
滑窗统计证据
能垒触发
最小开火时长
冷却期
风险温度 T
```

用于替代当前简单的：

```text
EMA / integrator → fire_state
```

逻辑。

---

# 二、设计目标

新增机制需满足：

```text
1. fire_advice 语义从“单帧开火”转为“提交一次 burst”
2. 对未来 burst 效果做统计评估
3. 避免瞬时 p_hit 峰值导致误触发
4. 支持拨盘启动后不可精确控弹
5. 保持现有 ProbabilityEngine 主体实现不变
6. 保持现有 p_hit / p_window 生成逻辑兼容
```

---

# 三、系统整体结构

新增后的结构：

```text
future p_hit_i
→ burst probability Pb
→ evidence ℓ
→ sliding evidence window E
→ normalized strength S
→ barrier decision
→ FIRE_COMMIT state
```

其中：

```text
p_hit_i：
第 i 发未来命中概率

Pb：
一次 burst 的综合成功概率

ℓ：
单帧统计证据

E：
滑窗累计证据

S：
归一化证据强度

T：
风险偏好温度

FIRE_COMMIT：
burst 提交状态
```

---

# 四、Burst 概率建模

## 4.1 单发命中概率

保留当前实现。

对于第 i 发：

```math
p_i = P(\text{bullet } i \text{ hits})
```

当前实现中：

```text
二维高斯误差
+
矩形命中区域积分
```

已具备统计意义。

因此：

```text
不修改现有 p_hit 计算
```

---

## 4.2 Burst 命中语义

当前实现：

```text
window_fusion = max
```

更偏向：

```text
“未来某一瞬间是否可能命中”
```

但实际系统为：

```text
一次启动后可能连续出 1~5 发
```

因此新增：

```text
burst probability Pb
```

定义：

设：

```math
Y_i \sim \text{Bernoulli}(p_i)
```

其中：

```math
Y_i = 1
```

表示第 i 发命中。

一次 burst 命中数：

```math
K = \sum_{i=1}^{N} Y_i
```

定义：

```math
P_b = P(K \ge m)
```

其中：

```text
N：
burst 预计发射数

m：
至少有效命中的最低发数
```

推荐：

```yaml
burst_bullet_count: 5
min_hit_count: 1
```

---

# 五、统计证据

## 5.1 参考概率 P0

定义：

```math
P_0
```

为：

```text
最低可接受 burst 命中率
```

推荐：

```yaml
reference_probability_p0: 0.60
```

---

## 5.2 单帧统计证据

定义：

```math
\ell_k
=
\log
\frac{
P_b^{(k)}(1-P_0)
}{
P_0(1-P_b^{(k)})
}
```

即：

```text
当前 burst 成功概率
相对于最低期望命中率 P0
的对数 odds ratio
```

语义：

```text
ℓ > 0：
当前帧支持开火

ℓ < 0：
当前帧反对开火
```

---

## 5.3 数值稳定性

由于：

```math
P_b \in [0,1]
```

而：

```math
\log\frac{P_b}{1-P_b}
```

在：

```text
Pb → 0 或 1
```

时会发散。

因此：

```math
P_b'
=
\mathrm{clip}(P_b,\epsilon,1-\epsilon)
```

推荐：

```yaml
epsilon: 1e-3
```

同时：

```math
\ell_k
\leftarrow
\mathrm{clip}(\ell_k,-L,L)
```

推荐：

```yaml
log_evidence_clip: 2.0
```

于是：

```math
\ell_k \in [-L,L]
```

---

# 六、滑窗统计证据

## 6.1 滑窗累计

定义：

```math
E_k
=
\sum_{j=k-M+1}^{k}
\ell_j
```

其中：

```math
M
=
\frac{T_w}{\Delta t}
```

即：

```text
最近窗口内累计的“支持开火”的统计证据
```

推荐：

```yaml
evidence_window_ms: 50
```

---

## 6.2 有界性

由于：

```math
\ell_j \in [-L,L]
```

因此：

```math
E_k \in [-ML,ML]
```

于是：

```text
总证据天然有界
```

---

# 七、证据百分比强度

定义：

```math
S_k
=
\frac{E_k+ML}{2ML}
```

于是：

```math
S_k \in [0,1]
```

语义：

```text
S≈0：
强烈反对开火

S≈0.5：
证据平衡

S≈1：
强烈支持开火
```

---

# 八、风险温度 T

定义：

```math
T \in [0,1]
```

其语义不是：

```text
命中率
```

而是：

```text
系统风险偏好
```

即：

```text
系统有多愿意承担 burst 风险
```

---

## 8.1 T 的作用

T 控制：

```text
1. 激发能垒
2. 开火维持阈值
3. 复位阈值
```

---

## 8.2 激发阈值

定义：

```math
\theta_{\text{on}}(T)
=
\theta_{\text{cold}}
-
(\theta_{\text{cold}}-\theta_{\text{hot}})T
```

推荐：

```yaml
theta_on_cold: 0.90
theta_on_hot: 0.75
```

---

## 8.3 维持与复位

定义：

```math
\theta_{\text{hold}}(T)
=
\theta_{\text{hold,cold}}
-
(\theta_{\text{hold,cold}}-\theta_{\text{hold,hot}})T
```

```math
\theta_{\text{reset}}(T)
=
\theta_{\text{reset,cold}}
-
(\theta_{\text{reset,cold}}-\theta_{\text{reset,hot}})T
```

推荐：

```yaml
theta_hold_cold: 0.70
theta_hold_hot: 0.55

theta_reset_cold: 0.45
theta_reset_hot: 0.35
```

---

# 九、Burst 提交状态机

新增：

```text
IDLE
FIRE_COMMIT
COOLDOWN
```

---

## 9.1 IDLE

若：

```math
S_k \ge \theta_{\text{on}}(T)
```

则：

```text
进入 FIRE_COMMIT
```

---

## 9.2 FIRE_COMMIT

进入后：

```text
至少维持 min_fire_ms
```

推荐：

```yaml
min_fire_ms: 20
```

最小开火时间结束后：

若：

```math
S_k \ge \theta_{\text{hold}}(T)
```

则：

```text
继续开火
```

否则：

```text
退出并进入 COOLDOWN
```

---

## 9.3 COOLDOWN

冷却阶段：

```text
禁止再次触发
```

持续：

```yaml
cooldown_ms: 80
```

冷却结束后：

若：

```math
S_k \le \theta_{\text{reset}}(T)
```

则：

```text
回到 IDLE
```

---

# 十、与当前实现的关系

本设计：

```text
不修改：
- p_hit
- sigma point
- covariance
- ballistic model
- normal velocity
- tracker uncertainty
```

仅替换：

```text
p_window → fire_state
```

部分。

即：

当前：

```text
p_window
→ lowpass/integrator
→ fire_state
```

替换为：

```text
p_window
→ Pb
→ log evidence
→ sliding evidence
→ normalized strength
→ burst commit state machine
→ fire_state
```

---

# 十一、推荐参数

```yaml
probability:

  future_window_ms: 40.0
  future_step_ms: 2.0

  burst:
    burst_bullet_count: 5
    min_hit_count: 1

  evidence:
    reference_probability_p0: 0.60
    evidence_window_ms: 50
    log_evidence_clip: 2.0

  temperature:
    value: 0.5

    theta_on_cold: 0.90
    theta_on_hot: 0.75

    theta_hold_cold: 0.70
    theta_hold_hot: 0.55

    theta_reset_cold: 0.45
    theta_reset_hot: 0.35

  commit:
    min_fire_ms: 20
    cooldown_ms: 80
```

---

# 十二、最终语义总结

系统不断估计：

```math
P_b
=
P(\text{当前提交一次 burst 后未来有效命中})
```

并与：

```math
P_0
```

比较，得到：

```text
支持 / 反对 burst 的统计证据
```

系统在滑动窗口内累计这些证据：

```math
E_k
```

再归一化为：

```math
S_k \in [0,1]
```

表示：

```text
最近窗口内支持 burst 的证据强度百分比
```

当：

```math
S_k
```

跨过由：

```math
T
```

控制的能垒时：

```text
提交一次 burst
```

并通过：

```text
最小开火时长
+
冷却期
```

适配实际下位机：

```text
启动后不可精确控弹
```

的特性。
