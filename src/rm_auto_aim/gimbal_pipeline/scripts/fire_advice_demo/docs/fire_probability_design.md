# 基于概率弹道与协方差退化模型的开火决策设计文档

## 1. 场景描述

当前自瞄系统中，`fire_advice` 主要由几何阈值生成：

```cpp
fire_advice =
    abs(yaw_error) < yaw_threshold &&
    abs(pitch_error) < pitch_threshold;
```

或者使用椭圆门限：

```math
\left(\frac{\Delta yaw}{yaw_{th}}\right)^2
+
\left(\frac{\Delta pitch}{pitch_{th}}\right)^2
< 1
```

该方法存在一个核心问题：

```text
fire_advice 是离散 bool 信号，容易因噪声在 true / false 之间抖动。
```

而下位机拨盘发弹通常需要连续的开火建议窗口。如果上位机输出的 `fire_advice` 频繁抖动，即使理论上某些瞬间满足击中条件，也可能无法达到下位机发弹的最小持续触发要求。

因此，需要将开火建议从：

```text
瞬时二值判定
```

升级为：

```text
连续命中概率 + 时间窗评分 + 滞回状态机
```

---

## 2. 设计目标

本方案目标如下：

1. 将 `fire_advice` 从离散阈值判断升级为连续概率判断；
2. 在枪管/云台系下建立简化弹道模型；
3. 将弹道误差建模为随飞行时间增长的协方差；
4. 将 tracker 的目标不确定性投影到装甲板平面；
5. 计算弹着点落入装甲板矩形区域的概率；
6. 对未来短时间窗内的命中概率进行融合；
7. 使用低通、积分和滞回生成稳定的 `fire_advice`；
8. 初版满足 120 FPS，计算耗时尽量控制在 1–2 ms 以内；
9. 预留 sigma-point 扩展，用于建模弹速误差和发射延时误差。

---

## 3. 总体方案

整体流程如下：

```text
tracker 在 odom 系预测目标
        ↓
选取未来候选发射时刻
        ↓
定义发射瞬间冻结的 barrel/shooter frame
        ↓
将预测装甲板转到该枪管系
        ↓
在枪管系计算理想弹道均值
        ↓
构造随飞行时间增长的弹道协方差
        ↓
将弹道协方差和 tracker 协方差投影到装甲板平面
        ↓
计算弹着点落入装甲板矩形区域的概率 P_hit
        ↓
对未来时间窗融合得到 P_window
        ↓
低通 / 积分 / 滞回
        ↓
输出 fire_advice
```

---

## 4. 坐标系与时间定义

### 4.1 坐标系

定义：

| 符号 | 含义 |
|---|---|
| \(O\) | odom/world 坐标系 |
| \(B_s\) | 发射时刻 \(t_s\) 冻结的枪管坐标系 |
| \(A\) | 装甲板局部坐标系 |

其中 \(B_s\) 是非常关键的概念。

在候选发射时刻 \(t_s\)，取枪管坐标系的姿态和位置作为一个冻结坐标系：

```text
B_s = shooter_link 在 t_s 时刻的坐标系
```

子弹出膛后不再跟随云台旋转，因此弹道计算应该在这个冻结的 \(B_s\) 中进行。

### 4.2 时间变量

设当前时刻为：

```math
t_0
```

未来候选发射时刻为：

```math
t_s = t_0 + \tau
```

其中：

```math
\tau \in \{0,\Delta \tau,2\Delta \tau,\cdots,T_{window}\}
```

例如：

```text
τ = 0 ms, 10 ms, 20 ms, 30 ms, 40 ms
```

子弹飞行时间为：

```math
t_f
```

系统延迟为：

```math
t_{sys}
```

下位机发射延迟为：

```math
t_d
```

则预测命中时刻为：

```math
t_{impact}
=
t_s + t_{sys} + t_d + t_f
```

tracker 应预测装甲板在 \(t_{impact}\) 时刻的状态。

---

## 5. 枪管系弹道均值模型

### 5.1 枪管系定义

在 \(B_s\) 中定义：

```text
x 轴：枪管前向
y 轴：水平横向，主要对应 yaw 误差
z 轴：竖直/俯仰方向，主要对应 pitch 补偿
```

理想初速度为：

```math
{}^{B_s}v_0 =
\begin{bmatrix}
v_0\\
0\\
0
\end{bmatrix}
```

其中 \(v_0\) 是弹丸初速度。

### 5.2 重力在枪管系中的表达

在 odom 系中，重力为：

```math
{}^O g =
\begin{bmatrix}
0\\
0\\
-g
\end{bmatrix}
```

将其变换到发射瞬间冻结的枪管系：

```math
{}^{B_s}g
=
{}^{B_s}R_O \ {}^O g
```

记：

```math
{}^{B_s}g =
\begin{bmatrix}
g_x\\
g_y\\
g_z
\end{bmatrix}
```

在地面近似水平、yaw 轴近似竖直的情况下，通常有：

```math
g_y \approx 0
```

如果枪管 pitch 角为 \(\theta\)，也可以近似写成：

```math
g_x \approx -g\sin\theta
```

```math
g_z \approx -g\cos\theta
```

但工程上更建议直接通过 TF 计算：

```math
{}^{B_s}g
=
{}^{B_s}R_O
\begin{bmatrix}
0\\
0\\
-g
\end{bmatrix}
```

这样可以避免 pitch 符号和安装误差带来的问题。

### 5.3 子弹位置均值

设枪口在 \(B_s\) 中的位置为：

```math
{}^{B_s}p_0
```

通常可以近似为：

```math
{}^{B_s}p_0 =
\begin{bmatrix}
0\\
0\\
0
\end{bmatrix}
```

忽略空气阻力时，子弹在 \(t\) 时刻的位置均值为：

```math
{}^{B_s}\mu_p(t)
=
{}^{B_s}p_0
+
\begin{bmatrix}
v_0 t\\
0\\
0
\end{bmatrix}
+
\frac{1}{2}
{}^{B_s}g t^2
```

展开为：

```math
{}^{B_s}\mu_p(t)
=
\begin{bmatrix}
p_{0x} + v_0 t + \frac{1}{2}g_x t^2\\
p_{0y} + \frac{1}{2}g_y t^2\\
p_{0z} + \frac{1}{2}g_z t^2
\end{bmatrix}
```

在水平地面简化条件下：

```math
g_y \approx 0
```

因此：

```math
\mu_x(t) \approx p_{0x}+v_0t+\frac{1}{2}g_x t^2
```

```math
\mu_y(t) \approx p_{0y}
```

```math
\mu_z(t) \approx p_{0z}+\frac{1}{2}g_z t^2
```

这对应：

```text
沿枪管方向的运动
+
pitch 方向的重力补偿
```

### 5.4 子弹速度均值

速度为位置对时间求导：

```math
{}^{B_s}\mu_v(t)
=
\frac{d}{dt}
{}^{B_s}\mu_p(t)
```

因此：

```math
{}^{B_s}\mu_v(t)
=
\begin{bmatrix}
v_0\\
0\\
0
\end{bmatrix}
+
{}^{B_s}g t
```

即：

```math
{}^{B_s}\mu_v(t)
=
\begin{bmatrix}
v_0 + g_x t\\
g_y t\\
g_z t
\end{bmatrix}
```

该速度可以用于计算入射法向速度：

```math
v_n(t)
=
\max
\left(
0,
-
{}^{B_s}\mu_v(t)^T
{}^{B_s}n_a(t)
\right)
```

其中 \(n_a\) 是装甲板法向。

---

## 6. 飞行时间求解

### 6.1 简化飞行时间

若目标装甲板中心在 \(B_s\) 中为：

```math
{}^{B_s}c_a =
\begin{bmatrix}
x_a\\
y_a\\
z_a
\end{bmatrix}
```

可以使用沿枪管方向距离 \(x_a\) 估计飞行时间：

```math
t_f \approx \frac{x_a}{v_0}
```

该形式计算最快，适合初版。

### 6.2 考虑 \(g_x\) 的飞行时间

更准确地，子弹沿枪管前向的位置满足：

```math
x_a
=
p_{0x}
+
v_0t_f
+
\frac{1}{2}g_x t_f^2
```

整理为：

```math
\frac{1}{2}g_x t_f^2
+
v_0 t_f
+
(p_{0x}-x_a)
=
0
```

若 \(p_{0x}=0\)，则：

```math
\frac{1}{2}g_x t_f^2
+
v_0t_f
-
x_a
=
0
```

取正根：

```math
t_f
=
\frac{
-v_0
+
\sqrt{
v_0^2 + 2g_x x_a
}
}{
g_x
}
```

当 \(g_x \to 0\) 时，退化为：

```math
t_f = \frac{x_a}{v_0}
```

工程中可写成：

```cpp
if (abs(gx) < eps) {
    tf = x / v0;
} else {
    tf = (-v0 + sqrt(v0 * v0 + 2.0f * gx * x)) / gx;
}
```

需要注意判别式：

```math
v_0^2 + 2g_xx_a \ge 0
```

若判别式异常，应退化到：

```math
t_f = \frac{x_a}{v_0}
```

---

## 7. 弹道随机误差模型

### 7.1 位置随机模型

将真实子弹位置建模为：

```math
{}^{B_s}p_b(t)
=
{}^{B_s}\mu_p(t)
+
\varepsilon_p(t)
```

其中：

```math
\varepsilon_p(t)
\sim
\mathcal{N}
\left(
0,
\Sigma_p^{B_s}(t)
\right)
```

\(\Sigma_p^{B_s}(t)\) 是子弹位置误差协方差。

### 7.2 协方差退化模型

初版可令：

```math
\Sigma_p^{B_s}(t)
=
\begin{bmatrix}
\sigma_x^2(t) & 0 & 0\\
0 & \sigma_y^2(t) & 0\\
0 & 0 & \sigma_z^2(t)
\end{bmatrix}
```

其中：

```math
\sigma_x(t)
=
\sigma_{x0}
+
k_x t
```

```math
\sigma_y(t)
=
\sigma_{y0}
+
k_y t
```

```math
\sigma_z(t)
=
\sigma_{z0}
+
k_z t
```

含义如下：

| 参数 | 含义 |
|---|---|
| \(\sigma_{x0}\) | 初始前向误差 |
| \(\sigma_{y0}\) | 初始横向误差 |
| \(\sigma_{z0}\) | 初始竖向误差 |
| \(k_x\) | 前向误差随飞行时间增长速率 |
| \(k_y\) | 横向误差随飞行时间增长速率 |
| \(k_z\) | 竖向误差随飞行时间增长速率 |

对于命中装甲板而言，主要关注：

```math
\sigma_y(t), \sigma_z(t)
```

因为它们对应横向和竖向弹着误差。

### 7.3 二次退化形式

如果希望远距离误差增长更快，也可以使用：

```math
\sigma_y^2(t)
=
\sigma_{y0}^2
+
q_y t^2
```

```math
\sigma_z^2(t)
=
\sigma_{z0}^2
+
q_z t^2
```

即：

```math
\Sigma_p^{B_s}(t)
=
\Sigma_{p0}
+
Q_p t^2
```

其中：

```math
\Sigma_{p0}
=
\begin{bmatrix}
\sigma_{x0}^2 & 0 & 0\\
0 & \sigma_{y0}^2 & 0\\
0 & 0 & \sigma_{z0}^2
\end{bmatrix}
```

```math
Q_p
=
\begin{bmatrix}
q_x & 0 & 0\\
0 & q_y & 0\\
0 & 0 & q_z
\end{bmatrix}
```

初版建议使用标准差线性增长：

```math
\sigma(t)=\sigma_0+kt
```

因为参数更直观。

---

## 8. 装甲板状态建模

tracker 在 odom 系中预测装甲板状态。

设装甲板中心在 odom 系为：

```math
{}^O c_a(t_{impact})
```

装甲板姿态为：

```math
{}^O R_a(t_{impact})
```

装甲板宽高为：

```math
w,\ h
```

将装甲板变换到发射瞬间冻结的枪管系 \(B_s\)：

```math
{}^{B_s}c_a
=
{}^{B_s}R_O
\left(
{}^O c_a
-
{}^O p_{B_s}
\right)
```

```math
{}^{B_s}R_a
=
{}^{B_s}R_O
{}^O R_a
```

装甲板局部方向为：

```math
{}^{B_s}r_a =
{}^{B_s}R_a
\begin{bmatrix}
1\\0\\0
\end{bmatrix}
```

```math
{}^{B_s}u_a =
{}^{B_s}R_a
\begin{bmatrix}
0\\1\\0
\end{bmatrix}
```

```math
{}^{B_s}n_a =
{}^{B_s}R_a
\begin{bmatrix}
0\\0\\1
\end{bmatrix}
```

其中：

| 向量 | 含义 |
|---|---|
| \(r_a\) | 装甲板水平方向 |
| \(u_a\) | 装甲板竖直方向 |
| \(n_a\) | 装甲板法向 |

---

## 9. 弹着点投影到装甲板平面

子弹均值位置与装甲板中心的差为：

```math
\Delta p(t_f)
=
{}^{B_s}\mu_p(t_f)
-
{}^{B_s}c_a(t_{impact})
```

将其投影到装甲板局部平面：

```math
e_u
=
{}^{B_s}r_a^T
\Delta p
```

```math
e_v
=
{}^{B_s}u_a^T
\Delta p
```

于是二维误差向量为：

```math
e_{uv}
=
\begin{bmatrix}
e_u\\
e_v
\end{bmatrix}
=
\begin{bmatrix}
{}^{B_s}r_a^T\\
{}^{B_s}u_a^T
\end{bmatrix}
\Delta p
```

定义投影矩阵：

```math
J_a
=
\begin{bmatrix}
{}^{B_s}r_a^T\\
{}^{B_s}u_a^T
\end{bmatrix}
```

则：

```math
e_{uv}
=
J_a
\Delta p
```

---

## 10. 弹道协方差投影到装甲板平面

子弹位置误差在 \(B_s\) 中为：

```math
\varepsilon_p
\sim
\mathcal{N}
(0,\Sigma_p^{B_s})
```

投影到装甲板局部平面后：

```math
\varepsilon_{uv}
=
J_a
\varepsilon_p
```

因此：

```math
\Sigma_{bullet}^{uv}
=
J_a
\Sigma_p^{B_s}
J_a^T
```

这表示弹道误差在装甲板平面内造成的二维弹着散布。

---

## 11. Tracker 不确定性投影

如果 tracker 能输出装甲板中心位置协方差：

```math
{}^O P_a
\in
\mathbb{R}^{3\times3}
```

先变换到枪管系：

```math
{}^{B_s}P_a
=
{}^{B_s}R_O
{}^O P_a
({}^{B_s}R_O)^T
```

再投影到装甲板平面：

```math
\Sigma_{tracker}^{uv}
=
J_a
{}^{B_s}P_a
J_a^T
```

如果 tracker 暂时没有协方差输出，可以先用固定值：

```math
{}^O P_a
=
\begin{bmatrix}
\sigma_{px}^2 & 0 & 0\\
0 & \sigma_{py}^2 & 0\\
0 & 0 & \sigma_{pz}^2
\end{bmatrix}
```

---

## 12. 总误差协方差

初版总协方差为：

```math
\Sigma_{total}^{uv}
=
\Sigma_{bullet}^{uv}
+
\Sigma_{tracker}^{uv}
```

如果后续引入控制误差、外参误差、sigma 点传播误差，可以扩展为：

```math
\Sigma_{total}^{uv}
=
\Sigma_{bullet}^{uv}
+
\Sigma_{tracker}^{uv}
+
\Sigma_{control}^{uv}
+
\Sigma_{sigma}^{uv}
```

---

## 13. 命中概率建模

弹着点在装甲板平面内的误差可建模为：

```math
X_{uv}
\sim
\mathcal{N}
\left(
e_{uv},
\Sigma_{total}^{uv}
\right)
```

装甲板矩形区域为：

```math
\mathcal{R}
=
\left\{
(u,v)
\mid
-\frac{w}{2} < u < \frac{w}{2},
-\frac{h}{2} < v < \frac{h}{2}
\right\}
```

命中概率为二维高斯分布落入矩形区域的概率：

```math
P_{hit}
=
P(X_{uv}\in\mathcal{R})
```

即：

```math
P_{hit}
=
\int_{-w/2}^{w/2}
\int_{-h/2}^{h/2}
\mathcal{N}
\left(
\begin{bmatrix}
u\\v
\end{bmatrix};
e_{uv},
\Sigma_{total}^{uv}
\right)
du\,dv
```

这是该方案最核心的概率建模公式。

---

## 14. 独立近似下的快速计算

为了满足实时性，初版可以忽略 \(u,v\) 的相关性。

设：

```math
\Sigma_{total}^{uv}
=
\begin{bmatrix}
\sigma_u^2 & 0\\
0 & \sigma_v^2
\end{bmatrix}
```

则：

```math
P_{hit}
=
P_u P_v
```

其中：

```math
P_u
=
\Phi
\left(
\frac{\frac{w}{2}-e_u}{\sigma_u}
\right)
-
\Phi
\left(
\frac{-\frac{w}{2}-e_u}{\sigma_u}
\right)
```

```math
P_v
=
\Phi
\left(
\frac{\frac{h}{2}-e_v}{\sigma_v}
\right)
-
\Phi
\left(
\frac{-\frac{h}{2}-e_v}{\sigma_v}
\right)
```

\(\Phi(x)\) 为标准正态分布 CDF：

```math
\Phi(x)
=
\frac{1}{2}
\left[
1+
\operatorname{erf}
\left(
\frac{x}{\sqrt{2}}
\right)
\right]
```

工程实现：

```cpp
inline float normalCDF(float x) {
    return 0.5f * (1.0f + std::erf(x * 0.70710678f));
}

float probabilityInside1D(float mean, float sigma, float half_size) {
    sigma = std::max(sigma, 1e-4f);

    float upper = ( half_size - mean) / sigma;
    float lower = (-half_size - mean) / sigma;

    return normalCDF(upper) - normalCDF(lower);
}

float computeHitProbability(
    const Eigen::Vector2f& e_uv,
    float sigma_u,
    float sigma_v,
    float width,
    float height
) {
    float pu = probabilityInside1D(e_uv.x(), sigma_u, width  * 0.5f);
    float pv = probabilityInside1D(e_uv.y(), sigma_v, height * 0.5f);

    return std::clamp(pu * pv, 0.0f, 1.0f);
}
```

---

## 15. 法向速度加权

如果希望考虑子弹入射角，可以计算法向有效速度。

子弹速度为：

```math
{}^{B_s}\mu_v(t_f)
```

装甲板法向为：

```math
{}^{B_s}n_a
```

定义有效法向速度：

```math
v_n
=
\max
\left(
0,
-
{}^{B_s}\mu_v(t_f)^T
{}^{B_s}n_a
\right)
```

若子弹正向撞击装甲板，则 \(v_n\) 较大；若掠射或方向不合适，则 \(v_n\) 较小。

定义归一化权重：

```math
w_n
=
\operatorname{clip}
\left(
\frac{v_n}{v_{ref}},
w_{min},
1
\right)
```

增强后的命中评分为：

```math
P_{hit}' =
P_{hit} \cdot w_n
```

初版可以先不启用，或者只作为调试输出。

---

## 16. 时间窗命中概率融合

对于多个候选发射时刻：

```math
\tau_i
\in
\{0,\Delta\tau,2\Delta\tau,\cdots,T_{window}\}
```

分别计算：

```math
P_i = P_{hit}(\tau_i)
```

由于同一发子弹在不同时间点的事件不是完全独立的，不建议简单求和：

```math
\sum_i P_i
```

初版推荐使用最大值：

```math
P_{window}
=
\max_i P_i
```

含义是：

```text
未来短时间窗内是否存在一个较好的开火机会。
```

如果希望更平滑，可以使用 softmax-like 融合：

```math
P_{window}
=
\frac{
\sum_i P_i \exp(\beta P_i)
}{
\sum_i \exp(\beta P_i)
}
```

其中 \(\beta\) 越大，越接近 max。

初版建议：

```math
P_{window}
=
\max_i P_i
```

---

## 17. Fire Score 低通与积分

### 17.1 指数低通

定义当前帧开火评分：

```math
S_k
```

使用低通滤波：

```math
S_k
=
\alpha S_{k-1}
+
(1-\alpha)P_{window,k}
```

其中：

```math
0<\alpha<1
```

推荐：

```text
alpha = 0.8 ~ 0.95
```

高帧率下可以取更大值，使信号更平滑。

### 17.2 积分式评分

为了更贴近下位机拨盘的连续触发需求，也可以使用积分式评分：

```math
S_k
=
\operatorname{clip}
\left(
S_{k-1}
+
\Delta t
\left[
c_+
\max(0,P_{window,k}-P_{base})
-
c_-
\max(0,P_{base}-P_{window,k})
\right],
0,
1
\right)
```

其中：

| 参数 | 含义 |
|---|---|
| \(P_{base}\) | 基础概率阈值 |
| \(c_+\) | 积分上升速度 |
| \(c_-\) | 积分下降速度 |
| \(\Delta t\) | 当前帧间隔 |

该方法适合解决：

```text
瞬时满足开火条件，但持续时间不足，导致拨盘无法发弹
```

的问题。

---

## 18. 滞回状态机

最终输出 `fire_advice` 不直接由 \(P_{window}\) 决定，而由 \(S_k\) 决定：

```math
fire\_advice =
\begin{cases}
true, & S_k > S_{on}\\
false, & S_k < S_{off}\\
保持不变, & S_{off} \le S_k \le S_{on}
\end{cases}
```

其中：

```math
S_{on} > S_{off}
```

例如：

```yaml
fire_gate:
  fire_on_th: 0.65
  fire_off_th: 0.35
```

这样可以避免 `fire_advice` 在阈值附近频繁抖动。

---

## 19. Sigma-Point 扩展：弹速误差与发射延迟误差

初版不必须使用 sigma point。但弹速误差和发射延迟误差很适合作为扩展项。

### 19.1 扩展误差状态

定义误差状态：

```math
x =
\begin{bmatrix}
\Delta v_0\\
\Delta t_d
\end{bmatrix}
```

其中：

| 状态 | 含义 |
|---|---|
| \(\Delta v_0\) | 弹速误差 |
| \(\Delta t_d\) | 发射延迟误差 |

假设：

```math
x
\sim
\mathcal{N}
(0,\Sigma_x)
```

其中：

```math
\Sigma_x
=
\begin{bmatrix}
\sigma_{v_0}^2 & 0\\
0 & \sigma_{t_d}^2
\end{bmatrix}
```

若认为弹速误差和延迟误差存在相关性，也可以写成：

```math
\Sigma_x
=
\begin{bmatrix}
\sigma_{v_0}^2 & \rho\sigma_{v_0}\sigma_{t_d}\\
\rho\sigma_{v_0}\sigma_{t_d} & \sigma_{t_d}^2
\end{bmatrix}
```

### 19.2 Sigma 点生成

状态维度：

```math
n=2
```

sigma 点数量：

```math
2n+1=5
```

Unscented Transform 中：

```math
\lambda
=
\alpha^2(n+\kappa)-n
```

sigma 点为：

```math
\chi_0 = \mu_x
```

```math
\chi_i
=
\mu_x
+
\left[
\sqrt{(n+\lambda)\Sigma_x}
\right]_i,
\quad i=1,\cdots,n
```

```math
\chi_{i+n}
=
\mu_x
-
\left[
\sqrt{(n+\lambda)\Sigma_x}
\right]_i,
\quad i=1,\cdots,n
```

对应权重：

```math
W_0^{(m)}
=
\frac{\lambda}{n+\lambda}
```

```math
W_0^{(c)}
=
\frac{\lambda}{n+\lambda}
+
1-\alpha^2+\beta
```

```math
W_i^{(m)}
=
W_i^{(c)}
=
\frac{1}{2(n+\lambda)},
\quad i=1,\cdots,2n
```

工程初版也可以简化为固定 5 点：

```math
\chi_0 =
\begin{bmatrix}
0\\0
\end{bmatrix}
```

```math
\chi_1 =
\begin{bmatrix}
+\sigma_{v_0}\\0
\end{bmatrix},
\quad
\chi_2 =
\begin{bmatrix}
-\sigma_{v_0}\\0
\end{bmatrix}
```

```math
\chi_3 =
\begin{bmatrix}
0\\+\sigma_{t_d}
\end{bmatrix},
\quad
\chi_4 =
\begin{bmatrix}
0\\-\sigma_{t_d}
\end{bmatrix}
```

### 19.3 Sigma 点传播到装甲板平面

对于第 \(j\) 个 sigma 点：

```math
\chi_j
=
\begin{bmatrix}
\Delta v_{0,j}\\
\Delta t_{d,j}
\end{bmatrix}
```

修正弹速：

```math
v_{0,j}
=
v_0
+
\Delta v_{0,j}
```

修正发射延迟：

```math
t_{d,j}
=
t_d
+
\Delta t_{d,j}
```

重新求飞行时间：

```math
t_{f,j}
=
solveFlightTime(x_a,v_{0,j},g_x)
```

命中时刻：

```math
t_{impact,j}
=
t_s
+
t_{sys}
+
t_{d,j}
+
t_{f,j}
```

tracker 预测：

```math
{}^O c_{a,j}
=
predictArmorCenter(t_{impact,j})
```

```math
{}^O R_{a,j}
=
predictArmorRotation(t_{impact,j})
```

转到 \(B_s\)：

```math
{}^{B_s}c_{a,j}
=
{}^{B_s}R_O
(
{}^O c_{a,j}
-
{}^O p_{B_s}
)
```

计算该 sigma 点下的子弹位置：

```math
{}^{B_s}\mu_{p,j}
=
{}^{B_s}p_0
+
\begin{bmatrix}
v_{0,j}t_{f,j}\\
0\\
0
\end{bmatrix}
+
\frac{1}{2}
{}^{B_s}g t_{f,j}^2
```

装甲板局部投影矩阵：

```math
J_{a,j}
=
\begin{bmatrix}
{}^{B_s}r_{a,j}^T\\
{}^{B_s}u_{a,j}^T
\end{bmatrix}
```

误差：

```math
e_j
=
J_{a,j}
\left(
{}^{B_s}\mu_{p,j}
-
{}^{B_s}c_{a,j}
\right)
```

其中：

```math
e_j
=
\begin{bmatrix}
e_{u,j}\\
e_{v,j}
\end{bmatrix}
```

### 19.4 Sigma 点得到附加均值和协方差

传播所有 sigma 点后，得到：

```math
e_0,e_1,\cdots,e_{2n}
```

加权均值：

```math
\mu_{\sigma}^{uv}
=
\sum_{j=0}^{2n}
W_j^{(m)}
e_j
```

加权协方差：

```math
\Sigma_{\sigma}^{uv}
=
\sum_{j=0}^{2n}
W_j^{(c)}
\left(
e_j-\mu_{\sigma}^{uv}
\right)
\left(
e_j-\mu_{\sigma}^{uv}
\right)^T
```

最终可以用：

```math
e_{uv}
=
\mu_{\sigma}^{uv}
```

```math
\Sigma_{total}^{uv}
=
\Sigma_{bullet}^{uv}
+
\Sigma_{tracker}^{uv}
+
\Sigma_{\sigma}^{uv}
```

再计算：

```math
P_{hit}
=
P
\left(
X_{uv}\in\mathcal{R}
\right)
```

### 19.5 避免重复计数

如果 \(\Sigma_{bullet}^{uv}\) 已经通过实弹数据包含了弹速波动和延迟误差，那么不应再加入：

```math
\Sigma_{\sigma}^{uv}
```

否则会重复放大误差。

推荐拆分为：

```math
\Sigma_{bullet}^{uv}
```

描述：

```text
枪口散布、控制残差、未建模小误差
```

```math
\Sigma_{\sigma}^{uv}
```

描述：

```text
弹速误差与发射延迟误差经过非线性传播造成的平面误差
```

---

## 20. 推荐初版实现

初版建议不启用 sigma point，先使用：

```math
{}^{B_s}\mu_p(t)
=
{}^{B_s}p_0
+
\begin{bmatrix}
v_0t\\0\\0
\end{bmatrix}
+
\frac{1}{2}
{}^{B_s}g t^2
```

```math
\Sigma_p^{B_s}(t)
=
\begin{bmatrix}
\sigma_x^2(t) & 0 & 0\\
0 & \sigma_y^2(t) & 0\\
0 & 0 & \sigma_z^2(t)
\end{bmatrix}
```

```math
\Sigma_{bullet}^{uv}
=
J_a
\Sigma_p^{B_s}
J_a^T
```

```math
\Sigma_{tracker}^{uv}
=
J_a
{}^{B_s}P_a
J_a^T
```

```math
\Sigma_{total}^{uv}
=
\Sigma_{bullet}^{uv}
+
\Sigma_{tracker}^{uv}
```

```math
P_{hit}
=
\int_{-w/2}^{w/2}
\int_{-h/2}^{h/2}
\mathcal{N}
\left(
x;
e_{uv},
\Sigma_{total}^{uv}
\right)
dx
```

再通过：

```math
P_{window}
=
\max_i P_{hit}(\tau_i)
```

```math
S_k
=
\alpha S_{k-1}
+
(1-\alpha)P_{window,k}
```

生成最终：

```math
fire\_advice
```

---

## 21. 推荐参数

```yaml
fire_probability:
  future_window_ms: 50
  future_step_ms: 10

  ballistic:
    bullet_speed: 28.0

  ballistic_uncertainty:
    sigma_x0: 0.010
    sigma_y0: 0.015
    sigma_z0: 0.015
    growth_x: 0.03
    growth_y: 0.06
    growth_z: 0.08
    min_sigma: 0.005
    max_sigma: 0.200

  tracker_uncertainty:
    use_tracker_covariance: true
    fallback_sigma_x: 0.020
    fallback_sigma_y: 0.020
    fallback_sigma_z: 0.030

  normal_velocity_weight:
    enable: false
    v_ref: 28.0
    w_min: 0.5

  fire_gate:
    alpha: 0.85
    fire_on_th: 0.65
    fire_off_th: 0.35

  sigma_point_extension:
    enable: false
    sigma_v0: 0.3
    sigma_delay: 0.005
```

---

## 22. 关键伪代码

```cpp
FireResult FireProbabilityEstimator::update(
    const TrackerState& tracker,
    const GimbalState& gimbal,
    const BallisticParams& ballistic,
    double now
) {
    float best_p = 0.0f;

    for (float tau : future_time_offsets_) {
        double fire_time = now + tau;

        // 1. 构造发射瞬间冻结的 barrel frame
        Transform T_Bs_O = getFrozenBarrelTransform(fire_time);

        // 2. 重力转到 barrel frame
        Eigen::Vector3f g_B =
            T_Bs_O.rotation() * Eigen::Vector3f(0, 0, -9.81f);

        // 3. 初步预测目标，估计飞行时间
        ArmorState armor_guess_O =
            tracker.predict(fire_time + system_delay + fire_delay);

        ArmorState armor_guess_B =
            transformArmor(armor_guess_O, T_Bs_O);

        float x = armor_guess_B.center.x();
        float tf = solveFlightTime(x, ballistic.v0, g_B.x());

        // 4. 预测真正命中时刻的装甲板
        double impact_time =
            fire_time + system_delay + fire_delay + tf;

        ArmorState armor_O =
            tracker.predict(impact_time);

        ArmorState armor_B =
            transformArmor(armor_O, T_Bs_O);

        // 5. 计算理想子弹位置和速度
        Eigen::Vector3f bullet_pos =
            muzzle_pos_B
            + Eigen::Vector3f(ballistic.v0 * tf, 0, 0)
            + 0.5f * g_B * tf * tf;

        Eigen::Vector3f bullet_vel =
            Eigen::Vector3f(ballistic.v0, 0, 0)
            + g_B * tf;

        // 6. 投影到装甲板平面
        Eigen::Vector3f delta =
            bullet_pos - armor_B.center;

        Eigen::Vector2f e_uv;
        e_uv.x() = delta.dot(armor_B.right);
        e_uv.y() = delta.dot(armor_B.up);

        // 7. 弹道协方差
        float sx = sigma_x0 + growth_x * tf;
        float sy = sigma_y0 + growth_y * tf;
        float sz = sigma_z0 + growth_z * tf;

        sx = std::clamp(sx, min_sigma, max_sigma);
        sy = std::clamp(sy, min_sigma, max_sigma);
        sz = std::clamp(sz, min_sigma, max_sigma);

        Eigen::Matrix3f Sigma_bullet_B =
            Eigen::Matrix3f::Zero();

        Sigma_bullet_B(0,0) = sx * sx;
        Sigma_bullet_B(1,1) = sy * sy;
        Sigma_bullet_B(2,2) = sz * sz;

        Eigen::Matrix<float, 2, 3> J;
        J.row(0) = armor_B.right.transpose();
        J.row(1) = armor_B.up.transpose();

        Eigen::Matrix2f Sigma_bullet_uv =
            J * Sigma_bullet_B * J.transpose();

        // 8. tracker 协方差投影
        Eigen::Matrix3f P_tracker_B =
            T_Bs_O.rotation()
            * tracker.positionCovariance()
            * T_Bs_O.rotation().transpose();

        Eigen::Matrix2f Sigma_tracker_uv =
            J * P_tracker_B * J.transpose();

        Eigen::Matrix2f Sigma_total =
            Sigma_bullet_uv + Sigma_tracker_uv;

        // 9. 初版：取对角独立近似
        float sigma_u = std::sqrt(std::max(Sigma_total(0,0), 1e-8f));
        float sigma_v = std::sqrt(std::max(Sigma_total(1,1), 1e-8f));

        float p_hit =
            computeHitProbability(
                e_uv,
                sigma_u,
                sigma_v,
                armor_B.width,
                armor_B.height
            );

        // 10. 可选法向速度权重
        if (enable_normal_velocity_weight) {
            float vn =
                std::max(0.0f, -bullet_vel.dot(armor_B.normal));

            float wn =
                std::clamp(vn / v_ref, w_min, 1.0f);

            p_hit *= wn;
        }

        best_p = std::max(best_p, p_hit);
    }

    // 11. 时间滤波
    fire_score_ =
        alpha * fire_score_
        +
        (1.0f - alpha) * best_p;

    // 12. 滞回输出
    if (!fire_state_ && fire_score_ > fire_on_th) {
        fire_state_ = true;
    }

    if (fire_state_ && fire_score_ < fire_off_th) {
        fire_state_ = false;
    }

    FireResult result;
    result.p_hit_window = best_p;
    result.fire_score = fire_score_;
    result.fire_advice = fire_state_;

    return result;
}
```

---

## 23. 调试输出建议

建议发布以下 debug 信息：

```text
p_hit_now
p_hit_window
fire_score
fire_advice

e_u
e_v

sigma_u
sigma_v

flight_time
target_distance

bullet_pos_B
bullet_vel_B

armor_center_B
armor_normal_B

normal_velocity
normal_velocity_weight
```

如果出现：

```text
肉眼看能打中，但 p_hit 很低
```

优先检查：

```text
坐标系方向、装甲板 right/up/normal、e_uv 符号、sigma 是否过小
```

如果出现：

```text
p_hit 很高，但实际打不中
```

优先检查：

```text
系统偏差、枪口外参、弹速、fire_delay、sigma 是否过大
```

如果出现：

```text
fire_advice 抖动
```

优先调整：

```text
alpha、fire_on_th、fire_off_th、积分器参数
```

---

## 24. 后续实弹标定升级

初版参数可以人工设置。后续建议通过实弹数据标定。

在距离：

```text
3.0m, 3.5m, 4.0m, ..., 7.0m
```

每个距离打：

```text
30 ~ 100 发
```

记录弹着点相对瞄准点误差：

```math
e_y,\ e_z
```

统计：

```math
\mu_y(d),\ \mu_z(d)
```

```math
\sigma_y(d),\ \sigma_z(d)
```

得到距离相关模型：

```math
e(d)
\sim
\mathcal{N}
\left(
\mu(d),
\Sigma(d)
\right)
```

其中：

```math
\mu(d)
=
\begin{bmatrix}
\mu_y(d)\\
\mu_z(d)
\end{bmatrix}
```

```math
\Sigma(d)
=
\begin{bmatrix}
\sigma_y^2(d) & \rho\sigma_y(d)\sigma_z(d)\\
\rho\sigma_y(d)\sigma_z(d) & \sigma_z^2(d)
\end{bmatrix}
```

后续可以用查表模型替代：

```math
\sigma(t)=\sigma_0+kt
```

即：

```text
distance / flight_time → μ(d), Σ(d)
```

---

## 25. 总结

本方案将原始的二值开火建议：

```math
fire\_advice
=
|\Delta yaw|<yaw_{th}
\land
|\Delta pitch|<pitch_{th}
```

升级为概率模型：

```math
P_{hit}
=
\int_{\mathcal{R}}
\mathcal{N}
(x;e_{uv},\Sigma_{total}^{uv})dx
```

其中：

```math
e_{uv}
=
J_a
\left(
{}^{B_s}\mu_p(t_f)
-
{}^{B_s}c_a(t_{impact})
\right)
```

```math
\Sigma_{total}^{uv}
=
J_a
\Sigma_p^{B_s}
J_a^T
+
J_a
{}^{B_s}P_a
J_a^T
```

再通过：

```math
P_{window}
=
\max_i P_{hit}(\tau_i)
```

```math
S_k
=
\alpha S_{k-1}
+
(1-\alpha)P_{window,k}
```

最终生成稳定的：

```math
fire\_advice
```

该方案具有以下优点：

```text
1. 物理意义清晰
2. 计算量小
3. 可以满足实时性
4. 能自然处理弹道散布和 tracker 不确定性
5. 能解决 fire_advice 离散抖动问题
6. 后续可扩展 sigma point、实弹标定和法向速度加权
```

初版建议实现：

```text
枪管系理想弹道
+
随飞行时间增长的弹道协方差
+
tracker 协方差投影
+
二维高斯矩形命中概率
+
时间窗 max
+
fire_score 低通/积分
+
滞回 fire_advice
```

后续再扩展：

```text
弹速误差 Δv0
+
发射延迟误差 Δtd
+
sigma-point 传播到装甲板平面误差
```
