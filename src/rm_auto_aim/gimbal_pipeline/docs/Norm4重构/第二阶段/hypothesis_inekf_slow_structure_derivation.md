# Hypothesis-InEKF 与慢结构参数更新推导方案

> 适用场景：Norm4 / Backend UKF V2 重构中的四装甲板自旋目标跟踪。  
> 核心目标：在“离散 panel 假设枚举 + NIS / likelihood 裁决 + TopK shadow + Top1 commit”的框架下，引入 InEKF / invariant error-state EKF 作为连续状态估计器，并将 `r1 / r2 / dza` 作为慢结构参数进行保守更新。

---

## 0. 核心结论

对于四装甲板自旋目标，**InEKF + 慢结构参数更新是可行的**，但它不应替代离散假设层。

更推荐的整体形式是：

```text
Discrete Hypothesis Layer
  panel / pair / order 枚举

Fast State Filter
  UKF / IEKF / InEKF
  更新 center、velocity、yaw、yaw_rate

Slow Structure Estimator
  低频更新 r1、r2、dza
  依赖高置信 TopK、多帧窗口、结构先验

Commit Policy
  NIS / likelihood / confidence / margin / null hypothesis

Mode FSM
  structured / ambiguous / fallback / lost
```

也就是说，InEKF 更适合作为 V2 后端中的**快状态估计器**，而 `r1 / r2 / dza` 更适合作为**欧氏慢参数**处理。

最终推荐形式：

```text
Hypothesis-InEKF Backend
+
Slow Structure Parameter Estimator
```

---

## 1. 普通 EKF 的局限

普通 EKF 默认状态位于欧氏空间：

$$
x \in \mathbb{R}^n
$$

系统模型为：

$$
x_{k+1} = f(x_k, u_k) + w_k
$$

观测模型为：

$$
z_k = h(x_k) + v_k
$$

在估计值 $\hat{x}$ 附近线性化：

$$
\delta x_{k+1} \approx F_k \delta x_k + G_k w_k
$$

$$
\nu_k = z_k - h(\hat{x}_k) \approx H_k \delta x_k + v_k
$$

其中：

$$
F_k = \frac{\partial f}{\partial x}\bigg|_{\hat{x}_k}
$$

$$
H_k = \frac{\partial h}{\partial x}\bigg|_{\hat{x}_k}
$$

标准 EKF 预测：

$$
P_{k+1|k} = F_k P_{k|k} F_k^T + Q_k
$$

标准 EKF 更新：

$$
S_k = H_k P_{k|k-1} H_k^T + R_k
$$

$$
K_k = P_{k|k-1}H_k^T S_k^{-1}
$$

$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k\nu_k
$$

问题在于：姿态、yaw、SE(3) 位姿等状态并不是真正的欧氏向量，直接相减 $x-\hat{x}$ 并不严格。

例如：

$$
\psi = 179^\circ,\quad \hat{\psi} = -179^\circ
$$

直接相减：

$$
\delta \psi = 358^\circ
$$

但真实误差应为：

$$
\delta \psi = -2^\circ
$$

所以，对包含旋转 / 位姿的跟踪问题，更合理的是在李群上定义状态误差。

---

## 2. 李群状态表示

设系统状态不是普通向量，而是矩阵李群上的元素：

$$
X \in G
$$

例如：

$$
G = SO(3),\ SE(3),\ SE_2(3)
$$

李代数为：

$$
\xi \in \mathfrak{g}
$$

通常用向量表示：

$$
\xi \in \mathbb{R}^n
$$

通过帽算子映射到矩阵李代数：

$$
\xi^\wedge \in \mathfrak{g}
$$

通过指数映射回李群：

$$
\operatorname{Exp}(\xi) = \exp(\xi^\wedge) \in G
$$

对小扰动，可以写成：

$$
X = \operatorname{Exp}(\delta \xi)\hat{X}
$$

或：

$$
X = \hat{X}\operatorname{Exp}(\delta \xi)
$$

这对应两类不同的误差定义。

---

## 3. 左扰动与右扰动

### 3.1 左扰动

定义真实状态为：

$$
X = \operatorname{Exp}(\delta \xi)\hat{X}
$$

则误差为：

$$
\eta_R = X\hat{X}^{-1}
$$

因为：

$$
\eta_R = \operatorname{Exp}(\delta \xi)
$$

该误差在右乘坐标变换下不变：

$$
(XA)(\hat{X}A)^{-1}
=
XAA^{-1}\hat{X}^{-1}
=
X\hat{X}^{-1}
$$

因此它称为**右不变误差**。

### 3.2 右扰动

定义真实状态为：

$$
X = \hat{X}\operatorname{Exp}(\delta \xi)
$$

则误差为：

$$
\eta_L = \hat{X}^{-1}X
$$

该误差在左乘坐标变换下不变：

$$
(A\hat{X})^{-1}(AX)
=
\hat{X}^{-1}A^{-1}AX
=
\hat{X}^{-1}X
$$

因此它称为**左不变误差**。

---

## 4. InEKF 的核心思想

普通 EKF 在线性化：

$$
\delta x = x - \hat{x}
$$

InEKF 在线性化：

$$
\eta = X\hat{X}^{-1}
$$

或：

$$
\eta = \hat{X}^{-1}X
$$

再通过对数映射得到最小误差向量：

$$
\delta \xi = \operatorname{Log}(\eta)
$$

也就是说：

```text
普通 EKF:
  在欧氏空间中做加法误差

InEKF:
  在李群上做乘法误差
```

状态更新也不是：

$$
\hat{x}^+ = \hat{x} + \delta x
$$

而是：

$$
\hat{X}^+ = \operatorname{Exp}(\delta \xi)\hat{X}
$$

或：

$$
\hat{X}^+ = \hat{X}\operatorname{Exp}(\delta \xi)
$$

---

## 5. 连续时间 InEKF 误差动力学推导

假设系统动力学为：

$$
\dot{X} = X\Omega^\wedge
$$

其中：

$$
\Omega \in \mathbb{R}^n
$$

是输入或系统速度。

估计状态满足：

$$
\dot{\hat{X}} = \hat{X}\Omega^\wedge
$$

定义左不变误差：

$$
\eta = X^{-1}\hat{X}
$$

对它求导：

$$
\dot{\eta}
=
\frac{d}{dt}(X^{-1}\hat{X})
$$

根据：

$$
\frac{d}{dt}X^{-1}
=
-X^{-1}\dot{X}X^{-1}
$$

得到：

$$
\dot{\eta}
=
-X^{-1}\dot{X}X^{-1}\hat{X}
+
X^{-1}\dot{\hat{X}}
$$

代入：

$$
\dot{X} = X\Omega^\wedge
$$

$$
\dot{\hat{X}} = \hat{X}\Omega^\wedge
$$

有：

$$
\dot{\eta}
=
-X^{-1}X\Omega^\wedge X^{-1}\hat{X}
+
X^{-1}\hat{X}\Omega^\wedge
$$

即：

$$
\dot{\eta}
=
-\Omega^\wedge \eta
+
\eta\Omega^\wedge
$$

整理为：

$$
\dot{\eta}
=
\eta\Omega^\wedge
-
\Omega^\wedge\eta
$$

当误差较小时：

$$
\eta \approx I + \delta \xi^\wedge
$$

代入：

$$
\dot{\eta}
\approx
\dot{\delta \xi}^{\wedge}
$$

右边展开：

$$
\eta\Omega^\wedge - \Omega^\wedge\eta
\approx
(I+\delta\xi^\wedge)\Omega^\wedge
-
\Omega^\wedge(I+\delta\xi^\wedge)
$$

$$
=
\Omega^\wedge
+
\delta\xi^\wedge\Omega^\wedge
-
\Omega^\wedge
-
\Omega^\wedge\delta\xi^\wedge
$$

所以：

$$
\dot{\delta \xi}^{\wedge}
=
\delta\xi^\wedge\Omega^\wedge
-
\Omega^\wedge\delta\xi^\wedge
$$

即李括号：

$$
\dot{\delta \xi}^{\wedge}
=
[\delta\xi^\wedge,\Omega^\wedge]
$$

根据伴随表示：

$$
[\Omega^\wedge,\delta\xi^\wedge]
=
(\operatorname{ad}_{\Omega}\delta\xi)^\wedge
$$

因此：

$$
[\delta\xi^\wedge,\Omega^\wedge]
=
-(\operatorname{ad}_{\Omega}\delta\xi)^\wedge
$$

得到误差动力学：

$$
\dot{\delta\xi}
=
-\operatorname{ad}_{\Omega}\delta\xi
$$

也就是：

$$
\dot{\delta\xi} = A\delta\xi
$$

其中：

$$
A = -\operatorname{ad}_{\Omega}
$$

关键点是：  
**InEKF 的误差动力学矩阵 $A$ 不直接依赖当前估计状态 $\hat{X}$，而主要依赖输入或系统速度。**

这通常比普通 EKF 更稳定。

---

## 6. 加入过程噪声

如果系统为：

$$
\dot{X} = X(\Omega + w)^\wedge
$$

其中：

$$
w \sim \mathcal{N}(0,Q)
$$

则误差线性化为：

$$
\dot{\delta\xi}
=
A\delta\xi + Gw
$$

离散化：

$$
\delta\xi_{k+1}
=
\Phi_k\delta\xi_k + G_kw_k
$$

其中：

$$
\Phi_k = \exp(A_k\Delta t)
$$

协方差预测：

$$
P_{k+1|k}
=
\Phi_k P_{k|k}\Phi_k^T
+
G_kQ_kG_k^T
$$

这里的 $P$ 不是普通状态 $x$ 的协方差，而是**李代数误差 $\delta\xi$ 的协方差**。

---

## 7. InEKF 观测更新推导

设观测模型为：

$$
z = h(X) + v
$$

估计观测为：

$$
\hat{z} = h(\hat{X})
$$

定义创新：

$$
\nu = z - \hat{z}
$$

由于：

$$
X = \operatorname{Exp}(\delta\xi)\hat{X}
$$

所以：

$$
h(X)
=
h(\operatorname{Exp}(\delta\xi)\hat{X})
$$

对 $\delta\xi = 0$ 一阶展开：

$$
h(\operatorname{Exp}(\delta\xi)\hat{X})
\approx
h(\hat{X}) + H\delta\xi
$$

因此：

$$
\nu
=
z - h(\hat{X})
\approx
H\delta\xi + v
$$

其中：

$$
H
=
\frac{\partial h(\operatorname{Exp}(\delta\xi)\hat{X})}
{\partial \delta\xi}
\bigg|_{\delta\xi=0}
$$

Kalman 更新：

$$
S = HPH^T + R
$$

$$
K = PH^TS^{-1}
$$

$$
\widehat{\delta\xi} = K\nu
$$

状态更新：

$$
\hat{X}^+
=
\operatorname{Exp}(\widehat{\delta\xi})\hat{X}
$$

协方差更新推荐 Joseph 形式：

$$
P^+
=
(I-KH)P(I-KH)^T
+
KRK^T
$$

如果修正量较大，可以加入误差重置 Jacobian：

$$
P^+ \leftarrow \Gamma P^+\Gamma^T
$$

工程中如果每帧更新量较小，可近似：

$$
\Gamma \approx I
$$

---

## 8. NIS 与 likelihood

InEKF 的观测创新仍然可用于 NIS 检验。

创新：

$$
\nu = z - h(\hat{X})
$$

创新协方差：

$$
S = HPH^T + R
$$

NIS：

$$
\operatorname{NIS}
=
\nu^TS^{-1}\nu
$$

log-likelihood：

$$
\log p(z|X)
=
-\frac{1}{2}
\left(
\nu^TS^{-1}\nu
+
\log |S|
+
m\log(2\pi)
\right)
$$

其中 $m$ 是观测维度。

这与 V2 后端中的 hypothesis 排序完全兼容：

```text
每个 hypothesis:
  计算 ν, S, NIS, log-likelihood
  ↓
排序
  ↓
TopK
  ↓
Top1 满足 gate 才 commit
```

---

## 9. 四装甲板自旋目标状态建模

目标快状态可以写为：

$$
X_f = (R,p,v)
$$

其中：

$$
R = R_z(\psi)
$$

是目标 yaw 对应的旋转矩阵；

$$
p = [x_c,y_c,z_c]^T
$$

是机器人中心；

$$
v = [v_x,v_y,v_z]^T
$$

是中心速度。

可以将其放入扩展位姿群：

$$
SE_2(3)
$$

矩阵形式为：

$$
X_f =
\begin{bmatrix}
R & v & p \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

同时保留普通欧氏慢参数：

$$
\beta = \dot{\psi}
$$

$$
\theta = [r_1,r_2,dza]^T
$$

完整状态为：

$$
\mathcal{X}
=
(X_f,\beta,\theta)
$$

其中：

$$
X_f \in SE_2(3)
$$

$$
\beta \in \mathbb{R}
$$

$$
\theta \in \mathbb{R}^3
$$

这不是纯 InEKF，而是更适合该项目的：

```text
Hybrid InEKF / invariant error-state EKF
```

即：

```text
位姿、速度、yaw 用李群误差；
yaw_rate、r1、r2、dza 用普通加法误差。
```

---

## 10. 自旋目标动力学

设：

$$
R_k = R_z(\psi_k)
$$

$$
\beta_k = \dot{\psi}_k
$$

离散时间模型：

$$
R_{k+1}
=
R_k\operatorname{Exp}(e_z\beta_k\Delta t)
$$

$$
p_{k+1}
=
p_k + v_k\Delta t
$$

$$
v_{k+1}
=
v_k
$$

$$
\beta_{k+1}
=
\beta_k
$$

慢结构参数：

$$
\theta_{k+1}
=
\theta_k
$$

加入过程噪声：

$$
R_{k+1}
=
R_k\operatorname{Exp}(e_z(\beta_k\Delta t+w_\psi))
$$

$$
p_{k+1}
=
p_k + v_k\Delta t + w_p
$$

$$
v_{k+1}
=
v_k + w_v
$$

$$
\beta_{k+1}
=
\beta_k + w_\beta
$$

$$
\theta_{k+1}
=
\theta_k + w_\theta
$$

其中慢结构噪声满足：

$$
Q_\theta \ll Q_p,\ Q_v,\ Q_\psi,\ Q_\beta
$$

也就是：

```text
r1、r2、dza 允许缓慢变化，
但不能被单帧观测剧烈拉动。
```

---

## 11. 误差状态定义

定义误差状态：

$$
\delta x =
[
\delta\rho^T,
\delta v^T,
\delta\psi,
\delta\beta,
\delta\theta^T
]^T
$$

其中：

$$
\delta\rho \in \mathbb{R}^3
$$

表示位置误差；

$$
\delta v \in \mathbb{R}^3
$$

表示速度误差；

$$
\delta\psi \in \mathbb{R}
$$

表示 yaw 的李代数小扰动；

$$
\delta\beta \in \mathbb{R}
$$

表示 yaw_rate 误差；

$$
\delta\theta = [\delta r_1,\delta r_2,\delta dza]^T
$$

表示结构参数误差。

如果使用左扰动：

$$
R = \operatorname{Exp}(\delta\psi e_z)\hat{R}
$$

$$
p = \hat{p} + \delta\rho
$$

$$
v = \hat{v} + \delta v
$$

$$
\beta = \hat{\beta} + \delta\beta
$$

$$
\theta = \hat{\theta} + \delta\theta
$$

则误差预测：

$$
\delta x_{k+1}
=
F_k\delta x_k + G_kw_k
$$

对简单常速度 + 常角速度模型：

$$
\delta\rho_{k+1}
=
\delta\rho_k + \Delta t\delta v_k
$$

$$
\delta v_{k+1}
=
\delta v_k
$$

$$
\delta\psi_{k+1}
=
\delta\psi_k + \Delta t\delta\beta_k
$$

$$
\delta\beta_{k+1}
=
\delta\beta_k
$$

$$
\delta\theta_{k+1}
=
\delta\theta_k
$$

因此：

$$
F_k =
\begin{bmatrix}
I_3 & \Delta t I_3 & 0 & 0 & 0 \\
0 & I_3 & 0 & 0 & 0 \\
0 & 0 & 1 & \Delta t & 0 \\
0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & I_3
\end{bmatrix}
$$

这是工程上可直接实现的 error-state 传播矩阵。

---

## 12. 装甲板观测模型

对某个 panel hypothesis $i$，定义相位：

$$
\phi_i \in \{0,\frac{\pi}{2},\pi,\frac{3\pi}{2}\}
$$

例如：

$$
\phi_0 = 0
$$

$$
\phi_1 = \frac{\pi}{2}
$$

$$
\phi_2 = \pi
$$

$$
\phi_3 = \frac{3\pi}{2}
$$

定义半径选择：

$$
r_i =
\begin{cases}
r_1, & i \in \{0,2\} \\
r_2, & i \in \{1,3\}
\end{cases}
$$

定义高度偏置：

$$
z_i =
\begin{cases}
-\frac{1}{2}dza, & i \in \{0,2\} \\
+\frac{1}{2}dza, & i \in \{1,3\}
\end{cases}
$$

在目标本体系下，第 $i$ 块装甲板中心偏移为：

$$
b_i(\theta)
=
\begin{bmatrix}
r_i\cos \phi_i \\
r_i\sin \phi_i \\
z_i
\end{bmatrix}
$$

世界系预测装甲板位置：

$$
p_i
=
p + Rb_i(\theta)
$$

预测装甲板 yaw：

$$
\psi_i
=
\psi + \phi_i + \gamma
$$

其中 $\gamma$ 是装甲板法向和机器人 yaw 零点之间的固定偏置。

观测模型：

$$
h_i(\mathcal{X})
=
\begin{bmatrix}
p + Rb_i(\theta) \\
\psi + \phi_i + \gamma
\end{bmatrix}
$$

若 PnP 输出为：

$$
z_i =
\begin{bmatrix}
p_i^{obs} \\
\psi_i^{obs}
\end{bmatrix}
$$

则创新为：

$$
\nu_i
=
\begin{bmatrix}
p_i^{obs} - (\hat{p}+\hat{R}b_i(\hat{\theta})) \\
\operatorname{wrap}(\psi_i^{obs}-(\hat{\psi}+\phi_i+\gamma))
\end{bmatrix}
$$

---

## 13. 观测 Jacobian 推导

对位置观测：

$$
p_i = p + Rb_i(\theta)
$$

扰动后：

$$
p_i'
=
p+\delta\rho
+
\operatorname{Exp}(\delta\psi e_z)\hat{R}
\left(
b_i(\hat{\theta}) + B_i\delta\theta
\right)
$$

一阶展开：

$$
\operatorname{Exp}(\delta\psi e_z)
\approx
I + \delta\psi e_z^\wedge
$$

所以：

$$
p_i'
\approx
\hat{p}
+
\delta\rho
+
(I+\delta\psi e_z^\wedge)
\hat{R}
\left(
\hat{b}_i+B_i\delta\theta
\right)
$$

忽略二阶项：

$$
p_i'
\approx
\hat{p}
+
\hat{R}\hat{b}_i
+
\delta\rho
+
\delta\psi e_z^\wedge\hat{R}\hat{b}_i
+
\hat{R}B_i\delta\theta
$$

因此位置创新的一阶形式为：

$$
\nu_p
\approx
\delta\rho
+
\delta\psi(e_z \times \hat{R}\hat{b}_i)
+
\hat{R}B_i\delta\theta
+
v_p
$$

所以位置观测 Jacobian 为：

$$
H_{p,i}
=
\begin{bmatrix}
I_3
&
0_{3\times 3}
&
e_z^\wedge \hat{R}\hat{b}_i
&
0
&
\hat{R}B_i
\end{bmatrix}
$$

其中：

$$
B_i = \frac{\partial b_i}{\partial \theta}
$$

如果：

$$
\theta = [r_1,r_2,dza]^T
$$

则对于 $i \in \{0,2\}$：

$$
\frac{\partial b_i}{\partial r_1}
=
\begin{bmatrix}
\cos \phi_i \\
\sin \phi_i \\
0
\end{bmatrix}
$$

$$
\frac{\partial b_i}{\partial r_2}
=
0
$$

对于 $i \in \{1,3\}$：

$$
\frac{\partial b_i}{\partial r_2}
=
\begin{bmatrix}
\cos \phi_i \\
\sin \phi_i \\
0
\end{bmatrix}
$$

$$
\frac{\partial b_i}{\partial r_1}
=
0
$$

高度项：

$$
\frac{\partial b_i}{\partial dza}
=
\begin{cases}
[0,0,-\frac{1}{2}]^T, & i \in \{0,2\} \\
[0,0,+\frac{1}{2}]^T, & i \in \{1,3\}
\end{cases}
$$

yaw 观测为：

$$
\psi_i = \psi + \phi_i + \gamma
$$

所以：

$$
\nu_\psi
\approx
\delta\psi + v_\psi
$$

因此 yaw 观测 Jacobian：

$$
H_{\psi,i}
=
\begin{bmatrix}
0_{1\times 3}
&
0_{1\times 3}
&
1
&
0
&
0_{1\times 3}
\end{bmatrix}
$$

最终单块装甲板观测 Jacobian：

$$
H_i
=
\begin{bmatrix}
H_{p,i} \\
H_{\psi,i}
\end{bmatrix}
$$

---

## 14. 单观测 InEKF 更新

单观测维度为：

$$
m=4
$$

即：

$$
z_i = [x,y,z,yaw]^T
$$

对某个 hypothesis `panel_id = i`：

$$
\nu_i = z_i - h_i(\hat{\mathcal{X}})
$$

$$
S_i = H_iPH_i^T + R_i
$$

$$
NIS_i = \nu_i^T S_i^{-1}\nu_i
$$

$$
K_i = PH_i^T S_i^{-1}
$$

$$
\delta\hat{x}_i = K_i\nu_i
$$

拆开：

$$
\delta\hat{x}_i
=
[
\delta\hat{\rho}^T,
\delta\hat{v}^T,
\delta\hat{\psi},
\delta\hat{\beta},
\delta\hat{\theta}^T
]^T
$$

状态更新：

$$
\hat{R}^+
=
\operatorname{Exp}(\delta\hat{\psi}e_z)\hat{R}
$$

$$
\hat{p}^+
=
\hat{p}+\delta\hat{\rho}
$$

$$
\hat{v}^+
=
\hat{v}+\delta\hat{v}
$$

$$
\hat{\beta}^+
=
\hat{\beta}+\delta\hat{\beta}
$$

结构参数如果正常更新：

$$
\hat{\theta}^+
=
\hat{\theta}+\delta\hat{\theta}
$$

但实际工程中更推荐：

$$
\hat{\theta}^+
=
\hat{\theta}+\Lambda_\theta\delta\hat{\theta}
$$

其中：

$$
\Lambda_\theta
=
\operatorname{diag}(\alpha_{r1},\alpha_{r2},\alpha_{dza})
$$

推荐：

$$
\alpha_{r1},\alpha_{r2} \in [0.02,0.10]
$$

$$
\alpha_{dza} \in [0.00,0.05]
$$

单观测不稳定时：

$$
\Lambda_\theta = 0
$$

也就是冻结结构参数。

---

## 15. 双观测 batch update

如果同一帧有两块装甲板观测：

$$
z_a,\ z_b
$$

假设 pair 为：

$$
(i,j)
$$

则 batch 观测为：

$$
z_{ij}
=
\begin{bmatrix}
z_a \\
z_b
\end{bmatrix}
$$

预测为：

$$
h_{ij}(\mathcal{X})
=
\begin{bmatrix}
h_i(\mathcal{X}) \\
h_j(\mathcal{X})
\end{bmatrix}
$$

创新：

$$
\nu_{ij}
=
\begin{bmatrix}
\nu_i \\
\nu_j
\end{bmatrix}
$$

Jacobian：

$$
H_{ij}
=
\begin{bmatrix}
H_i \\
H_j
\end{bmatrix}
$$

协方差：

$$
R_{ij}
=
\begin{bmatrix}
R_i & 0 \\
0 & R_j
\end{bmatrix}
$$

如果考虑同一帧 PnP 误差相关性，也可以写成：

$$
R_{ij}
=
\begin{bmatrix}
R_i & R_{ab} \\
R_{ba} & R_j
\end{bmatrix}
$$

然后：

$$
S_{ij}
=
H_{ij}PH_{ij}^T + R_{ij}
$$

$$
NIS_{ij}
=
\nu_{ij}^TS_{ij}^{-1}\nu_{ij}
$$

$$
K_{ij}
=
PH_{ij}^TS_{ij}^{-1}
$$

$$
\delta\hat{x}_{ij}
=
K_{ij}\nu_{ij}
$$

双观测应该与 V2 架构中的 pair 枚举结合：

```text
pair 01:
  obsA -> 0, obsB -> 1
  obsA -> 1, obsB -> 0

pair 12:
  obsA -> 1, obsB -> 2
  obsA -> 2, obsB -> 1

pair 23:
  obsA -> 2, obsB -> 3
  obsA -> 3, obsB -> 2

pair 30:
  obsA -> 3, obsB -> 0
  obsA -> 0, obsB -> 3
```

每个 assignment 都单独计算：

$$
\nu,\ S,\ NIS,\ \log likelihood
$$

但只有最终 commit 的 Top1 才真正更新主状态。

---

## 16. Hypothesis 层与 InEKF 的组合

对于每个离散假设：

$$
\mathcal{H}_l
$$

有：

$$
\mathcal{H}_l = \{obs \rightarrow panel_i\}
$$

或：

$$
\mathcal{H}_l =
\{
obs_a \rightarrow panel_i,\ 
obs_b \rightarrow panel_j
\}
$$

InEKF 对每个假设输出：

$$
\nu_l
$$

$$
S_l
$$

$$
NIS_l
$$

$$
\log p(z|\mathcal{H}_l)
$$

其中：

$$
\log p(z|\mathcal{H}_l)
=
-\frac{1}{2}
\left(
\nu_l^TS_l^{-1}\nu_l
+
\log|S_l|
+
m\log(2\pi)
\right)
$$

如果有 evidence prior：

$$
\log w_l^{prior}
$$

则总分为：

$$
\log w_l
=
\log p(z|\mathcal{H}_l)
+
\log w_l^{prior}
$$

归一化权重：

$$
w_l
=
\frac{\exp(\log w_l)}
{\sum_j \exp(\log w_j)}
$$

Top1 置信度：

$$
confidence_{top1} = w_{top1}
$$

Top1 / Top2 margin：

$$
margin_{12}
=
\log w_{top1}
-
\log w_{top2}
$$

commit 条件：

$$
NIS_{top1} < \tau_{nis}
$$

$$
confidence_{top1} > \tau_{conf}
$$

$$
margin_{12} > \tau_{margin}
$$

否则：

```text
不更新 / ambiguous / predict only
```

---

## 17. 慢结构参数的严格处理

对结构参数：

$$
\theta = [r_1,r_2,dza]^T
$$

不建议将其作为快状态强更新。更严格的方式是引入慢时间尺度：

$$
\theta_{k+1} = \theta_k + w_{\theta,k}
$$

且：

$$
w_{\theta,k} \sim \mathcal{N}(0,Q_\theta)
$$

其中：

$$
Q_\theta
$$

非常小。

同时加入结构先验：

$$
\theta \sim \mathcal{N}(\theta_0,\Sigma_\theta)
$$

等价于伪观测：

$$
z_\theta = \theta_0
$$

$$
h_\theta(\mathcal{X}) = \theta
$$

创新：

$$
\nu_\theta = \theta_0-\hat{\theta}
$$

Jacobian：

$$
H_\theta =
\begin{bmatrix}
0 & I_3
\end{bmatrix}
$$

先验更新：

$$
S_\theta = H_\theta PH_\theta^T + \Sigma_\theta
$$

$$
K_\theta = PH_\theta^TS_\theta^{-1}
$$

$$
\delta x_\theta = K_\theta\nu_\theta
$$

这样结构参数会被温和拉回合理范围，而不是无限漂移。

---

## 18. 慢结构参数的工程实现

理论上可以通过：

$$
Q_\theta,\ R,\ \Sigma_\theta
$$

来控制慢更新。

工程上也可以显式缩放结构参数更新量：

$$
\delta\theta^+ = \Lambda_\theta\delta\theta
$$

其中：

$$
\Lambda_\theta
=
\begin{bmatrix}
\alpha_{r1} & 0 & 0 \\
0 & \alpha_{r2} & 0 \\
0 & 0 & \alpha_{dza}
\end{bmatrix}
$$

然后：

$$
\hat{\theta}^+
=
\operatorname{clamp}(\hat{\theta}+\Lambda_\theta\delta\theta)
$$

例如：

$$
r_1 \in [r_{min},r_{max}]
$$

$$
r_2 \in [r_{min},r_{max}]
$$

$$
dza \in [dza_{min},dza_{max}]
$$

注意：

```text
直接缩放 Kalman 增益是工程启发式；
更严格的统计方式是调 Qθ、R、结构先验 Σθ。
```

推荐组合：

```text
Qθ 很小
结构先验存在
单观测 θ 增益冻结或极小
双观测 θ 增益保守放开
高置信窗口内 θ 才慢更新
```

---

## 19. 结构参数允许更新的条件

建议只有当以下条件满足时，才允许更新 `r1 / r2 / dza`：

```text
Top1 NIS 通过 gate
confidence_top1 足够高
Top1 与 Top2 margin 足够大
当前不是 ambiguous 模式
不是刚初始化的前几帧
观测距离没有过远
PnP yaw 没有明显跳变
结构参数更新后仍在物理范围内
```

更严格的优先级：

```text
双观测 > 单观测
多帧同一相位稳定 > 单帧
窗口优化 > 直接单帧更新
```

可以设计为：

```cpp
if (dual_obs && top1_confident) {
  allow_slow_structure_update();
} else if (single_obs && stable_for_N_frames) {
  allow_very_weak_structure_update();
} else {
  freeze_structure();
}
```

---

## 20. 完整算法流程

```text
输入:
  当前观测 obs_k
  上一帧状态 R, p, v, beta, theta
  上一帧协方差 P

1. 预测:
  R_{k|k-1} = R_{k-1} Exp(e_z beta Δt)
  p_{k|k-1} = p_{k-1} + v_{k-1} Δt
  v_{k|k-1} = v_{k-1}
  beta_{k|k-1} = beta_{k-1}
  theta_{k|k-1} = theta_{k-1}

2. 误差协方差预测:
  P_{k|k-1} = F_k P_{k-1} F_k^T + Q_k

3. 枚举 hypothesis:
  单观测: panel 0,1,2,3
  双观测: pair 01,12,23,30 × two orders

4. 对每个 hypothesis:
  计算 h_l(X)
  计算 innovation ν_l
  计算 Jacobian H_l
  计算 S_l = H_l P H_l^T + R_l
  计算 NIS_l = ν_l^T S_l^{-1}ν_l
  计算 log-likelihood_l

5. TopK 排序:
  log w_l = log-likelihood_l + prior_log_weight_l

6. CommitPolicy:
  如果 Top1 通过 NIS / confidence / margin:
      commit Top1
  否则:
      predict only 或 ambiguous

7. InEKF 更新:
  K = P H^T S^{-1}
  δx = Kν

  R^+     = Exp(δψ e_z) R
  p^+     = p + δρ
  v^+     = v + δv
  beta^+  = beta + δbeta
  theta^+ = theta + Λθδtheta

8. 协方差更新:
  P^+ = (I-KH)P(I-KH)^T + KRK^T

9. 结构先验 / clamp:
  theta 拉回合理范围
  r1,r2,dza 限幅

10. 输出:
  center pose
  yaw / yaw_rate
  r1,r2,dza
  selected panel
  NIS
  confidence
  margin
  mode
```

---

## 21. 与 UKF 的关系

UKF 和 InEKF 的区别可以概括为：

```text
UKF:
  用 sigma points 传播非线性
  不显式求 Jacobian
  对强非线性观测传播比较友好

InEKF:
  在李群误差上做线性化
  误差定义更符合姿态 / 位姿几何
  对旋转、坐标系、yaw wrap 更稳定
```

在该项目中可以这样理解：

```text
UKF 更适合处理非线性观测传播；
InEKF 更适合处理姿态 / 位姿误差一致性。
```

如果当前最主要问题是：

```text
panel 错绑
NIS 爆炸
双观测重复消费
结构参数被拉偏
```

那么优先级仍然应该是：

```text
hypothesis + NIS + TopK + null hypothesis
```

如果之后发现：

```text
高速旋转下 yaw 线性化不稳定
姿态误差 wrap 问题明显
坐标变换误差导致 P 不一致
```

再引入 InEKF 会更有价值。

---

## 22. 推荐最终数学模型

推荐最终状态：

$$
\mathcal{X}
=
(R,p,v,\beta,\theta)
$$

其中：

$$
R \in SO(2)\ \text{或}\ SO(3)
$$

$$
p,v \in \mathbb{R}^3
$$

$$
\beta \in \mathbb{R}
$$

$$
\theta = [r_1,r_2,dza]^T \in \mathbb{R}^3
$$

误差状态：

$$
\delta x =
[
\delta\rho^T,
\delta v^T,
\delta\psi,
\delta\beta,
\delta r_1,
\delta r_2,
\delta dza
]^T
$$

预测：

$$
\hat{R}_{k+1}
=
\hat{R}_{k}\operatorname{Exp}(e_z\hat{\beta}_k\Delta t)
$$

$$
\hat{p}_{k+1}
=
\hat{p}_{k}+\hat{v}_{k}\Delta t
$$

$$
\hat{v}_{k+1}
=
\hat{v}_{k}
$$

$$
\hat{\beta}_{k+1}
=
\hat{\beta}_{k}
$$

$$
\hat{\theta}_{k+1}
=
\hat{\theta}_{k}
$$

观测：

$$
h_i(\mathcal{X})
=
\begin{bmatrix}
p + Rb_i(\theta) \\
\psi + \phi_i + \gamma
\end{bmatrix}
$$

误差线性化：

$$
\nu_i
\approx
H_i\delta x + v_i
$$

NIS：

$$
NIS_i
=
\nu_i^T(H_iPH_i^T+R_i)^{-1}\nu_i
$$

更新：

$$
\delta\hat{x}=K\nu
$$

$$
\hat{R}^+
=
\operatorname{Exp}(\delta\hat{\psi}e_z)\hat{R}
$$

$$
\hat{p}^+
=
\hat{p}+\delta\hat{\rho}
$$

$$
\hat{v}^+
=
\hat{v}+\delta\hat{v}
$$

$$
\hat{\beta}^+
=
\hat{\beta}+\delta\hat{\beta}
$$

$$
\hat{\theta}^+
=
\hat{\theta}+\Lambda_\theta\delta\hat{\theta}
$$

其中：

$$
\Lambda_\theta \ll I
$$

---

## 23. 实施建议

建议分阶段落地：

```text
P0:
  保留 UKF，先完成 hypothesis shadow evaluate + TopK + NIS 日志。

P1:
  启用 Top1 commit + null hypothesis + ambiguous 降级。

P2:
  加结构参数慢更新:
    r1 / r2 / dza 弱增益
    结构先验
    物理范围 clamp

P3:
  如果 yaw 高速旋转下 UKF 线性化 / 统计一致性仍不好，
  再把快状态部分替换为 IEKF / InEKF。

P4:
  引入窗口结构优化或短窗口因子图，专门慢更新结构参数。
```

---

## 24. 最终结论

InEKF 的数学本质是：

```text
不用 x - x_hat 定义误差，
而用 X X_hat^{-1} 或 X_hat^{-1} X 定义误差。
```

它的更新本质是：

```text
不用 x_hat += δx，
而用 X_hat ← Exp(δξ) X_hat
或 X_hat ← X_hat Exp(δξ)。
```

套到四装甲板自旋跟踪系统中，最合理的是：

```text
位姿 / yaw / 速度:
  用 InEKF 或 invariant error-state 处理

r1 / r2 / dza:
  作为欧氏慢参数处理

panel id / pair / order:
  仍然通过 hypothesis 枚举处理

是否更新:
  仍然由 NIS / likelihood / TopK / confidence / margin 决定
```

最终系统不是“纯 InEKF 替代 UKF”，而是：

```text
Hypothesis-InEKF Backend
+
Slow Structure Parameter Estimator
```

这与 Norm4 / Backend UKF V2 重构方案兼容，并且比直接把 `r1 / r2 / dza` 强行塞进每帧滤波更新更稳。
