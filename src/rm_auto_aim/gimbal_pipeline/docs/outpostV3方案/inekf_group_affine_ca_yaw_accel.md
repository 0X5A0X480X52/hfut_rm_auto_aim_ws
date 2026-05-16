# 结构参数已知条件下的 InEKF 建模说明

## 1. 问题背景

在 RoboMaster 装甲板目标跟踪问题中，如果目标的结构参数已经全部已知，例如：

- 装甲板相对于目标中心的偏移量；
- 装甲板编号与 yaw 相位关系；
- 装甲板高度；
- 小装甲 / 大装甲尺寸；
- 相机外参；
- 装甲板刚体结构关系；

那么滤波器不需要再估计结构参数，而可以只估计目标中心的运动状态。

此时，装甲板观测可以被看作：

> 对一个已知刚体上固定点的观测。

这会显著简化 InEKF 的建模，并使系统更接近标准的 group-affine 形式。

---

## 2. 推荐状态设计

在“结构参数已知 + CA 中心运动 + yaw 匀加速”的设定下，可以将状态设计为：

\[
X = \{R_z(\psi),\ p,\ v,\ a,\ \omega,\ \alpha\}
\]

其中：

| 符号 | 含义 |
|---|---|
| \(R_z(\psi)\) | 目标绕 z 轴的 yaw 旋转 |
| \(p\) | 目标中心位置 |
| \(v\) | 目标中心速度 |
| \(a\) | 目标中心加速度 |
| \(\omega\) | yaw 角速度 |
| \(\alpha\) | yaw 角加速度 |

该状态可以理解为：

- 平动部分：中心位置、速度、加速度；
- 转动部分：yaw、yaw 角速度、yaw 角加速度；
- 结构部分：不进入状态，只作为已知常量参与观测模型。

---

## 3. 预测模型

采用 CA 中心运动和 yaw 匀加速模型：

\[
\dot p = v
\]

\[
\dot v = a
\]

\[
\dot a = 0
\]

\[
\dot\psi = \omega
\]

\[
\dot\omega = \alpha
\]

\[
\dot\alpha = 0
\]

也可以合并写作：

\[
\dot p=v,\quad \dot v=a,\quad \dot a=0,\quad
\dot\psi=\omega,\quad \dot\omega=\alpha,\quad \dot\alpha=0
\]

这个模型本质上由两条积分链组成：

\[
p \leftarrow v \leftarrow a
\]

\[
\psi \leftarrow \omega \leftarrow \alpha
\]

其中第一条描述目标中心的平动，第二条描述目标 yaw 的转动。

---

## 4. 离散形式

若时间间隔为 \(\Delta t\)，则可以写成：

\[
p_{k+1} = p_k + v_k\Delta t + \frac{1}{2}a_k\Delta t^2
\]

\[
v_{k+1} = v_k + a_k\Delta t
\]

\[
a_{k+1} = a_k
\]

\[
\psi_{k+1} = \psi_k + \omega_k\Delta t + \frac{1}{2}\alpha_k\Delta t^2
\]

\[
\omega_{k+1} = \omega_k + \alpha_k\Delta t
\]

\[
\alpha_{k+1} = \alpha_k
\]

实际工程中可以在 \(a\) 和 \(\alpha\) 上加入过程噪声，表示目标机动的不确定性：

\[
a_{k+1} = a_k + w_a
\]

\[
\alpha_{k+1} = \alpha_k + w_\alpha
\]

---

## 5. group-affine 条件是否可以满足

### 5.1 结论

在该建模方式下：

> 结构参数已知 + CA 中心运动 + yaw 匀加速，可以构造成满足 group-affine 条件的 InEKF 预测模型。

核心原因是：

1. 中心运动是规整的线性积分链；
2. yaw-only 转动位于 \(SO(2)\)，其结构相对简单；
3. 加速度和 yaw 角加速度作为状态的一部分，仅承担积分链中的高阶状态；
4. 结构参数不进入预测模型，只作为观测模型中的已知常量。

---

### 5.2 group-affine 条件形式

InEKF 中常见的 group-affine 条件可以写作：

\[
f(XY)=f(X)Y + Xf(Y) - Xf(I)Y
\]

其中：

- \(X, Y\) 是 Lie 群上的状态；
- \(f(\cdot)\) 是系统动力学；
- \(I\) 是群单位元。

满足该条件后，不变误差的传播可以与当前估计值解耦，误差动力学具有较好的 log-linear 性质。

---

## 6. 误差传播形式

若定义误差状态为：

\[
\delta x =
\begin{bmatrix}
\delta p \\
\delta v \\
\delta a \\
\delta \psi \\
\delta \omega \\
\delta \alpha
\end{bmatrix}
\]

则误差传播近似为：

\[
\delta \dot p = \delta v
\]

\[
\delta \dot v = \delta a
\]

\[
\delta \dot a = 0
\]

\[
\delta \dot \psi = \delta \omega
\]

\[
\delta \dot \omega = \delta \alpha
\]

\[
\delta \dot \alpha = 0
\]

即：

\[
\delta \dot x = F\delta x
\]

其中：

\[
F =
\begin{bmatrix}
0 & I & 0 & 0 & 0 & 0 \\
0 & 0 & I & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\]

这个 \(F\) 的关键特点是：

> 它不依赖当前估计状态。

这正是 InEKF 相比普通 EKF 更稳定的重要原因之一。

---

## 7. 观测模型

由于结构参数已知，第 \(i\) 块装甲板相对于目标中心的偏移量 \(r_i\) 是常量。

装甲板位置观测可以写成：

\[
z_i = p + R_z(\psi)r_i + n
\]

其中：

| 符号 | 含义 |
|---|---|
| \(z_i\) | 第 \(i\) 块装甲板的观测位置 |
| \(p\) | 目标中心位置 |
| \(R_z(\psi)\) | 目标 yaw 旋转 |
| \(r_i\) | 第 \(i\) 块装甲板相对目标中心的已知偏移 |
| \(n\) | 观测噪声 |

也可以构造不变残差：

\[
e_i = \hat R^\top (z_i - \hat p) - r_i
\]

该残差表示：

> 将观测到的装甲板位置变换回目标自身坐标系后，与已知结构偏移 \(r_i\) 进行比较。

这种形式与刚体 Lie 群作用比较匹配。

---

## 8. 为什么 yaw 匀加速不会破坏结构

这里的关键是：目标只估计 yaw，而不是完整三维姿态。

yaw-only 对应 \(SO(2)\)，其角度组合形式较简单：

\[
\psi_1 \oplus \psi_2 = \psi_1 + \psi_2
\]

因此：

\[
\dot\psi = \omega
\]

\[
\dot\omega = \alpha
\]

\[
\dot\alpha = 0
\]

可以自然地作为积分链加入系统。

如果估计完整 \(SO(3)\) 姿态，并引入三维角速度和角加速度，则由于三维旋转不可交换，模型会复杂很多，不一定能保持简单的 group-affine 结构。

---

## 9. 哪些情况会破坏 group-affine

虽然当前组合可以构造得很干净，但以下情况会破坏严格的 group-affine 条件。

### 9.1 结构参数作为状态估计

如果将结构参数也放入状态，例如：

\[
X = \{R_z(\psi), p, v, a, \omega, \alpha, r_i\}
\]

并在线慢更新：

\[
r_i \leftarrow r_i + \Delta r_i
\]

那么状态就不再是单纯的刚体运动状态，而变成：

> 刚体运动状态 + 慢变结构参数。

这种形式通常难以严格满足标准 group-affine 条件。

---

### 9.2 引入经验机动模型

如果加速度不是状态积分链，而是某个经验函数：

\[
\dot v = g(p, v, \psi, \omega)
\]

例如根据速度方向、残差大小、预测误差、目标类别进行启发式修正，那么误差传播会依赖当前估计状态，不再是标准 group-affine。

---

### 9.3 引入 IMM 模式切换

如果同时使用：

- CV；
- CA；
- CTRV；
- CS；
- abrupt maneuver model；

并通过 IMM 进行模式融合，则每个子模型可能是 InEKF-compatible 的，但整体不再是一个单一的标准 group-affine 系统。

更准确地说，它是：

> 多个 InEKF-compatible 子模型组成的工程化混合滤波器。

---

### 9.4 直接使用像素重投影观测

若观测模型直接写成：

\[
u = \pi(T_{cw}(p + R_z(\psi)r_i))
\]

其中 \(\pi(\cdot)\) 是相机投影函数，则观测模型会引入更强的非线性。

这不一定破坏预测模型的 group-affine 性质，但会使观测更新更接近普通非线性 EKF 的线性化形式。

工程上更推荐先通过 PnP 得到装甲板 3D 观测，再使用：

\[
z_i = p + R_z(\psi)r_i + n
\]

进行滤波更新。

---

## 10. 推荐工程实现方式

### 10.1 严格核心模型

建议将严格 InEKF 核心保持为：

\[
X = \{R_z(\psi), p, v, a, \omega, \alpha\}
\]

预测：

\[
\dot p=v,\quad \dot v=a,\quad \dot a=0
\]

\[
\dot\psi=\omega,\quad \dot\omega=\alpha,\quad \dot\alpha=0
\]

观测：

\[
z_i = p + R_z(\psi)r_i + n
\]

其中 \(r_i\) 固定不变。

---

### 10.2 工程增强项放在滤波器外部

以下内容建议放在滤波器外层，而不是塞进严格 InEKF 状态模型：

- 多假设数据关联；
- 装甲板编号候选；
- NIS / Mahalanobis gate；
- 目标丢失与重捕获；
- 发散检测；
- 机动模式判定；
- 结构参数慢更新；
- 经验规则修正。

也就是说，推荐架构是：

```text
PnP / BA 观测
    ↓
多假设数据关联
    ↓
InEKF 核心预测与更新
    ↓
NIS / 残差一致性检查
    ↓
目标选择与火控预测
```

其中 InEKF 只负责一个干净、规整、可解释的状态估计核心。

---

## 11. 最终结论

在以下条件下：

- 结构参数全部已知；
- 目标中心采用 CA 运动模型；
- yaw 采用匀加速模型；
- 装甲板偏移 \(r_i\) 不参与估计；
- 不引入强经验动力学；
- 不把 IMM、结构慢更新塞进同一个严格模型；

则可以构造一个比较标准的 InEKF，并且其预测模型可以满足 group-affine 条件。

最终推荐模型为：

\[
X = \{R_z(\psi), p, v, a, \omega, \alpha\}
\]

\[
\dot p=v,\quad \dot v=a,\quad \dot a=0
\]

\[
\dot\psi=\omega,\quad \dot\omega=\alpha,\quad \dot\alpha=0
\]

\[
z_i = p + R_z(\psi)r_i + n
\]

因此，对于当前装甲板跟踪问题，这一组合是合理且理论上比较干净的建模方案。
