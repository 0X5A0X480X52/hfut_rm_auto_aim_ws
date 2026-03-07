# MPC 轨迹规划（包含控制延迟）

---

## - 设计思路

### 1️⃣ 状态估计与预测

* 与原方案相同：使用 **卡尔曼滤波 / CA / Singer 模型**预测每个装甲板未来状态：
  \[
  X_{predict} = {x_{target}[t+1], \dots, x_{target}[t+N]}
  \]
* 输出每个装甲板的 **yaw/pitch 目标**及 **速度**。

---

### 2️⃣ 预测窗口内选板策略

* 将装甲板划分为若干扇区，每个未来时刻选择扇区最大面向的板作为目标：
  \[
  x_{target}^{sel}[t+k], \quad k = 1, \dots, N
  \]
* 考虑自旋和跳变，保证每步唯一目标。

---

### 3️⃣ 弹道解算器

* 对每个选板目标，使用弹道解算器计算期望的 yaw/pitch：
  \[
  X_{ref}[k] = \begin{bmatrix} \text{yaw}*{ref}[k] \ \text{pitch}*{ref}[k] \end{bmatrix}, \quad k = 1..N
  \]
* 弹道解算考虑飞行时间、空气阻力等。

---

### 4️⃣ 控制延迟建模

假设控制延迟为 \( \tau \) 秒（或 \( d \) 步预测窗口）：

* 实际下位机执行的控制输入：
  \[
  u_{exec}[t] = u[t - d]
  \]

* 为考虑延迟，可在 MPC 中**扩展预测模型**：
  \[
  x_{gimbal}[t+1] = A x_{gimbal}[t] + B u_{exec}[t] = A x_{gimbal}[t] + B u[t-d]
  \]

* 对未来 N 步预测，可用**延迟映射矩阵** ( \mathcal{B}*d ) 替换原预测矩阵 ( \mathcal{B} )：
  \[
  X*{gimbal} = \mathcal{A} x_{gimbal}[t] + \mathcal{B}_d U
  \]

* \(\mathcal{B}_d\) 的构建：

  * 前 d 步的控制输入尚未影响状态（因延迟），因此对应列为 0。
  * 第 d+1 步开始，控制输入逐步生效：
    \[
    \mathcal{B}_d =
    \begin{bmatrix}
    0 & 0 & \cdots & 0 & B & 0 & \cdots & 0 \
    0 & 0 & \cdots & 0 & AB & B & \cdots & 0 \
    \vdots & \vdots & & \vdots & \vdots & \vdots & & \vdots \
    \end{bmatrix}
    \]

---

### 5️⃣ 云台状态与控制定义

* 云台状态：
  \[
  x_{gimbal}[k] =
  \begin{bmatrix} yaw[k] \ pitch[k] \ \dot{yaw}[k] \ \dot{pitch}[k] \end{bmatrix}
  \]

* 控制输入：
  \[
  u[k] =
  \begin{bmatrix} \ddot{yaw}[k] \ \ddot{pitch}[k] \end{bmatrix}
  \]

* 预测模型：
  \[
  x_{gimbal}[k+1] = A x_{gimbal}[k] + B u[k-d] \quad (\text{考虑延迟 } d)
  \]

---

### 6️⃣ MPC 代价函数（带延迟）

\[
J = \sum_{k=1}^{N}
(x_{gimbal}[k] - X_{ref}[k])^T Q (x_{gimbal}[k] - X_{ref}[k]) + u[k]^T R u[k] + (u[k] - u[k-1])^T S (u[k] - u[k-1])
\]

- 与原代价函数相同，只是未来状态预测 (x_{gimbal}[k]) 已包含延迟映射。

---

### 7️⃣ 转化为二次规划 (QP)

* 代价函数标准形式：
  \[
  J = \frac{1}{2} U^T H U + f^T U
  \]

* Hessian 矩阵：
  \[
  H = 2 (\mathcal{B}_d^T Q \mathcal{B}_d + R + D^T S D)
  \]

* 线性项：
  \[
  f = 2 \mathcal{B}_d^T Q (\mathcal{A} x_{gimbal}[t] - X_{ref})
  \]

* 差分矩阵 D 保持不变：
  \[
  D =
  \begin{bmatrix}
  1 & -1 & 0 & \cdots \
  0 & 1 & -1 & \cdots \
  \vdots & \vdots & \ddots & \vdots \
  0 & 0 & \cdots & 1
  \end{bmatrix}
  \]

* 约束：
  \[
  G U \le h
  \]
  包括云台角度、速度、加速度限制。

---

### 8️⃣ 求解与执行

1. 使用 QP 求解器（OSQP / qpOASES）求解：
   \[
   U^* = { u^*[t], u^*[t+1], \dots, u^*[t+N-1] }
   \]

2. 实际执行控制：
   \[
   u_{exec}[t] = u^*[t - d]
   \]

3. 更新云台状态，滚动预测窗口，重复优化，实现 **考虑延迟的实时 MPC**。

---

### 9️⃣ 特点

* **考虑控制延迟**：通过延迟映射矩阵 (\mathcal{B}_d)，优化结果提前补偿延迟。
* **平滑提前瞄准**：即使目标跳变，MPC 也会在延迟内平滑转向。
* **双通道优化**：yaw/pitch 同时优化，参考弹道解算结果。
* **选板简洁**：扇区法保证每时刻唯一目标。
* **可调权重**：Q、R、S 权重可根据实际需求调整，平衡跟踪精度与控制平滑性。
