# 当前 ProbabilityEngine 尚未实现部分分析文档

## 1. 文档目的

当前 `ProbabilityEngine` 已经实现：

```text
P_hit
+
时间窗融合
+
fire_score
+
fire_state 滞回
```

并已经成功解决：

```text
fire_advice 离散抖动
```

问题。

但当前实现仍属于：

```text
基于角度误差的概率门限模型
```

而不是完整的：

```text
随机弹道概率模型
```

本文档用于整理当前系统尚未真正实现的部分，以及后续推荐升级路径。

---

# 2. 当前系统已经实现的内容

当前系统已经具备以下能力：

## 2.1 连续命中概率输出

当前系统已经实现：

```math
P_{hit}=P_uP_v
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

即：

```text
二维独立高斯矩形命中概率
```

---

## 2.2 协方差退化

当前已经实现：

```math
\sigma(t)=\sigma_0+kt
```

即：

```text
飞行时间越长
→ 不确定性越大
```

该模型虽然简化，但工程上合理。

---

## 2.3 时间窗概率融合

当前已经实现：

```text
未来时间窗 τ_i
逐点评估 P_hit
```

并支持：

```text
max fusion
softmax fusion
```

即：

```math
P_{window}=\max_i P_i
```

或者：

```math
P_{window}
=
\frac{
\sum_i P_i e^{\beta P_i}
}{
\sum_i e^{\beta P_i}
}
```

---

## 2.4 Fire Score 与滞回状态机

当前已经实现：

```math
S_k
=
\alpha S_{k-1}
+
(1-\alpha)P_k
```

以及积分模式：

```math
S_k
=
S_{k-1}
+
\Delta t(c_+-c_-)
```

最终：

```math
fire\_state=
\begin{cases}
true,&S>S_{on}\\
false,&S<S_{off}
\end{cases}
```

这是当前系统中最重要且最成功的部分。

---

# 3. 当前系统本质上是什么模型

当前系统本质上是：

```text
角度误差概率门限模型
```

其核心为：

```math
e_u=d\tan(\Delta yaw)
```

```math
e_v=d\tan(\Delta pitch)
```

然后：

```math
u\sim\mathcal N(0,\sigma_u^2)
```

```math
v\sim\mathcal N(0,\sigma_v^2)
```

再计算：

```math
P_{hit}
=
P
\left(
|u-e_u|<w/2,
|v-e_v|<h/2
\right)
```

该模型已经可以有效解决：

```text
fire_advice 离散抖动
```

但它仍不是：

```text
完整随机弹道模型
```

---

# 4. 当前尚未真正实现的部分

当前缺失的核心部分如下。

---

# 5. 缺失部分一：真实弹道动力学

## 5.1 当前问题

当前系统并没有真正计算：

```text
子弹在空间中的位置
```

当前：

```math
e_u=d\tan(\Delta yaw)
```

```math
e_v=d\tan(\Delta pitch)
```

本质上只是：

```text
角度误差投影
```

并不是：

```text
真实子弹飞行轨迹
```

---

## 5.2 当前缺失的核心量

当前缺失：

```math
p_b(t)
```

即：

```text
子弹在 t 时刻的位置
```

以及：

```math
v_b(t)
```

即：

```text
子弹速度
```

---

## 5.3 推荐升级方案

建议建立真正的枪管系弹道：

```math
{}^{B_s}p_b(t)
=
{}^{B_s}p_0
+
\begin{bmatrix}
v_0t\\0\\0
\end{bmatrix}
+
\frac12{}^{B_s}gt^2
```

其中：

```math
{}^{B_s}g
=
{}^{B_s}R_O
\begin{bmatrix}
0\\0\\-g
\end{bmatrix}
```

---

## 5.4 推荐新增接口

建议新增：

```cpp
struct BulletState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};
```

新增函数：

```cpp
BulletState predictBullet(
    double flight_time,
    const ShooterState& shooter,
    double bullet_speed
);
```

---

# 6. 缺失部分二：真实装甲板未来预测

## 6.1 当前问题

当前系统在时间窗内：

```text
armor_center 不变
armor_normal 不变
```

即：

```text
未来 τ 的目标状态没有重新预测
```

因此当前实际上是：

```text
当前目标
+
不同 sigma
```

而不是：

```text
未来目标
+
未来弹道
```

---

## 6.2 推荐升级

对于每个未来时间点：

```math
t_s=t_0+\tau
```

应该预测：

```math
armor(t_{impact})
```

其中：

```math
t_{impact}
=
t_s+t_{delay}+t_f
```

---

## 6.3 推荐目标预测模型

初版可使用：

### 位置预测

```math
c_a(t+\Delta t)
=
c_a(t)+v_a\Delta t
```

### 姿态预测

若角速度为：

```math
\omega_a
```

则：

```math
R_a(t+\Delta t)
\approx
R_a(t)\exp([\omega_a]\Delta t)
```

初版也可以先保持姿态不变。

---

## 6.4 推荐新增接口

```cpp
class ArmorPredictor {
public:
    virtual ArmorState predict(double future_time) const = 0;
};
```

---

# 7. 缺失部分三：真实装甲板平面投影

## 7.1 当前问题

当前：

```text
e_u/e_v 直接由 yaw_error/pitch_error 得到
```

而不是：

```text
真实弹着点投影到装甲板平面
```

---

## 7.2 推荐升级

应建立：

```math
\Delta p
=
p_b-c_a
```

装甲板局部坐标：

```math
r_a,u_a,n_a
```

定义：

```math
J_a=
\begin{bmatrix}
r_a^T\\u_a^T
\end{bmatrix}
```

则：

```math
e_{uv}=J_a\Delta p
```

即：

```math
e_u=r_a^T\Delta p
```

```math
e_v=u_a^T\Delta p
```

这才是真正的：

```text
装甲板平面弹着误差
```

---

# 8. 缺失部分四：真实协方差投影

## 8.1 当前问题

当前：

```math
\sigma_u
```

```math
\sigma_v
```

直接经验设置。

而不是：

```text
真实三维协方差投影
```

---

## 8.2 推荐升级

建立三维弹道协方差：

```math
\Sigma_b(t)
=
\begin{bmatrix}
\sigma_x^2(t)&0&0\\
0&\sigma_y^2(t)&0\\
0&0&\sigma_z^2(t)
\end{bmatrix}
```

然后投影到装甲板平面：

```math
\Sigma_{uv}
=
J_a\Sigma_bJ_a^T
```

tracker 协方差：

```math
P_a
```

同样投影：

```math
\Sigma_{tracker}^{uv}
=
J_aP_aJ_a^T
```

最终：

```math
\Sigma_{total}^{uv}
=
\Sigma_b^{uv}+\Sigma_{tracker}^{uv}
```

---

# 9. 缺失部分五：真实 flight time 求解

## 9.1 当前问题

当前：

```text
flight_time 只是输入参数
```

没有真正由目标距离和弹道决定。

---

## 9.2 推荐升级

建立：

```math
x_a=v_0t_f+\frac12g_xt_f^2
```

求解：

```math
t_f
=
\frac{
-v_0+
\sqrt{v_0^2+2g_xx_a}
}{g_x}
```

当：

```math
g_x\to0
```

退化为：

```math
t_f=x_a/v_0
```

---

# 10. 缺失部分六：法向速度建模

## 10.1 当前问题

当前没有真正使用：

```text
入射方向
```

以及：

```text
子弹与装甲板法向夹角
```

---

## 10.2 推荐升级

建立：

```math
v_n
=
\max(0,-v_b^Tn_a)
```

定义权重：

```math
w_n
=
clip(v_n/v_{ref},w_{min},1)
```

最终：

```math
P'_{hit}=P_{hit}w_n
```

---

# 11. 缺失部分七：真正的 sigma-point 传播

## 11.1 当前问题

当前：

```cpp
sigma_u += sigma_v0 * c
```

只是：

```text
经验性增大 sigma
```

并不是：

```text
Unscented Transform
```

---

## 11.2 推荐升级

定义随机参数：

```math
x=
\begin{bmatrix}
\Delta v_0\\
\Delta t_d
\end{bmatrix}
```

生成 sigma points：

```math
\chi_i
```

传播：

```math
e_i=f(\chi_i)
```

统计：

```math
\mu_e
```

```math
\Sigma_e
```

最终：

```math
\Sigma_{total}^{uv}
+=\Sigma_e
```

---

# 12. 缺失部分八：真实随机过程相关性

## 12.1 当前问题

当前不同时间点：

```text
被视为独立评估
```

但实际上：

```text
同一发子弹不同时间点高度相关
```

因为：

```text
同一个初速误差会影响整条轨迹
```

---

## 12.2 当前近似是否合理

合理。

当前：

```math
P_{window}=\max_iP_i
```

已经是比较合理的工程近似。

不建议：

```math
1-\prod_i(1-P_i)
```

因为这要求事件独立。

---

# 13. 推荐升级优先级

推荐升级顺序如下。

---

## 第一阶段（最重要）

真正建立：

```math
p_b(t)
```

实现：

```text
真实 bullet position
真实 bullet velocity
```

替代：

```math
e_u=d\tan(\Delta yaw)
```

---

## 第二阶段

实现：

```text
未来目标预测
```

即：

```math
armor(t+\tau)
```

---

## 第三阶段

实现：

```math
\Sigma_b(t)
```

并投影：

```math
\Sigma_{uv}=J\Sigma J^T
```

---

## 第四阶段

实现：

```text
法向速度权重
```

---

## 第五阶段

实现：

```text
sigma-point propagation
```

---

# 14. 当前模型的工程评价

当前模型虽然还不是真正的随机弹道模型，但：

```text
方向完全正确
```

因为当前已经真正实现：

```text
连续概率
+
时间窗
+
fire_score
+
fire_state
```

这已经成功解决：

```text
fire_advice 离散抖动
```

问题。

因此当前系统已经具备：

```text
第一代概率 fire gate
```

能力。

真正下一步重点不是继续调 gate 参数，而是：

```text
建立真实 bullet trajectory
```

以及：

```text
真实装甲板平面弹着投影
```

这将使系统从：

```text
角度误差概率模型
```

真正升级为：

```text
随机弹道概率模型
```

