# 装甲板的损失函数设计

考虑到英雄机器人发射机构质量较大，云台不适合进行较大的转动，设计的损失函数应能反应以下特性：

1. 其每次yaw和pitch的转动应该在一定阈值内
2. 选择目标装甲板时其yaw和pitch移动应尽可能小
3. yaw应尽可能瞄准向目标机器人的旋转中心

其中第一项可以通过设定转动阈值解决：

- 设置一个转动上限，防止转角过大，云台不好控制。
- 设置一个转动下限，控制机构可能存在虚位，过小的转动幅度可能也不能精确控制。

下面我们主要关心第二点和第三点。考虑到云台移动幅度越大，控制延时和精度都会下降。因此我们假设云台移动越大，打击效果的精准度越差。

不妨将当 $t$ 时刻的云台状态记为 $\boldsymbol{\theta} = ( \theta_{yaw},\theta_{pitch})$ 为定值。下一时刻的云台状态记为 $\boldsymbol{\theta}' = (\theta_{yaw}'，\theta_{pitch}' )$，通过待选装甲板的位置坐标解算得到，是该问题的变量。此外，记云台正对目标装甲板旋转中心的方向为基准方向，装甲板与旋转中心连线方向与该基准方向的夹角为 $\theta_{center}$，也是该问题的一个变量。

为描述第二点和第三点特性，可写出以下两个损失函数：
$$\begin{align*}
    & \min d_{center} = \theta_{center} \\
    & \min d_{armor} = \| \boldsymbol{\theta} - \boldsymbol{\theta}' \|_2 = ( \theta_{yaw} - \theta_{yaw}' )^2 + ( \theta_{pitch} - \theta_{pitch}' )^2
\end{align*}$$

基于多目标优化的思想，可以引入一个融合系数 $\lambda$，通过其对上述两个损失函数加权平均后得到一个多目标的损失函数:
$$ \min loss = \lambda d_{center} + (1-\lambda) d_{armor} $$

此外，为了增强损失函数的表达能力，引入两个因子 $\alpha_{center}$ 和 $\alpha_{armor}$ 作为损失函数的系数，用于描述更多的特性。损失函数改写为:
$$ \min loss = \lambda \alpha_{center}d_{center} + (1-\lambda) \alpha_{armor}d_{armor} $$

### 融合系数 $\lambda$ 的设计

融合系数 $\lambda$ 应该满足归一化特性，即 $\lambda \in [0,1]$。

越远离基准方向，我们希望回正的趋势越强，而打击的精准度似乎就没那么重要了，可以适当降低要求。反之，如果在基准方向附近，打击精度就会更加重要。

可以设 $\lambda$ 是 $d_{center}$ 的一个函数，$d_{center}$ 越小，其值也越小，因此 $\lambda(d_{center})$ 应该近似为一个增函数函数。

此外，我们希望超过一定阈值后，$d_{center}$ 其较大的作用；未超过该阈值，可近似忽略 $d_{center}$ 的作用。因此， $\lambda(d_{center})$ 的图像应该近似一个 “锅”，靠近中心部分平坦，超过某一阈值后函数值快速上升。

神经网络中的 sigmoid 函数就是满足上述性质的函数。

例如，可以将 $\lambda(d_{center})$ 设计为以下的一些函数：

$$\lambda(d_{center}) = \frac{1}{2} \left[ \tanh\left( \frac{|d_{center}| - R}{σ} \right) + \frac{\pi}{2} \right]，\sigma>0$$

$$\lambda(d_{center}) = \dfrac{1}{1+e^{-k(|d_{center}| - R)}}，k,R>0$$

$$\lambda(d_{center}) = \dfrac{1}{c} \ln( 1 + e^{c(|d_{center}| - R)} ) \operatorname{mod} 1，c>0$$

### 因子 $\alpha_{center}$ 的设计

可以用 $\alpha_{center}$ 增强回正趋势，即距基准方向越远，其回正项产生的效用越大。由于是求极小化，则要求 $d_{center}$ 越大，$\alpha_{center}$ 越小。

例如：
$$\alpha_{center} = 1 - \left(\dfrac{d_{center}}{\sigma} \right)^2$$

### 因子 $\alpha_{armor}$ 的设计

可以用 $\alpha_{armor}$ 增强回瞄准趋势，即移动幅度越小，其瞄准项产生的效用越大。由于是求极小化，则要求 $d_{center}$ 越小，$\alpha_{armor}$ 也越小。

例如：
$$\alpha_{armor} = - e^{-\left(\frac{d_{armor}}{\sigma}\right)^2}$$
$$\alpha_{armor} = \ln (w|d_{armor}| + v)，w>0$$