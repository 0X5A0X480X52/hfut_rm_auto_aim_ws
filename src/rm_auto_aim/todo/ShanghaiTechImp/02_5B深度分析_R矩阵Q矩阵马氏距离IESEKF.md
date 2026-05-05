# 5B 深度分析：从观测噪声建模到 IESEKF 的改进路径

> 对应参考方案 §5B.1–§5B.6，涵盖 R 矩阵、Q 矩阵、马氏距离匹配、IESEKF、双装甲板观测处理

## 一、观测噪声 R 矩阵

### 1.1 当前实现

**文件**: [gimbal_pipeline/src/max_entropy_tracker/filters/dual_radius_spin_ukf.cpp:247-257](../../src/max_entropy_tracker/filters/dual_radius_spin_ukf.cpp)

```cpp
double np_ = config_.ukf.obs_noise_pos;  // 默认 0.03 (YAML)
double ny = config_.ukf.obs_noise_yaw;   // 默认 0.03 (YAML)
Eigen::Matrix4d R = Eigen::Vector4d(np_*np_, np_*np_, np_*np_, ny*ny).asDiagonal();
```

**问题**：
1. X、Y、Z 三个空间维度使用同一个标量噪声 std——但深度方向的 PnP 误差远大于横向
2. R 是对角矩阵——假设 X/Y/Z/yaw 观测噪声互相独立。实际中一个像素的方位角抖动会同时影响 X 和 Y 坐标
3. 噪声不随目标距离动态变化——1° 方位角误差在 1m 处对应 1.7cm，在 8m 处对应 14cm

唯一的动态调整基于置信度（第 252-257 行）：
```cpp
if (height_confidence < 0.3) R(2, 2) *= 100.0;     // 高度不可信时放大 Z 噪声
if (position_confidence < 0.8) {
  double scale = 100.0 / std::max(position_confidence, 0.1);
  R(0, 0) *= scale; R(1, 1) *= scale;               // 位置不可信时放大 X/Y 噪声
}
```

### 1.2 参考方案的核心思路

观测噪声的物理来源天然在球坐标（Yaw-Pitch-Distance，简称 YPD）中是对角的：

| 参数 | 含义 | 物理来源 | 参考值 |
|------|------|---------|--------|
| `σ_azi` | 方位角 std | 像素水平抖动 / 焦距 | ~0.063 rad (≈3.6°) |
| `σ_ele` | 俯仰角 std | 像素垂直抖动 / 焦距 | ~0.063 rad (≈3.6°) |
| `σ_dist_coeff` | 距离噪声系数 | PnP tvec 深度不确定度 | 0.05–0.20 (距离的 5-20%) |
| `σ_yaw` | 装甲板朝向 std | NN 解算 yaw 本征精度 | ~0.30 rad (≈17°) |

在 YPD 系下，R 是对角的：

```
R_ypd = diag(σ_azi², σ_ele², σ_dist², σ_yaw²)
```

然后通过雅可比 J = ∂(XYZ)/∂(YPD) 旋转到笛卡尔坐标系：

```
R_xyz = J × R_ypd × J^T
```

核心效果：
- 方位角噪声按弧长 `dist × σ_angle` 投射到垂直于视线的方向
- 俯仰角噪声投射到高度方向
- 深度噪声投射到视线方向
- 噪声量随距离线性放大（弧长 = r × θ）

### 1.3 改进方案

改动位置：`dual_radius_spin_ukf.cpp` 中单观测 R 矩阵构造处。

```cpp
// === YPD 球坐标观测噪声建模 ===
// 步骤 1: 球坐标下的独立噪声参数
const double sigma_azi  = config_.ukf.ypd_sigma_azi;   // 建议默认 0.063 rad
const double sigma_ele  = config_.ukf.ypd_sigma_ele;   // 建议默认 0.063 rad
const double sigma_dist_coeff = config_.ukf.ypd_sigma_dist_coeff; // 建议 0.10
const double sigma_yaw  = config_.ukf.obs_noise_yaw;   // 复用现有 yaw 噪声参数

// 步骤 2: 从当前观测计算球坐标参数
double x = z_pred(0), y = z_pred(1), z = z_pred(2);
double dist_xy = std::sqrt(x*x + y*y);
double dist_3d = std::sqrt(dist_xy*dist_xy + z*z);
double cos_ele = dist_xy / dist_3d;  // cos(俯仰角)
double sin_ele = z / dist_3d;
double cos_azi = (dist_xy > 1e-6) ? x / dist_xy : 1.0;
double sin_azi = (dist_xy > 1e-6) ? y / dist_xy : 0.0;

double sigma_dist = sigma_dist_coeff * dist_3d;  // 距离越远噪声越大

// 步骤 3: 构造 YPD 到 XYZ 的雅可比
// x = dist * cos(ele) * cos(azi)
// y = dist * cos(ele) * sin(azi)
// z = dist * sin(ele)
Eigen::Matrix4d J = Eigen::Matrix4d::Zero();
// ∂x/∂azi = -dist * cos(ele) * sin(azi) = -y
J(0, 0) = -y;
// ∂x/∂ele = -dist * sin(ele) * cos(azi) = -x * z / dist_xy
J(0, 1) = (dist_xy > 1e-6) ? -x * z / dist_xy : 0.0;
// ∂x/∂dist = cos(ele) * cos(azi) = x / dist_3d
J(0, 2) = (dist_3d > 1e-6) ? x / dist_3d : 1.0;

// ∂y/∂azi = dist * cos(ele) * cos(azi) = x
J(1, 0) = x;
// ∂y/∂ele = -dist * sin(ele) * sin(azi) = -y * z / dist_xy
J(1, 1) = (dist_xy > 1e-6) ? -y * z / dist_xy : 0.0;
// ∂y/∂dist = cos(ele) * sin(azi) = y / dist_3d
J(1, 2) = (dist_3d > 1e-6) ? y / dist_3d : 0.0;

// ∂z/∂ele = dist * cos(ele) = dist_xy
J(2, 1) = dist_xy;
// ∂z/∂dist = sin(ele) = z / dist_3d
J(2, 2) = (dist_3d > 1e-6) ? z / dist_3d : 0.0;

// yaw 维度独立（假设 pitch/roll 固定后 yaw 与位置不相关）
J(3, 3) = 1.0;

// 步骤 4: YPD 对角噪声 → XYZ 满矩阵
Eigen::Vector4d r_ypd(sigma_azi * sigma_azi,
                       sigma_ele * sigma_ele,
                       sigma_dist * sigma_dist,
                       sigma_yaw * sigma_yaw);
Eigen::Matrix4d R = J * r_ypd.asDiagonal() * J.transpose();

// 步骤 5: 保留现有的置信度缩放（叠加到 YPD 之上）
if (height_confidence < 0.3) R(2, 2) *= 100.0;
if (position_confidence < 0.8) {
  double scale = 100.0 / std::max(position_confidence, 0.1);
  R(0, 0) *= scale; R(1, 1) *= scale;
}
```

**关键效果**：
- 核心效果：距离越远，R 矩阵自动放大（通过 `σ_dist_coeff * dist_3d` 和雅可比中的距离因子）
- X 和 Y 之间出现非零协方差（J 的第 0 列同时影响 X 和 Y）
- Z 方向的噪声通过 `σ_ele` 和 `σ_dist` 的混合贡献
- 改动仅影响 R 矩阵构造，不涉及 UKF 其余逻辑

---

## 二、过程噪声 Q 矩阵

### 2.1 当前实现已是 Piecewise White Noise

当前实现的 Q 矩阵构造与参考方案描述完全一致：

| 运动模型 | Q 结构 | 文件位置 |
|---------|--------|---------|
| CV (匀速) | `Q = q² · [[dt³/3, dt²/2], [dt²/2, dt]]` | `translation.hpp:49-61` |
| CA (匀加速) | `Q = q² · [[dt⁵/20, dt⁴/8, dt³/6], [dt⁴/8, dt³/3, dt²/2], [dt³/6, dt²/2, dt]]` | `translation.hpp:105-119` |
| Singer | 时间相关加速度解析 Q（3×3） | `translation.hpp:173-211` |
| 旋转 CV/CA | 同结构 | `rotation.hpp:36-80` |
| 结构参数 | 随机游走：`Q = diag(qr²·dt, qr²·dt, qd²·dt·0.1)` | `structural.hpp:31-37` |

### 2.2 Q 参数对标参考方案

参考方案 sp25 的参考值：

| 参数 | sp25 参考 | 含义 | 当前 YAML 值 | 分析 |
|------|----------|------|-------------|------|
| `p_coord` (v1) | 100 | 平移加速度方差, σ_a = 10 m/s² (≈1g) | `singer_sigma: 0.5` | **偏保守** — 当前预期加速度 std 仅 0.5 m/s²，对于高速机动目标响应慢 |
| `p_yaw` (v2) | 400 | 角加速度方差, σ_α = 20 rad/s² | `spin_process_noise_delta_acc: 45.0` | **偏激进** — 当前预期角加速度 std 约 6.7 rad/s²，对旋转变化的跟随更快但可能引入抖动 |
| `p_r` | 小值 | 半径变化方差 | `process_noise_r: 0.002` | 合理 — 机械上半径不应突变 |
| `p_dz` | 小值 | 高度差变化方差 | `process_noise_dz: 0.00005` | 合理 |

**关键差距**：`singer_sigma = 0.5` 意味着预期目标平移加速度 std 仅 0.5 m/s²。参考方案用 `v1=100`（σ_a = 10 m/s² ≈ 1g），覆盖步兵从静止到 10 m/s 的 1 秒变速。如果实际目标（如哨兵、英雄）机动能力更强，当前 Q 会导致跟踪滞后明显增大。

**建议**：
- `motion.singer_sigma` 从 0.5 提高到 2.0–5.0（根据实测目标机动能力调整）
- 或直接切换到 CA 模型，将 `ca_process_noise_acc` 从 0.1 提高到 1.0–3.0

---

## 三、马氏距离匹配

### 3.1 当前实现

代码已完整实现马氏距离门控（`dual_radius_spin_ukf.cpp:598-631`）：

```cpp
double chi2_yaw = (innov(3) * innov(3)) / Pzz(3, 3);
bool yaw_pass = (chi2_yaw <= threshold);       // 默认 9.49 = χ²(1) 99.8% 界

Eigen::Vector3d innov_pos = innov.head<3>();
Eigen::Matrix3d Pzz_pos = Pzz.topLeftCorner<3, 3>();
double chi2_pos = innov_pos.transpose() * Pzz_pos.inverse() * innov_pos;
bool pos_pass = (chi2_pos <= threshold * 3.0);  // χ²(3) 的 99.8% 界

return yaw_pass || pos_pass;  // OR 逻辑：任一通过即接受
```

**但默认关闭**：`ukf.enable_innovation_gating: false`

### 3.2 改进建议

1. **立即启用**：将 `enable_innovation_gating: true`

2. **调整阈值**：当前 9.49 对应 χ²(1) 的 99.8% 置信界——非常严格，大量合法观测可能被拒绝。建议降低：
   - P0 验证阶段：`6.64`（χ²(1) 99%）或更宽松的 `5.02`（χ²(1) 97.5%）
   - 线上稳定后：根据 NIS 统计回归到合适值

3. **依赖 YPD 噪声建模**：门控的效果强依赖 R 矩阵的准确性。做完"改进一"后，S = HPH^T + R 中的 R 更准确地反映各方向真实噪声尺度，门控的区分度会显著提升。

4. **OR 逻辑保持**：当前 yaw_pass || pos_pass 的 OR 逻辑是合理的——当 yaw 通过但位置未通过时（观测位置跳变但朝向连续），不应拒绝；反之亦然。不需要改为 AND。

---

## 四、IESEKF 迭代更新

### 4.1 当前状态

当前实现使用 UKF 而非 EKF。UKF 通过无迹变换（sigma 点传播）达到 2 阶泰勒精度，而 EKF 只有 1 阶。这是 UKF 的内生优势——不需要 IESEKF 来补偿一阶线性化误差。

但是，在大机动或跟踪初始化阶段，状态分布可能高度非高斯，此时即使 UKF 的更新也可能不够精确。参考方案 IESEKF 的"多次迭代、逐步逼近真值"思路仍然适用。

### 4.2 改进方案：Iterated UKF (IUKF)

在 `dual_radius_spin_ukf.cpp` 的 `update()` 函数中，对单观测更新增加可选迭代循环：

```cpp
// 伪代码示意
Eigen::VectorXd x_upd = x_pred;
Eigen::MatrixXd P_upd = P_pred;

for (int iter = 0; iter < config_.ukf.iukf_max_iterations; ++iter) {  // 默认 2-3
  // 在当前 x_upd 处重新生成 sigma 点
  generateSigmaPoints(x_upd, P_upd);

  // 重新计算观测预测和交叉协方差
  predictObservation();
  computeCrossCovariance();

  // 检查收敛（创新量变化 < 阈值）
  if (innovation_change < config_.ukf.iukf_convergence_threshold) break;

  // 执行 UKF 更新
  x_upd = x_pred + K * (z - z_pred - H * (x_pred - x_upd));
  P_upd = P_pred - K * Pzz * K.transpose();
}
```

迭代次数建议：
- 正常帧：1 次（等同于当前行为，无额外开销）
- 初始化帧或机动检测触发时：3 次
- 最大迭代：5 次（防止不收敛）

改动量约 30 行，不涉及新的矩阵分解，计算开销可控。

---

## 五、双装甲板观测处理

### 5.1 当前实现优于参考方案

参考方案 sp25 使用顺序更新的方式处理双板：遍历匹配的装甲板，每个调用一次 `ekf_.update()`。参考方案文档自己指出了问题——第二次更新使用第一次更新后的状态重新线性化，不是标准的顺序更新，也不是联合更新的等价形式。

当前实现使用 6D 几何观测模型：

```
z_dual = [xc, yc, zc, r1, r2, dza]
```

两块装甲板的 4D 位置通过 {R1, R2, DZA} 参数化后，观测模型一次性更新旋转中心和结构参数。这不存在：
- 线性化点偏移问题（所有观测在同一个预测点处线性化）
- yaw 信息被重复计入的风险（几何观测不包含 yaw）
- 交叉相关处理问题（所有 6 个维度在一个 R_geo 中统一处理）

**结论**：当前双板处理方案在数学严密性上优于参考方案，无需修改。

### 5.2 可选的增强

如果未来需要增强双板观测的稳健性，可以考虑：
- 在双板几何观测前，先对各板分别做马氏距离门控，剔除异常单板
- 当 `fit_double` 联合优化 yaw 时，该 yaw 仅通过单板通道更新（避免"double dip"）

---

## 六、改进优先级与实施路线

| 优先级 | 改进项 | 改动量 | 涉及文件 | 预期收益 |
|--------|--------|--------|---------|---------|
| **P0** | YPD 球坐标 R 矩阵 | ~60 行 + 4 个配置参数 | `dual_radius_spin_ukf.cpp`, `config.hpp`, YAML | 从根本上解决观测噪声建模 |
| **P1** | 启用马氏距离门控 | 1 行 YAML | `gimbal_pipeline.yaml` | 剔除异常观测 |
| **P2** | Q 参数对标调整 | 几行 YAML | `gimbal_pipeline.yaml` | 改善机动跟踪响应 |
| **P3** | IUKF 迭代更新 | ~30 行 | `dual_radius_spin_ukf.cpp` | 大机动/初始化时精度提升 |

P0 是核心改进，需要在 `UKFParameters` 中新增 3 个配置参数：
```yaml
ukf:
  ypd_sigma_azi: 0.063       # 方位角噪声 std (rad), ~3.6°
  ypd_sigma_ele: 0.063       # 俯仰角噪声 std (rad), ~3.6°
  ypd_sigma_dist_coeff: 0.10 # 距离噪声系数 (相对值)
  # 原有参数保留
  obs_noise_pos: 0.03        # 作为 YPD 模型关闭时的回退
  obs_noise_yaw: 0.03
```

P1 可以直接做，当前代码已经完整，只是开关没打开。建议在 P0 完成后再启用（门控效果依赖准确的 R 矩阵）。

P2 的核心是调整 `motion.singer_sigma`（0.5 → 2.0–5.0），需配合实弹测试验证。

P3 是锦上添花，建议在 P0-P2 稳定后再实施。

---

## 七、参数生效与分批上线约束（审计补充）

1. 当前 `bringup_pipeline` 的参数加载是“包默认 + 机器人覆盖”，因此 P0/P1/P2 的 YAML 改动应同步覆盖：
   - `rm_auto_aim/gimbal_pipeline/config/gimbal_pipeline.yaml`
   - `rm_bringup/config/leg/gimbal_pipeline.yaml`
   - `rm_bringup/config/hero/gimbal_pipeline.yaml`
   - `rm_bringup/config/infantry_4_1/gimbal_pipeline.yaml`
2. 启动默认 `robot: 'leg'`，若只改包默认而不改 `rm_bringup/config/leg/gimbal_pipeline.yaml`，现场可能看不到变更效果。
3. `enable_innovation_gating` 建议按车型分批开启，不建议同一迭代全量开启，避免某单车型观测拒绝率异常导致错误回滚结论。
