# 观测噪声 R 更新设计方案

## 1. 设计目标

当前 UKF 中观测噪声通常使用固定对角矩阵：

```cpp
double np = config_.ukf.obs_noise_pos;
double ny = config_.ukf.obs_noise_yaw;
Eigen::Matrix4d R = Eigen::Vector4d(
  np * np,
  np * np,
  np * np,
  ny * ny
).asDiagonal();
```

该实现存在以下问题：

1. `x/y/z` 使用同一个噪声，无法表达深度方向 PnP 误差通常更大的事实。
2. `R` 是对角矩阵，无法表达 `x/y/z/yaw` 之间的相关性。
3. 噪声不随目标距离、目标像素尺寸、PnP/BA 几何退化程度变化。
4. 没有利用 BA / PnP 的协方差信息。
5. 如果直接从固定 `R` 切换到动态 `R`，可能导致滤波器不稳定。

因此，本方案目标是设计一个：

```text
可回退、可迁移、可标定、可逐步增强的观测噪声 R 建模框架。
```

---

## 2. 总体公式

推荐最终观测噪声设计为：

```text
R_dynamic = (1 - w) R_YPD + w · s_BA · R_BA
```

```text
R_final = R_floor + (1 - λ) R_fixed + λ R_dynamic
```

其中：

| 符号 | 含义 |
|---|---|
| `R_floor` | 最小噪声下限，防止滤波器过度自信 |
| `R_fixed` | 旧版固定基准观测噪声，作为稳定锚点 |
| `R_YPD` | 基于 Yaw-Pitch-Distance 的物理 / 标定先验噪声 |
| `R_BA` | BA / PnP 输出的 `xyz + yaw` 协方差 |
| `w` | BA / PnP 协方差可信度权重 |
| `s_BA` | BA 协方差膨胀系数，防止 BA 过度自信 |
| `λ` | 动态模型启用比例 |

推荐理解：

```text
R_floor：保证安全下限
R_fixed：保证迁移过程稳定
R_dynamic：提供距离、几何、BA 退化等自适应能力
```

---

## 3. 各部分职责

### 3.1 R_floor：最小噪声下限

`R_floor` 只负责防止最终 `R` 过小，不负责主要观测噪声建模。

```text
R_floor = diag(
  σ_x_floor²,
  σ_y_floor²,
  σ_z_floor²,
  σ_yaw_floor²
)
```

建议初值：

```yaml
r_floor:
  sigma_x: 0.005
  sigma_y: 0.005
  sigma_z: 0.010
  sigma_yaw: 0.010
```

含义：

```text
x/y 最小 5 mm
z 最小 1 cm
yaw 最小 0.01 rad
```

---

### 3.2 R_fixed：固定基准噪声

`R_fixed` 可以理解为旧版固定 `R` 的升级版，用于迁移过程中的稳定参考。

```text
R_fixed = diag(
  σ_x_fixed²,
  σ_y_fixed²,
  σ_z_fixed²,
  σ_yaw_fixed²
)
```

建议初值：

```yaml
r_fixed:
  sigma_x: 0.03
  sigma_y: 0.03
  sigma_z: 0.05
  sigma_yaw: 0.05
```

与 `R_floor` 的区别：

```text
R_floor：很小，只防止过度自信
R_fixed：较大，是稳定的固定基准模型
```

在迁移早期，`R_fixed` 很有价值，因为它可以避免动态模型估计异常时导致 `R_final` 剧烈波动。

---

### 3.3 R_YPD：球坐标物理先验噪声

将观测位置看作球坐标：

```text
x = ρ cos(ele) cos(azi)
y = ρ cos(ele) sin(azi)
z = ρ sin(ele)
```

在 YPD 空间中定义：

```text
R_YPD_local = diag(
  σ_azi²,
  σ_ele²,
  σ_dist²,
  σ_yaw²
)
```

再通过雅可比转换到 `xyz + yaw` 空间：

```text
R_YPD = J · R_YPD_local · Jᵀ
```

其中：

```text
J = ∂(x, y, z, yaw) / ∂(azi, ele, dist, yaw)
```

这样可以表达：

1. 方位角噪声随距离变成横向位置噪声。
2. 俯仰角噪声影响高度方向。
3. 距离噪声沿视线方向传播。
4. `x/y/z` 之间出现合理相关性。
5. 噪声随目标距离和像素尺寸动态变化。

---

### 3.4 R_BA：BA / PnP 动态协方差

BA / PnP solver 建议输出：

```cpp
struct BaPnpResult {
  bool valid = false;
  bool cov_valid = false;

  Eigen::Vector4d pose_xyz_yaw;
  Eigen::Matrix4d cov_xyz_yaw;

  double confidence = 0.0;
  double reproj_rms = 0.0;
  double condition_number = 0.0;

  int num_observations = 0;
  int num_inliers = 0;
};
```

其中：

```text
cov_xyz_yaw = Cov([x, y, z, yaw])
```

可以近似来自：

```text
Cov ≈ σ_px² · (Jᵀ W J)^-1
```

但 `R_BA` 不能裸用，必须经过：

1. 有效性检查。
2. 正定性检查。
3. 特征值 clamp。
4. Hessian 条件数检查。
5. 重投影 RMS 检查。
6. scale 放大。
7. confidence 权重控制。

推荐使用：

```text
R_BA_used = s_BA · clamp(R_BA)
```

---

## 4. 权重设计

### 4.1 BA 权重 w

推荐：

```text
w = confidence_BA²
```

而不是直接：

```text
w = confidence_BA
```

原因是 BA covariance 往往偏乐观，用平方后更保守。

| confidence | w = confidence² |
|---:|---:|
| 0.2 | 0.04 |
| 0.5 | 0.25 |
| 0.8 | 0.64 |
| 1.0 | 1.0 |

如果仍然不稳定，可以使用：

```text
w = confidence_BA³
```

---

### 4.2 动态模型权重 λ

`λ` 控制动态模型启用程度。

```text
λ = 0：
R_final = R_floor + R_fixed

λ = 1：
R_final = R_floor + R_dynamic

0 < λ < 1：
固定基准模型和动态模型平滑融合
```

建议迁移过程：

```text
λ = 0.0   旧版固定 R
λ = 0.25  少量引入动态模型
λ = 0.5   固定模型和动态模型各占一部分
λ = 0.75  动态模型为主
λ = 1.0   完全使用动态模型，但仍保留 R_floor
```

第一阶段建议：

```yaml
dynamic_blend:
  lambda: 0.3
```

后续根据 NIS、跟踪稳定性和发弹效果逐步调大。

---

## 5. 阶段推进路线

## 阶段一：camera_info + 手工参数的 YPD 建模 + BA/PnP 协方差

### 5.1 阶段目标

先实现完整框架：

```text
R_final = R_floor + (1 - λ) R_fixed + λ [(1 - w) R_YPD + w · s_BA · R_BA]
```

其中 `R_YPD` 来自：

```text
camera_info + armor_size + 少量像素级手工参数
```

---

### 5.2 camera_info 使用方式

可以从 `camera_info.yaml` 读取：

```yaml
image_width: 1280
image_height: 1024

camera_matrix:
  data: [1556.34704, 0, 610.59754,
         0, 1557.43488, 503.80001,
         0, 0, 1]

projection_matrix:
  data: [1541.14038, 0, 610.38965, 0,
         0, 1544.15491, 502.2795, 0,
         0, 0, 1, 0]
```

使用原则：

```text
原始畸变图像上的检测点：使用 camera_matrix K
矫正图像上的检测点：使用 projection_matrix P
如果图像 resize / letterbox：需要同步变换内参，或将检测点映射回原图坐标
```

长期更推荐订阅 `/camera_info`，但第一阶段直接引用文件也可行。

---

### 5.3 armor_size 第一阶段先写死

```yaml
armor_geometry:
  small:
    width: 0.135
    height: 0.055
  large:
    width: 0.230
    height: 0.055
```

后续可以从：

```text
rm_bringup/config/<robot>/armor_geometry.yaml
robot_description / xacro
robot-specific config
```

中读取。

---

### 5.4 手工参数只保留像素级参数

不建议直接手填：

```text
σ_azi
σ_ele
σ_dist
σ_yaw
```

建议手填：

```yaml
ypd_prior:
  source: "camera_model"

  sigma_center_px: 2.0
  sigma_size_px: 2.0
  sigma_corner_px: 1.5

  sigma_azi_min: 0.0005
  sigma_azi_max: 0.020

  sigma_ele_min: 0.0005
  sigma_ele_max: 0.020

  sigma_dist_min: 0.02
  sigma_dist_max: 1.00

  sigma_yaw_min: 0.03
  sigma_yaw_max: 0.50
  sigma_yaw_scale: 10.0

  global_scale: 1.0
```

这些参数比直接调 `R(0,0)` 更可解释。

---

### 5.5 从 camera_info 计算 YPD 标准差

设检测结果为：

```text
u_center, v_center, width_px, height_px
```

角噪声：

```text
azi = atan((u - cx) / fx)
ele = atan((v - cy) / fy)
```

近似：

```text
σ_azi ≈ σ_center_u_px / fx
σ_ele ≈ σ_center_v_px / fy
```

更精确：

```cpp
double du = u_center - cx;
double dv = v_center - cy;

double sigma_azi =
    std::abs(fx / (fx * fx + du * du)) * sigma_center_px;

double sigma_ele =
    std::abs(fy / (fy * fy + dv * dv)) * sigma_center_px;
```

距离噪声：

```text
dist_w ≈ fx · armor_width / width_px
σ_dist_w ≈ dist_w / width_px · σ_size_px
```

```text
dist_h ≈ fy · armor_height / height_px
σ_dist_h ≈ dist_h / height_px · σ_size_px
```

宽高融合：

```text
1 / σ_dist² = 1 / σ_dist_w² + 1 / σ_dist_h²
```

yaw 先验：

```text
σ_yaw ≈ sigma_yaw_scale · σ_corner_px / max(width_px, height_px)
```

然后进行 clamp。

---

### 5.6 构造 R_YPD

局部噪声：

```text
R_YPD_local = diag(
  σ_azi²,
  σ_ele²,
  σ_dist²,
  σ_yaw²
)
```

雅可比：

```cpp
Eigen::Matrix4d J = Eigen::Matrix4d::Zero();

// d(x,y,z)/d(azi)
J(0, 0) = -y;
J(1, 0) =  x;
J(2, 0) =  0.0;

// d(x,y,z)/d(ele)
if (dist_xy > 1e-6) {
  J(0, 1) = -x * z / dist_xy;
  J(1, 1) = -y * z / dist_xy;
} else {
  J(0, 1) = 0.0;
  J(1, 1) = 0.0;
}
J(2, 1) = dist_xy;

// d(x,y,z)/d(dist)
J(0, 2) = x / dist_3d;
J(1, 2) = y / dist_3d;
J(2, 2) = z / dist_3d;

// yaw independent
J(3, 3) = 1.0;
```

最终：

```cpp
Eigen::Matrix4d R_ypd = J * R_ypd_local * J.transpose();
R_ypd = 0.5 * (R_ypd + R_ypd.transpose());
```

---

## 阶段二：通过 PnP 静止观测数据估计 R_YPD

### 6.1 阶段目标

将阶段一中的手工 `R_YPD` 推进为：

```text
camera_info + detector + PnP 静止统计得到的随机噪声表
```

也就是：

```text
R_YPD source = pnp_statistics
```

---

### 6.2 只估计随机噪声

静止目标统计可以可靠估计：

```text
σ_azi
σ_ele
σ_dist
σ_yaw
```

但没有外部真值时，不能可靠估计系统偏差。

例如：

```text
真实距离：4.0 m
PnP 输出：4.18 m ± 0.03 m
```

静止统计只能说明：

```text
随机噪声约 0.03 m
```

不能说明：

```text
系统偏差 0.18 m
```

因此第二阶段建议：

```text
随机噪声：通过 PnP 静止数据统计
系统偏差：仍手工设置或只记录，不自动修正
```

---

### 6.3 数据采集

每个组合单独标定：

```text
camera_info
detector_version
armor_type
image_resolution
robot_type
```

每个距离采集：

```text
500 ~ 3000 帧
```

推荐距离：

```text
1.5 m / 3 m / 5 m / 7 m / 9 m
```

每帧记录：

```yaml
frame:
  timestamp: ...

  detection:
    u_center: ...
    v_center: ...
    width_px: ...
    height_px: ...
    confidence: ...
    corners: ...

  pnp:
    x: ...
    y: ...
    z: ...
    yaw: ...
    reproj_rms: ...

  ba:
    cov_xyz_yaw: ...
    confidence: ...
```

---

### 6.4 转换到 YPD 空间

每帧 PnP 输出：

```text
x_i, y_i, z_i, yaw_i
```

转换为：

```text
azi_i  = atan2(y_i, x_i)
ele_i  = atan2(z_i, sqrt(x_i² + y_i²))
dist_i = sqrt(x_i² + y_i² + z_i²)
yaw_i  = yaw_i
```

然后统计：

```text
σ_azi
σ_ele
σ_dist
σ_yaw
```

建议使用 MAD 鲁棒标准差：

```text
σ ≈ 1.4826 × median(|x_i - median(x)|)
```

避免误检或跳点污染普通标准差。

---

### 6.5 标定表格式

```yaml
ypd_noise_calibration:
  source: "pnp_static_statistics"
  camera_name: narrow_stereo
  camera_info_hash: "fx1556_fy1557_1280x1024"
  detector_name: armor_detector_nn
  armor_type: small

  bins:
    - median_distance: 2.0
      mean_size_px: 105.0

      sigma_azi: 0.0006
      sigma_ele: 0.0008
      sigma_dist: 0.04
      sigma_yaw: 0.05

      bias_dist_manual: 0.00
      bias_yaw_manual: 0.00

      reproj_rms_median: 0.7
      outlier_rate: 0.01

    - median_distance: 4.0
      mean_size_px: 52.0

      sigma_azi: 0.0011
      sigma_ele: 0.0014
      sigma_dist: 0.13
      sigma_yaw: 0.09

      bias_dist_manual: 0.00
      bias_yaw_manual: 0.00

      reproj_rms_median: 0.9
      outlier_rate: 0.03
```

在线使用时：

```text
current_distance → 查表 / 插值得到 YPD std
```

如果查表失败，则回退：

```text
pnp_statistics → camera_model → fixed fallback
```

---

## 阶段三：引入 3D LiDAR / 离线 factor graph 标定

### 7.1 阶段目标

第三阶段不建议一开始将 LiDAR 放进实时链路，而是将它作为：

```text
外部几何参考
离线联合标定工具
R_YPD / R_BA scale / bias 校准工具
```

实时公式仍然不变：

```text
R_final = R_floor + (1 - λ) R_fixed + λ R_dynamic
```

---

### 7.2 首先标定 camera-lidar 外参

需要得到：

```text
T_cam_lidar
```

即将 LiDAR 点云转换到 camera 坐标系的变换。

输出：

```yaml
camera_lidar_extrinsic:
  T_cam_lidar:
    translation: [tx, ty, tz]
    rotation_quat: [qx, qy, qz, qw]
  valid: true
  calibration_rms: ...
```

---

### 7.3 用 LiDAR 估计 PnP / BA 误差

每帧：

```text
p_cam_pnp    = PnP / BA 输出的目标位置
p_lidar      = LiDAR 提取的目标位置
p_cam_lidar  = T_cam_lidar · p_lidar
```

误差：

```text
e_xyz  = p_cam_pnp - p_cam_lidar
e_dist = dist_pnp - dist_lidar
```

可统计：

```text
bias_xyz
Cov_xyz
σ_dist
bias_dist
```

这可以补充第二阶段无法估计的系统偏差。

---

### 7.4 离线 factor graph / BA 联合优化

可以只在标定时使用，不进入实时链路。

变量：

```text
X_k：第 k 帧目标位姿
T_CL：camera-lidar 外参
可选：detector corner bias
可选：armor size correction
```

残差：

```text
1. 图像角点重投影残差
2. LiDAR 目标中心残差
3. LiDAR 点到目标平面残差
4. 静止目标约束
```

优化目标：

```text
min Σ ||r_image||²
  + Σ ||r_lidar_pos||²
  + Σ ||r_lidar_plane||²
  + Σ ||r_static||²
```

离线优化输出更可信的：

```text
X_k^opt
```

然后比较：

```text
e_k = X_k^pnp - X_k^opt
```

用于重新标定：

```text
R_YPD table
R_BA scale
bias table
```

---

### 7.5 LiDAR 阶段输出

```yaml
lidar_assisted_observation_noise_calibration:
  camera_name: narrow_stereo
  lidar_name: lidar_xxx
  armor_type: small

  extrinsic:
    T_cam_lidar: ...

  ypd_noise_model:
    type: "distance_table"
    bins:
      - distance: 2.0
        sigma_azi: 0.0007
        sigma_ele: 0.0009
        sigma_dist: 0.035
        sigma_yaw: 0.05
        bias_dist: 0.02
        bias_yaw: 0.01

      - distance: 4.0
        sigma_azi: 0.0012
        sigma_ele: 0.0015
        sigma_dist: 0.10
        sigma_yaw: 0.08
        bias_dist: 0.06
        bias_yaw: 0.02

  ba_cov_calibration:
    scale: 5.0
    nis_mean_before_scale: 22.0
    nis_mean_after_scale: 4.5
```

---

## 8. 推荐配置结构

```yaml
observation_noise:
  mode: "fixed_dynamic_blend"

  r_floor:
    sigma_x: 0.005
    sigma_y: 0.005
    sigma_z: 0.010
    sigma_yaw: 0.010

  r_fixed:
    sigma_x: 0.03
    sigma_y: 0.03
    sigma_z: 0.05
    sigma_yaw: 0.05

  dynamic_blend:
    lambda: 0.3

  ypd_prior:
    enable: true

    # camera_model | pnp_statistics | lidar_assisted
    source: "camera_model"

    camera_info_source: "file"   # file | topic
    camera_info_url: "package://rm_bringup/config/infantry_4_1/camera_info.yaml"
    use_projection_matrix: false

    armor_size_source: "config"
    armor_sizes:
      small:
        width: 0.135
        height: 0.055
      large:
        width: 0.230
        height: 0.055

    sigma_center_px: 2.0
    sigma_size_px: 2.0
    sigma_corner_px: 1.5

    sigma_azi_min: 0.0005
    sigma_azi_max: 0.020
    sigma_ele_min: 0.0005
    sigma_ele_max: 0.020
    sigma_dist_min: 0.02
    sigma_dist_max: 1.00
    sigma_yaw_min: 0.03
    sigma_yaw_max: 0.50
    sigma_yaw_scale: 10.0

    global_scale: 1.0

    calibration_table_url: ""

  ba_cov:
    enable: true
    scale: 4.0
    confidence_weight_mode: "square"

    min_eigenvalue: 1.0e-8
    max_eigenvalue: 1.0

    reproj_hard_reject: 5.0
    cond_hard_reject: 1.0e8

  final_R:
    min_eigenvalue: 1.0e-8
    max_eigenvalue: 10.0
```

阶段切换主要修改：

```yaml
ypd_prior:
  source: "camera_model"
```

到：

```yaml
ypd_prior:
  source: "pnp_statistics"
  calibration_table_url: "package://..."
```

再到：

```yaml
ypd_prior:
  source: "lidar_assisted"
  calibration_table_url: "package://..."
```

---

## 9. 核心实现伪代码

```cpp
Eigen::Matrix4d computeObservationR(
    const ObservationNoiseInput& input,
    const ObservationNoiseConfig& cfg)
{
  Eigen::Matrix4d R_floor = computeFloorR(cfg);
  Eigen::Matrix4d R_fixed = computeFixedR(cfg);

  Eigen::Matrix4d R_ypd;

  if (cfg.ypd_prior.source == "pnp_statistics" &&
      ypd_table_.available()) {
    R_ypd = computeYpdRFromTable(input, cfg);
  } else if (cfg.ypd_prior.source == "lidar_assisted" &&
             lidar_table_.available()) {
    R_ypd = computeYpdRFromLidarTable(input, cfg);
  } else {
    R_ypd = computeYpdRFromCameraModel(input, cfg);
  }

  double w = 0.0;
  Eigen::Matrix4d R_ba = Eigen::Matrix4d::Zero();

  if (cfg.ba_cov.enable && input.ba_pnp.has_value()) {
    const auto& ba = input.ba_pnp.value();

    if (isBaCovUsable(ba, cfg)) {
      Eigen::Matrix4d cov = clampCovariance(
          ba.cov_xyz_yaw,
          cfg.ba_cov.min_eigenvalue,
          cfg.ba_cov.max_eigenvalue);

      double conf = std::clamp(ba.confidence, 0.0, 1.0);
      w = conf * conf;

      R_ba = cfg.ba_cov.scale * cov;
    }
  }

  Eigen::Matrix4d R_dynamic =
      (1.0 - w) * R_ypd
    + w * R_ba;

  double lambda = std::clamp(cfg.dynamic_blend.lambda, 0.0, 1.0);

  Eigen::Matrix4d R_final =
      R_floor
    + (1.0 - lambda) * R_fixed
    + lambda * R_dynamic;

  R_final = 0.5 * (R_final + R_final.transpose());

  R_final = clampCovariance(
      R_final,
      cfg.final_R.min_eigenvalue,
      cfg.final_R.max_eigenvalue);

  return R_final;
}
```

---

## 10. 调试与验证指标

建议 debug 模式输出：

```text
distance
armor_type
u_center, v_center
width_px, height_px

std(R_floor)
std(R_fixed)
std(R_YPD)
std(R_BA)
std(R_dynamic)
std(R_final)

BA confidence
BA weight w
BA reproj_rms
BA condition_number

lambda
NIS
innovation
accepted / rejected
```

其中 `std(R)` 输出：

```cpp
sqrt(R(i, i))
```

比方差更直观。

---

## 11. NIS 验证

对于观测：

```text
z = [x, y, z, yaw]
```

观测维度为 4，NIS 理论均值大致为：

```text
E[NIS] ≈ 4
```

现象判断：

| 现象 | 可能原因 |
|---|---|
| NIS 长期远大于 4 | R 偏小，或模型 / 关联错误 |
| NIS 长期远小于 4 | R 偏大，滤波器过度保守 |
| NIS 几百 / 几千 | 大概率存在严重错误 |
| 动态 R 启用后抖动增大 | `λ` 太大、`s_BA` 太小、`w` 太激进 |
| 跟踪滞后明显 | R 过大或 Q 过小 |
| 远距离 NIS 爆炸 | `σ_dist` / `σ_yaw` 低估 |

---

## 12. 关键风险与约束

### 12.1 坐标系一致性

`R_YPD`、`R_BA`、`z_obs` 必须在同一观测空间：

```text
[x, y, z, yaw]
```

如果 BA covariance 在 camera frame，而 UKF 使用 odom frame，需要：

```text
Cov_odom = J_tf · Cov_camera · J_tfᵀ
```

第一阶段建议先保证所有观测都在同一坐标系下处理。

---

### 12.2 K / P 不能混用

```text
原始图像检测点：使用 K + D
矫正图像检测点：使用 P
```

图像 resize / letterbox 时，要同步修正内参或将检测点映射回原图。

---

### 12.3 系统偏差不要直接混进 R

`R` 表示随机噪声，不是系统偏差。

系统偏差建议单独记录：

```yaml
manual_bias:
  dist_bias: 0.0
  yaw_bias: 0.0
```

第二阶段先手工设置或只记录。第三阶段有 LiDAR 参考后，再考虑 bias table。

---

### 12.4 BA covariance 不能裸用

不要：

```cpp
R = R_BA;
```

应该：

```cpp
R_dynamic = (1 - w) R_YPD + w · s_BA · clamp(R_BA);
```

并且始终经过：

```text
valid check
condition number check
reprojection RMS check
positive definite check
eigenvalue clamp
scale
confidence weight
```

---

## 13. 推荐落地顺序

### 13.1 第一阶段

实现：

```text
1. R_floor
2. R_fixed
3. CameraInfo 读取
4. armor_size 配置
5. camera_model R_YPD
6. BA/PnP covariance 接口
7. R_dynamic 融合
8. R_final 融合
9. NIS/debug 日志
```

建议先跑：

```text
λ = 0.0
```

验证旧模型稳定性，然后逐步：

```text
λ = 0.3 → 0.5 → 0.7
```

---

### 13.2 第二阶段

实现离线脚本：

```text
rosbag/csv
→ 提取 detector + PnP 数据
→ 转换到 YPD
→ 分距离/尺寸桶
→ 鲁棒统计随机噪声
→ 生成 ypd_noise_calibration.yaml
```

在线切换：

```yaml
ypd_prior:
  source: "pnp_statistics"
```

---

### 13.3 第三阶段

先不改实时 tracker。

离线增加：

```text
1. camera-lidar 外参标定
2. LiDAR target extraction
3. PnP/BA vs LiDAR 对照统计
4. 可选 factor graph 联合优化
5. 输出 lidar_assisted_observation_noise.yaml
```

实时仍然只读表，不跑 factor graph。

---

## 14. 最终总结

推荐最终方案：

```text
R_dynamic = (1 - w) R_YPD + w · s_BA · R_BA
```

```text
R_final = R_floor + (1 - λ) R_fixed + λ R_dynamic
```

推进路线：

```text
阶段 1：
R_YPD = camera_info + armor_size + 手工像素噪声参数
R_BA = BA/PnP covariance + confidence + clamp + scale

阶段 2：
R_YPD = PnP 静止观测统计得到的随机噪声表
系统偏差仍手工设置或只记录

阶段 3：
引入 3D LiDAR / factor graph，仅在离线标定时使用
用于标定 camera-lidar 外参、R_YPD、bias、R_BA scale
实时系统仍然查表构造 R
```

该路线的优点：

```text
1. 在线计算轻量
2. 每阶段都可独立上线
3. 动态模型失败时可回退
4. λ 可以控制迁移比例
5. R_fixed 保证稳定性
6. R_floor 防止过度自信
7. R_YPD 提供物理先验
8. R_BA 提供当前几何和优化质量信息
```

整体上，该方案适合当前 `hypothesis + NIS / likelihood / TopK + UKF` 框架，能够在不破坏现有实时链路的前提下逐步提升观测噪声建模的物理合理性与工程稳定性。
