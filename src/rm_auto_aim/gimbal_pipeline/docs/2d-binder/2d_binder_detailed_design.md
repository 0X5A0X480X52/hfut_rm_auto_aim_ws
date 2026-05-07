# 2D Binder 详细方案设计（面向 gimbal_pipeline）

## 1. 背景与目标

当前 `gimbal_pipeline` 主链路依赖 `rm_interfaces/msg/Armors` 中的 3D `pose` 进行跟踪和绑定，`binder` 的决策主要使用 `z/yaw/jump` 等 3D 证据。

本方案新增一条 **2D 观测证据链**：

1. 在检测消息中补充 `bbox/corners` 等 2D 几何字段；
2. 在 `binder` 内部实现独立 `2d-tracker`（实例级 `track_id`）；
3. `2d-tracker` 除 ID 关联外，额外维护：
- 同一 `track_id` 在 Odom 系下的 `z` 观测均值/方差；
- 基于 2D 框运动方向的机器人整体运动趋势辅助判别；
4. 将 2D 证据以“融合项”接入现有 binder pipeline，而非直接替代 3D 证据。

## 2. 关键原则

1. **`track_id` 与装甲板编号解耦**
- `track_id`: 仅代表 2D 时序实例（短时稳定身份）；
- `armor.number`: 仍代表业务语义（"1"~"5"/"outpost"/"base" 等）。

2. **`Armor.msg` 不传输 `track_id`**
- 保持 detector 端无状态/弱状态职责；
- 避免跨节点传播实例ID导致语义漂移与调参耦合。

3. **2D-tracker 放在 binder 域内**
- detector 只负责“本帧观测”；
- binder 统一负责“跨帧绑定决策与证据融合”。

4. **渐进融合**
- 第一阶段作为辅助证据（加权）；
- 行为稳定后再评估是否提升到强约束。

## 3. 非目标

1. 不在本阶段替换现有 3D tracker/UKF 主状态估计。
2. 不在 detector 节点维护长期全局 `track_id`。
3. 不将 2D 轨迹直接用于控制输出（先经 binder/tracker 融合）。

## 4. 链路改造范围

## 4.1 上游观测（detector）

需要在 `rm_interfaces/msg/Armor.msg` 扩展 2D 几何字段（示例，具体命名可再统一）：

1. `float32[4] bbox_xywh` 或 `float32[4] bbox_xyxy`
2. `geometry_msgs/Point32[4] image_corners`
3. `float32 detection_confidence`
4. `uint8 corners_ordering`（约定角点顺序）
5. `bool has_image_geometry`

约束：

1. `armor_detector` 与 `armor_detector_nn` 字段语义一致；
2. 坐标系统一为原始图像像素坐标；
3. 角点顺序固定（如左下、左上、右上、右下或明确文档约定）；
4. 无有效2D信息时 `has_image_geometry=false`。

## 4.2 gimbal_pipeline/binder

在 `max_entropy_tracker/binder` 下新增 2D 子模块（建议目录）：

1. `binder/2d_track/model/*`
2. `binder/2d_track/association/*`
3. `binder/2d_track/core/*`
4. `binder/2d_track/debug/*`

并在 `Norm4BinderBridge/OutpostBinderBridge` 的输入构造处注入 2D 证据摘要。

## 5. 2D-tracker 设计

## 5.1 输入输出

输入（每帧）：

1. 当前帧所有装甲观测的 `bbox/corners/confidence`；
2. 与之时间对齐的 3D `obs(x,y,z,yaw)`（若可用）；
3. 时间戳 `t`。

输出（每帧）：

1. 每个观测对应的 `track_id`（仅 binder 内部使用）；
2. `TrackEvidence`：
- `z_mean/z_var/z_count`（按 track 维护）；
- `img_velocity`（像素速度）；
- `img_heading_confidence`（方向可信度）；
- `motion_trend`（左/右/近/远/不确定）；
- `association_quality`（匹配质量分）。

## 5.2 状态与生命周期

每个 2D track 维护：

1. `id`（自增）
2. `last_bbox`, `last_corners`
3. `age`, `hits`, `missed`
4. `velocity_px`（中心点速度）
5. `z_stats`（Welford 在线均值方差）
6. `last_update_time`
7. `quality_score`

生命周期：

1. `Tentative`（初建）
2. `Confirmed`（命中达阈值）
3. `Lost`（短时丢失）
4. `Removed`（超时剔除）

## 5.3 关联策略

推荐两层门控 + 匈牙利匹配：

1. 预门控
- IOU gate
- 中心距离 gate
- 尺度变化 gate
- 类别一致性软约束（`armor.type`）

2. 代价函数
- `cost = w_iou*(1-iou) + w_center*d_center + w_scale*d_scale + w_shape*d_shape`

3. 后验拒配
- 低于最小匹配质量阈值则拒绝

4. 新生与死亡
- unmatched det -> new track
- unmatched track -> `missed++`，超阈值删除

说明：可复用 `armor_detector_nn` 当前 IOU 逻辑或 `muit_obj_tracker` 的 SORT 组件思想，但最终实现归属 binder。

## 6. 2D-tracker 承担的 3D 辅助职责

## 6.1 同一 track 的 Odom-Z 统计

目标：为 dz 识别提供更鲁棒的局部先验，而不是仅靠瞬时 `z`。

方法：

1. 若该观测存在有效 3D 位置（经过 TF 后），将 `obs.z` 输入 track 的 `z_stats`；
2. 维护：
- `z_mean`
- `z_var`
- `z_count`
- `z_last`
3. 对异常值做鲁棒门控（MAD/IQR/阈值截断）；
4. 当 `z_count` 不足时降低权重。

在 binder 中用途：

1. jump 候选打分时增加 `|z_obs - z_mean(track)|` 一致性项；
2. dual-obs 情况下，用各 track 的 z 统计区分上下层候选；
3. 为 outpost 三层高度判别提供短窗稳定先验。

## 6.2 基于 ID 的 2D 运动趋势辅助

目标：利用图像平面轨迹方向，辅助判断机器人整体运动趋势（特别在 3D yaw/velocity 不稳定时）。

输出建议：

1. `trend_lateral`: LEFT/RIGHT/UNKNOWN
2. `trend_radial`: APPROACH/LEAVE/UNKNOWN（结合框面积变化）
3. `trend_confidence`: [0,1]

判别特征：

1. `dx, dy`（中心位移）
2. `dA`（bbox面积变化率）
3. `dAspect`（宽高比变化）
4. 时间窗一致性（最近 N 帧符号一致比例）

在 binder 中用途：

1. 当多个 panel 候选分数接近时，作为 tie-breaker；
2. 与 spin_direction/yaw_rate 的符号一致性联合打分；
3. 低置信趋势不参与硬决策，仅作为 soft evidence。

## 7. 与现有 BinderPipeline 的融合方式

## 7.1 不改核心状态机语义

现有 `BinderPipeline(decoder -> id_binder -> scorer -> fsm)` 保持不变。

新增方式：

1. 在 `BinderFrameInput` 扩展 `two_d_evidence`（结构体）；
2. decoder/id_binder/scorer 按需读取该字段；
3. 缺失 2D 证据时完全回退现有行为。

## 7.2 融合点建议

1. `id_binder`：候选目标接近时增加 2D 连续性加分；
2. `scorer`：健康评分中加入 `association_quality` 与 `trend_consistency`；
3. `decoder`：jump 判定时引入 `z_mean residual` 的软门控。

## 7.3 证据权重策略

1. `w_2d_base` 基础权重；
2. 根据质量自适应缩放：
- `high z_var` -> 降权
- `low association_quality` -> 降权
- `lost/reacquire` 初期 -> 降权

## 8. 参数设计（建议）

建议新增参数组：`tracker.two_d.*`

1. `enable`
2. `iou_threshold`
3. `max_center_dist_px`
4. `min_hits`
5. `max_missed`
6. `cost_weights.{iou,center,scale,shape}`
7. `z_stats.min_samples`
8. `z_stats.outlier_gate`
9. `trend.window_size`
10. `trend.min_confidence`
11. `fusion.weight_base`
12. `fusion.weight_max`

## 9. 失败模式与风险

1. 遮挡/交叉导致 2D ID switch
- 缓解：reacquire降权 + z统计一致性复核

2. 透视变化导致面积趋势误判
- 缓解：仅作为软证据，且需多帧一致

3. detector 双实现字段不一致
- 缓解：统一接口定义 + 单元测试 + bag回放一致性检查

4. 消息扩展引发下游兼容问题
- 缓解：新增字段默认值可兼容旧逻辑；严格禁止复用 `number` 存实例ID

5. TF 抖动污染 z 统计
- 缓解：对 `z` 输入增加时间同步与异常门控

## 10. 验证与评估方案

## 10.1 离线指标

1. 2D track 连续性：IDSW / MOTA-like 指标
2. binder 绑定稳定性：switch false positive / false negative
3. outpost dz 识别准确率（含转场阶段）
4. 模式切换抖动率（短时来回切换）

## 10.2 在线指标

1. `binder_debug_snapshot` 增加 2D evidence 字段日志
2. 关键统计：
- `2d_assoc_quality_avg`
- `2d_trend_confidence_avg`
- `z_mean_residual`

3. A/B 对照：
- A: 仅3D旧逻辑
- B: 3D + 2D融合
- 对比命中率、误切换率、控制稳定性

## 11. 分阶段落地计划（文档级）

1. Phase 1: 接口与观测打通
- 扩展 `Armor.msg`（仅 2D 几何字段）
- detector / detector_nn 同步发布

2. Phase 2: binder 内 2D-tracker 基础版
- IOU + Hungarian + 生命周期
- 只输出内部 `track_id` 与质量分

3. Phase 3: 3D辅助统计
- 每 track 的 `z_mean/z_var` 维护
- 趋势估计模块

4. Phase 4: 融合接入
- 注入 scorer/id_binder 的 soft evidence
- 参数化开关与降级回退

5. Phase 5: A/B 与参数固化
- 多场景bag回放
- 给出机器人类型分配置模板

## 12. 与当前系统兼容性结论

该方案与现有架构兼容，且满足以下约束：

1. 不改变 `number` 的业务语义；
2. 不向 `Armor.msg` 注入 `track_id`；
3. binder 内新增能力可配置启停，默认可完全回退旧逻辑；
4. 对 `bringup_pipeline` 链路改动可控，主要集中在消息字段与 binder 处理逻辑。

---

## 附录 A：建议新增内部结构（示意）

```cpp
struct TwoDEvidence {
  bool valid = false;
  int internal_track_id = -1;
  double assoc_quality = 0.0;

  bool has_z_stats = false;
  double z_mean = 0.0;
  double z_var = 0.0;
  int z_count = 0;

  int trend_lateral = 0; // -1 left, 0 unknown, +1 right
  int trend_radial = 0;  // -1 leave,0 unknown,+1 approach
  double trend_confidence = 0.0;
};
```

## 附录 B：消息字段命名建议（示意）

```text
# Armor.msg (append-only)
float32 detection_confidence
bool has_image_geometry
float32[4] bbox_xywh
geometry_msgs/Point32[4] image_corners
uint8 corners_ordering
```

说明：最终字段名以接口评审结果为准，但需保证 detector 与 binder 对语义一致。
