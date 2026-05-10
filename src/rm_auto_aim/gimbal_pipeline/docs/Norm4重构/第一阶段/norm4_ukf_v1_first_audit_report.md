# Norm4 第一阶段第一次审计报告

## 1. 审计目标与范围

本报告用于核对 `Norm4` 第一阶段实现与设计计划的一致性，重点关注：

1. `UKF backend V1 + evaluate all hypotheses + Top1 commit + TopK log + dual raw batch` 是否落地。
2. 接入链路是否可用：`tracker.implementation=norm4_v2`、`TrackerManager`、参数解析。
3. 关键风险是否被控制：返回语义、降级链路、调试可见性、阈值可配置性。

本次审计范围：

```text
src/rm_auto_aim/gimbal_pipeline/include/max_entropy_tracker/trackers/norm4_v3/*
src/rm_auto_aim/gimbal_pipeline/src/max_entropy_tracker/trackers/norm4_v3/*
src/rm_auto_aim/gimbal_pipeline/include/max_entropy_tracker/tracker_manager.hpp
src/rm_auto_aim/gimbal_pipeline/include/max_entropy_tracker/core/config.hpp
src/rm_auto_aim/gimbal_pipeline/src/gimbal_pipeline_node.cpp
src/rm_auto_aim/gimbal_pipeline/src/pybind/offline_tracker_replay.cpp
```

构建验证：

```text
colcon build --packages-select gimbal_pipeline 通过
```

## 2. 对齐综述

### 2.1 已对齐（核心目标）

1. 新增独立 `Norm4ArmorTrackerV2` 并接入 `tracker.implementation=norm4_v2`。
2. 实现单观测 `4` 假设枚举与双观测 `8` 有序相邻假设枚举。
3. 所有假设在同一个 `PredictContext` 上进行评估。
4. 支持 `TopK` 排序、置信度与 margin 计算。
5. 双观测采用 `8D raw batch` 更新，不再使用 geometry pseudo measurement。
6. `commit()` 仅在 `trial.success` 时写主状态，具备基本 non-destructive 评估框架。

### 2.2 未对齐（计划新增但未落地）

1. `2D-tracker -> 3D-tracker` 单板桥接输出链路尚未实现。
2. `ambiguous/structured` 双链路路由尚未实现。
3. `0/1` 双种子 warmup 与内部浅更新机制尚未实现。
4. `Top1 confidence / margin / reconstruction` 尚未纳入 commit 判决门控。
5. 关键阈值与 dual R scale 未配置化。

## 3. 主要缺陷（按严重度）

### 3.1 高风险

1. `update` 返回语义会导致“有观测但未commit”被上层误判失败。  
证据：
`norm4_tracker_v2.cpp:166` 返回 `committed`。  
`tracker_manager.hpp:222-223` 仅在 `ok=true` 更新 `last_update_time`。  
影响：
连续 reject 帧可能触发过早 stale/lost，与“predict/shallow 降级可持续运行”目标冲突。

2. `ambiguous` 单板降级主链路未实现。  
证据：
`norm4_v3` 中无 `single_plate_bridge / ambiguous backend` 相关实现；  
`rg "warmup|shallow|mode_routing|single_plate_bridge"` 在 `norm4_v3` 无命中。  
影响：
当前无法达到计划中的“ambiguous 对外单板输出、structured 对外UKF输出”的双通道切换。

### 3.2 中高风险

1. `Top1` 质量指标未参与提交判决。  
证据：
`norm4_tracker_v2.cpp:90-92` 计算了 `top1_confidence/top1_top2_margin`；  
`norm4_tracker_v2.cpp:122` 提交条件仅 `trial.success && trial.posterior_sanity_pass`。  
`reconstruction_pos_error` 仅在后端计算（`norm4_ukf_backend_v1.cpp:515,653`），未用于 reject。  
影响：
存在“可提交但低分辨率/低置信”的错误收敛风险。

2. 调试链路无法消费 V2 结果。  
证据：
`gimbal_pipeline_node.cpp:3258,3344` 仅 `dynamic_cast<const Norm4ArmorTracker*>`；  
`offline_tracker_replay.cpp:152` 仅识别旧 `Norm4ArmorTracker`。  
影响：
`TopK/NIS/decision` 诊断不完整，难以回放定位误关联与退化原因。

### 3.3 中风险

1. gate 与 dual R scale 硬编码，无法按 bag 迭代调优。  
证据：
`norm4_ukf_backend_v1.cpp:271-274,391-393` 固定阈值；  
`norm4_ukf_backend_v1.cpp:337,595` 固定 `r_scale=1.5`。  
影响：
不同相机、PnP 噪声、距离工况下难以稳定复现和调参。

2. 矩阵求逆使用 `S.inverse()`，数值稳定性可继续优化。  
证据：
`norm4_ukf_backend_v1.cpp:488,625`。  
影响：
在病态协方差帧更易出现数值噪声放大；建议统一使用分解求解。

3. 初始化仍偏单点种子。  
证据：
`norm4_tracker_v2.cpp:28` 缺省 `panel_id` 用 `0`。  
影响：
首帧语义错位时，后续若无强证据易进入错误 basin。

4. ambiguous 语义声明与行为不一致。  
证据：
`norm4_tracker_v2.hpp:38` 声明支持 ambiguous single semantics；  
`norm4_tracker_v2.cpp:198` `is_ambiguous_single_mode()` 固定 `false`。  
影响：
上层无法正确区分当前输出语义模式。

## 4. 需完善项清单

### 4.1 必做（进入 PR 0）

1. 修正 `update()` 返回语义：有观测且流程执行完返回 `true`。
2. 提交门控加入：
`confidence`、`top1/top2 margin`、`reconstruction error`。
3. 参数化：
single/dual gate 阈值、`dual_raw_R_scale`、commit 阈值。
4. V2 调试接入：
node/replay 支持 V2（或抽象统一 debug provider）。

### 4.2 必做（进入 PR 2/PR 5）

1. 实现 `0/1` 双种子 warmup：
内部双假设 shallow 更新，对外保持 ambiguous single。
2. 实现模式路由：
`AMBIGUOUS` 输出单板 3D，UKF shallow/predict；  
`STRUCTURED` 输出UKF，单板链路 shallow 保温。
3. 实现 `2D-tracker -> 3D-tracker` 单板桥接：
以 `track2d_id` 维护同一装甲板语义。

### 4.3 建议优化（可并行）

1. `S.inverse()` 替换为分解求解。
2. 统一 `measurement gate` 与 `posterior sanity` 输出 reject reason 枚举化。
3. 补单元测试：warmup 收敛/不收敛、模式路由切换、reject 帧返回语义。

## 5. 修复优先级建议

1. `PR 0`：运行语义与可观测性修复。  
目标：先让链路“可持续运行 + 可调试”。

2. `PR 2`：warmup 双种子 + ambiguous 输出骨架。  
目标：先落模式语义，不急于调最优阈值。

3. `PR 4/5`：提交门控完善 + dual raw batch 收敛标定。  
目标：降低错误提交和抖动。

4. `PR 6`：回放统计与阈值固化。  
目标：把参数从“经验值”收敛到“数据值”。

## 6. 阶段结论

当前实现已经完成第一阶段核心主干（多假设评估、Top1提交、dual raw batch、V2接入），方向正确。  
但“可运行”与“可部署”之间仍有关键缺口，主要集中在：

1. 返回语义与上层生命周期耦合。
2. ambiguous 输出链路尚未真正落地。
3. 提交门控与调试可见性不足。

在完成 `PR 0` 与模式路由补齐前，不建议直接用于实车主链路替换。
