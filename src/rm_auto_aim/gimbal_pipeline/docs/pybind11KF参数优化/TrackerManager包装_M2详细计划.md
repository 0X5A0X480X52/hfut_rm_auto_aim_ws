# pybind11 KF 参数优化：TrackerManager 包装 M2 详细计划

## 1. 文档目的

本计划用于指导在 `gimbal_pipeline` 中落地一个“可跑通完整流程”的离线优化 Demo（至 M2）。
核心原则：
- 优先保持与线上行为一致；
- 优先复用现有 `TrackerManager` 帧级流程；
- 以固定 `robot_id` 先打通单链路闭环，再扩展多目标场景。

---

## 2. 设计决策（结论先行）

1. **pybind 不直接只包单个 filter/tracker 的 update()**。
2. **pybind 主入口包 `TrackerManager` 的帧级语义**（predict/update/missing/stale）。
3. M2 阶段固定测试 `robot_id`（如 `outpost` 或某个 norm4 id），减少多目标耦合影响。
4. Python 优化脚本以“日志重放 -> 指标计算 -> 参数搜索 -> 导出补丁”为主线。

这样可以最大限度保证：
- 对外接口语义与线上一致；
- 离线结论可迁移回线上配置；
- 实施复杂度可控。

---

## 3. 范围定义

### 3.1 In Scope（M2）
- C++ 新增 pybind 模块（离线专用）。
- 包装 `TrackerManager` 的最小可重放 API。
- 支持固定 `robot_id` 的时序重放（包含空帧）。
- 在 `scripts/` 新增评估与优化脚本。
- 支持导出参数结果与 ROS 参数 patch。

### 3.2 Out of Scope（M2 不做）
- 多 robot_id 同时优化。
- 在线节点热更新参数。
- 新增模型结构（仅调参，不改状态机拓扑）。

---

## 4. 总体架构

```text
CSV logs (observation/state)
        |
        v
Python replay loader  --->  pybind manager wrapper  ---> TrackerManager
        |                              |                     |
        |                              |                     +--> Norm4ArmorTracker / OutpostTrackerV2
        |                              |
        +------ metrics & objective <--+
                       |
                       v
                 scipy optimizer
                       |
                       v
             best_params.json + yaml patch
```

---

## 5. C++ 侧详细方案（pybind 模块）

## 5.1 新增模块建议

建议新增离线绑定目录（示例）：
- `src/rm_auto_aim/gimbal_pipeline/src/pybind/`
- `src/rm_auto_aim/gimbal_pipeline/include/gimbal_pipeline/pybind/`（如需头文件）

建议模块名：
- `gimbal_pipeline_kf_opt`

## 5.2 对外包装类（核心）

建议新增 `OfflineTrackerReplay`（C++ 类，供 pybind 导出）：

职责：
- 持有 `UnifiedConfig` 与 `TrackerManager`。
- 在固定 `robot_id` 约束下执行帧级 replay。
- 输出最小状态快照与诊断快照。

建议接口：
1. `reset(config_dict_like, dt, default_r1, default_r2, default_dza, timeout)`
2. `feed(timestamp_sec, observations)`
   - observations: `std::vector<ObservationData>`
   - 内部构造 `obs_by_robot = {{fixed_robot_id, observations}}`
   - 调用 `process_frame(...)`
3. `feed_empty(timestamp_sec)`
   - 内部构造空 `obs_by_robot`，用于丢帧推进状态机
4. `snapshot()`
   - 若 tracker 不存在/未初始化，返回 `valid=false`
   - 否则返回：
     - center/vel/yaw/r1/r2/dza
     - innov_x/y/z, innov_yaw, nis, update_type
     - track_state/lost_count/frame_count
     - outpost/norm4 debug snapshot（若可用）
5. `set_fixed_robot_id(robot_id)`

## 5.3 为什么不是只包 `tracker->update(obs)`

只包 `update(obs)` 会遗漏：
- `predict_all()` 的时间推进
- `notify_missing()` 的失配状态迁移
- `remove_stale()/remove_lost()` 的生命周期管理

这些正是线上行为一致性的关键。M2 必须复用 `process_frame()` 级语义。

## 5.4 pybind 暴露对象

1. `PyObservationData`
- `x, y, z, yaw`
- `panel_id`（可空）
- `confidence`
- `timestamp`（可空）
- `layer`（可空）

2. `PyReplaySnapshot`
- 通用状态量（center/vel/yaw/radii/...）
- UKF 诊断量（nis/innov/update_type）
- debug 字段（模式、candidate、margin、switch 统计等）

3. `PyOfflineTrackerReplay`
- 绑定 `OfflineTrackerReplay` 所有核心方法

---

## 6. Python 侧详细方案（scripts）

目标目录：`src/rm_auto_aim/gimbal_pipeline/scripts`

建议新增脚本：

1. `kf_opt_data_loader.py`
- 读取 `observation_log_*.csv` / `tracker_state_log_*.csv`
- 按 `(timestamp_ns, robot_id)` 分组
- 输出帧序列对象（含空帧策略）

2. `kf_opt_replay_eval.py`
- 加载参数
- 调 pybind wrapper 执行 replay
- 计算指标与综合损失
- 输出 `eval_report.json`

3. `kf_opt_objective.py`
- 封装 objective(params) -> scalar loss
- 支持多日志汇总（均值 + 方差惩罚）

4. `kf_opt_m2.py`
- 优化入口
- 两阶段搜索：
  - Phase A：随机/LHS 粗搜
  - Phase B：局部精调（Nelder-Mead/Powell）
- 保存 `best_params.json`

5. `kf_opt_export_yaml.py`
- 参数映射到 ROS2 参数层级
- 导出 `ros2_params_patch.yaml`

6. `kf_opt_run_demo.sh`（可选）
- baseline -> optimize -> export 一键串联

---

## 7. 数据协议与重放规则

## 7.1 输入数据

主输入：
- `observation_log_*.csv`

可选监督参考：
- `tracker_state_log_*.csv`

## 7.2 时间与分组

- 按 `timestamp_ns` 升序。
- 限定 `robot_id == fixed_robot_id`。
- 同一 timestamp 的所有观测作为一个 `obs_list` 传 `feed()`。

## 7.3 空帧处理

当相邻时间戳间出现目标缺测：
- 通过 `feed_empty(ts)` 推进状态机。
- 空帧时间点可按固定周期插值（如 100Hz）或按日志原生时序策略执行。

## 7.4 坐标系约束

M2 假设日志是 `target_frame`（通常 `odom`）。
脚本参数必须显式声明 `--frame odom`，并在报告中固化该元信息。

---

## 8. 评估指标与目标函数（M2 版）

## 8.1 核心精度项

1. `E_pos_horizon`
- 预测 50/100/150 ms 的 center position RMSE

2. `E_yaw_horizon`
- 同预测窗口的 yaw RMSE（角差 wrap）

## 8.2 稳定性项

1. `E_nis_tail`
- NIS 超阈比例（按 `update_type` 可分组）

2. `E_innov_tail`
- `||innov_xyz||` 的 P95/P99

## 8.3 语义/模式项

outpost 重点：
- `switch_chatter_rate`
- `low_margin_ratio`
- `z_audit_conflict_ratio`

norm4 重点：
- `degraded_mode_ratio`
- `switch_chatter_rate`
- `low_binding_conf_ratio`

## 8.4 综合损失

建议初版：
\[
L = 0.45 E_{pos} + 0.20 E_{yaw} + 0.15 E_{nis} + 0.10 E_{innov} + 0.10 E_{semantic}
\]

并加入：
- 参数边界罚项
- 跨日志方差罚项（防止过拟合单场景）

---

## 9. 参数空间设计

## 9.1 第一批优化参数（建议 6~10 个）

通用：
- `ukf.obs_noise_pos`
- `ukf.obs_noise_yaw`
- `motion.cv_process_noise_vel` 或 `motion.ca_process_noise_acc`
- `motion.process_noise_r`
- `motion.process_noise_dz`
- `spin.spin_process_noise_delta_rate`
- `spin.spin_process_noise_delta_acc`

outpost 可选补充：
- `outpost.softmax_temperature`
- `outpost.binding_min_candidate_prob`
- `outpost.binding_min_candidate_margin`

## 9.2 参数变换

- 正值噪声参数建议在 log-space 搜索。
- 概率/阈值参数在线性空间搜索并做 [0,1] 或指定范围裁剪。

---

## 10. 优化流程（M2）

## 10.1 Baseline 评估

- 使用当前参数跑全量日志集。
- 生成 `baseline_report.json`。

## 10.2 Phase A 粗搜

- 随机/LHS 采样 N=200~500。
- 选 Top-K（如 K=10）。

## 10.3 Phase B 精搜

- 对 Top-K 分别做局部优化。
- 选择全局最优候选。

## 10.4 结果导出

- `best_params.json`
- `ros2_params_patch.yaml`
- `opt_report.json`（含与 baseline 对比）

---

## 11. 验收标准（M2）

必须满足：
1. 完整链路可运行：重放、评估、优化、导出均成功。
2. 综合损失相对 baseline 下降 >= 10%。
3. 关键稳态指标不退化：
   - 模式切换抖动不明显增加；
   - NIS 尾部不过度恶化。
4. 多日志评估方差在可接受范围。

---

## 12. 任务拆分与工期建议

### Day 1
- pybind 工程接入
- `OfflineTrackerReplay` 最小接口
- 单日志 replay 跑通

### Day 2
- 指标计算脚本与 baseline 报告
- 参数映射与边界定义

### Day 3
- 两阶段优化实现
- 结果导出 + 对比报告

### Day 4（缓冲）
- 稳定性修正
- 文档补齐与使用说明

---

## 13. 风险与应对

1. **重放时序与线上不一致**
- 应对：统一走 `process_frame`，避免仅调用 `update(obs)`。

2. **日志信息不足（frame 未显式记录）**
- 应对：M2 强约定 + 报告写明；M3 补 logger 元信息字段。

3. **优化过慢**
- 应对：先小参数集 + 粗搜缩小范围，再局部精搜。

4. **离线收益无法迁移线上**
- 应对：加入稳定性/语义损失，避免单纯拟合误差最小。

---

## 14. 后续扩展（M3+）

- 扩展多 `robot_id` 联合评估。
- 引入场景标签（距离/速度/遮挡）分桶优化。
- 增加自动回归流水线（每次改参数自动离线评分）。

