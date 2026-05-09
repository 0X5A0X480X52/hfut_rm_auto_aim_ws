# JLU 公共模块移植与 ROS2 替换调研

日期：2026-05-09  
范围：`tmp/jlu_vision_26` 与 `src/rm_auto_aim/auto_buff`

## 1. 结论摘要

`auto_buff` 可以接入，但不建议整体迁移 `tools/common_defs`。推荐路线为：

1. 仅移植算法相关最小子集（几何、拟合、轨迹）。
2. 通信/TF/状态监听改为 ROS2 原生实现。
3. 删除 iceoryx 体系及其镜像消息依赖。

## 2. 依赖关系（JLU 原仓库）

`auto_buff -> tools + common_defs -> iceoryx (+ fast_tf, quill, rfl...)`

证据：
- `tmp/jlu_vision_26/src/auto_buff/xmake.lua`
- `tmp/jlu_vision_26/src/tools/xmake.lua`
- `tmp/jlu_vision_26/src/common_defs/xmake.lua`
- `tmp/jlu_vision_26/xmake.lua`

## 3. 可移植性分级

### A. 可直接移植（低风险）

- `tools/basic` 中无通信耦合工具。
- `tools/math` 中通用数学与弹道工具。
- `buff_tracker` 中纯算法代码（`factors/targets/trajectory/buff_fitter`）。

### B. 建议 ROS2 替换（高收益）

- `tools/hardware/*listener`：改为 ROS2 topic/subscription。
- `tools/transform/tf_listener` + `fast_tf`：改 `tf2_ros::Buffer/TransformListener`。
- `common_defs/msgs/*`：改为 `rm_interfaces` + 标准 ROS2 消息。
- `TaskMode` 链路：改为 `rm_interfaces::srv::SetMode` 或统一模式 topic。

### C. 建议删除（不必要依赖）

- `iceoryx_deps` 与 `iceoryx_posh/hoofs/platform`。
- `common_defs/types` 中大量 `Sample` 镜像构造。
- `fast_tf`（在采用 tf2 后）。
- `cxxopts`（改 ROS2 参数后可去除）。
- `quill`（可统一用 `rclcpp` 日志）。
- `reflect-cpp(rfl)`（改参数体系后可去除）。

## 4. 与当前工程接口对齐建议

优先使用 `rm_interfaces` 现有消息进行对齐：

- Rune 检测输出：优先映射 `rm_interfaces/msg/RuneTarget.msg`。
- 云台状态输入：从 `rm_interfaces/msg/SerialReceiveData.msg` / `GimbalState.msg` 获取。
- 云台控制输出：映射 `rm_interfaces/msg/GimbalCmd.msg`。

## 5. 推荐迁移路线（执行顺序）

### Phase 0: 基础清理

- `auto_buff` 保持 `package.xml + CMakeLists.txt`。
- 移除 xmake/iceoryx 入口。

### Phase 1: Tracker 先 ROS2 化

- 保留 tracker 算法本体。
- 重写 `hardware/*listener` 为 ROS2 订阅。
- 重写 TF 查询到 tf2。
- 打通 tracker 输入/输出消息。

### Phase 2: Detector ROS2 化

- 图像输入改 `sensor_msgs/Image`。
- 模式切换改 ROS2 服务或参数。
- 输出统一成 `RuneTarget`（或适配器中间层）。

### Phase 3: 接入 gimbal_pipeline

- 适配器将 buff 结果转 `TrackedRobot(s)`。
- buff 策略与 armor 策略解耦并可配置开关。

## 6. 风险与控制

- 风险1：消息语义不一致。  
  控制：先定义字段映射表，再做代码迁移。

- 风险2：TF 时间戳/坐标系不一致。  
  控制：统一 `odom/camera` 时基和查询容差。

- 风险3：一次性改动过大。  
  控制：按 Phase 拆分，先 tracker 后 detector，逐层验收。
