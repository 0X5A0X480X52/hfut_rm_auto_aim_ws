# auto_buff 依赖排查（2026-05-09）

## 1) 已完成改造

- 已新增 ROS2 包结构文件：
  - `package.xml`
  - `CMakeLists.txt`
- 已停止使用 xmake `iceoryx_deps` 规则（`xmake.lua` 已移除）。

## 2) 当前源码可见依赖（按 include/链接统计）

- 基础构建：`ament_cmake`
- 数学/视觉：`Eigen3`、`OpenCV`
- 配置与参数：`yaml-cpp`、`cxxopts`、`reflect-cpp (rfl)`
- 日志：`quill`
- 优化/图优化：`Ceres`、`GTSAM`
- 推理：`OpenVINO`（`openvino/openvino.hpp`）
- 通信：`iceoryx_posh`、`iceoryx_hoofs`
- 线程：`Threads`

## 3) 当前阻塞项（代码级缺失，不是系统包名问题）

本包引用了大量头文件，但在 `src/rm_auto_aim/auto_buff` 与当前仓库内未找到定义：

- `basic/*`
- `hardware/*`
- `math/*`
- `msgs/*`
- `types/*`（例如 `types/TaskMode.hpp`）
- `confs/*`
- `transform/*`

这说明 `auto_buff` 目前仍依赖一套外部公共代码（很可能来自原始 JLU 工程的 common_defs/tools/msgs 等模块）。

## 4) 建议下一步

1. 先补齐上述缺失模块来源（作为独立 ROS2 包或 vendored 子目录）。
2. 再执行 `colcon build --packages-select auto_buff` 做二次依赖收敛。
3. 第二轮会进一步暴露精确的链接缺口（例如 quill/rfl/openvino 的库符号或 cmake target 名称）。
