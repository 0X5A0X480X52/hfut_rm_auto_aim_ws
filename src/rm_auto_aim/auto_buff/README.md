# 能量机关组件

包含：

- 扇叶识别节点
- 能量机关追踪即弹道解算节点

详见各个节点自己的README。

## 构建方式

本目录已改为 ROS2 `ament_cmake` 包管理：

- `package.xml`
- `CMakeLists.txt`

并已移除 `xmake.lua`（不再使用 `iceoryx_deps` 规则）。

依赖排查见 `DEPENDENCY_AUDIT.md`。
