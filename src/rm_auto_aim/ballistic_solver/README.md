# Ballistic Solver (弹道解算器)

## 概述

`ballistic_solver` 是 RoboMaster 自瞄系统解耦架构中的弹道解算模块。它负责计算击中目标所需的云台 pitch 和 yaw 角度，考虑重力和空气阻力的影响。

## 功能特性

- **迭代弹道求解**: 通过迭代方法精确计算击中目标所需的发射角度
- **重力补偿**: 考虑子弹飞行过程中的重力下坠
- **空气阻力模型**: 支持理想模型和带空气阻力的模型
- **移动目标预测**: 支持对移动目标进行位置预测后求解弹道
- **ROS2 服务接口**: 提供标准服务接口，方便与其他模块集成

## 架构设计

```
ballistic_solver/
├── include/ballistic_solver/
│   ├── ballistic_solver.hpp          # 核心解算器类
│   ├── ballistic_solver_node.hpp     # ROS2 节点封装
│   └── compensator/                  # 弹道补偿器模块
│       ├── trajectory_compensator.hpp    # 补偿器基类
│       ├── ideal_compensator.hpp         # 理想模型补偿器
│       ├── resistance_compensator.hpp    # 空气阻力模型补偿器
│       └── manual_compensator.hpp        # 手动补偿器
├── src/
│   ├── ballistic_solver.cpp          # 核心解算器实现
│   ├── ballistic_solver_node.cpp     # ROS2 节点实现
│   ├── main.cpp                      # 节点入口
│   └── compensator/                  # 补偿器实现
│       ├── trajectory_compensator.cpp
│       ├── ideal_compensator.cpp
│       ├── resistance_compensator.cpp
│       ├── manual_compensator.cpp
│       └── compensator_factory.cpp
├── config/
│   └── ballistic_solver.yaml         # 参数配置
├── launch/
│   └── ballistic_solver.launch.py    # 启动文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 核心类

### 补偿器模块 (compensator/)

#### TrajectoryCompensator (弹道补偿器基类)

提供弹道计算的基本接口:

```cpp
class TrajectoryCompensator {
public:
  // 计算击中目标所需的 pitch 角
  bool compensate(const Eigen::Vector3d& target_position, double& pitch) const;
  
  // 获取飞行时间
  virtual double getFlyingTime(const Eigen::Vector3d& target_position) const = 0;
  
  // 参数
  double velocity;     // 子弹速度
  double gravity;      // 重力加速度
  double resistance;   // 空气阻力系数
};
```

#### IdealCompensator (理想模型)

不考虑空气阻力的简化模型，适用于短距离射击。

#### ResistanceCompensator (空气阻力模型)

考虑空气阻力的模型，适用于中远距离射击。

#### ManualCompensator (手动补偿器)

基于距离-高度二维映射表的角度补偿器，用于弥补机械误差或实验微调:

```cpp
class ManualCompensator {
public:
  // 根据距离和高度查询补偿角度
  std::vector<double> angleHardCorrect(double dist, double height) const;
  
  // 通过字符串配置补偿映射
  // 格式: "dist_lower dist_upper height_lower height_upper pitch_offset yaw_offset"
  bool updateMapByStr(const std::string& str);
};
```

### BallisticSolver (弹道解算器)

封装完整的弹道解算流程:

```cpp
class BallisticSolver {
public:
  // 静态目标求解
  BallisticResult solve(const Eigen::Vector3d& target_position) const;
  
  // 移动目标求解
  BallisticResult solveMovingTarget(
    const Eigen::Vector3d& target_position,
    const Eigen::Vector3d& target_velocity) const;
  
  // 获取手动补偿器 (用于配置)
  ManualCompensator& getManualCompensator();
};
```

## 弹道模型

### 理想模型 (IdealCompensator)

不考虑空气阻力的简化模型:

$$y = v \sin(\theta) \cdot t - \frac{1}{2} g t^2$$

其中:
- $v$: 子弹初速度
- $\theta$: 发射角度
- $t$: 飞行时间
- $g$: 重力加速度

### 空气阻力模型 (ResistanceCompensator)

考虑空气阻力的模型，飞行时间计算为:

$$t = \frac{e^{r \cdot x} - 1}{r \cdot v \cdot \cos(\theta)}$$

其中 $r$ 是空气阻力系数。

## 服务接口

### SolveBallistic.srv

```srv
# 请求
geometry_msgs/Point target_position      # 目标3D位置
geometry_msgs/Vector3 target_velocity    # 目标速度
float64 bullet_speed                     # 子弹速度 (m/s)

---

# 响应
float64 pitch            # 云台pitch角 (弧度)
float64 yaw              # 云台yaw角 (弧度)
float64 flight_time      # 飞行时间 (秒)
bool success             # 是否成功求解
string message           # 错误或成功信息
```

## 参数配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `bullet_speed` | double | 28.0 | 子弹速度 (m/s) |
| `gravity` | double | 9.8 | 重力加速度 (m/s²) |
| `air_resistance` | double | 0.001 | 空气阻力系数 |
| `max_iterations` | int | 50 | 最大迭代次数 |
| `convergence_threshold` | double | 0.001 | 收敛阈值 (m) |
| `compensator_type` | string | "resistance" | 补偿器类型 |

## 使用方法

### 编译

```bash
cd /path/to/workspace
colcon build --packages-select ballistic_solver
source install/setup.bash
```

### 启动节点

```bash
# 使用默认参数
ros2 launch ballistic_solver ballistic_solver.launch.py

# 指定子弹速度
ros2 launch ballistic_solver ballistic_solver.launch.py bullet_speed:=30.0

# 使用理想模型
ros2 launch ballistic_solver ballistic_solver.launch.py compensator_type:=ideal
```

### 服务调用示例

```bash
# 静态目标 (位于 x=5m, y=0m, z=1m)
ros2 service call /ballistic_solver/solve rm_interfaces/srv/SolveBallistic \
    "{target_position: {x: 5.0, y: 0.0, z: 1.0}, target_velocity: {x: 0.0, y: 0.0, z: 0.0}, bullet_speed: 28.0}"

# 移动目标 (以 1m/s 向 y 方向移动)
ros2 service call /ballistic_solver/solve rm_interfaces/srv/SolveBallistic \
    "{target_position: {x: 5.0, y: 0.0, z: 1.0}, target_velocity: {x: 0.0, y: 1.0, z: 0.0}, bullet_speed: 28.0}"
```

### C++ 客户端示例

```cpp
#include "rm_interfaces/srv/solve_ballistic.hpp"

// 创建服务客户端
auto client = node->create_client<rm_interfaces::srv::SolveBallistic>("/ballistic_solver/solve");

// 构建请求
auto request = std::make_shared<rm_interfaces::srv::SolveBallistic::Request>();
request->target_position.x = 5.0;
request->target_position.y = 0.0;
request->target_position.z = 1.0;
request->target_velocity.x = 0.0;
request->target_velocity.y = 0.0;
request->target_velocity.z = 0.0;
request->bullet_speed = 28.0;

// 发送请求
auto future = client->async_send_request(request);

// 处理响应
if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready) {
  auto response = future.get();
  if (response->success) {
    double pitch = response->pitch;  // 弧度
    double yaw = response->yaw;      // 弧度
    double flight_time = response->flight_time;
  }
}
```

## 坐标系约定

- **X 轴**: 前方 (正向为目标方向)
- **Y 轴**: 左侧
- **Z 轴**: 上方
- **Pitch**: 绕 Y 轴旋转 (正值为抬头)
- **Yaw**: 绕 Z 轴旋转 (正值为向左转)

## 与原 armor_solver 的关系

本模块从原 `armor_solver` 包中的 `TrajectoryCompensator` 类解耦而来:

| 原模块 | 新模块 |
|--------|--------|
| `rm_utils/math/trajectory_compensator.hpp` | `ballistic_solver/ballistic_solver.hpp` |
| `Solver::solve()` 中的弹道计算部分 | `BallisticSolver::solve()` |

## 测试

```bash
# 运行单元测试 (如果有)
colcon test --packages-select ballistic_solver

# 查看测试结果
colcon test-result --verbose
```

## 开发计划

- [ ] 添加单元测试
- [ ] 支持更多弹道模型 (如二次阻力模型)
- [ ] 添加弹道可视化功能
- [ ] 支持弹道预测轨迹输出

## License

Apache-2.0
