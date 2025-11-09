# Kalman Filter 工厂模式实现总结

## 概述

为 `src/kalmanFilters` 包实现了完整的工厂方法模式，提供统一的接口创建各种 Kalman 滤波器模型。

## 实现的功能

### 1. 核心工厂框架

**文件位置**: `src/kalmanFilters/models/`

- **model_factory.h/cpp**: 
  - `ModelConfig`: 模型配置参数结构
  - `ModelFactoryBase`: 抽象工厂基类
  - `ModelFactoryRegistry`: 工厂注册器（单例模式）
  - `AutoRegisterFactory`: 自动注册辅助类

### 2. 基本模型工厂

**文件位置**: `src/kalmanFilters/basic_models/`

- `basic_model_factories.h/cpp`: 实现了以下工厂
  - `CA_KF_Factory`: 恒加速度模型工厂
  - `CV_KF_Factory`: 恒速度模型工厂
  - `CS_KF_Factory`: CS 模型工厂
  - `CTRV_EKF_Factory`: 恒转角速度 EKF 模型工厂
  - `Singer_KF_Factory`: Singer 模型工厂

### 3. 组合模型工厂

**文件位置**: `src/kalmanFilters/combined_models/`

- `combined_model_factories.h/cpp`: 实现了以下工厂
  - `IMM_CV_CA_CS_3Dim_Factory`
  - `IMM_CV_CA_CS_Singer_3Dim_Factory`
  - `IMM_CV_CA_CTRV_Factory`
  - `IMM_CS_Parallel_Factory`
  - `IMM_Factory`: 通用 IMM 工厂，支持自定义子模型组合

### 4. 配置文件加载器

**文件位置**: `src/kalmanFilters/models/`

- **model_config_loader.h/cpp**: 
  - 支持从 YAML 文件加载模型配置
  - 自动解析矩阵和向量
  - 直接从配置文件创建模型实例

### 5. 配置文件示例

**文件位置**: `src/kalmanFilters/models/config/`

提供了多个配置文件示例：
- `ca_kf_config.yaml`: CA 模型配置
- `cv_kf_config.yaml`: CV 模型配置
- `singer_kf_config.yaml`: Singer 模型配置
- `imm_cv_ca_cs_3dim_config.yaml`: IMM 组合模型配置
- `imm_custom_config.yaml`: 自定义 IMM 配置

### 6. 示例程序

**文件位置**: `src/kalmanFilters/models/src/`

- **factory_demo.cpp**: 基础工厂使用演示
- **complete_example.cpp**: 完整使用示例
- **kalman_filter_node_example.cpp**: ROS2 节点示例

### 7. 文档

**文件位置**: `src/kalmanFilters/models/docs/`

- **FACTORY_PATTERN.md**: 详细的工厂模式使用文档

## 使用方法

### 方法 1: 直接使用工厂

```cpp
#include "models/model_factory.h"
#include "basic_models/basic_model_factories.h"

ModelConfig config;
config.T = 0.01;
config.Dim = 3;
config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005;
config.X_0 = Eigen::VectorXd::Zero(9);

auto model = ModelFactoryRegistry::getInstance().createModel("CA_KF", config);
```

### 方法 2: 从配置文件创建

```cpp
#include "models/model_config_loader.h"

auto model = ModelConfigLoader::createModelFromYaml("ca_kf_config.yaml");
```

### 方法 3: 在 ROS2 节点中使用

```cpp
class MyNode : public rclcpp::Node {
    std::unique_ptr<Models> model_;
    
    void init() {
        std::string model_type;
        this->get_parameter("model_type", model_type);
        
        ModelConfig config;
        // ... 设置配置 ...
        
        model_ = ModelFactoryRegistry::getInstance().createModel(model_type, config);
    }
};
```

## 扩展新模型

1. 创建模型类（继承自 `Models`）
2. 创建工厂类（继承自 `ModelFactoryBase`）
3. 使用宏注册工厂：

```cpp
// MyModel_Factory.cpp
#include "my_package/MyModel_Factory.h"

namespace {
    REGISTER_MODEL_FACTORY(MyModel_Factory, "MyModel");
}
```

4. 创建配置文件（可选）

## 设计模式

### 工厂方法模式
每个具体模型有自己的工厂类，实现 `createModel()` 方法。

### 注册器模式
使用 `ModelFactoryRegistry` 管理所有工厂，支持运行时查询和创建。

### 单例模式
`ModelFactoryRegistry` 使用单例模式确保全局唯一。

### 自动注册
使用静态初始化和宏实现自动注册，无需手动调用注册函数。

## 优势

1. **统一接口**: 所有模型通过相同方式创建
2. **配置驱动**: 支持外部配置文件
3. **易于扩展**: 新增模型只需实现工厂并注册
4. **类型安全**: 使用智能指针管理内存
5. **运行时切换**: 支持动态切换模型类型
6. **自动注册**: 简化工厂注册流程

## 依赖

- **Eigen3**: 矩阵运算库
- **yaml-cpp**: YAML 配置文件解析
- **ROS2** (可选): 如果在 ROS2 环境中使用

## 构建

```bash
cd /home/amatrix/Userfiles/Robomaster/hfut_rm_auto_aim_ws
colcon build --packages-select models basic_models combined_models
```

## 运行示例

```bash
# 基础演示
./build/models/factory_demo

# 完整示例
./build/models/complete_example

# ROS2 节点示例（需要先取消注释 CMakeLists.txt 中的相关代码）
ros2 run models kalman_filter_node_example
```

## 文件结构

```
src/kalmanFilters/
├── models/
│   ├── include/models/
│   │   ├── models.h                    # 抽象基类
│   │   ├── model_factory.h             # 工厂框架
│   │   └── model_config_loader.h       # 配置加载器
│   ├── src/
│   │   ├── models.cpp
│   │   ├── model_factory.cpp
│   │   ├── model_config_loader.cpp
│   │   ├── factory_demo.cpp            # 演示程序
│   │   ├── complete_example.cpp        # 完整示例
│   │   └── kalman_filter_node_example.cpp  # ROS2 示例
│   ├── config/
│   │   ├── ca_kf_config.yaml
│   │   ├── cv_kf_config.yaml
│   │   ├── singer_kf_config.yaml
│   │   ├── imm_cv_ca_cs_3dim_config.yaml
│   │   └── imm_custom_config.yaml
│   ├── docs/
│   │   └── FACTORY_PATTERN.md          # 详细文档
│   ├── CMakeLists.txt
│   └── package.xml
├── basic_models/
│   ├── include/basic_models/
│   │   ├── CA_KF.h
│   │   ├── CV_KF.h
│   │   ├── CS_KF.h
│   │   ├── CTRV_EKF.h
│   │   ├── Singer_KF.h
│   │   └── basic_model_factories.h     # 基本模型工厂
│   ├── src/
│   │   ├── CA_KF.cpp
│   │   ├── CV_KF.cpp
│   │   ├── CS_KF.cpp
│   │   ├── CTRV_EKF.cpp
│   │   ├── Singer_KF.cpp
│   │   └── basic_model_factories.cpp   # 工厂实现
│   ├── CMakeLists.txt
│   └── package.xml
└── combined_models/
    ├── include/combined_models/
    │   ├── IMM.h
    │   ├── IMM_CV_CA_CS_3Dim.h
    │   ├── IMM_CV_CA_CS_Singer_3Dim.h
    │   ├── IMM_CV_CA_CTRV.h
    │   ├── IMM_CS_Parallel.h
    │   └── combined_model_factories.h  # 组合模型工厂
    ├── src/
    │   ├── IMM.cpp
    │   ├── IMM_CV_CA_CS_3Dim.cpp
    │   ├── IMM_CV_CA_CS_Singer_3Dim.cpp
    │   ├── IMM_CV_CA_CTRV.cpp
    │   ├── IMM_CS_Parallel.cpp
    │   └── combined_model_factories.cpp # 工厂实现
    ├── CMakeLists.txt
    └── package.xml
```

## 注意事项

1. 确保在使用前已链接相应的库（basic_models 或 combined_models）
2. IMM 模型的子模型指针生命周期由 IMM 类管理
3. 配置文件路径相对于程序运行目录
4. 矩阵和向量的维度必须与模型要求一致
5. 如需在 ROS2 中使用节点示例，需在 package.xml 中添加 rclcpp 依赖

## 后续改进建议

1. 添加参数验证功能
2. 支持更多配置格式（JSON, XML 等）
3. 添加模型性能监控和日志
4. 实现模型状态的序列化和反序列化
5. 提供更多预定义配置模板
6. 添加单元测试
