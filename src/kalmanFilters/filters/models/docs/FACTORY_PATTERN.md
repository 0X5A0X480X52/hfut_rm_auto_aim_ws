# Kalman Filter Model Factory

这个工厂模式实现为 Kalman 滤波器模型提供了统一的创建接口。

## 架构设计

### 工厂方法模式

采用工厂方法模式，每个具体模型都有对应的工厂类：

```
ModelFactoryBase (抽象工厂)
    ├── CA_KF_Factory
    ├── CV_KF_Factory
    ├── CS_KF_Factory
    ├── CTRV_EKF_Factory
    ├── Singer_KF_Factory
    ├── IMM_CV_CA_CS_3Dim_Factory
    ├── IMM_CV_CA_CS_Singer_3Dim_Factory
    ├── IMM_CV_CA_CTRV_Factory
    ├── IMM_CS_Parallel_Factory
    └── IMM_Factory (通用 IMM 工厂)
```

### 核心组件

1. **ModelConfig**: 模型配置参数结构体
2. **ModelFactoryBase**: 工厂基类
3. **ModelFactoryRegistry**: 工厂注册器（单例模式）
4. **ModelConfigLoader**: 配置文件加载器
5. **AutoRegisterFactory**: 自动注册辅助类

## 使用方法

### 1. 直接使用工厂创建模型

```cpp
#include "models/model_factory.h"
#include "basic_models/basic_model_factories.h"

// 配置参数
ModelConfig config;
config.T = 0.01;                                    // 采样时间
config.Dim = 3;                                     // 状态维度
config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005; // 观测噪声协方差
config.X_0 = Eigen::VectorXd::Zero(9);              // 初始状态

// 创建 CA_KF 模型
auto model = ModelFactoryRegistry::getInstance().createModel("CA_KF", config);

// 使用模型
Eigen::VectorXd measurement(3);
measurement << 1.0, 2.0, 3.0;
Eigen::VectorXd result = model->KalmanFilterIterator(measurement);
```

### 2. 从配置文件创建模型

#### 创建配置文件 (YAML)

```yaml
# ca_kf_config.yaml
model_type: "CA_KF"

T: 0.01
Dim: 3

R:
  rows: 3
  cols: 3
  data: [
    [0.005, 0.001, 0.001],
    [0.001, 0.005, 0.001],
    [0.001, 0.001, 0.005]
  ]

X_0:
  size: 9
  data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

#### 加载配置并创建模型

```cpp
#include "models/model_config_loader.h"

// 从配置文件创建模型
auto model = ModelConfigLoader::createModelFromYaml("ca_kf_config.yaml");
```

### 3. 创建组合模型 (IMM)

#### 使用预定义的 IMM 模型

```cpp
ModelConfig config;
config.T = 0.01;
config.Dim = 3;
config.R = Eigen::MatrixXd::Identity(3, 3) * 0.005;
config.X_0 = Eigen::VectorXd::Zero(9);

// 创建 IMM_CV_CA_CS_3Dim 模型
auto model = ModelFactoryRegistry::getInstance().createModel(
    "IMM_CV_CA_CS_3Dim", config
);
```

#### 使用通用 IMM 工厂自定义子模型组合

```yaml
# imm_custom_config.yaml
model_type: "IMM"

T: 0.01
Dim: 3

sub_models:
  - "CV_KF"
  - "CA_KF"
  - "CS_KF"

R:
  rows: 3
  cols: 3
  data: [[0.005, 0.001, 0.001],
         [0.001, 0.005, 0.001],
         [0.001, 0.001, 0.005]]

X_0:
  size: 9
  data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

H:
  rows: 3
  cols: 9
  data: [[1, 0, 0, 0, 0, 0, 0, 0, 0],
         [0, 0, 0, 1, 0, 0, 0, 0, 0],
         [0, 0, 0, 0, 0, 0, 1, 0, 0]]

transform_rate_mat:
  rows: 3
  cols: 3
  data: [[0.60, 0.20, 0.20],
         [0.20, 0.60, 0.20],
         [0.20, 0.20, 0.60]]
```

```cpp
auto model = ModelConfigLoader::createModelFromYaml("imm_custom_config.yaml");
```

### 4. 查询已注册的工厂

```cpp
// 获取所有已注册的工厂名称
auto factories = ModelFactoryRegistry::getInstance().getRegisteredFactories();
for (const auto& name : factories) {
    std::cout << name << std::endl;
}

// 检查特定工厂是否存在
if (ModelFactoryRegistry::getInstance().hasFactory("CA_KF")) {
    std::cout << "CA_KF factory is available" << std::endl;
}
```

## 扩展新模型

### 1. 为新模型创建工厂类

```cpp
// include/my_package/MyModel_Factory.h
#include "models/model_factory.h"
#include "my_package/MyModel.h"

class MyModel_Factory : public ModelFactoryBase {
public:
    std::unique_ptr<Models> createModel(const ModelConfig& config) const override {
        auto model = std::make_unique<MyModel>(config.T, config.Dim, config.R);
        model->KalmanFilterInit(config.X_0);
        return model;
    }
    
    std::string getFactoryName() const override {
        return "MyModel";
    }
};
```

### 2. 注册工厂

```cpp
// src/MyModel_Factory.cpp
#include "my_package/MyModel_Factory.h"

// 自动注册
namespace {
    REGISTER_MODEL_FACTORY(MyModel_Factory, "MyModel");
}
```

### 3. 使用新模型

```cpp
ModelConfig config;
// ... 设置配置 ...

auto model = ModelFactoryRegistry::getInstance().createModel("MyModel", config);
```

## 配置文件格式

### 基本模型配置

```yaml
model_type: "模型名称"  # 必需
T: 0.01                # 采样时间
Dim: 3                 # 状态维度

R:                     # 观测噪声协方差矩阵
  rows: 3
  cols: 3
  data: [[...]]

X_0:                   # 初始状态向量
  size: 9
  data: [...]

extra_params:          # 可选：模型特定参数
  param_name: value
```

### IMM 模型配置

```yaml
model_type: "IMM"
T: 0.01
Dim: 3

sub_models:            # 子模型列表
  - "CV_KF"
  - "CA_KF"

H:                     # 观测矩阵
  rows: 3
  cols: 9
  data: [[...]]

transform_rate_mat:    # 转移概率矩阵
  rows: 3
  cols: 3
  data: [[...]]
```

## 已支持的模型

### 基本模型 (basic_models)
- **CA_KF**: 恒加速度模型
- **CV_KF**: 恒速度模型
- **CS_KF**: 恒速度 + Singer 模型
- **CTRV_EKF**: 恒转角速度模型（EKF）
- **Singer_KF**: Singer 模型

### 组合模型 (combined_models)
- **IMM_CV_CA_CS_3Dim**: CV + CA + CS 三维 IMM
- **IMM_CV_CA_CS_Singer_3Dim**: CV + CA + CS + Singer 三维 IMM
- **IMM_CV_CA_CTRV**: CV + CA + CTRV IMM
- **IMM_CS_Parallel**: CS 并行 IMM
- **IMM**: 通用 IMM（支持自定义子模型组合）

## 优势

1. **统一接口**: 所有模型通过相同的方式创建和使用
2. **配置驱动**: 支持通过配置文件创建模型，便于参数调整
3. **易于扩展**: 新增模型只需实现工厂类并注册
4. **类型安全**: 使用智能指针管理内存，避免内存泄漏
5. **自动注册**: 使用宏简化工厂注册过程

## 注意事项

1. 确保在使用工厂之前已加载对应的工厂实现（链接了对应的库）
2. IMM 模型的子模型指针管理需要注意生命周期
3. 配置文件路径是相对于程序运行目录的
4. 矩阵和向量的维度必须与模型要求一致

## 示例程序

参考 `src/factory_demo.cpp` 了解完整的使用示例。

## 依赖

- Eigen3: 矩阵运算
- yaml-cpp: YAML 配置文件解析
- ROS2 (可选): 如果需要在 ROS2 环境中使用
