#include "models/model_factory.h"
#include "models/model_config_loader.h"
#include "basic_models/basic_model_factories.h"
#include "combined_models/combined_model_factories.h"
#include <iostream>

int main() {
    std::cout << "Factory demo (examples package)." << std::endl;
    // This demo creates a CA_KF instance and prints the registered factories.
    auto factories = ModelFactoryRegistry::getInstance().getRegisteredFactories();
    std::cout << "Registered factories:" << std::endl;
    for (const auto &name : factories) std::cout << "  - " << name << std::endl;

    ModelConfig config;
    config.T = 0.01;
    config.Dim = 3;
    config.R = Eigen::MatrixXd::Identity(3,3) * 0.005;
    config.X_0 = Eigen::VectorXd::Zero(9);

    auto m = ModelFactoryRegistry::getInstance().createModel("CA_KF", config);
    std::cout << "Created CA_KF with state dim: " << m->get_Dim() << std::endl;
    return 0;
}
