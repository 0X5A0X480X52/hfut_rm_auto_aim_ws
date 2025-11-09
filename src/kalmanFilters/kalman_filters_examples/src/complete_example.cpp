#include "models/model_factory.h"
#include "basic_models/basic_model_factories.h"
#include <iostream>
#include <vector>

int main() {
    std::cout << "Complete example (examples package)." << std::endl;
    ModelConfig config;
    config.T = 0.01;
    config.Dim = 3;
    config.R = Eigen::MatrixXd::Identity(3,3) * 0.005;
    config.X_0 = Eigen::VectorXd::Zero(9);

    auto ca = ModelFactoryRegistry::getInstance().createModel("CA_KF", config);
    std::vector<Eigen::VectorXd> measurements;
    for (int i=0;i<3;i++) { Eigen::VectorXd z(3); z << i*0.1, i*0.1, i*0.1; measurements.push_back(z); }
    auto res = ca->KalmanFilterWholeProcess(measurements);
    std::cout << "Processed " << res.rows() << " steps." << std::endl;
    return 0;
}
