#include <rclcpp/rclcpp.hpp>
#include "models/model_factory.h"
#include "basic_models/basic_model_factories.h"
#include <vector>

class MinimalNode : public rclcpp::Node {
public:
    MinimalNode(): Node("minimal_kf_node") {
        ModelConfig config; config.T=0.01; config.Dim=3; config.R=Eigen::MatrixXd::Identity(3,3)*0.005; config.X_0=Eigen::VectorXd::Zero(9);
        model_ = ModelFactoryRegistry::getInstance().createModel("CA_KF", config);
        RCLCPP_INFO(this->get_logger(), "Minimal KF node started");
    }
    Eigen::VectorXd process(const Eigen::VectorXd& z) { return model_->KalmanFilterIterator(z); }
private:
    std::unique_ptr<Models> model_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MinimalNode>();
    Eigen::VectorXd z(3); z << 1.0, 2.0, 3.0;
    auto r = node->process(z);
    RCLCPP_INFO(node->get_logger(), "Filtered: [%.3f, %.3f, %.3f]", r(0), r(1), r(2));
    rclcpp::shutdown();
    return 0;
}
