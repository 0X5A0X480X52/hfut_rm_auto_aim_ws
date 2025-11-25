#include "muit_obj_tracker/model/kalman_model.hpp"

namespace muit_obj_tracker {

KalmanModel::KalmanModel(std::shared_ptr<Models> kf_impl, MeasurementExtractor extractor)
    : kf(kf_impl), extractor(extractor) {}

KalmanModel::KalmanModel(const std::string& model_name, const ModelConfig& config, MeasurementExtractor extractor)
    : extractor(extractor) {
    kf = ModelFactoryRegistry::getInstance().createModel(model_name, config);
}

void KalmanModel::predict() {
    if (kf) {
        kf->performPredict();
    }
}

void KalmanModel::update(const Detection& det) {
    if (kf && extractor) {
        Eigen::VectorXd Z = extractor(det);
        kf->performUpdate(Z);
    }
}

Eigen::VectorXd KalmanModel::getState() const {
    if (kf) {
        return kf->get_X_after();
    }
    return Eigen::VectorXd();
}

} // namespace muit_obj_tracker
