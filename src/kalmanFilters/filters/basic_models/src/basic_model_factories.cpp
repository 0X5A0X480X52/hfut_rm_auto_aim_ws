#include "basic_models/basic_model_factories.h"

// 自动注册所有基本模型工厂
namespace {
    REGISTER_MODEL_FACTORY(CA_KF_Factory, "CA_KF");
    REGISTER_MODEL_FACTORY(CV_KF_Factory, "CV_KF");
    REGISTER_MODEL_FACTORY(CS_KF_Factory, "CS_KF");
    REGISTER_MODEL_FACTORY(CTRV_EKF_Factory, "CTRV_EKF");
    REGISTER_MODEL_FACTORY(Singer_KF_Factory, "Singer_KF");
}
