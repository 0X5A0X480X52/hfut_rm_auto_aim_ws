#include "combined_models/combined_model_factories.h"

// 自动注册所有组合模型工厂
namespace {
    REGISTER_MODEL_FACTORY(IMM_CV_CA_CS_3Dim_Factory, "IMM_CV_CA_CS_3Dim");
    REGISTER_MODEL_FACTORY(IMM_CV_CA_CS_Singer_3Dim_Factory, "IMM_CV_CA_CS_Singer_3Dim");
    REGISTER_MODEL_FACTORY(IMM_CV_CA_CTRV_Factory, "IMM_CV_CA_CTRV");
    REGISTER_MODEL_FACTORY(IMM_CS_Parallel_Factory, "IMM_CS_Parallel");
    REGISTER_MODEL_FACTORY(IMM_Factory, "IMM");
}
