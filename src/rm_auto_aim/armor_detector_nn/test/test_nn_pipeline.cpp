#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

#include "armor_detector_nn/backend/openvino_backend.hpp"
#include "armor_detector_nn/postprocess/decode_strategy_factory.hpp"

namespace fyt::auto_aim {

TEST(OpenVinoBackend, RejectsMissingModel) {
  BackendConfig config;
  config.model_path = "/tmp/armor_detector_nn_model_does_not_exist.onnx";
  OpenVINOBackend backend;
  EXPECT_THROW(backend.load(config), std::runtime_error);
}

TEST(RobotPilotsDecode, EmptyTensorListProducesNoDetections) {
  auto decoder = DecodeStrategyFactory::create();
  ASSERT_NE(decoder, nullptr);

  ImageMeta image_meta;
  image_meta.original_width = 1280;
  image_meta.original_height = 1024;
  PostprocessConfig config;
  EXPECT_TRUE(decoder->decode({}, image_meta, config).empty());
}

TEST(RobotPilotsDecode, InvalidTensorShapeFailsFast) {
  auto decoder = DecodeStrategyFactory::create();
  ASSERT_NE(decoder, nullptr);

  TensorOutput output;
  output.info.shape = {1, 22};
  output.host_data.resize(22);
  ImageMeta image_meta;
  PostprocessConfig config;
  EXPECT_THROW(decoder->decode({output}, image_meta, config), std::runtime_error);
}

}  // namespace fyt::auto_aim
