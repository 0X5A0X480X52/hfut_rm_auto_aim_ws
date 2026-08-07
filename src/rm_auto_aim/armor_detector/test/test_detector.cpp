// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "armor_detector/armor_detector.hpp"
#include "rm_utils/url_resolver.hpp"

using namespace fyt;
using namespace fyt::auto_aim;
TEST(ArmorDetector, EmptyFrameProducesNoArmors) {
  constexpr int binary_thres = 160;
  Detector::LightParams l_params = {
    .min_ratio = 0.08, .max_ratio = 0.4, .max_angle = 40.0, .color_diff_thresh = 25};
  Detector::ArmorParams a_params = {.min_light_ratio = 0.6,
                                    .min_small_center_distance = 0.8,
                                    .max_small_center_distance = 3.2,
                                    .min_large_center_distance = 3.2,
                                    .max_large_center_distance = 5.0,
                                    .max_angle = 35.0};

  auto detector = std::make_unique<Detector>(binary_thres, EnemyColor::RED, l_params, a_params);

  namespace fs = std::filesystem;
  const fs::path model_path =
    utils::URLResolver::getResolvedPath("package://armor_detector/model/lenet.onnx");
  const fs::path label_path =
    utils::URLResolver::getResolvedPath("package://armor_detector/model/label.txt");
  ASSERT_TRUE(fs::is_regular_file(model_path));
  ASSERT_TRUE(fs::is_regular_file(label_path));

  detector->classifier = std::make_unique<NumberClassifier>(
    model_path, label_path, 0.6, std::vector<std::string>{"negative"});

  const cv::Mat empty_frame = cv::Mat::zeros(1024, 1280, CV_8UC3);
  EXPECT_TRUE(detector->detect(empty_frame).empty());
}
