#ifndef ARMOR_DETECTOR_NN_DECODE_STRATEGY_HPP_
#define ARMOR_DETECTOR_NN_DECODE_STRATEGY_HPP_

#include <vector>

#include "armor_detector_nn/core/detection_types.hpp"
#include "armor_detector_nn/core/detector_config.hpp"

namespace fyt::auto_aim {

struct ImageMeta {
  int original_width{0};
  int original_height{0};
  float scale_x{1.0F};
  float scale_y{1.0F};
  float pad_left{0.0F};
  float pad_top{0.0F};
};

class IDecodeStrategy {
public:
  virtual ~IDecodeStrategy() = default;

  virtual std::vector<RawDetection> decode(
    const std::vector<TensorOutput>& outputs,
    const ImageMeta& image_meta,
    const PostprocessConfig& config) = 0;

protected:
  IDecodeStrategy() = default;
  IDecodeStrategy(const IDecodeStrategy&) = delete;
  IDecodeStrategy& operator=(const IDecodeStrategy&) = delete;
};

}  // namespace fyt::auto_aim

#endif
