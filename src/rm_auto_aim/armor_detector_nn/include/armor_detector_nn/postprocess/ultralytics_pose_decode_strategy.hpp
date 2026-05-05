#ifndef ARMOR_DETECTOR_NN_ULTRALYTICS_POSE_DECODE_STRATEGY_HPP_
#define ARMOR_DETECTOR_NN_ULTRALYTICS_POSE_DECODE_STRATEGY_HPP_

#include "armor_detector_nn/postprocess/decode_strategy.hpp"

namespace fyt::auto_aim {

class UltralyticsPoseDecodeStrategy : public IDecodeStrategy {
public:
  UltralyticsPoseDecodeStrategy() = default;
  ~UltralyticsPoseDecodeStrategy() override = default;

  std::vector<RawDetection> decode(
    const std::vector<TensorOutput>& outputs,
    const ImageMeta& image_meta,
    const PostprocessConfig& config) override;

private:
  bool decodeCandidate(
    const float* data,
    int index,
    int num_candidates,
    const PostprocessConfig& config,
    RawDetection& detection) const;

  void restoreBbox(
    RawDetection& detection,
    const ImageMeta& meta,
    const PostprocessConfig& config) const;

  void restoreKeypoints(
    RawDetection& detection,
    const ImageMeta& meta,
    const PostprocessConfig& config) const;
};

}  // namespace fyt::auto_aim

#endif
