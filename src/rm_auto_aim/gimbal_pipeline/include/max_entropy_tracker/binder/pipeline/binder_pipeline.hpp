// Copyright (C) Max Entropy Tracker. Licensed under the MIT License.
#ifndef MAX_ENTROPY_TRACKER_BINDER_PIPELINE_BINDER_PIPELINE_HPP_
#define MAX_ENTROPY_TRACKER_BINDER_PIPELINE_BINDER_PIPELINE_HPP_

#include <memory>
#include <optional>

#include "max_entropy_tracker/binder/core/binding_fsm.hpp"
#include "max_entropy_tracker/binder/debug/binder_debug_snapshot.hpp"
#include "max_entropy_tracker/binder/decoder/jump_event_decoder.hpp"
#include "max_entropy_tracker/binder/id_binder/id_binder.hpp"
#include "max_entropy_tracker/binder/scorer/hypothesis_scorer.hpp"

namespace fyt::auto_aim::binder {

struct BinderPipelineConfig {
  BindingFSMConfig fsm;
};

class BinderPipeline {
 public:
  BinderPipeline(std::unique_ptr<JumpEventDecoder> decoder,
                 std::unique_ptr<IDBinder> id_binder,
                 std::unique_ptr<HypothesisScorer> scorer,
                 const BinderPipelineConfig & config);

  BinderOutput step(const BinderFrameInput & input);

  void reset(int init_panel_id, HeightLabel init_label,
             std::optional<double> obs_z = std::nullopt);

  const BinderDebugSnapshot & debug_snapshot() const { return debug_; }
  const DecoderContext & decoder_context() const { return decoder_ctx_; }
  const BinderContext & binder_context() const { return binder_ctx_; }

  void set_confidence(double conf) { fsm_.set_confidence(conf); }

 private:
  std::unique_ptr<JumpEventDecoder> decoder_;
  std::unique_ptr<IDBinder> id_binder_;
  std::unique_ptr<HypothesisScorer> scorer_;
  BinderPipelineConfig config_;
  BindingFSM fsm_;

  DecoderContext decoder_ctx_;
  BinderContext binder_ctx_;
  ScorerContext scorer_ctx_;
  BinderDebugSnapshot debug_;
};

}  // namespace fyt::auto_aim::binder

#endif  // MAX_ENTROPY_TRACKER_BINDER_PIPELINE_BINDER_PIPELINE_HPP_
