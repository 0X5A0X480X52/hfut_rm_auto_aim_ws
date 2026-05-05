#include "armor_detector_nn/debug/profiler.hpp"

namespace fyt::auto_aim {

ScopedTimer::ScopedTimer(double& target_ms)
  : target_(target_ms)
  , start_(std::chrono::steady_clock::now())
{
}

ScopedTimer::~ScopedTimer() {
  auto end = std::chrono::steady_clock::now();
  target_ = std::chrono::duration<double, std::milli>(end - start_).count();
}

Profiler::Profiler(size_t window_size)
  : window_size_(window_size)
  , history_(window_size)
{
}

void Profiler::record(const ProfilerEntry& entry) {
  latest_ = entry;
  history_[write_index_] = entry;
  write_index_ = (write_index_ + 1) % window_size_;
  if (count_ < window_size_) ++count_;
}

double Profiler::avgPreprocessMs() const {
  if (count_ == 0) return 0.0;
  double sum = 0.0;
  for (size_t i = 0; i < count_; ++i) sum += history_[i].preprocess_ms;
  return sum / count_;
}

double Profiler::avgInferMs() const {
  if (count_ == 0) return 0.0;
  double sum = 0.0;
  for (size_t i = 0; i < count_; ++i) sum += history_[i].infer_ms;
  return sum / count_;
}

double Profiler::avgTotalMs() const {
  if (count_ == 0) return 0.0;
  double sum = 0.0;
  for (size_t i = 0; i < count_; ++i) sum += history_[i].total_ms;
  return sum / count_;
}

double Profiler::avgFPS() const {
  double avg = avgTotalMs();
  if (avg <= 0.0) return 0.0;
  return 1000.0 / avg;
}

const ProfilerEntry& Profiler::latest() const {
  return latest_;
}

void Profiler::reset() {
  write_index_ = 0;
  count_ = 0;
  latest_ = ProfilerEntry{};
}

}  // namespace fyt::auto_aim
