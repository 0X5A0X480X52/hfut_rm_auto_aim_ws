#ifndef ARMOR_DETECTOR_NN_PROFILER_HPP_
#define ARMOR_DETECTOR_NN_PROFILER_HPP_

#include <chrono>
#include <cstddef>
#include <vector>

namespace fyt::auto_aim {

struct ProfilerEntry {
  double preprocess_ms{0.0};
  double infer_ms{0.0};
  double decode_ms{0.0};
  double nms_ms{0.0};
  double pose_ms{0.0};
  double total_ms{0.0};

  int raw_candidates{0};
  int after_conf{0};
  int after_nms{0};
  int published{0};

  int batch_size{1};
  double batch_wait_ms{0.0};
  double observation_age_ms{0.0};
  int queue_size{0};
  int dropped_old{0};
  int dropped_stale{0};
};

class ScopedTimer {
public:
  explicit ScopedTimer(double& target_ms);
  ~ScopedTimer();

private:
  double& target_;
  std::chrono::steady_clock::time_point start_;
};

class Profiler {
public:
  explicit Profiler(size_t window_size = 100);

  void record(const ProfilerEntry& entry);

  double avgPreprocessMs() const;
  double avgInferMs() const;
  double avgTotalMs() const;
  double avgFPS() const;

  const ProfilerEntry& latest() const;

  void reset();

private:
  size_t window_size_;
  std::vector<ProfilerEntry> history_;
  size_t write_index_{0};
  size_t count_{0};
  ProfilerEntry latest_;
};

}  // namespace fyt::auto_aim

#endif
