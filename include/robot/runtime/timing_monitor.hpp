#pragma once

#include <algorithm>
#include <cstdint>

#include "robot/core/frame.hpp"

namespace robot {

struct TimingConfig {
  double nominal_period_s{0.010};
  double min_math_dt_s{0.001};
  double max_math_dt_s{0.050};
  double deadline_dt_s{0.015};
};

struct TimingSample {
  FrameHeader h;
  double raw_dt_s{};
  double math_dt_s{};
  double exec_s{};
  double jitter_s{};
  bool deadline_missed{};
  std::uint32_t consecutive_misses{};
};

struct TimingSummary {
  double min_dt_s{};
  double max_dt_s{};
  double max_exec_s{};
  std::uint64_t tick_count{};
  std::uint64_t missed_count{};
  std::uint32_t max_consecutive_misses{};
};

class TimingMonitor {
 public:
  explicit TimingMonitor(TimingConfig config) : config_(config) {}

  TimingSample begin(const FrameHeader& header) {
    TimingSample sample{};
    sample.h = header;
    sample.raw_dt_s =
        initialized_
            ? static_cast<double>(header.time_us - previous_start_us_) * 1e-6
            : config_.nominal_period_s;
    sample.math_dt_s = std::clamp(sample.raw_dt_s, config_.min_math_dt_s,
                                  config_.max_math_dt_s);
    sample.jitter_s = sample.raw_dt_s - config_.nominal_period_s;
    previous_start_us_ = header.time_us;
    initialized_ = true;
    return sample;
  }

  void finish(TimingSample& sample, TimeUs end_us) {
    sample.exec_s = end_us >= sample.h.time_us
                        ? static_cast<double>(end_us - sample.h.time_us) * 1e-6
                        : 0.0;
    sample.deadline_missed = sample.raw_dt_s > config_.deadline_dt_s ||
                             sample.exec_s > config_.nominal_period_s;
    consecutive_misses_ = sample.deadline_missed ? consecutive_misses_ + 1 : 0;
    sample.consecutive_misses = consecutive_misses_;

    if (summary_.tick_count == 0 || sample.raw_dt_s < summary_.min_dt_s)
      summary_.min_dt_s = sample.raw_dt_s;
    summary_.max_dt_s = std::max(summary_.max_dt_s, sample.raw_dt_s);
    summary_.max_exec_s = std::max(summary_.max_exec_s, sample.exec_s);
    ++summary_.tick_count;
    if (sample.deadline_missed) ++summary_.missed_count;
    summary_.max_consecutive_misses =
        std::max(summary_.max_consecutive_misses, consecutive_misses_);
  }

  const TimingSummary& summary() const noexcept { return summary_; }

 private:
  TimingConfig config_{};
  TimeUs previous_start_us_{};
  std::uint32_t consecutive_misses_{};
  bool initialized_{};
  TimingSummary summary_{};
};

}  // namespace robot
