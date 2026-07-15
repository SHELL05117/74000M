#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "robot/telemetry/log_frame.hpp"
#include "robot/telemetry/spsc_ring.hpp"

namespace robot {

class TelemetrySink {
 public:
  virtual bool write(const LogFrame* frames, std::size_t count) = 0;
  virtual ~TelemetrySink() = default;
};

template <std::size_t RingCapacity, std::size_t BatchSize>
class TelemetryDrain {
  static_assert(BatchSize > 0);

 public:
  TelemetryDrain(SpscRing<LogFrame, RingCapacity>& ring, TelemetrySink& sink)
      : ring_(ring), sink_(sink) {}

  std::size_t drainOnce() {
    std::size_t count = 0;
    while (count < BatchSize && ring_.tryPop(batch_[count])) ++count;
    if (count == 0) return 0;
    if (!sink_.write(batch_.data(), count)) {
      ++sink_failure_count_;
      discarded_frames_ += static_cast<std::uint32_t>(count);
    }
    return count;
  }

  std::uint32_t sinkFailureCount() const noexcept {
    return sink_failure_count_;
  }
  std::uint32_t discardedFrames() const noexcept { return discarded_frames_; }

 private:
  SpscRing<LogFrame, RingCapacity>& ring_;
  TelemetrySink& sink_;
  std::array<LogFrame, BatchSize> batch_{};
  std::uint32_t sink_failure_count_{};
  std::uint32_t discarded_frames_{};
};

}  // namespace robot
