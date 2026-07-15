#pragma once

#include <cstdint>

#include "robot/telemetry/log_frame.hpp"

namespace robot {

enum IntegrityFault : std::uint32_t {
  kIntegrityOk = 0,
  kBadLogMagic = 1u << 0,
  kBadLogSchema = 1u << 1,
  kBadLogFrameSize = 1u << 2,
  kLogSequenceGap = 1u << 3,
  kLogTimeRegression = 1u << 4,
  kLogEpochRegression = 1u << 5,
  kLogRunMismatch = 1u << 6,
};

struct IntegrityReport {
  std::uint64_t observed_frames{};
  std::uint64_t missing_frames{};
  std::uint64_t time_regressions{};
  std::uint32_t fault_bits{};
};

class LogIntegrityTracker {
 public:
  explicit LogIntegrityTracker(std::uint32_t expected_run_id_hash)
      : expected_run_id_hash_(expected_run_id_hash) {}

  std::uint32_t observe(const LogFrame& frame) noexcept {
    std::uint32_t faults = kIntegrityOk;
    if (frame.header.magic != kLogMagic) faults |= kBadLogMagic;
    if (frame.header.schema_major != kLogSchemaMajor)
      faults |= kBadLogSchema;
    if (frame.header.frame_size_bytes != sizeof(LogFrame))
      faults |= kBadLogFrameSize;
    if (frame.header.run_id_hash != expected_run_id_hash_)
      faults |= kLogRunMismatch;

    if (initialized_) {
      if (frame.header.sequence != previous_sequence_ + 1) {
        faults |= kLogSequenceGap;
        if (frame.header.sequence > previous_sequence_ + 1)
          report_.missing_frames +=
              frame.header.sequence - previous_sequence_ - 1;
      }
      if (frame.header.time_us < previous_time_us_) {
        faults |= kLogTimeRegression;
        ++report_.time_regressions;
      }
      if (frame.header.mode_epoch < previous_epoch_)
        faults |= kLogEpochRegression;
    }

    previous_sequence_ = frame.header.sequence;
    previous_time_us_ = frame.header.time_us;
    previous_epoch_ = frame.header.mode_epoch;
    initialized_ = true;
    ++report_.observed_frames;
    report_.fault_bits |= faults;
    return faults;
  }

  const IntegrityReport& report() const noexcept { return report_; }

 private:
  std::uint32_t expected_run_id_hash_{};
  std::uint32_t previous_sequence_{};
  TimeUs previous_time_us_{};
  std::uint32_t previous_epoch_{};
  bool initialized_{};
  IntegrityReport report_{};
};

}  // namespace robot
