#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "robot/telemetry/recording.hpp"
#include "robot/telemetry/recording_file.hpp"

namespace robot {

class ProsRecordingSessionSink final : public RecordingSessionSink {
 public:
  ProsRecordingSessionSink(const char* robot_id,
                           std::uint32_t robot_id_hash) noexcept;

  bool begin(std::uint32_t session_sequence,
             std::uint32_t start_time_ms) override;
  bool write(const LogFrame* frames, std::size_t count) override;
  bool finish(std::uint32_t producer_drops) override;
  void abort() override;
  RecordingError error() const noexcept override { return error_; }

 private:
  bool ensureDirectory(const char* path);
  bool writeExact(const void* data, std::size_t bytes,
                  RecordingError error);
  void setError(RecordingError error) noexcept;

  char robot_id_[16]{};
  char directory_[128]{};
  char temporary_path_[160]{};
  char committed_path_[160]{};
  std::uint32_t robot_id_hash_{};
  std::uint32_t block_sequence_{};
  std::uint32_t total_frames_{};
  std::uint32_t first_frame_sequence_{};
  std::uint32_t last_frame_sequence_{};
  TimeUs first_time_us_{};
  TimeUs last_time_us_{};
  RecordingError error_{RecordingError::None};
  std::FILE* file_{};
};

}  // namespace robot
