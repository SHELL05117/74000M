#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "robot/telemetry/recording.hpp"
#include "robot/telemetry/recording_codec.hpp"

namespace robot {

class ProsRecordingSessionSink final : public RecordingSessionSink,
                                       private RecordingByteWriter {
 public:
  explicit ProsRecordingSessionSink(
      RecordingMetadata metadata) noexcept;

  bool begin(std::uint32_t session_sequence,
             std::uint32_t start_time_ms) override;
  bool write(const LogFrame* frames, std::size_t count) override;
  bool finish(std::uint32_t producer_drops) override;
  void abort() override;
  RecordingError error() const noexcept override { return error_; }

 private:
  bool ensureDirectory(const char* path);
  bool writeBytes(const void* data, std::size_t bytes) override;
  void setError(RecordingError error) noexcept;

  RecordingMetadata metadata_{};
  char directory_[128]{};
  char temporary_path_[160]{};
  char committed_path_[160]{};
  RecordingFileEncoder encoder_{};
  RecordingError error_{RecordingError::None};
  std::FILE* file_{};
  std::uint32_t blocks_since_flush_{};
};

}  // namespace robot
