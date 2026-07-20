#include "robot/platform/pros_recording_sink.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

#include "pros/misc.h"

namespace robot {

ProsRecordingSessionSink::ProsRecordingSessionSink(
    const char* robot_id, std::uint32_t robot_id_hash) noexcept
    : robot_id_hash_(robot_id_hash) {
  if (robot_id != nullptr)
    std::snprintf(robot_id_, sizeof(robot_id_), "%s", robot_id);
}

bool ProsRecordingSessionSink::ensureDirectory(const char* path) {
  if (::mkdir(path, 0777) == 0 || errno == EEXIST) return true;
  setError(RecordingError::DirectoryCreate);
  return false;
}

bool ProsRecordingSessionSink::begin(std::uint32_t session_sequence,
                                     std::uint32_t start_time_ms) {
  abort();
  error_ = RecordingError::None;
  block_sequence_ = 0;
  total_frames_ = 0;
  first_frame_sequence_ = 0;
  last_frame_sequence_ = 0;
  first_time_us_ = 0;
  last_time_us_ = 0;

  if (pros::c::usd_is_installed() != 1) {
    setError(RecordingError::CardMissing);
    return false;
  }
  if (robot_id_[0] == '\0' || !ensureDirectory("/usd/FLIGHT")) {
    if (error_ == RecordingError::None)
      setError(RecordingError::DirectoryCreate);
    return false;
  }

  char robot_root[64]{};
  std::snprintf(robot_root, sizeof(robot_root), "/usd/FLIGHT/%s", robot_id_);
  if (!ensureDirectory(robot_root)) return false;

  bool created = false;
  std::uint32_t allocated = session_sequence;
  for (std::uint32_t attempt = 0; attempt < 10000; ++attempt, ++allocated) {
    std::snprintf(directory_, sizeof(directory_), "%s/R%06lu_T%010lu",
                  robot_root, static_cast<unsigned long>(allocated),
                  static_cast<unsigned long>(start_time_ms));
    if (::mkdir(directory_, 0777) == 0) {
      created = true;
      break;
    }
    if (errno != EEXIST) break;
  }
  if (!created) {
    setError(RecordingError::DirectoryCreate);
    return false;
  }

  std::snprintf(temporary_path_, sizeof(temporary_path_), "%s/DATA.TMP",
                directory_);
  std::snprintf(committed_path_, sizeof(committed_path_), "%s/DATA.V5L",
                directory_);
  file_ = std::fopen(temporary_path_, "wb");
  if (file_ == nullptr) {
    setError(RecordingError::FileOpen);
    return false;
  }

  RecordingFileHeader header{};
  header.session_sequence = allocated;
  header.start_time_ms = start_time_ms;
  header.robot_id_hash = robot_id_hash_;
  header.header_crc32 = telemetryCrc32(
      reinterpret_cast<const std::uint8_t*>(&header),
      offsetof(RecordingFileHeader, header_crc32));
  if (!writeExact(&header, sizeof(header), RecordingError::HeaderWrite))
    return false;
  if (std::fflush(file_) != 0) {
    setError(RecordingError::Flush);
    return false;
  }
  return true;
}

bool ProsRecordingSessionSink::write(const LogFrame* frames,
                                     std::size_t count) {
  if (file_ == nullptr || frames == nullptr || count == 0) {
    setError(RecordingError::Internal);
    return false;
  }
  RecordingBlockHeader block{};
  block.block_sequence = ++block_sequence_;
  block.first_frame_sequence = frames[0].header.sequence;
  block.frame_count = static_cast<std::uint32_t>(count);
  block.payload_bytes =
      static_cast<std::uint32_t>(count * sizeof(LogFrame));
  block.payload_crc32 = telemetryCrc32(
      reinterpret_cast<const std::uint8_t*>(frames), block.payload_bytes);
  block.header_crc32 = telemetryCrc32(
      reinterpret_cast<const std::uint8_t*>(&block),
      offsetof(RecordingBlockHeader, header_crc32));
  if (!writeExact(&block, sizeof(block), RecordingError::DataWrite) ||
      !writeExact(frames, block.payload_bytes, RecordingError::DataWrite))
    return false;

  if (total_frames_ == 0) {
    first_frame_sequence_ = frames[0].header.sequence;
    first_time_us_ = frames[0].header.time_us;
  }
  total_frames_ += static_cast<std::uint32_t>(count);
  last_frame_sequence_ = frames[count - 1].header.sequence;
  last_time_us_ = frames[count - 1].header.time_us;
  return true;
}

bool ProsRecordingSessionSink::finish(std::uint32_t producer_drops) {
  if (file_ == nullptr) {
    setError(RecordingError::Internal);
    return false;
  }
  RecordingFileFooter footer{};
  footer.total_frames = total_frames_;
  footer.first_frame_sequence = first_frame_sequence_;
  footer.last_frame_sequence = last_frame_sequence_;
  footer.first_time_us = first_time_us_;
  footer.last_time_us = last_time_us_;
  footer.producer_drops = producer_drops;
  footer.block_count = block_sequence_;
  footer.footer_crc32 = telemetryCrc32(
      reinterpret_cast<const std::uint8_t*>(&footer),
      offsetof(RecordingFileFooter, footer_crc32));
  if (!writeExact(&footer, sizeof(footer), RecordingError::FooterWrite))
    return false;
  if (std::fflush(file_) != 0) {
    setError(RecordingError::Flush);
    return false;
  }
  if (std::fclose(file_) != 0) {
    file_ = nullptr;
    setError(RecordingError::Close);
    return false;
  }
  file_ = nullptr;
  if (std::rename(temporary_path_, committed_path_) != 0) {
    setError(RecordingError::CommitRename);
    return false;
  }
  return true;
}

void ProsRecordingSessionSink::abort() {
  if (file_ != nullptr) {
    std::fflush(file_);
    std::fclose(file_);
    file_ = nullptr;
  }
}

bool ProsRecordingSessionSink::writeExact(const void* data, std::size_t bytes,
                                          RecordingError error) {
  if (file_ != nullptr && std::fwrite(data, 1, bytes, file_) == bytes)
    return true;
  setError(error);
  return false;
}

void ProsRecordingSessionSink::setError(RecordingError error) noexcept {
  if (error_ == RecordingError::None) error_ = error;
}

}  // namespace robot
