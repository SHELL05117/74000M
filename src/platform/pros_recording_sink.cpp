#include "robot/platform/pros_recording_sink.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>

#include "pros/misc.h"

namespace robot {

namespace {
constexpr std::uint32_t kPeriodicFlushBlocks = 50;
}

ProsRecordingSessionSink::ProsRecordingSessionSink(
    RecordingMetadata metadata) noexcept
    : metadata_(metadata) {}

bool ProsRecordingSessionSink::ensureDirectory(const char* path) {
  if (::mkdir(path, 0777) == 0 || errno == EEXIST) return true;
  setError(RecordingError::DirectoryCreate);
  return false;
}

bool ProsRecordingSessionSink::begin(std::uint32_t session_sequence,
                                     std::uint32_t start_time_ms) {
  abort();
  error_ = RecordingError::None;
  encoder_.reset();
  blocks_since_flush_ = 0;

  if (pros::c::usd_is_installed() != 1) {
    setError(RecordingError::CardMissing);
    return false;
  }
  if (metadata_.robot_id[0] == '\0' ||
      !ensureDirectory("/usd/FLIGHT")) {
    if (error_ == RecordingError::None)
      setError(RecordingError::DirectoryCreate);
    return false;
  }

  char robot_root[64]{};
  std::snprintf(robot_root, sizeof(robot_root), "/usd/FLIGHT/%s",
                metadata_.robot_id);
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

  if (!encoder_.begin(*this, metadata_, session_sequence, allocated,
                      start_time_ms)) {
    setError(RecordingError::HeaderWrite);
    return false;
  }
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
  if (!encoder_.append(frames, count)) {
    setError(RecordingError::DataWrite);
    return false;
  }
  if (++blocks_since_flush_ >= kPeriodicFlushBlocks) {
    if (std::fflush(file_) != 0) {
      setError(RecordingError::Flush);
      return false;
    }
    blocks_since_flush_ = 0;
  }
  return true;
}

bool ProsRecordingSessionSink::finish(std::uint32_t producer_drops) {
  if (file_ == nullptr) {
    setError(RecordingError::Internal);
    return false;
  }
  if (!encoder_.finish(producer_drops)) {
    setError(RecordingError::FooterWrite);
    return false;
  }
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

bool ProsRecordingSessionSink::writeBytes(const void* data,
                                          std::size_t bytes) {
  return file_ != nullptr &&
         std::fwrite(data, 1, bytes, file_) == bytes;
}

void ProsRecordingSessionSink::setError(RecordingError error) noexcept {
  if (error_ == RecordingError::None) error_ = error;
}

}  // namespace robot
