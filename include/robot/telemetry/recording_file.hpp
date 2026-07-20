#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "robot/telemetry/log_frame.hpp"

namespace robot {

constexpr std::uint32_t kRecordingFileMagic = 0x314C3556u;  // "V5L1"
constexpr std::uint32_t kRecordingBlockMagic = 0x314B4C42u; // "BLK1"
constexpr std::uint32_t kRecordingFooterMagic = 0x31444E45u;// "END1"

struct RecordingFileHeader {
  std::uint32_t magic{kRecordingFileMagic};
  std::uint16_t format_major{1};
  std::uint16_t format_minor{0};
  std::uint16_t log_schema_major{kLogSchemaMajor};
  std::uint16_t log_schema_minor{kLogSchemaMinor};
  std::uint32_t frame_size_bytes{sizeof(LogFrame)};
  std::uint32_t session_sequence{};
  std::uint32_t start_time_ms{};
  std::uint32_t robot_id_hash{};
  std::uint32_t header_crc32{};
};

struct RecordingBlockHeader {
  std::uint32_t magic{kRecordingBlockMagic};
  std::uint32_t block_sequence{};
  std::uint32_t first_frame_sequence{};
  std::uint32_t frame_count{};
  std::uint32_t payload_bytes{};
  std::uint32_t payload_crc32{};
  std::uint32_t header_crc32{};
};

struct RecordingFileFooter {
  std::uint32_t magic{kRecordingFooterMagic};
  std::uint32_t total_frames{};
  std::uint32_t first_frame_sequence{};
  std::uint32_t last_frame_sequence{};
  TimeUs first_time_us{};
  TimeUs last_time_us{};
  std::uint32_t producer_drops{};
  std::uint32_t block_count{};
  std::uint32_t footer_crc32{};
};

inline std::uint32_t telemetryCrc32(const std::uint8_t* data,
                                    std::size_t size) noexcept {
  std::uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1u) ^
            (0xEDB88320u & (0u - static_cast<std::uint32_t>(crc & 1u)));
  }
  return ~crc;
}

static_assert(std::is_trivially_copyable_v<RecordingFileHeader>);
static_assert(std::is_trivially_copyable_v<RecordingBlockHeader>);
static_assert(std::is_trivially_copyable_v<RecordingFileFooter>);

}  // namespace robot
