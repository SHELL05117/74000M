#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "robot/telemetry/recording_file.hpp"

namespace robot {

class RecordingByteWriter {
 public:
  virtual bool writeBytes(const void* data, std::size_t size) = 0;
  virtual ~RecordingByteWriter() = default;
};

class RecordingFileEncoder {
 public:
  bool begin(RecordingByteWriter& writer,
             const RecordingMetadata& metadata,
             std::uint32_t session_sequence,
             std::uint32_t storage_sequence,
             std::uint32_t start_time_ms) noexcept {
    reset();
    writer_ = &writer;
    header_ = makeRecordingFileHeader(
        metadata, session_sequence, storage_sequence, start_time_ms);
    return writer_->writeBytes(&header_, sizeof(header_));
  }

  bool append(const LogFrame* frames, std::size_t count) noexcept {
    if (writer_ == nullptr || frames == nullptr || count == 0 ||
        count > UINT32_MAX / sizeof(LogFrame))
      return false;
    RecordingBlockHeader block{};
    block.block_sequence = ++block_sequence_;
    block.first_frame_sequence = frames[0].header.sequence;
    block.frame_count = static_cast<std::uint32_t>(count);
    block.payload_bytes =
        static_cast<std::uint32_t>(count * sizeof(LogFrame));
    block.payload_crc32 = telemetryCrc32(
        reinterpret_cast<const std::uint8_t*>(frames),
        block.payload_bytes);
    block.header_crc32 = telemetryCrc32(
        reinterpret_cast<const std::uint8_t*>(&block),
        offsetof(RecordingBlockHeader, header_crc32));
    if (!writer_->writeBytes(&block, sizeof(block)) ||
        !writer_->writeBytes(frames, block.payload_bytes))
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

  bool finish(std::uint32_t producer_drops) noexcept {
    if (writer_ == nullptr) return false;
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
    const bool ok = writer_->writeBytes(&footer, sizeof(footer));
    writer_ = nullptr;
    return ok;
  }

  const RecordingFileHeader& header() const noexcept { return header_; }

  void reset() noexcept {
    writer_ = nullptr;
    header_ = {};
    block_sequence_ = 0;
    total_frames_ = 0;
    first_frame_sequence_ = 0;
    last_frame_sequence_ = 0;
    first_time_us_ = 0;
    last_time_us_ = 0;
  }

 private:
  RecordingByteWriter* writer_{};
  RecordingFileHeader header_{};
  std::uint32_t block_sequence_{};
  std::uint32_t total_frames_{};
  std::uint32_t first_frame_sequence_{};
  std::uint32_t last_frame_sequence_{};
  TimeUs first_time_us_{};
  TimeUs last_time_us_{};
};

enum RecordingVerifyFault : std::uint32_t {
  kRecordingVerifyOk = 0,
  kRecordingTruncated = 1u << 0,
  kRecordingBadHeader = 1u << 1,
  kRecordingUnsupportedFormat = 1u << 2,
  kRecordingBadBlockHeader = 1u << 3,
  kRecordingBadPayload = 1u << 4,
  kRecordingSequenceGap = 1u << 5,
  kRecordingTimeRegression = 1u << 6,
  kRecordingRunMismatch = 1u << 7,
  kRecordingBadFooter = 1u << 8,
  kRecordingFooterMismatch = 1u << 9,
  kRecordingTrailingData = 1u << 10,
};

struct RecordingVerifyReport {
  std::uint32_t fault_bits{};
  std::uint32_t recoverable_frames{};
  std::uint32_t valid_blocks{};
  std::uint32_t producer_drops{};
  std::size_t valid_bytes{};
  bool complete{};
  RecordingFileHeader header{};
};

inline RecordingVerifyReport verifyRecordingFile(
    const std::uint8_t* data, std::size_t size) noexcept {
  RecordingVerifyReport report{};
  if (data == nullptr || size < sizeof(RecordingFileHeader)) {
    report.fault_bits = kRecordingTruncated;
    return report;
  }
  std::memcpy(&report.header, data, sizeof(report.header));
  const bool header_crc_ok =
      telemetryCrc32(data, offsetof(RecordingFileHeader, header_crc32)) ==
      report.header.header_crc32;
  if (!header_crc_ok || report.header.magic != kRecordingFileMagic) {
    report.fault_bits |= kRecordingBadHeader;
    return report;
  }
  if (report.header.format_major != 2 ||
      report.header.endian_marker != kRecordingEndianMarker ||
      report.header.header_size_bytes != sizeof(RecordingFileHeader) ||
      report.header.frame_size_bytes != sizeof(LogFrame) ||
      report.header.log_schema_major != kLogSchemaMajor ||
      report.header.log_schema_minor != kLogSchemaMinor) {
    report.fault_bits |= kRecordingUnsupportedFormat;
    return report;
  }

  std::size_t offset = sizeof(RecordingFileHeader);
  report.valid_bytes = offset;
  std::uint32_t expected_block = 1;
  std::uint32_t previous_sequence{};
  std::uint32_t first_sequence{};
  TimeUs previous_time{};
  TimeUs first_time{};
  bool have_frame{};
  while (offset < size) {
    if (size - offset < sizeof(std::uint32_t)) {
      report.fault_bits |= kRecordingTruncated;
      break;
    }
    std::uint32_t magic{};
    std::memcpy(&magic, data + offset, sizeof(magic));
    if (magic == kRecordingBlockMagic) {
      if (size - offset < sizeof(RecordingBlockHeader)) {
        report.fault_bits |= kRecordingTruncated;
        break;
      }
      RecordingBlockHeader block{};
      std::memcpy(&block, data + offset, sizeof(block));
      if (telemetryCrc32(
              data + offset,
              offsetof(RecordingBlockHeader, header_crc32)) !=
              block.header_crc32 ||
          block.block_sequence != expected_block ||
          block.frame_count == 0 ||
          block.payload_bytes !=
              block.frame_count * sizeof(LogFrame)) {
        report.fault_bits |= kRecordingBadBlockHeader;
        break;
      }
      const std::size_t payload_offset = offset + sizeof(block);
      if (size - payload_offset < block.payload_bytes) {
        report.fault_bits |= kRecordingTruncated;
        break;
      }
      if (telemetryCrc32(data + payload_offset, block.payload_bytes) !=
          block.payload_crc32) {
        report.fault_bits |= kRecordingBadPayload;
        break;
      }
      for (std::uint32_t index = 0; index < block.frame_count; ++index) {
        LogFrame frame{};
        std::memcpy(&frame,
                    data + payload_offset + index * sizeof(LogFrame),
                    sizeof(frame));
        if (frame.header.magic != kLogMagic ||
            frame.header.frame_size_bytes != sizeof(LogFrame) ||
            frame.header.schema_major != kLogSchemaMajor ||
            frame.header.schema_minor != kLogSchemaMinor) {
          report.fault_bits |= kRecordingBadPayload;
          return report;
        }
        if (frame.header.run_id_hash != report.header.run_id_hash)
          report.fault_bits |= kRecordingRunMismatch;
        if (have_frame) {
          if (frame.header.sequence != previous_sequence + 1)
            report.fault_bits |= kRecordingSequenceGap;
          if (frame.header.time_us < previous_time)
            report.fault_bits |= kRecordingTimeRegression;
        }
        previous_sequence = frame.header.sequence;
        previous_time = frame.header.time_us;
        if (!have_frame) {
          first_sequence = frame.header.sequence;
          first_time = frame.header.time_us;
        }
        have_frame = true;
      }
      report.recoverable_frames += block.frame_count;
      ++report.valid_blocks;
      ++expected_block;
      offset = payload_offset + block.payload_bytes;
      report.valid_bytes = offset;
      continue;
    }
    if (magic == kRecordingFooterMagic) {
      if (size - offset < sizeof(RecordingFileFooter)) {
        report.fault_bits |= kRecordingTruncated;
        break;
      }
      RecordingFileFooter footer{};
      std::memcpy(&footer, data + offset, sizeof(footer));
      if (telemetryCrc32(
              data + offset,
              offsetof(RecordingFileFooter, footer_crc32)) !=
          footer.footer_crc32) {
        report.fault_bits |= kRecordingBadFooter;
        break;
      }
      if (footer.total_frames != report.recoverable_frames ||
          footer.block_count != report.valid_blocks ||
          (have_frame &&
           (footer.first_frame_sequence != first_sequence ||
            footer.first_time_us != first_time ||
            footer.last_frame_sequence != previous_sequence ||
            footer.last_time_us != previous_time))) {
        report.fault_bits |= kRecordingFooterMismatch;
      }
      report.producer_drops = footer.producer_drops;
      offset += sizeof(footer);
      report.valid_bytes = offset;
      report.complete =
          (report.fault_bits & (kRecordingTruncated |
                                kRecordingBadHeader |
                                kRecordingUnsupportedFormat |
                                kRecordingBadBlockHeader |
                                kRecordingBadPayload |
                                kRecordingBadFooter |
                                kRecordingFooterMismatch)) == 0;
      if (offset != size) {
        report.fault_bits |= kRecordingTrailingData;
        report.complete = false;
      }
      return report;
    }
    report.fault_bits |= kRecordingBadBlockHeader;
    break;
  }
  if (offset == size) report.fault_bits |= kRecordingTruncated;
  return report;
}

inline bool copyVerifiedRecordingFrame(const std::uint8_t* data,
                                       std::size_t size,
                                       std::uint32_t frame_index,
                                       LogFrame& output) noexcept {
  const RecordingVerifyReport report = verifyRecordingFile(data, size);
  if (frame_index >= report.recoverable_frames ||
      (report.fault_bits &
       (kRecordingBadHeader | kRecordingUnsupportedFormat |
        kRecordingBadBlockHeader | kRecordingBadPayload)) != 0)
    return false;
  std::size_t offset = sizeof(RecordingFileHeader);
  std::uint32_t base = 0;
  while (offset + sizeof(RecordingBlockHeader) <= report.valid_bytes) {
    RecordingBlockHeader block{};
    std::memcpy(&block, data + offset, sizeof(block));
    if (block.magic != kRecordingBlockMagic) break;
    if (frame_index < base + block.frame_count) {
      const std::size_t index = frame_index - base;
      std::memcpy(&output, data + offset + sizeof(block) +
                               index * sizeof(LogFrame),
                  sizeof(output));
      return true;
    }
    base += block.frame_count;
    offset += sizeof(block) + block.payload_bytes;
  }
  return false;
}

}  // namespace robot
