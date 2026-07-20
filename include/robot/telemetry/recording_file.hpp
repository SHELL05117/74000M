#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "robot/telemetry/log_frame.hpp"

namespace robot {

constexpr std::uint32_t kRecordingFileMagic = 0x324C3556u;   // "V5L2"
constexpr std::uint32_t kRecordingBlockMagic = 0x314B4C42u;  // "BLK1"
constexpr std::uint32_t kRecordingFooterMagic = 0x31444E45u; // "END1"
constexpr std::uint32_t kRecordingEndianMarker = 0x01020304u;

struct RecordingMetadata {
  std::uint64_t boot_id{};
  std::uint32_t robot_id_hash{};
  std::uint32_t config_hash{};
  std::uint32_t software_identity_hash{};
  std::uint32_t hardware_revision{};
  std::uint32_t config_schema{};
  std::uint32_t calibration_revision{};
  std::uint8_t hardware_verification{};
  bool dirty_build{true};
  char robot_id[16]{};
  char software_version[16]{};
  char source_commit[41]{};
};

struct RecordingFileHeader {
  std::uint32_t magic{kRecordingFileMagic};
  std::uint16_t format_major{2};
  std::uint16_t format_minor{0};
  std::uint16_t log_schema_major{kLogSchemaMajor};
  std::uint16_t log_schema_minor{kLogSchemaMinor};
  std::uint32_t endian_marker{kRecordingEndianMarker};
  std::uint32_t header_size_bytes{sizeof(RecordingFileHeader)};
  std::uint32_t frame_size_bytes{sizeof(LogFrame)};
  std::uint64_t boot_id{};
  std::uint32_t session_sequence{};
  std::uint32_t storage_sequence{};
  std::uint32_t start_time_ms{};
  std::uint32_t run_id_hash{};
  std::uint32_t robot_id_hash{};
  std::uint32_t config_hash{};
  std::uint32_t software_identity_hash{};
  std::uint32_t hardware_revision{};
  std::uint32_t config_schema{};
  std::uint32_t calibration_revision{};
  std::uint8_t hardware_verification{};
  bool dirty_build{true};
  std::uint16_t reserved{};
  char robot_id[16]{};
  char software_version[16]{};
  char source_commit[41]{};
  std::uint8_t reserved_tail[3]{};
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

inline std::uint32_t telemetryCrc32Update(
    std::uint32_t state, const std::uint8_t* data,
    std::size_t size) noexcept {
  std::uint32_t crc = state;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1u) ^
            (0xEDB88320u & (0u - static_cast<std::uint32_t>(crc & 1u)));
  }
  return crc;
}

inline std::uint32_t telemetryCrc32(const std::uint8_t* data,
                                    std::size_t size) noexcept {
  return ~telemetryCrc32Update(0xFFFFFFFFu, data, size);
}

inline std::uint32_t telemetryFnv1a(const void* data, std::size_t size,
                                    std::uint32_t seed =
                                        2166136261u) noexcept {
  const auto* bytes = static_cast<const std::uint8_t*>(data);
  std::uint32_t hash = seed;
  for (std::size_t index = 0; index < size; ++index) {
    hash ^= bytes[index];
    hash *= 16777619u;
  }
  return hash;
}

inline std::uint32_t recordingRunId(std::uint64_t boot_id,
                                    std::uint32_t session_sequence) noexcept {
  std::uint32_t hash = telemetryFnv1a(&boot_id, sizeof(boot_id));
  return telemetryFnv1a(&session_sequence, sizeof(session_sequence), hash);
}

template <std::size_t N>
inline void copyRecordingText(const char* source,
                              char (&destination)[N]) noexcept {
  if (source == nullptr || N == 0) return;
  std::size_t index = 0;
  while (index + 1 < N && source[index] != '\0') {
    destination[index] = source[index];
    ++index;
  }
  destination[index] = '\0';
}

inline std::uint32_t hashRobotConfig(const RobotConfig& config) noexcept {
  std::uint32_t hash = 2166136261u;
  auto mix = [&](const void* data, std::size_t size) {
    hash = telemetryFnv1a(data, size, hash);
  };
  auto mixText = [&](const char* text) {
    if (text != nullptr) mix(text, std::strlen(text) + 1);
  };
  mixText(config.identity.robot_id);
  mix(&config.identity.hardware_revision,
      sizeof(config.identity.hardware_revision));
  mix(&config.identity.config_schema, sizeof(config.identity.config_schema));
  mix(&config.identity.calibration_revision,
      sizeof(config.identity.calibration_revision));
  auto mixMotor = [&](const MotorPortConfig& motor) {
    mix(&motor.smart_port, sizeof(motor.smart_port));
    mix(&motor.reversed, sizeof(motor.reversed));
    mix(&motor.cartridge_rpm, sizeof(motor.cartridge_rpm));
    mix(&motor.motor_rev_per_wheel_rev,
        sizeof(motor.motor_rev_per_wheel_rev));
  };
  for (const auto& motor : config.hardware.left) mixMotor(motor);
  for (const auto& motor : config.hardware.right) mixMotor(motor);
  mix(&config.hardware.imu.installed,
      sizeof(config.hardware.imu.installed));
  mix(&config.hardware.imu.smart_port,
      sizeof(config.hardware.imu.smart_port));
  auto mixRotation = [&](const RotationSensorConfig& sensor) {
    mix(&sensor.installed, sizeof(sensor.installed));
    mix(&sensor.smart_port, sizeof(sensor.smart_port));
    mix(&sensor.reversed, sizeof(sensor.reversed));
    mix(&sensor.offset_m, sizeof(sensor.offset_m));
  };
  mixRotation(config.hardware.parallel_rotation);
  mixRotation(config.hardware.lateral_rotation);
  mix(&config.geometry.nominal_drive_wheel_diameter_m,
      sizeof(config.geometry.nominal_drive_wheel_diameter_m));
  mix(&config.geometry.nominal_track_width_m,
      sizeof(config.geometry.nominal_track_width_m));
  mix(&config.calibration.left_m_per_motor_rad,
      sizeof(config.calibration.left_m_per_motor_rad));
  mix(&config.calibration.right_m_per_motor_rad,
      sizeof(config.calibration.right_m_per_motor_rad));
  mix(&config.calibration.effective_track_width_m,
      sizeof(config.calibration.effective_track_width_m));
  mix(&config.runtime.nominal_period_s,
      sizeof(config.runtime.nominal_period_s));
  mix(&config.runtime.min_math_dt_s,
      sizeof(config.runtime.min_math_dt_s));
  mix(&config.runtime.max_math_dt_s,
      sizeof(config.runtime.max_math_dt_s));
  mix(&config.runtime.request_ttl_us,
      sizeof(config.runtime.request_ttl_us));
  mix(&config.runtime.output_ttl_us,
      sizeof(config.runtime.output_ttl_us));
  mix(&config.electrical.max_command_voltage_V,
      sizeof(config.electrical.max_command_voltage_V));
  const auto verification =
      static_cast<std::uint8_t>(config.hardware_verification);
  mix(&verification, sizeof(verification));
  mix(&config.capabilities.hardware_output,
      sizeof(config.capabilities.hardware_output));
  mix(&config.capabilities.driver_control,
      sizeof(config.capabilities.driver_control));
  mix(&config.capabilities.pose_good,
      sizeof(config.capabilities.pose_good));
  mix(&config.capabilities.autonomous_chassis_velocity,
      sizeof(config.capabilities.autonomous_chassis_velocity));
  mix(&config.capabilities.autonomous_motion,
      sizeof(config.capabilities.autonomous_motion));
  mix(&config.capabilities.competition_routes,
      sizeof(config.capabilities.competition_routes));
  mix(&config.selected_route, sizeof(config.selected_route));
  return hash;
}

inline RecordingMetadata makeRecordingMetadata(
    const RobotConfig& config, std::uint64_t boot_id,
    const char* source_commit = "UNKNOWN",
    bool dirty_build = true) noexcept {
  RecordingMetadata metadata{};
  metadata.boot_id = boot_id;
  metadata.robot_id_hash =
      telemetryFnv1a(config.identity.robot_id,
                     config.identity.robot_id == nullptr
                         ? 0
                         : std::strlen(config.identity.robot_id));
  metadata.config_hash = hashRobotConfig(config);
  metadata.hardware_revision = config.identity.hardware_revision;
  metadata.config_schema = config.identity.config_schema;
  metadata.calibration_revision =
      config.identity.calibration_revision;
  metadata.hardware_verification =
      static_cast<std::uint8_t>(config.hardware_verification);
  metadata.dirty_build = dirty_build;
  copyRecordingText(config.identity.robot_id, metadata.robot_id);
  copyRecordingText(config.identity.software_version,
                    metadata.software_version);
  copyRecordingText(source_commit, metadata.source_commit);
  metadata.software_identity_hash = telemetryFnv1a(
      metadata.software_version, sizeof(metadata.software_version));
  metadata.software_identity_hash = telemetryFnv1a(
      metadata.source_commit, sizeof(metadata.source_commit),
      metadata.software_identity_hash);
  metadata.software_identity_hash = telemetryFnv1a(
      &metadata.dirty_build, sizeof(metadata.dirty_build),
      metadata.software_identity_hash);
  return metadata;
}

inline RecordingFileHeader makeRecordingFileHeader(
    const RecordingMetadata& metadata, std::uint32_t session_sequence,
    std::uint32_t storage_sequence,
    std::uint32_t start_time_ms) noexcept {
  RecordingFileHeader header{};
  header.boot_id = metadata.boot_id;
  header.session_sequence = session_sequence;
  header.storage_sequence = storage_sequence;
  header.start_time_ms = start_time_ms;
  header.run_id_hash =
      recordingRunId(metadata.boot_id, session_sequence);
  header.robot_id_hash = metadata.robot_id_hash;
  header.config_hash = metadata.config_hash;
  header.software_identity_hash = metadata.software_identity_hash;
  header.hardware_revision = metadata.hardware_revision;
  header.config_schema = metadata.config_schema;
  header.calibration_revision = metadata.calibration_revision;
  header.hardware_verification = metadata.hardware_verification;
  header.dirty_build = metadata.dirty_build;
  std::memcpy(header.robot_id, metadata.robot_id,
              sizeof(header.robot_id));
  std::memcpy(header.software_version, metadata.software_version,
              sizeof(header.software_version));
  std::memcpy(header.source_commit, metadata.source_commit,
              sizeof(header.source_commit));
  header.header_crc32 = telemetryCrc32(
      reinterpret_cast<const std::uint8_t*>(&header),
      offsetof(RecordingFileHeader, header_crc32));
  return header;
}

static_assert(std::is_trivially_copyable_v<RecordingMetadata>);
static_assert(std::is_trivially_copyable_v<RecordingFileHeader>);
static_assert(std::is_trivially_copyable_v<RecordingBlockHeader>);
static_assert(std::is_trivially_copyable_v<RecordingFileFooter>);
static_assert(sizeof(RecordingMetadata) == 112,
              "Recording metadata ABI changed");
static_assert(sizeof(RecordingFileHeader) == 160,
              "V5L2 file header ABI changed");
static_assert(sizeof(RecordingBlockHeader) == 28,
              "V5L2 block header ABI changed");
static_assert(sizeof(RecordingFileFooter) == 48,
              "V5L2 footer ABI changed");

}  // namespace robot
