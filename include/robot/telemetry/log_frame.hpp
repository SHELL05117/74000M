#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "robot/config/robot_config.hpp"
#include "robot/core/fault.hpp"
#include "robot/core/frame.hpp"
#include "robot/core/quality.hpp"
#include "robot/state/raw_inputs.hpp"

namespace robot {

constexpr std::uint32_t kLogMagic = 0x374D3030u;  // "7M00"
constexpr std::uint16_t kLogSchemaMajor = 3;
constexpr std::uint16_t kLogSchemaMinor = 1;

struct LogHeader {
  std::uint32_t magic{kLogMagic};
  std::uint16_t schema_major{kLogSchemaMajor};
  std::uint16_t schema_minor{kLogSchemaMinor};
  std::uint32_t frame_size_bytes{};
  TimeUs time_us{};
  std::uint32_t sequence{};
  std::uint32_t mode_epoch{};
  std::uint32_t run_id_hash{};
};

struct MotorLogSample {
  std::uint8_t smart_port{};
  std::uint8_t api_ok_mask{};
  Quality quality{Quality::Invalid};
  std::uint8_t reserved{};
  double position_rad{};
  double velocity_radps{};
  double current_A{};
  double temperature_C{};
  double applied_voltage_V{};
  TimeUs position_time_us{};
  TimeUs velocity_time_us{};
  TimeUs current_time_us{};
  TimeUs temperature_time_us{};
  TimeUs applied_voltage_time_us{};
  std::uint32_t position_status{};
  std::uint32_t velocity_status{};
  std::uint32_t current_status{};
  std::uint32_t temperature_status{};
  std::uint32_t applied_voltage_status{};
  std::uint32_t api_faults{};
  std::uint32_t reject_bits{};
};

enum MotorApiOkBits : std::uint8_t {
  kMotorPositionOk = 1u << 0,
  kMotorVelocityOk = 1u << 1,
  kMotorCurrentOk = 1u << 2,
  kMotorTemperatureOk = 1u << 3,
  kMotorAppliedVoltageOk = 1u << 4,
  kMotorFaultsOk = 1u << 5,
};

struct TimingLog {
  double raw_dt_s{};
  double math_dt_s{};
  double exec_s{};
  double jitter_s{};
  TimeUs sensor_age_us{};
  TimeUs request_age_us{};
  TimeUs actuator_age_us{};
  std::uint32_t consecutive_overruns{};
  std::uint32_t overrun_total{};
  std::uint32_t ring_depth{};
  std::uint32_t log_dropped_total{};
};

struct RequestLog {
  std::uint8_t source{};
  std::uint8_t payload_kind{};
  std::uint16_t owner_id{};
  std::uint32_t owner_lease{};
  std::uint32_t reject_bits{};
  TimeUs request_time_us{};
  TimeUs ttl_us{};
  double forward{};
  double steering{};
  double vx_mps{};
  double omega_radps{};
  double requested_left_V{};
  double requested_right_V{};
};

struct ActuatorLog {
  double unallocated_left_V{};
  double unallocated_right_V{};
  double allocated_left_V{};
  double allocated_right_V{};
  double final_left_V{};
  double final_right_V{};
  double derate_target{};
  double derate_applied{};
  std::array<double, kMotorsPerSide * 2>
      final_motor_voltage_V{};
  std::uint32_t final_motor_valid_mask{};
  std::uint32_t applied_limits{};
  std::uint32_t write_reject_bits{};
  std::uint32_t last_written_sequence{};
  bool write_attempted{};
  bool write_ok{};
};

struct FaultLog {
  FaultBits active_fault_bits{};
  FaultBits latched_fault_bits{};
  FaultBits enter_fault_bits{};
  FaultBits exit_fault_bits{};
  std::uint32_t affected_motor_mask{};
  std::uint8_t severity{};
  std::uint8_t safety_state{};
  bool clear_authorized{};
  std::uint8_t reserved{};
};

struct TrackingLogSample {
  double position_rad{};
  double velocity_radps{};
  TimeUs position_time_us{};
  TimeUs velocity_time_us{};
  std::uint32_t position_status{};
  std::uint32_t velocity_status{};
  bool configured{};
  bool position_api_ok{};
  bool velocity_api_ok{};
  std::uint8_t reserved{};
};

struct RawIoLog {
  TimeUs acquisition_end_us{};
  TimeUs imu_rotation_time_us{};
  TimeUs imu_rate_time_us{};
  TimeUs battery_time_us{};
  std::uint32_t imu_rotation_status{};
  std::uint32_t imu_rate_status{};
  std::uint32_t battery_status{};
  bool imu_rotation_api_ok{};
  bool imu_rate_api_ok{};
  bool imu_status_api_ok{};
  bool imu_calibrating{};
  bool battery_api_ok{};
  std::uint8_t reserved[3]{};
};

struct ControllerLog {
  double left_x{};
  double left_y{};
  double right_x{};
  double right_y{};
  std::uint32_t buttons{};
  std::uint8_t mode{};
  bool connected{};
  bool api_ok{};
  bool enabled{};
  bool field_connected{};
  std::uint8_t reserved[3]{};
};

struct RecordingLog {
  std::uint32_t session_sequence{};
  std::uint32_t event_bits{};
  std::uint8_t state{};
  std::uint8_t error{};
  std::uint16_t reserved{};
};

struct SystemEventLog {
  std::uint32_t event_sequence{};
  std::uint32_t event_bits{};
  std::uint32_t mechanism_motor_request_mask{};
  std::uint32_t pneumatic_request_mask{};
  std::uint32_t automatic_program_id{};
  bool drive_consumed{};
  bool mechanisms_valid{};
  bool pneumatics_valid{};
  bool automatic_program_valid{};
};

enum ControlTraceAvailability : std::uint32_t {
  kTraceNoData = 0,
  kTraceRawInputs = 1u << 0,
  kTraceValidatedState = 1u << 1,
  kTracePoseState = 1u << 2,
  kTraceDriverMapping = 1u << 3,
  kTraceRequestCandidate = 1u << 4,
  kTraceArbitration = 1u << 5,
  kTraceActuatorIntent = 1u << 6,
  kTraceOutputStatus = 1u << 7,
  kTraceFaultManager = 1u << 8,
  kTracePidTerms = 1u << 9,
  kTraceMechanisms = 1u << 10,
  kTracePneumatics = 1u << 11,
  kTraceAutonomousProgram = 1u << 12,
  kTraceCompetitionInput = 1u << 13,
};

// Schema 3.1 append-only extension. Availability bits distinguish a real
// zero from a subsystem that is not composed in the current firmware.
struct ControlTraceLog {
  double mapped_throttle{};
  double mapped_turn{};
  double output_derate{};
  TimeUs request_age_us{};
  TimeUs mode_transition_time_us{};
  std::uint32_t availability_bits{};
  std::uint32_t arbitration_reject_bits{};
  std::uint32_t arbitration_rejected_count{};
  std::uint32_t selected_request_sequence{};
  std::uint32_t selected_owner_lease{};
  std::uint32_t mode_fault_bits{};
  std::uint32_t ring_high_watermark{};
  std::uint8_t selected_source{};
  std::uint8_t stop_mode{};
  std::uint8_t output_action{};
  std::uint8_t drive_state{};
  bool quick_turn_active{};
  bool request_candidate_present{};
  bool request_selected{};
  bool deadline_missed{};
  bool competition_disabled{};
  bool competition_autonomous{};
  bool competition_api_ok{};
  std::uint8_t reserved{};
};

struct LogFrame {
  LogHeader header{};
  std::array<MotorLogSample, kMotorsPerSide> left_motor{};
  std::array<MotorLogSample, kMotorsPerSide> right_motor{};
  std::array<TrackingLogSample, kMaxTrackingWheels> tracking{};
  RawIoLog raw_io{};
  double imu_rotation_rad{};
  double imu_rate_radps{};
  double battery_V{};
  double pose_x_m{};
  double pose_y_m{};
  double pose_theta_rad{};
  double body_vx_mps{};
  double body_vy_mps{};
  double body_omega_radps{};
  Quality translation_quality{Quality::Invalid};
  Quality heading_quality{Quality::Invalid};
  std::uint16_t reserved{};
  ControllerLog controller{};
  SystemEventLog system_event{};
  RecordingLog recording{};
  RequestLog request{};
  ActuatorLog actuator{};
  TimingLog timing{};
  FaultLog fault{};
  ControlTraceLog trace{};
};

inline LogFrame makeEmptyLogFrame(const FrameHeader& header,
                                  std::uint32_t run_id_hash) noexcept {
  LogFrame frame{};
  frame.header.frame_size_bytes = sizeof(LogFrame);
  frame.header.time_us = header.time_us;
  frame.header.sequence = header.sequence;
  frame.header.mode_epoch = header.mode_epoch;
  frame.header.run_id_hash = run_id_hash;
  return frame;
}

static_assert(std::is_trivially_copyable_v<LogFrame>);
static_assert(sizeof(ControlTraceLog) == 80,
              "ControlTraceLog schema 3.1 ABI changed");
static_assert(sizeof(LogFrame) == 1536,
              "LogFrame schema 3.1 ARM ABI changed; version the schema and "
              "update the PC decoder before accepting new logs");

}  // namespace robot
