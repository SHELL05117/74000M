#pragma once

#include <cstddef>
#include <cstdint>

#include "robot/drive/actuator_frame.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/runtime/timing_monitor.hpp"
#include "robot/state/raw_inputs.hpp"
#include "robot/telemetry/recording.hpp"

namespace robot {

class FlightRecorderPort {
 public:
  virtual void capture(const ModeSnapshot& mode,
                       const ControllerSnapshot& controller,
                       LogFrame& frame) noexcept = 0;
  virtual ~FlightRecorderPort() = default;
};

inline void copyMotorLog(const MotorSample& source,
                         MotorLogSample& destination) noexcept {
  destination.smart_port = source.smart_port;
  destination.valid =
      source.position_rad.api_ok && source.velocity_radps.api_ok &&
      source.current_A.api_ok && source.temperature_C.api_ok &&
      source.applied_voltage_V.api_ok && source.faults_api_ok;
  destination.quality =
      destination.valid ? Quality::Good : Quality::Invalid;
  destination.position_rad = source.position_rad.value;
  destination.velocity_radps = source.velocity_radps.value;
  destination.current_A = source.current_A.value;
  destination.temperature_C = source.temperature_C.value;
  destination.applied_voltage_V = source.applied_voltage_V.value;
  destination.api_faults = source.faults;
}

inline LogFrame makeControlLogFrame(
    const FrameHeader& header, std::uint32_t run_id_hash,
    const ModeSnapshot& mode, const RawDriveInputs& raw,
    const ControllerSnapshot& controller, const ActuatorFrame& actuator,
    const TimingSample& timing, std::uint32_t ring_depth,
    std::uint32_t dropped) noexcept {
  LogFrame frame = makeEmptyLogFrame(header, run_id_hash);
  for (std::size_t index = 0; index < kMotorsPerSide; ++index) {
    copyMotorLog(raw.left.motor[index], frame.left_motor[index]);
    copyMotorLog(raw.right.motor[index], frame.right_motor[index]);
  }
  frame.imu_rotation_rad = raw.imu.rotation_rad.value;
  frame.imu_rate_radps = raw.imu.yaw_rate_radps.value;
  frame.battery_V = raw.battery_V.value;
  for (std::size_t index = 0; index < kMaxTrackingWheels; ++index) {
    const TrackingWheelRaw& source = raw.tracking[index];
    TrackingLogSample& destination = frame.tracking[index];
    destination.position_rad = source.position_rad.value;
    destination.velocity_radps = source.velocity_radps.value;
    destination.position_time_us = source.position_rad.sample_time_us;
    destination.velocity_time_us = source.velocity_radps.sample_time_us;
    destination.position_status = source.position_rad.device_status;
    destination.velocity_status = source.velocity_radps.device_status;
    destination.configured = source.configured;
    destination.position_api_ok = source.position_rad.api_ok;
    destination.velocity_api_ok = source.velocity_radps.api_ok;
  }
  frame.raw_io.acquisition_end_us = raw.acquisition_end_us;
  frame.raw_io.imu_rotation_time_us =
      raw.imu.rotation_rad.sample_time_us;
  frame.raw_io.imu_rate_time_us =
      raw.imu.yaw_rate_radps.sample_time_us;
  frame.raw_io.battery_time_us = raw.battery_V.sample_time_us;
  frame.raw_io.imu_rotation_status =
      raw.imu.rotation_rad.device_status;
  frame.raw_io.imu_rate_status =
      raw.imu.yaw_rate_radps.device_status;
  frame.raw_io.battery_status = raw.battery_V.device_status;
  frame.raw_io.imu_rotation_api_ok = raw.imu.rotation_rad.api_ok;
  frame.raw_io.imu_rate_api_ok = raw.imu.yaw_rate_radps.api_ok;
  frame.raw_io.imu_status_api_ok = raw.imu.status_api_ok;
  frame.raw_io.imu_calibrating = raw.imu.calibrating;
  frame.raw_io.battery_api_ok = raw.battery_V.api_ok;
  frame.controller.left_x = controller.left_x;
  frame.controller.left_y = controller.left_y;
  frame.controller.right_x = controller.right_x;
  frame.controller.right_y = controller.right_y;
  frame.controller.buttons = controller.buttons;
  frame.controller.connected = controller.connected;
  frame.controller.api_ok = controller.api_ok;
  frame.controller.mode = static_cast<std::uint8_t>(mode.mode);
  frame.controller.enabled = mode.enabled;
  frame.controller.field_connected = mode.field_connected;
  frame.actuator.final_left_V = actuator.left_V;
  frame.actuator.final_right_V = actuator.right_V;
  frame.actuator.applied_limits = actuator.applied_limits;
  frame.timing.raw_dt_s = timing.raw_dt_s;
  frame.timing.math_dt_s = timing.math_dt_s;
  frame.timing.exec_s = timing.exec_s;
  frame.timing.jitter_s = timing.jitter_s;
  frame.timing.consecutive_overruns = timing.consecutive_misses;
  frame.timing.ring_depth = ring_depth;
  frame.timing.log_dropped_total = dropped;
  return frame;
}

template <std::size_t RingCapacity>
class FlightRecorderProducer final : public FlightRecorderPort {
 public:
  FlightRecorderProducer(RecordingControl& control,
                         SpscRing<LogFrame, RingCapacity>& ring)
      : control_(control), ring_(ring) {}

  void capture(const ModeSnapshot& mode,
               const ControllerSnapshot& controller,
               LogFrame& frame) noexcept override {
    RecordingObservation observed = control_.observe(controller, mode);
    if (last_state_ == RecordingState::Opening &&
        observed.state == RecordingState::Recording)
      observed.event_bits |= kRecordingStarted;
    frame.recording.state = static_cast<std::uint8_t>(observed.state);
    frame.recording.error = static_cast<std::uint8_t>(observed.error);
    frame.recording.session_sequence = observed.session_sequence;
    frame.recording.event_bits = observed.event_bits;
    frame.timing.ring_depth = static_cast<std::uint32_t>(ring_.depth());
    frame.timing.log_dropped_total = ring_.dropped();

    const bool capture =
        observed.state == RecordingState::Opening ||
        observed.state == RecordingState::Recording ||
        (observed.event_bits & kRecordingStopRequested) != 0;
    if (capture) ring_.tryPush(frame);
    last_state_ = observed.state;
  }

 private:
  RecordingControl& control_;
  SpscRing<LogFrame, RingCapacity>& ring_;
  RecordingState last_state_{RecordingState::Idle};
};

}  // namespace robot
