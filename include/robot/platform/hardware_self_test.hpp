#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/config/robot_config.hpp"
#include "robot/core/frame.hpp"
#include "robot/state/raw_inputs.hpp"

namespace robot {

constexpr TimeUs kMinimumStartupSelfCheckUs = 3000000;

enum StartupSelfCheckFault : std::uint32_t {
  kStartupSelfCheckOk = 0,
  kStartupBadConfig = 1u << 0,
  kStartupDriveInitializeFailed = 1u << 1,
  kStartupMotorMissing = 1u << 2,
  kStartupMotorFault = 1u << 3,
  kStartupBatteryInvalid = 1u << 4,
  kStartupImuMissing = 1u << 5,
  kStartupImuCalibrationFailed = 1u << 6,
  kStartupImuInvalid = 1u << 7,
  kStartupTrackingSensorMissing = 1u << 8,
};

struct StartupSelfCheckConfig {
  TimeUs minimum_duration_us{kMinimumStartupSelfCheckUs};
  TimeUs sensor_timeout_us{5000000};
};

struct StartupSelfCheckStatus {
  bool started{};
  bool complete{};
  bool healthy{};
  std::uint32_t fault_bits{};
  TimeUs elapsed_us{};
};

// Passive startup validation only. It never writes a motor and it deliberately
// waits for repeated HAL samples so a transient API error does not masquerade
// as a missing device. Known-impossible conditions (for example no configured
// IMU) still observe the mandatory three-second startup dwell.
class StartupSelfCheck {
 public:
  explicit StartupSelfCheck(StartupSelfCheckConfig config = {}) noexcept
      : config_(config) {
    if (config_.minimum_duration_us < kMinimumStartupSelfCheckUs)
      config_.minimum_duration_us = kMinimumStartupSelfCheckUs;
    if (config_.sensor_timeout_us < config_.minimum_duration_us)
      config_.sensor_timeout_us = config_.minimum_duration_us;
  }

  void begin(TimeUs now_us, const HardwareConfig& hardware,
             bool config_valid, bool drive_initialized,
             bool imu_calibration_started) noexcept {
    hardware_ = hardware;
    start_time_us_ = now_us;
    status_ = {};
    status_.started = true;
    motor_seen_.fill(false);
    tracking_seen_.fill(false);
    battery_seen_ = false;
    imu_seen_ = false;
    motor_fault_seen_ = false;

    if (!config_valid) status_.fault_bits |= kStartupBadConfig;
    if (!drive_initialized)
      status_.fault_bits |= kStartupDriveInitializeFailed;
    if (!hardware_.imu.installed) {
      status_.fault_bits |= kStartupImuMissing;
    } else if (!imu_calibration_started) {
      status_.fault_bits |= kStartupImuCalibrationFailed;
    }
  }

  StartupSelfCheckStatus tick(TimeUs now_us,
                              const RawDriveInputs& raw) noexcept {
    if (!status_.started || status_.complete) return status_;
    status_.elapsed_us =
        now_us >= start_time_us_ ? now_us - start_time_us_ : 0;

    observeMotors(raw.left.motor, hardware_.left, 0);
    observeMotors(raw.right.motor, hardware_.right, kMotorsPerSide);
    if (hardware_.lift.installed) {
      observeMotors(raw.lift, hardware_.lift.motors,
                    2 * kMotorsPerSide);
    }
    battery_seen_ = battery_seen_ || validScalar(raw.battery_V);

    if (hardware_.imu.installed &&
        (status_.fault_bits & kStartupImuCalibrationFailed) == 0) {
      const bool imu_valid = raw.imu.status_api_ok && !raw.imu.calibrating &&
                             validScalar(raw.imu.rotation_rad) &&
                             validScalar(raw.imu.yaw_rate_radps);
      imu_seen_ = imu_seen_ || imu_valid;
    }

    observeTracking(raw, hardware_.parallel_rotation, 0);
    observeTracking(raw, hardware_.lateral_rotation, 1);

    const bool reached_minimum =
        status_.elapsed_us >= config_.minimum_duration_us;
    const bool reached_timeout =
        status_.elapsed_us >= config_.sensor_timeout_us;
    const bool known_impossible =
        (status_.fault_bits & (kStartupBadConfig |
                               kStartupDriveInitializeFailed |
                               kStartupImuMissing |
                               kStartupImuCalibrationFailed)) != 0;
    const bool all_observed = allRequiredDevicesObserved();
    if (!reached_minimum || (!all_observed && !known_impossible &&
                             !reached_timeout)) {
      return status_;
    }

    finalizeFaults();
    status_.complete = true;
    status_.healthy = status_.fault_bits == kStartupSelfCheckOk;
    return status_;
  }

  StartupSelfCheckStatus status() const noexcept { return status_; }

 private:
  static bool validScalar(const ScalarSample& sample) noexcept {
    return sample.api_ok && std::isfinite(sample.value);
  }

  static bool validMotor(const MotorSample& sample,
                         const MotorPortConfig& expected) noexcept {
    return sample.smart_port ==
               static_cast<std::uint8_t>(expected.smart_port) &&
           validScalar(sample.position_rad) &&
           validScalar(sample.velocity_radps) &&
           validScalar(sample.current_A) &&
           validScalar(sample.temperature_C) &&
           validScalar(sample.applied_voltage_V) && sample.faults_api_ok;
  }

  template <std::size_t N>
  void observeMotors(const std::array<MotorSample, N>& samples,
                     const std::array<MotorPortConfig, N>& expected,
                     std::size_t offset) noexcept {
    for (std::size_t i = 0; i < N; ++i) {
      motor_seen_[offset + i] =
          motor_seen_[offset + i] || validMotor(samples[i], expected[i]);
      motor_fault_seen_ =
          motor_fault_seen_ ||
          (samples[i].faults_api_ok && samples[i].faults != 0);
    }
  }

  void observeTracking(const RawDriveInputs& raw,
                       const RotationSensorConfig& expected,
                       std::size_t index) noexcept {
    if (!expected.installed || index >= raw.tracking.size()) return;
    const TrackingWheelRaw& sample = raw.tracking[index];
    tracking_seen_[index] =
        tracking_seen_[index] ||
        (sample.configured && validScalar(sample.position_rad) &&
         validScalar(sample.velocity_radps));
  }

  bool allRequiredDevicesObserved() const noexcept {
    for (std::size_t i = 0; i < 2 * kMotorsPerSide; ++i)
      if (!motor_seen_[i]) return false;
    if (hardware_.lift.installed) {
      for (std::size_t i = 2 * kMotorsPerSide; i < motor_seen_.size(); ++i)
        if (!motor_seen_[i]) return false;
    }
    if (!battery_seen_) return false;
    if (hardware_.imu.installed && !imu_seen_) return false;
    if (hardware_.parallel_rotation.installed && !tracking_seen_[0])
      return false;
    if (hardware_.lateral_rotation.installed && !tracking_seen_[1])
      return false;
    return true;
  }

  void finalizeFaults() noexcept {
    const std::size_t required_motors =
        2 * kMotorsPerSide +
        (hardware_.lift.installed ? kLiftMotorCount : 0);
    for (std::size_t i = 0; i < required_motors; ++i) {
      if (!motor_seen_[i]) {
        status_.fault_bits |= kStartupMotorMissing;
        break;
      }
    }
    if (motor_fault_seen_) status_.fault_bits |= kStartupMotorFault;
    if (!battery_seen_) status_.fault_bits |= kStartupBatteryInvalid;
    if (hardware_.imu.installed && !imu_seen_)
      status_.fault_bits |= kStartupImuInvalid;
    if ((hardware_.parallel_rotation.installed && !tracking_seen_[0]) ||
        (hardware_.lateral_rotation.installed && !tracking_seen_[1])) {
      status_.fault_bits |= kStartupTrackingSensorMissing;
    }
  }

  StartupSelfCheckConfig config_{};
  HardwareConfig hardware_{};
  StartupSelfCheckStatus status_{};
  TimeUs start_time_us_{};
  std::array<bool, 2 * kMotorsPerSide + kLiftMotorCount> motor_seen_{};
  std::array<bool, 2> tracking_seen_{};
  bool battery_seen_{};
  bool imu_seen_{};
  bool motor_fault_seen_{};
};

enum class DirectionTestState : std::uint8_t {
  Locked,
  Ready,
  AwaitingObservation,
  Passed,
  Failed,
};

struct DirectionSelfTestLimits {
  double pulse_voltage_V{};
  TimeUs pulse_duration_us{};
  double minimum_positive_delta_rad{};
};

struct DirectionPulseIntent {
  std::uint8_t smart_port{};
  double voltage_V{};
  TimeUs duration_us{};
  std::uint32_t index{};
};

class DirectionSelfTest {
 public:
  bool begin(const HardwareConfig& hardware,
             const DirectionSelfTestLimits& limits, bool disabled,
             bool bench_authorized) noexcept {
    reset();
    if (!disabled || !bench_authorized ||
        !std::isfinite(limits.pulse_voltage_V) ||
        !std::isfinite(limits.minimum_positive_delta_rad) ||
        limits.pulse_voltage_V <= 0.0 || limits.pulse_voltage_V > 12.0 ||
        limits.pulse_duration_us == 0 ||
        limits.minimum_positive_delta_rad <= 0.0) {
      state_ = DirectionTestState::Locked;
      return false;
    }
    limits_ = limits;
    std::size_t index = 0;
    for (const auto& motor : hardware.left) ports_[index++] = motor.smart_port;
    for (const auto& motor : hardware.right) ports_[index++] = motor.smart_port;
    for (const auto port : ports_) {
      if (port < 1 || port > 21) {
        state_ = DirectionTestState::Failed;
        return false;
      }
    }
    state_ = DirectionTestState::Ready;
    return true;
  }

  bool nextIntent(DirectionPulseIntent& intent) noexcept {
    if (state_ != DirectionTestState::Ready || index_ >= ports_.size())
      return false;
    intent = {static_cast<std::uint8_t>(ports_[index_]),
              limits_.pulse_voltage_V, limits_.pulse_duration_us,
              static_cast<std::uint32_t>(index_)};
    state_ = DirectionTestState::AwaitingObservation;
    return true;
  }

  bool observe(std::uint8_t smart_port, double encoder_delta_rad,
               bool api_ok, bool emergency_stop) noexcept {
    if (state_ != DirectionTestState::AwaitingObservation) return false;
    if (emergency_stop || !api_ok || !std::isfinite(encoder_delta_rad) ||
        smart_port != static_cast<std::uint8_t>(ports_[index_]) ||
        encoder_delta_rad < limits_.minimum_positive_delta_rad) {
      failed_port_ = smart_port;
      state_ = DirectionTestState::Failed;
      return false;
    }
    ++index_;
    state_ = index_ == ports_.size() ? DirectionTestState::Passed
                                     : DirectionTestState::Ready;
    return true;
  }

  void reset() noexcept {
    ports_.fill(0);
    limits_ = {};
    index_ = 0;
    failed_port_ = 0;
    state_ = DirectionTestState::Locked;
  }

  DirectionTestState state() const noexcept { return state_; }
  std::uint8_t failedPort() const noexcept { return failed_port_; }

 private:
  std::array<std::int8_t, 2 * kMotorsPerSide> ports_{};
  DirectionSelfTestLimits limits_{};
  std::size_t index_{};
  std::uint8_t failed_port_{};
  DirectionTestState state_{DirectionTestState::Locked};
};

}  // namespace robot
