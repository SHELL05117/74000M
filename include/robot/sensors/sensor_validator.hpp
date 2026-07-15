#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/sensors/validated_inputs.hpp"
#include "robot/state/raw_inputs.hpp"

namespace robot {

struct ScalarValidationConfig {
  double min_value{};
  double max_value{};
  double max_abs_rate_per_s{};
  TimeUs max_age_us{};
  double freeze_epsilon{};
  TimeUs freeze_timeout_us{};
  std::uint16_t recovery_samples{};
};

inline bool validScalarConfig(const ScalarValidationConfig& config) noexcept {
  return std::isfinite(config.min_value) &&
         std::isfinite(config.max_value) &&
         config.min_value < config.max_value &&
         std::isfinite(config.max_abs_rate_per_s) &&
         config.max_abs_rate_per_s > 0.0 && config.max_age_us > 0 &&
         std::isfinite(config.freeze_epsilon) &&
         config.freeze_epsilon >= 0.0 && config.freeze_timeout_us > 0 &&
         config.recovery_samples > 0;
}

class ScalarValidator {
 public:
  ScalarValidator() = default;
  explicit ScalarValidator(ScalarValidationConfig config) noexcept
      : config_(config) {
    reset();
  }

  void configure(ScalarValidationConfig config) noexcept {
    config_ = config;
    reset();
  }

  bool valid() const noexcept { return validScalarConfig(config_); }

  CheckedScalar update(const ScalarSample& sample, TimeUs now_us,
                       bool expected_change) noexcept {
    CheckedScalar output{};
    output.value = sample.value;
    output.sample_time_us = sample.sample_time_us;
    if (!valid() || !sample.api_ok) output.reject_bits |= kSensorApiError;
    if (!std::isfinite(sample.value)) output.reject_bits |= kSensorNonfinite;
    if (sample.sample_time_us > now_us) {
      output.reject_bits |= kSensorFutureTimestamp;
    } else if (now_us - sample.sample_time_us > config_.max_age_us) {
      output.reject_bits |= kSensorStale;
    }
    if (std::isfinite(sample.value) &&
        (sample.value < config_.min_value ||
         sample.value > config_.max_value)) {
      output.reject_bits |= kSensorOutOfRange;
    }
    if (have_accepted_) {
      if (sample.sample_time_us <= accepted_time_us_) {
        output.reject_bits |= kSensorNonmonotonic;
      } else if (std::isfinite(sample.value)) {
        const double dt_s =
            static_cast<double>(sample.sample_time_us - accepted_time_us_) *
            1e-6;
        const double rate = std::abs(sample.value - accepted_value_) / dt_s;
        if (!std::isfinite(rate) || rate > config_.max_abs_rate_per_s)
          output.reject_bits |= kSensorImplausibleRate;

        if (expected_change &&
            std::abs(sample.value - accepted_value_) <=
                config_.freeze_epsilon) {
          if (!freeze_candidate_) {
            freeze_candidate_ = true;
            freeze_start_us_ = accepted_time_us_;
          }
          if (sample.sample_time_us - freeze_start_us_ >=
              config_.freeze_timeout_us) {
            output.reject_bits |= kSensorFrozen;
          }
        } else {
          freeze_candidate_ = false;
          freeze_start_us_ = 0;
        }
      }
    }

    constexpr std::uint32_t kHardRejectMask =
        kSensorApiError | kSensorNonfinite | kSensorFutureTimestamp |
        kSensorStale | kSensorNonmonotonic | kSensorOutOfRange |
        kSensorImplausibleRate | kSensorFrozen;
    if ((output.reject_bits & kHardRejectMask) != 0) {
      output.quality = Quality::Invalid;
      recovery_remaining_ = config_.recovery_samples;
      return output;
    }

    accepted_value_ = sample.value;
    accepted_time_us_ = sample.sample_time_us;
    have_accepted_ = true;
    if (sample.device_status != 0)
      output.reject_bits |= kSensorDeviceStatus;
    if (recovery_remaining_ > 0) {
      --recovery_remaining_;
      output.reject_bits |= kSensorWarmup;
      output.quality = Quality::Degraded;
    } else if ((output.reject_bits & kSensorDeviceStatus) != 0) {
      output.quality = Quality::Degraded;
    } else {
      output.quality = Quality::Good;
    }
    return output;
  }

  void reset() noexcept {
    accepted_value_ = 0.0;
    accepted_time_us_ = 0;
    freeze_start_us_ = 0;
    recovery_remaining_ = config_.recovery_samples;
    have_accepted_ = false;
    freeze_candidate_ = false;
  }

 private:
  ScalarValidationConfig config_{};
  double accepted_value_{};
  TimeUs accepted_time_us_{};
  TimeUs freeze_start_us_{};
  std::uint16_t recovery_remaining_{};
  bool have_accepted_{};
  bool freeze_candidate_{};
};

struct SensorValidatorConfig {
  ScalarValidationConfig motor_position_rad;
  ScalarValidationConfig motor_velocity_radps;
  ScalarValidationConfig motor_current_A;
  ScalarValidationConfig motor_temperature_C;
  ScalarValidationConfig motor_applied_voltage_V;
  ScalarValidationConfig tracking_position_rad;
  ScalarValidationConfig tracking_velocity_radps;
  ScalarValidationConfig imu_rotation_rad;
  ScalarValidationConfig imu_yaw_rate_radps;
  ScalarValidationConfig battery_V;
  TimeUs max_acquisition_age_us{};
};

struct MotionExpectation {
  bool left_drive{};
  bool right_drive{};
  bool tracking{};
  bool turning{};
};

class SensorValidator {
 public:
  explicit SensorValidator(const SensorValidatorConfig& config) noexcept
      : config_(config) {
    configureBanks();
  }

  bool valid() const noexcept {
    return validScalarConfig(config_.motor_position_rad) &&
           validScalarConfig(config_.motor_velocity_radps) &&
           validScalarConfig(config_.motor_current_A) &&
           validScalarConfig(config_.motor_temperature_C) &&
           validScalarConfig(config_.motor_applied_voltage_V) &&
           validScalarConfig(config_.tracking_position_rad) &&
           validScalarConfig(config_.tracking_velocity_radps) &&
           validScalarConfig(config_.imu_rotation_rad) &&
           validScalarConfig(config_.imu_yaw_rate_radps) &&
           validScalarConfig(config_.battery_V) &&
           config_.max_acquisition_age_us > 0;
  }

  ValidatedInputs update(const RawDriveInputs& raw, TimeUs now_us,
                         MotionExpectation expected = {}) noexcept {
    ValidatedInputs output{};
    output.h = raw.h;
    output.acquisition_end_us = raw.acquisition_end_us;

    if (!valid() || raw.h.time_us > now_us ||
        raw.acquisition_end_us < raw.h.time_us ||
        raw.acquisition_end_us > now_us ||
        now_us - raw.acquisition_end_us > config_.max_acquisition_age_us ||
        (have_frame_ && raw.h.mode_epoch == last_epoch_ &&
         raw.h.sequence <= last_sequence_)) {
      output.frame_reject_bits = kSensorFrameRejected;
      output.fault_bits |= faultBit(Fault::SensorInvalid);
      return output;
    }

    if (!have_frame_ || raw.h.mode_epoch != last_epoch_) resetBanks();
    have_frame_ = true;
    last_epoch_ = raw.h.mode_epoch;
    last_sequence_ = raw.h.sequence;

    for (std::size_t i = 0; i < kMotorsPerSide; ++i) {
      output.left[i] = validateMotor(raw.left.motor[i], left_[i], now_us,
                                     expected.left_drive);
      output.right[i] = validateMotor(raw.right.motor[i], right_[i], now_us,
                                      expected.right_drive);
      if (output.left[i].kinematic_quality == Quality::Invalid ||
          output.right[i].kinematic_quality == Quality::Invalid) {
        output.fault_bits |= faultBit(Fault::SensorInvalid);
      }
    }

    for (std::size_t i = 0; i < output.tracking.size(); ++i) {
      output.tracking[i].configured = raw.tracking[i].configured;
      if (!raw.tracking[i].configured) {
        output.tracking[i].position_rad.reject_bits = kSensorNotConfigured;
        output.tracking[i].velocity_radps.reject_bits = kSensorNotConfigured;
        continue;
      }
      output.tracking[i].position_rad = tracking_[i].position.update(
          raw.tracking[i].position_rad, now_us, expected.tracking);
      output.tracking[i].velocity_radps = tracking_[i].velocity.update(
          raw.tracking[i].velocity_radps, now_us, false);
      if (output.tracking[i].position_rad.quality == Quality::Invalid)
        output.fault_bits |= faultBit(Fault::SensorInvalid);
    }

    output.imu.calibrating = raw.imu.calibrating;
    if (raw.imu.calibrating || !raw.imu.status_api_ok) {
      imu_rotation_.reset();
      imu_rate_.reset();
      output.imu.rotation_rad.value = raw.imu.rotation_rad.value;
      output.imu.rotation_rad.sample_time_us =
          raw.imu.rotation_rad.sample_time_us;
      output.imu.yaw_rate_radps.value = raw.imu.yaw_rate_radps.value;
      output.imu.yaw_rate_radps.sample_time_us =
          raw.imu.yaw_rate_radps.sample_time_us;
      const std::uint32_t reason = raw.imu.calibrating
                                       ? kSensorCalibrating
                                       : kSensorApiError;
      output.imu.rotation_rad.reject_bits = reason;
      output.imu.yaw_rate_radps.reject_bits = reason;
      output.fault_bits |= faultBit(Fault::SensorInvalid);
    } else {
      output.imu.rotation_rad = imu_rotation_.update(
          raw.imu.rotation_rad, now_us, expected.turning);
      output.imu.yaw_rate_radps = imu_rate_.update(
          raw.imu.yaw_rate_radps, now_us, false);
      if (output.imu.rotation_rad.quality == Quality::Invalid)
        output.fault_bits |= faultBit(Fault::SensorInvalid);
    }

    output.battery_V = battery_.update(raw.battery_V, now_us, false);
    if (output.battery_V.quality == Quality::Invalid)
      output.fault_bits |= faultBit(Fault::SensorInvalid);
    return output;
  }

  void reset() noexcept {
    resetBanks();
    have_frame_ = false;
    last_epoch_ = 0;
    last_sequence_ = 0;
  }

 private:
  struct MotorBank {
    ScalarValidator position;
    ScalarValidator velocity;
    ScalarValidator current;
    ScalarValidator temperature;
    ScalarValidator applied_voltage;
  };

  struct TrackingBank {
    ScalarValidator position;
    ScalarValidator velocity;
  };

  void configureBanks() noexcept {
    for (auto& side : left_) configureMotor(side);
    for (auto& side : right_) configureMotor(side);
    for (auto& tracking : tracking_) {
      tracking.position.configure(config_.tracking_position_rad);
      tracking.velocity.configure(config_.tracking_velocity_radps);
    }
    imu_rotation_.configure(config_.imu_rotation_rad);
    imu_rate_.configure(config_.imu_yaw_rate_radps);
    battery_.configure(config_.battery_V);
  }

  void configureMotor(MotorBank& bank) noexcept {
    bank.position.configure(config_.motor_position_rad);
    bank.velocity.configure(config_.motor_velocity_radps);
    bank.current.configure(config_.motor_current_A);
    bank.temperature.configure(config_.motor_temperature_C);
    bank.applied_voltage.configure(config_.motor_applied_voltage_V);
  }

  static void resetMotor(MotorBank& bank) noexcept {
    bank.position.reset();
    bank.velocity.reset();
    bank.current.reset();
    bank.temperature.reset();
    bank.applied_voltage.reset();
  }

  void resetBanks() noexcept {
    for (auto& side : left_) resetMotor(side);
    for (auto& side : right_) resetMotor(side);
    for (auto& tracking : tracking_) {
      tracking.position.reset();
      tracking.velocity.reset();
    }
    imu_rotation_.reset();
    imu_rate_.reset();
    battery_.reset();
  }

  static ValidatedMotor validateMotor(const MotorSample& raw,
                                      MotorBank& bank, TimeUs now_us,
                                      bool expected_motion) noexcept {
    ValidatedMotor output{};
    output.smart_port = raw.smart_port;
    output.position_rad =
        bank.position.update(raw.position_rad, now_us, expected_motion);
    output.velocity_radps =
        bank.velocity.update(raw.velocity_radps, now_us, false);
    output.current_A = bank.current.update(raw.current_A, now_us, false);
    output.temperature_C =
        bank.temperature.update(raw.temperature_C, now_us, false);
    output.applied_voltage_V = bank.applied_voltage.update(
        raw.applied_voltage_V, now_us, false);
    output.kinematic_quality = worstQuality(output.position_rad.quality,
                                            output.velocity_radps.quality);
    output.health_quality =
        worstQuality(worstQuality(output.current_A.quality,
                                  output.temperature_C.quality),
                     output.applied_voltage_V.quality);
    output.device_faults = raw.faults;
    if (!raw.faults_api_ok) output.reject_bits |= kSensorApiError;
    output.reject_bits |= output.position_rad.reject_bits |
                          output.velocity_radps.reject_bits |
                          output.current_A.reject_bits |
                          output.temperature_C.reject_bits |
                          output.applied_voltage_V.reject_bits;
    return output;
  }

  SensorValidatorConfig config_{};
  std::array<MotorBank, kMotorsPerSide> left_{};
  std::array<MotorBank, kMotorsPerSide> right_{};
  std::array<TrackingBank, 2> tracking_{};
  ScalarValidator imu_rotation_{};
  ScalarValidator imu_rate_{};
  ScalarValidator battery_{};
  std::uint32_t last_epoch_{};
  std::uint32_t last_sequence_{};
  bool have_frame_{};
};

}  // namespace robot
