#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/config/robot_config.hpp"
#include "robot/platform/io.hpp"

namespace robot {

class FakeClock final : public Clock {
 public:
  TimeUs nowUs() const override { return now_us_; }
  std::uint32_t nowMs() const override {
    return static_cast<std::uint32_t>(now_us_ / 1000);
  }
  void delayUntilMs(std::uint32_t& previous_ms,
                    std::uint32_t period_ms) override {
    previous_ms += period_ms;
    now_us_ = static_cast<TimeUs>(previous_ms) * 1000;
  }
  void setNowUs(TimeUs now_us) noexcept { now_us_ = now_us; }
  void advanceUs(TimeUs delta_us) noexcept { now_us_ += delta_us; }

 private:
  TimeUs now_us_{};
};

struct FakeDriveModel {
  double motor_radps_per_volt{};
  double battery_V{};
};

class FakeDriveIO final : public DriveIO {
 public:
  FakeDriveIO(const HardwareConfig& hardware, FakeDriveModel model)
      : hardware_(hardware), model_(model) {
    for (std::size_t i = 0; i < kMotorsPerSide; ++i) {
      left_position_[i] = 0.0;
      right_position_[i] = 0.0;
    }
    lift_position_.fill(0.0);
  }

  bool initialize() override {
    initialized_ = std::isfinite(model_.motor_radps_per_volt) &&
                   std::isfinite(model_.battery_V) &&
                   model_.motor_radps_per_volt > 0.0 && model_.battery_V > 0.0;
    return initialized_;
  }

  bool beginImuCalibration() override {
    if (!hardware_.imu.installed) return false;
    imu_calibrating_ = true;
    return initialized_;
  }

  RawDriveInputs readAll(const FrameHeader& header) override {
    ++read_count_;
    RawDriveInputs raw{};
    raw.h = header;
    for (std::size_t i = 0; i < kMotorsPerSide; ++i) {
      fillMotor(raw.left.motor[i], hardware_.left[i], left_position_[i],
                left_command_V_, left_fault_mask_ & (1u << i), header.time_us);
      fillMotor(raw.right.motor[i], hardware_.right[i], right_position_[i],
                right_command_V_, right_fault_mask_ & (1u << i),
                header.time_us);
    }
    if (hardware_.lift.installed) {
      for (std::size_t i = 0; i < kLiftMotorCount; ++i) {
        fillMotor(raw.lift[i], hardware_.lift.motors[i], lift_position_[i],
                  lift_command_V_, lift_fault_mask_ & (1u << i),
                  header.time_us);
      }
    }
    const bool imu_ok = hardware_.imu.installed && !imu_failed_ &&
                        !imu_calibrating_;
    raw.imu.rotation_rad =
        {imu_rotation_rad_, header.time_us, imu_ok, 0};
    raw.imu.yaw_rate_radps =
        {imu_rate_radps_, header.time_us, imu_ok, 0};
    raw.imu.calibrating = hardware_.imu.installed && imu_calibrating_;
    raw.imu.status_api_ok = hardware_.imu.installed && !imu_failed_;
    raw.battery_V = {model_.battery_V, header.time_us, true, 0};
    raw.acquisition_end_us = header.time_us;
    return raw;
  }

  bool writeVoltage(double left_V, double right_V) override {
    ++write_count_;
    if (!initialized_ || !std::isfinite(left_V) || !std::isfinite(right_V))
      return false;
    left_command_V_ = left_V;
    right_command_V_ = right_V;
    stopped_ = false;
    return true;
  }

  bool stop(StopMode mode) override {
    ++stop_count_;
    left_command_V_ = 0.0;
    right_command_V_ = 0.0;
    last_stop_mode_ = mode;
    stopped_ = true;
    return initialized_;
  }

  bool zeroLiftAtLowerLimit() override {
    if (!initialized_) return false;
    lift_position_.fill(0.0);
    return !hardware_.lift.installed || initialized_;
  }

  bool writeLiftVoltage(double voltage_V) override {
    ++lift_write_count_;
    if (!initialized_ || !hardware_.lift.installed ||
        !std::isfinite(voltage_V)) {
      return false;
    }
    lift_command_V_ = voltage_V;
    lift_stopped_ = false;
    return true;
  }

  bool stopLift(StopMode mode) override {
    ++lift_stop_count_;
    lift_command_V_ = 0.0;
    last_lift_stop_mode_ = mode;
    lift_stopped_ = true;
    return initialized_;
  }

  void advance(double dt_s) noexcept {
    if (!initialized_ || !std::isfinite(dt_s) || dt_s <= 0.0) return;
    const double left_velocity = left_command_V_ * model_.motor_radps_per_volt;
    const double right_velocity =
        right_command_V_ * model_.motor_radps_per_volt;
    for (std::size_t i = 0; i < kMotorsPerSide; ++i) {
      if ((left_fault_mask_ & (1u << i)) == 0)
        left_position_[i] += left_velocity * dt_s;
      if ((right_fault_mask_ & (1u << i)) == 0)
        right_position_[i] += right_velocity * dt_s;
    }
    const double lift_velocity =
        lift_command_V_ * model_.motor_radps_per_volt;
    for (std::size_t i = 0; i < kLiftMotorCount; ++i) {
      if ((lift_fault_mask_ & (1u << i)) == 0)
        lift_position_[i] += lift_velocity * dt_s;
    }
  }

  void finishImuCalibration() noexcept { imu_calibrating_ = false; }
  void injectImuFailure(bool failed) noexcept { imu_failed_ = failed; }
  void injectLeftMotorFailure(std::size_t index, bool failed) noexcept {
    if (index >= kMotorsPerSide) return;
    if (failed)
      left_fault_mask_ |= (1u << index);
    else
      left_fault_mask_ &= ~(1u << index);
  }

  std::uint32_t readCount() const noexcept { return read_count_; }
  std::uint32_t writeCount() const noexcept { return write_count_; }
  std::uint32_t stopCount() const noexcept { return stop_count_; }
  std::uint32_t liftWriteCount() const noexcept { return lift_write_count_; }
  std::uint32_t liftStopCount() const noexcept { return lift_stop_count_; }
  double leftCommandV() const noexcept { return left_command_V_; }
  double rightCommandV() const noexcept { return right_command_V_; }
  StopMode lastStopMode() const noexcept { return last_stop_mode_; }
  double liftCommandV() const noexcept { return lift_command_V_; }
  StopMode lastLiftStopMode() const noexcept { return last_lift_stop_mode_; }
  void setLiftPositionRad(std::size_t index, double position_rad) noexcept {
    if (index < kLiftMotorCount) lift_position_[index] = position_rad;
  }

 private:
  void fillMotor(MotorSample& sample, const MotorPortConfig& config,
                 double position_rad, double command_V, bool failed,
                 TimeUs now_us) const noexcept {
    sample.smart_port = static_cast<std::uint8_t>(config.smart_port);
    const bool valid = initialized_ && !failed;
    sample.position_rad = {position_rad, now_us, valid, 0};
    sample.velocity_radps =
        {command_V * model_.motor_radps_per_volt, now_us, valid, 0};
    sample.current_A = {std::abs(command_V) * 0.1, now_us, valid, 0};
    sample.temperature_C = {25.0, now_us, valid, 0};
    sample.applied_voltage_V = {command_V, now_us, valid, 0};
    sample.faults = failed ? 1u : 0u;
    sample.faults_api_ok = valid;
  }

  HardwareConfig hardware_{};
  FakeDriveModel model_{};
  std::array<double, kMotorsPerSide> left_position_{};
  std::array<double, kMotorsPerSide> right_position_{};
  std::array<double, kLiftMotorCount> lift_position_{};
  double left_command_V_{};
  double right_command_V_{};
  double lift_command_V_{};
  double imu_rotation_rad_{};
  double imu_rate_radps_{};
  std::uint32_t left_fault_mask_{};
  std::uint32_t right_fault_mask_{};
  std::uint32_t lift_fault_mask_{};
  std::uint32_t read_count_{};
  std::uint32_t write_count_{};
  std::uint32_t stop_count_{};
  std::uint32_t lift_write_count_{};
  std::uint32_t lift_stop_count_{};
  StopMode last_stop_mode_{StopMode::Coast};
  StopMode last_lift_stop_mode_{StopMode::Hold};
  bool initialized_{};
  bool stopped_{true};
  bool lift_stopped_{true};
  bool imu_calibrating_{};
  bool imu_failed_{};
};

class FakeControllerIO final : public ControllerIO {
 public:
  ControllerSnapshot readOnce(const FrameHeader& header) override {
    ++read_count_;
    snapshot_.h = header;
    return snapshot_;
  }
  void set(const ControllerSnapshot& snapshot) { snapshot_ = snapshot; }
  std::uint32_t readCount() const noexcept { return read_count_; }

 private:
  ControllerSnapshot snapshot_{};
  std::uint32_t read_count_{};
};

class FakeCompetitionIO final : public CompetitionIO {
 public:
  CompetitionSnapshot readOnce(const FrameHeader& header) override {
    ++read_count_;
    snapshot_.h = header;
    return snapshot_;
  }
  void set(const CompetitionSnapshot& snapshot) { snapshot_ = snapshot; }
  std::uint32_t readCount() const noexcept { return read_count_; }

 private:
  CompetitionSnapshot snapshot_{};
  std::uint32_t read_count_{};
};

}  // namespace robot
