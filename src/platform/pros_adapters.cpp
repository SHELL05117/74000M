#include "robot/platform/pros_adapters.hpp"

#include <algorithm>
#include <cmath>

#include "pros/error.h"
#include "pros/imu.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "robot/core/units.hpp"

namespace robot {
namespace {

ScalarSample makeSample(double value, TimeUs time_us, bool api_ok,
                        std::uint32_t status = 0) {
  return {api_ok ? value : 0.0, time_us, api_ok, status};
}

double normalizeAxis(std::int32_t raw) {
  if (raw == PROS_ERR) return 0.0;
  return std::clamp(static_cast<double>(raw) / 127.0, -1.0, 1.0);
}

pros::motor_brake_mode_e_t toProsBrake(StopMode mode) {
  switch (mode) {
    case StopMode::Coast:
      return pros::E_MOTOR_BRAKE_COAST;
    case StopMode::Brake:
      return pros::E_MOTOR_BRAKE_BRAKE;
    case StopMode::Hold:
      return pros::E_MOTOR_BRAKE_HOLD;
  }
  return pros::E_MOTOR_BRAKE_BRAKE;
}

pros::motor_gearset_e_t toProsGearset(std::uint16_t cartridge_rpm) {
  switch (cartridge_rpm) {
    case 100:
      return pros::E_MOTOR_GEARSET_36;
    case 200:
      return pros::E_MOTOR_GEARSET_18;
    case 600:
      return pros::E_MOTOR_GEARSET_06;
    default:
      return pros::E_MOTOR_GEARSET_INVALID;
  }
}

}  // namespace

TimeUs ProsClock::nowUs() const { return pros::c::micros(); }

std::uint32_t ProsClock::nowMs() const { return pros::c::millis(); }

void ProsClock::delayUntilMs(std::uint32_t& previous_ms,
                             std::uint32_t period_ms) {
  pros::c::task_delay_until(&previous_ms, period_ms);
}

CompetitionSnapshot ProsCompetitionIO::readOnce(
    const FrameHeader& header) {
  CompetitionSnapshot snapshot{};
  snapshot.h = header;
  snapshot.disabled = pros::c::competition_is_disabled() != 0;
  snapshot.autonomous = pros::c::competition_is_autonomous() != 0;
  snapshot.field_connected = pros::c::competition_is_connected() != 0;
  snapshot.api_ok = true;
  return snapshot;
}

ControllerSnapshot ProsControllerIO::readOnce(const FrameHeader& header) {
  ControllerSnapshot snapshot{};
  snapshot.h = header;
  const std::int32_t connected =
      pros::c::controller_is_connected(pros::E_CONTROLLER_MASTER);
  snapshot.api_ok = connected != PROS_ERR;
  snapshot.connected = snapshot.api_ok && connected != 0;
  if (!snapshot.connected) return snapshot;

  snapshot.left_x = normalizeAxis(pros::c::controller_get_analog(
      pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_X));
  snapshot.left_y = normalizeAxis(pros::c::controller_get_analog(
      pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_LEFT_Y));
  snapshot.right_x = normalizeAxis(pros::c::controller_get_analog(
      pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_X));
  snapshot.right_y = normalizeAxis(pros::c::controller_get_analog(
      pros::E_CONTROLLER_MASTER, pros::E_CONTROLLER_ANALOG_RIGHT_Y));

  const pros::controller_digital_e_t buttons[] = {
      pros::E_CONTROLLER_DIGITAL_L1,    pros::E_CONTROLLER_DIGITAL_L2,
      pros::E_CONTROLLER_DIGITAL_R1,    pros::E_CONTROLLER_DIGITAL_R2,
      pros::E_CONTROLLER_DIGITAL_UP,    pros::E_CONTROLLER_DIGITAL_DOWN,
      pros::E_CONTROLLER_DIGITAL_LEFT,  pros::E_CONTROLLER_DIGITAL_RIGHT,
      pros::E_CONTROLLER_DIGITAL_X,     pros::E_CONTROLLER_DIGITAL_B,
      pros::E_CONTROLLER_DIGITAL_Y,     pros::E_CONTROLLER_DIGITAL_A,
  };
  for (std::size_t i = 0; i < 12; ++i) {
    const std::int32_t pressed =
        pros::c::controller_get_digital(pros::E_CONTROLLER_MASTER,
                                        buttons[i]);
    if (pressed == PROS_ERR) {
      snapshot.api_ok = false;
      continue;
    }
    if (pressed != 0) snapshot.buttons |= (1u << i);
  }
  return snapshot;
}

ProsDriveIO::ProsDriveIO(const RobotConfig& config,
                         const char* expected_robot_id,
                         std::uint32_t expected_schema)
    : hardware_(config.hardware), electrical_(config.electrical) {
  structurally_valid_ =
      validateConfig(config, expected_robot_id, expected_schema)
          .structurally_valid;
}

bool ProsDriveIO::configureMotor(const MotorPortConfig& motor) {
  const auto gearset = toProsGearset(motor.cartridge_rpm);
  if (gearset == pros::E_MOTOR_GEARSET_INVALID) return false;
  const bool gearing_ok =
      pros::c::motor_set_gearing(motor.smart_port, gearset) != PROS_ERR;
  const bool units_ok =
      pros::c::motor_set_encoder_units(
          motor.smart_port, pros::E_MOTOR_ENCODER_DEGREES) != PROS_ERR;
  return gearing_ok && units_ok;
}

bool ProsDriveIO::initialize() {
  if (!structurally_valid_) return false;
  bool success = true;
  for (const auto& motor : hardware_.left)
    success = configureMotor(motor) && success;
  for (const auto& motor : hardware_.right)
    success = configureMotor(motor) && success;
  // The 1690X commissioning policy keeps all six drivetrain brake modes in
  // Coast. Nonzero voltage commands still drive normally; the mode controls
  // only the behavior after an explicit brake/zero command.
  success = stop(StopMode::Coast) && success;
  return success;
}

bool ProsDriveIO::beginImuCalibration() {
  return structurally_valid_ && hardware_.imu.installed &&
         pros::c::imu_reset(hardware_.imu.smart_port) != PROS_ERR;
}

MotorSample ProsDriveIO::readMotor(const MotorPortConfig& motor) {
  MotorSample sample{};
  sample.smart_port = static_cast<std::uint8_t>(motor.smart_port);
  const double direction = motor.reversed ? -1.0 : 1.0;

  const double position_deg = pros::c::motor_get_position(motor.smart_port);
  const TimeUs position_time = pros::c::micros();
  const bool position_ok = std::isfinite(position_deg);
  sample.position_rad = makeSample(
      direction * units::degreesToRadians(position_deg), position_time,
      position_ok);

  const double velocity_rpm =
      pros::c::motor_get_actual_velocity(motor.smart_port);
  const TimeUs velocity_time = pros::c::micros();
  const bool velocity_ok = std::isfinite(velocity_rpm);
  sample.velocity_radps = makeSample(
      direction * units::rpmToRadiansPerSecond(velocity_rpm), velocity_time,
      velocity_ok);

  const std::int32_t current_mA =
      pros::c::motor_get_current_draw(motor.smart_port);
  sample.current_A =
      makeSample(units::milliampsToAmps(current_mA), pros::c::micros(),
                 current_mA != PROS_ERR && current_mA >= 0);

  const double temperature_C =
      pros::c::motor_get_temperature(motor.smart_port);
  sample.temperature_C = makeSample(temperature_C, pros::c::micros(),
                                    std::isfinite(temperature_C));

  const std::int32_t voltage_mV =
      pros::c::motor_get_voltage(motor.smart_port);
  sample.applied_voltage_V =
      makeSample(units::millivoltsToVolts(voltage_mV), pros::c::micros(),
                 voltage_mV != PROS_ERR);

  const std::uint32_t faults =
      pros::c::motor_get_faults(motor.smart_port);
  sample.faults_api_ok = faults != static_cast<std::uint32_t>(PROS_ERR);
  sample.faults = sample.faults_api_ok ? faults : 0;
  return sample;
}

RawDriveInputs ProsDriveIO::readAll(const FrameHeader& header) {
  RawDriveInputs inputs{};
  inputs.h = header;
  if (!structurally_valid_) return inputs;

  for (std::size_t i = 0; i < kMotorsPerSide; ++i) {
    inputs.left.motor[i] = readMotor(hardware_.left[i]);
    inputs.right.motor[i] = readMotor(hardware_.right[i]);
  }

  if (hardware_.imu.installed) {
    const auto imu_status =
        pros::c::imu_get_status(hardware_.imu.smart_port);
    inputs.imu.status_api_ok = imu_status != pros::E_IMU_STATUS_ERROR;
    inputs.imu.calibrating =
        inputs.imu.status_api_ok &&
        (static_cast<std::uint32_t>(imu_status) &
         static_cast<std::uint32_t>(pros::E_IMU_STATUS_CALIBRATING)) != 0;

    const double rotation_deg =
        pros::c::imu_get_rotation(hardware_.imu.smart_port);
    inputs.imu.rotation_rad =
        makeSample(units::degreesToRadians(rotation_deg), pros::c::micros(),
                   std::isfinite(rotation_deg) && !inputs.imu.calibrating);

    const pros::imu_gyro_s_t gyro =
        pros::c::imu_get_gyro_rate(hardware_.imu.smart_port);
    inputs.imu.yaw_rate_radps =
        makeSample(units::degreesToRadians(gyro.z), pros::c::micros(),
                   std::isfinite(gyro.z) && !inputs.imu.calibrating);
  } else {
    const TimeUs now_us = pros::c::micros();
    inputs.imu.status_api_ok = false;
    inputs.imu.calibrating = false;
    inputs.imu.rotation_rad = makeSample(0.0, now_us, false);
    inputs.imu.yaw_rate_radps = makeSample(0.0, now_us, false);
  }

  const std::int32_t battery_mV = pros::c::battery_get_voltage();
  inputs.battery_V =
      makeSample(units::millivoltsToVolts(battery_mV), pros::c::micros(),
                 battery_mV != PROS_ERR && battery_mV > 0);
  inputs.acquisition_end_us = pros::c::micros();
  return inputs;
}

bool ProsDriveIO::writeSide(
    const std::array<MotorPortConfig, kMotorsPerSide>& side,
    double voltage_V) {
  bool success = true;
  for (const auto& motor : side) {
    const double signed_voltage = motor.reversed ? -voltage_V : voltage_V;
    success =
        pros::c::motor_move_voltage(
            motor.smart_port,
            units::voltsToMotorMillivolts(signed_voltage)) != PROS_ERR &&
        success;
  }
  return success;
}

bool ProsDriveIO::writeVoltage(double left_V, double right_V) {
  if (!structurally_valid_ || !std::isfinite(left_V) ||
      !std::isfinite(right_V)) {
    return false;
  }
  const double limit = electrical_.max_command_voltage_V;
  const double left = std::clamp(left_V, -limit, limit);
  const double right = std::clamp(right_V, -limit, limit);
  const bool left_ok = writeSide(hardware_.left, left);
  const bool right_ok = writeSide(hardware_.right, right);
  return left_ok && right_ok;
}

bool ProsDriveIO::stop(StopMode mode) {
  if (!structurally_valid_) return false;
  const auto pros_mode = toProsBrake(mode);
  bool success = true;
  for (const auto& motor : hardware_.left) {
    success = pros::c::motor_set_brake_mode(motor.smart_port, pros_mode) !=
                  PROS_ERR &&
              success;
    success = pros::c::motor_brake(motor.smart_port) != PROS_ERR && success;
  }
  for (const auto& motor : hardware_.right) {
    success = pros::c::motor_set_brake_mode(motor.smart_port, pros_mode) !=
                  PROS_ERR &&
              success;
    success = pros::c::motor_brake(motor.smart_port) != PROS_ERR && success;
  }
  return success;
}

}  // namespace robot
