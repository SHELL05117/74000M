#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/health/fault_manager.hpp"
#include "robot/sensors/validated_inputs.hpp"

namespace robot {

struct MotorProtectionConfig {
  double temperature_enter_C{};
  double temperature_exit_C{};
  double current_enter_A{};
  double current_exit_A{};
  double stall_enter_voltage_V{};
  double stall_exit_voltage_V{};
  double stall_enter_speed_radps{};
  double stall_exit_speed_radps{};
  double stall_min_current_A{};
};

inline bool validMotorProtectionConfig(
    const MotorProtectionConfig& config) noexcept {
  return std::isfinite(config.temperature_enter_C) &&
         std::isfinite(config.temperature_exit_C) &&
         config.temperature_exit_C < config.temperature_enter_C &&
         std::isfinite(config.current_enter_A) &&
         config.current_enter_A > 0.0 &&
         std::isfinite(config.current_exit_A) &&
         config.current_exit_A >= 0.0 &&
         config.current_exit_A < config.current_enter_A &&
         std::isfinite(config.stall_enter_voltage_V) &&
         config.stall_enter_voltage_V > 0.0 &&
         config.stall_enter_voltage_V <= 12.0 &&
         std::isfinite(config.stall_exit_voltage_V) &&
         config.stall_exit_voltage_V >= 0.0 &&
         config.stall_exit_voltage_V < config.stall_enter_voltage_V &&
         std::isfinite(config.stall_enter_speed_radps) &&
         config.stall_enter_speed_radps >= 0.0 &&
         std::isfinite(config.stall_exit_speed_radps) &&
         config.stall_exit_speed_radps > config.stall_enter_speed_radps &&
         std::isfinite(config.stall_min_current_A) &&
         config.stall_min_current_A >= 0.0;
}

class MotorProtectionMonitor {
 public:
  explicit MotorProtectionMonitor(MotorProtectionConfig config) noexcept
      : config_(config) {}

  bool valid() const noexcept { return validMotorProtectionConfig(config_); }

  FaultEvidence update(const ValidatedInputs& inputs, double left_command_V,
                       double right_command_V) noexcept {
    FaultEvidence evidence{};
    if (!valid() || !std::isfinite(left_command_V) ||
        !std::isfinite(right_command_V)) {
      evidence.known_bits = faultBit(Fault::ThermalDerate) |
                            faultBit(Fault::MotorOverCurrent) |
                            faultBit(Fault::Stall);
      evidence.asserted_bits = evidence.known_bits;
      evidence.affected_motor_mask = 0x3Fu;
      return evidence;
    }
    bool all_temperature_known = true;
    bool all_current_known = true;
    bool all_stall_known = true;
    for (std::size_t i = 0; i < kMotorsPerSide; ++i) {
      updateMotor(inputs.left[i], left_command_V, state_[i], i, evidence);
      updateMotor(inputs.right[i], right_command_V,
                  state_[i + kMotorsPerSide], i + kMotorsPerSide, evidence);
      accumulateKnown(inputs.left[i], all_temperature_known,
                      all_current_known, all_stall_known);
      accumulateKnown(inputs.right[i], all_temperature_known,
                      all_current_known, all_stall_known);
    }
    const FaultBits thermal = faultBit(Fault::ThermalDerate);
    const FaultBits current = faultBit(Fault::MotorOverCurrent);
    const FaultBits stall = faultBit(Fault::Stall);
    if (all_temperature_known || (evidence.asserted_bits & thermal) != 0)
      evidence.known_bits |= thermal;
    if (all_current_known || (evidence.asserted_bits & current) != 0)
      evidence.known_bits |= current;
    if (all_stall_known || (evidence.asserted_bits & stall) != 0)
      evidence.known_bits |= stall;
    return evidence;
  }

  void reset() noexcept { state_ = {}; }

 private:
  struct MotorState {
    bool hot{};
    bool over_current{};
    bool stalled{};
  };

  static void accumulateKnown(const ValidatedMotor& motor,
                              bool& all_temperature_known,
                              bool& all_current_known,
                              bool& all_stall_known) noexcept {
    const bool temperature_known =
        motor.temperature_C.quality != Quality::Invalid &&
        std::isfinite(motor.temperature_C.value);
    const bool current_known =
        motor.current_A.quality != Quality::Invalid &&
        std::isfinite(motor.current_A.value);
    const bool velocity_known =
        motor.velocity_radps.quality != Quality::Invalid &&
        std::isfinite(motor.velocity_radps.value);
    all_temperature_known = all_temperature_known && temperature_known;
    all_current_known = all_current_known && current_known;
    all_stall_known = all_stall_known && current_known && velocity_known;
  }

  void updateMotor(const ValidatedMotor& motor, double command_V,
                   MotorState& state, std::size_t index,
                   FaultEvidence& evidence) noexcept {
    const bool temperature_valid =
        motor.temperature_C.quality != Quality::Invalid &&
        std::isfinite(motor.temperature_C.value);
    if (temperature_valid) {
      if (!state.hot &&
          motor.temperature_C.value >= config_.temperature_enter_C)
        state.hot = true;
      else if (state.hot &&
               motor.temperature_C.value <= config_.temperature_exit_C)
        state.hot = false;
    }

    const bool current_valid =
        motor.current_A.quality != Quality::Invalid &&
        std::isfinite(motor.current_A.value);
    if (current_valid) {
      if (!state.over_current &&
          motor.current_A.value >= config_.current_enter_A)
        state.over_current = true;
      else if (state.over_current &&
               motor.current_A.value <= config_.current_exit_A)
        state.over_current = false;
    }

    const bool velocity_valid =
        motor.velocity_radps.quality != Quality::Invalid &&
        std::isfinite(motor.velocity_radps.value);
    const bool stall_inputs_valid = current_valid && velocity_valid;
    if (stall_inputs_valid) {
      if (!state.stalled &&
          std::abs(command_V) >= config_.stall_enter_voltage_V &&
          std::abs(motor.velocity_radps.value) <=
              config_.stall_enter_speed_radps &&
          motor.current_A.value >= config_.stall_min_current_A) {
        state.stalled = true;
      } else if (state.stalled &&
                 (std::abs(command_V) <= config_.stall_exit_voltage_V ||
                  std::abs(motor.velocity_radps.value) >=
                      config_.stall_exit_speed_radps)) {
        state.stalled = false;
      }
    }

    const std::uint32_t motor_bit = 1u << index;
    if (state.hot) {
      evidence.asserted_bits |= faultBit(Fault::ThermalDerate);
      evidence.affected_motor_mask |= motor_bit;
    }
    if (state.over_current) {
      evidence.asserted_bits |= faultBit(Fault::MotorOverCurrent);
      evidence.affected_motor_mask |= motor_bit;
    }
    if (state.stalled) {
      evidence.asserted_bits |= faultBit(Fault::Stall);
      evidence.affected_motor_mask |= motor_bit;
    }
  }

  MotorProtectionConfig config_{};
  std::array<MotorState, 2 * kMotorsPerSide> state_{};
};

}  // namespace robot
