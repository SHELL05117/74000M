#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/config/robot_config.hpp"
#include "robot/core/frame.hpp"

namespace robot {

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
