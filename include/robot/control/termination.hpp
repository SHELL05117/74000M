#pragma once

#include <cmath>
#include <cstdint>

#include "robot/core/frame.hpp"

namespace robot {

enum class MotionTerminationState : std::uint8_t {
  Idle,
  Running,
  Settling,
  Succeeded,
  TimedOut,
  Stalled,
  StateInvalid,
};

struct MotionTerminationConfig {
  double error_band{};
  double velocity_band{};
  TimeUs settle_time_us{};
  TimeUs timeout_us{};
  double stall_min_effort{};
  double stall_max_velocity{};
  TimeUs stall_time_us{};
};

inline bool validMotionTerminationConfig(
    const MotionTerminationConfig& config) noexcept {
  return std::isfinite(config.error_band) && config.error_band >= 0.0 &&
         std::isfinite(config.velocity_band) && config.velocity_band >= 0.0 &&
         config.settle_time_us > 0 && config.timeout_us > 0 &&
         std::isfinite(config.stall_min_effort) &&
         config.stall_min_effort >= 0.0 &&
         std::isfinite(config.stall_max_velocity) &&
         config.stall_max_velocity >= 0.0 && config.stall_time_us > 0;
}

class MotionTerminationMonitor {
 public:
  explicit MotionTerminationMonitor(MotionTerminationConfig config) noexcept
      : config_(config) {}

  bool start(TimeUs now_us) noexcept {
    reset();
    if (!validMotionTerminationConfig(config_)) return false;
    start_us_ = now_us;
    last_us_ = now_us;
    state_ = MotionTerminationState::Running;
    return true;
  }

  MotionTerminationState update(TimeUs now_us, double error,
                                double velocity, double effort,
                                bool state_valid) noexcept {
    if (terminal() || state_ == MotionTerminationState::Idle) return state_;
    if (!state_valid || now_us < last_us_ || !std::isfinite(error) ||
        !std::isfinite(velocity) || !std::isfinite(effort)) {
      state_ = MotionTerminationState::StateInvalid;
      return state_;
    }
    last_us_ = now_us;
    if (now_us - start_us_ >= config_.timeout_us) {
      state_ = MotionTerminationState::TimedOut;
      return state_;
    }

    const bool stalled = std::abs(effort) >= config_.stall_min_effort &&
                         std::abs(velocity) <= config_.stall_max_velocity;
    if (stalled) {
      if (!stall_timing_) {
        stall_timing_ = true;
        stall_start_us_ = now_us;
      } else if (now_us - stall_start_us_ >= config_.stall_time_us) {
        state_ = MotionTerminationState::Stalled;
        return state_;
      }
    } else {
      stall_timing_ = false;
    }

    const bool in_band = std::abs(error) <= config_.error_band &&
                         std::abs(velocity) <= config_.velocity_band;
    if (!in_band) {
      settling_ = false;
      state_ = MotionTerminationState::Running;
      return state_;
    }
    if (!settling_) {
      settling_ = true;
      settle_start_us_ = now_us;
      state_ = MotionTerminationState::Settling;
    } else if (now_us - settle_start_us_ >= config_.settle_time_us) {
      state_ = MotionTerminationState::Succeeded;
    }
    return state_;
  }

  void reset() noexcept {
    state_ = MotionTerminationState::Idle;
    start_us_ = 0;
    last_us_ = 0;
    settle_start_us_ = 0;
    stall_start_us_ = 0;
    settling_ = false;
    stall_timing_ = false;
  }

  MotionTerminationState state() const noexcept { return state_; }
  bool terminal() const noexcept {
    return state_ == MotionTerminationState::Succeeded ||
           state_ == MotionTerminationState::TimedOut ||
           state_ == MotionTerminationState::Stalled ||
           state_ == MotionTerminationState::StateInvalid;
  }

 private:
  MotionTerminationConfig config_{};
  MotionTerminationState state_{MotionTerminationState::Idle};
  TimeUs start_us_{};
  TimeUs last_us_{};
  TimeUs settle_start_us_{};
  TimeUs stall_start_us_{};
  bool settling_{};
  bool stall_timing_{};
};

}  // namespace robot
