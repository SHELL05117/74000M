#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robot/core/quality.hpp"
#include "robot/state/pose2d.hpp"

namespace robot {

enum class HeadingAssistState : std::uint8_t {
  Disarmed,
  Candidate,
  Locked,
};

struct HeadingAssistConfig {
  bool enabled{};
  double raw_turn_enter{};
  double raw_turn_release{};
  double min_throttle{};
  double speed_enter_mps{};
  double speed_exit_mps{};
  double lock_delay_s{};
  double kP{};
  double kD{};
  double max_turn{};
  Quality required_heading_quality{Quality::Good};
  Quality required_translation_quality{Quality::Good};
};

inline bool validHeadingAssistConfig(
    const HeadingAssistConfig& config) noexcept {
  return std::isfinite(config.raw_turn_enter) &&
         config.raw_turn_enter >= 0.0 &&
         std::isfinite(config.raw_turn_release) &&
         config.raw_turn_release >= config.raw_turn_enter &&
         config.raw_turn_release < 1.0 &&
         std::isfinite(config.min_throttle) && config.min_throttle >= 0.0 &&
         config.min_throttle <= 1.0 &&
         std::isfinite(config.speed_enter_mps) &&
         config.speed_enter_mps > 0.0 &&
         std::isfinite(config.speed_exit_mps) &&
         config.speed_exit_mps >= 0.0 &&
         config.speed_exit_mps < config.speed_enter_mps &&
         std::isfinite(config.lock_delay_s) && config.lock_delay_s > 0.0 &&
         std::isfinite(config.kP) && config.kP >= 0.0 &&
         std::isfinite(config.kD) && config.kD >= 0.0 &&
         std::isfinite(config.max_turn) && config.max_turn > 0.0 &&
         config.max_turn <= 1.0;
}

inline bool qualityMeets(Quality actual, Quality required) noexcept {
  return static_cast<std::uint8_t>(actual) <=
         static_cast<std::uint8_t>(required);
}

struct HeadingAssistInput {
  double raw_turn_centered{};
  double shaped_turn{};
  double throttle{};
  double heading_rad{};
  double yaw_rate_radps{};
  double speed_mps{};
  Quality heading_quality{Quality::Invalid};
  Quality translation_quality{Quality::Invalid};
  bool quick_turn{};
  bool state_fresh{};
  double dt_s{};
};

struct HeadingAssistOutput {
  HeadingAssistState state{HeadingAssistState::Disarmed};
  bool active{};
  double turn{};
  double reference_rad{};
  double error_rad{};
};

class HeadingAssist {
 public:
  explicit HeadingAssist(HeadingAssistConfig config) noexcept
      : config_(config) {}

  bool valid() const noexcept { return validHeadingAssistConfig(config_); }

  HeadingAssistOutput update(const HeadingAssistInput& input) noexcept {
    if (!valid() || !config_.enabled || !finiteInput(input)) {
      reset();
      return output();
    }
    const bool state_good =
        input.state_fresh &&
        qualityMeets(input.heading_quality,
                     config_.required_heading_quality) &&
        qualityMeets(input.translation_quality,
                     config_.required_translation_quality);
    const bool release =
        std::abs(input.raw_turn_centered) > config_.raw_turn_release ||
        input.shaped_turn != 0.0 ||
        std::abs(input.throttle) < config_.min_throttle ||
        std::abs(input.speed_mps) < config_.speed_exit_mps ||
        !state_good || input.quick_turn;
    if (release) {
      reset();
      return output();
    }

    const bool can_enter =
        std::abs(input.raw_turn_centered) <= config_.raw_turn_enter &&
        std::abs(input.speed_mps) >= config_.speed_enter_mps;
    if (state_ == HeadingAssistState::Disarmed && can_enter) {
      state_ = HeadingAssistState::Candidate;
      candidate_time_s_ = 0.0;
    }
    if (state_ == HeadingAssistState::Candidate) {
      if (!can_enter) {
        reset();
      } else {
        candidate_time_s_ += input.dt_s;
        if (candidate_time_s_ >= config_.lock_delay_s) {
          reference_rad_ = input.heading_rad;
          state_ = HeadingAssistState::Locked;
        }
      }
    }
    if (state_ != HeadingAssistState::Locked) return output();

    const double error = wrapPi(reference_rad_ - input.heading_rad);
    const double turn =
        std::clamp(config_.kP * error - config_.kD * input.yaw_rate_radps,
                   -config_.max_turn, config_.max_turn);
    return {state_, true, turn, reference_rad_, error};
  }

  void reset() noexcept {
    state_ = HeadingAssistState::Disarmed;
    candidate_time_s_ = 0.0;
    reference_rad_ = 0.0;
  }

  HeadingAssistState state() const noexcept { return state_; }

 private:
  static bool finiteInput(const HeadingAssistInput& input) noexcept {
    return std::isfinite(input.raw_turn_centered) &&
           std::isfinite(input.shaped_turn) &&
           std::isfinite(input.throttle) &&
           std::isfinite(input.heading_rad) &&
           std::isfinite(input.yaw_rate_radps) &&
           std::isfinite(input.speed_mps) && std::isfinite(input.dt_s) &&
           input.dt_s >= 0.0;
  }

  HeadingAssistOutput output() const noexcept {
    return {state_, false, 0.0, reference_rad_, 0.0};
  }

  HeadingAssistConfig config_{};
  HeadingAssistState state_{HeadingAssistState::Disarmed};
  double candidate_time_s_{};
  double reference_rad_{};
};

}  // namespace robot
