#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robot/drive/drive_request.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/state/robot_state.hpp"

namespace robot {

enum class CharacterizationKind : std::uint8_t {
  Quasistatic,
  Dynamic,
};

enum class CharacterizationDirection : std::int8_t {
  Reverse = -1,
  Forward = 1,
};

enum class CharacterizationPhase : std::uint8_t {
  Idle,
  Running,
  Stopping,
  Complete,
  Aborted,
};

enum CharacterizationStopReason : std::uint32_t {
  kCharacterizationNoStop = 0,
  kCharacterizationTimeLimit = 1u << 0,
  kCharacterizationDistanceLimit = 1u << 1,
  kCharacterizationModeLost = 1u << 2,
  kCharacterizationStateInvalid = 1u << 3,
  kCharacterizationBadConfig = 1u << 4,
};

struct CharacterizationConfig {
  CharacterizationKind kind{CharacterizationKind::Quasistatic};
  CharacterizationDirection direction{CharacterizationDirection::Forward};
  double ramp_V_per_s{};
  double dynamic_step_V{};
  double max_voltage_V{};
  TimeUs max_duration_us{};
  double max_distance_m{};
  TimeUs state_ttl_us{};
  TimeUs request_ttl_us{};
};

inline bool validCharacterizationConfig(
    const CharacterizationConfig& config) noexcept {
  return std::isfinite(config.ramp_V_per_s) && config.ramp_V_per_s > 0.0 &&
         std::isfinite(config.dynamic_step_V) &&
         config.dynamic_step_V > 0.0 &&
         std::isfinite(config.max_voltage_V) &&
         config.max_voltage_V > 0.0 && config.max_voltage_V <= 12.0 &&
         config.dynamic_step_V <= config.max_voltage_V &&
         config.max_duration_us > 0 &&
         std::isfinite(config.max_distance_m) &&
         config.max_distance_m > 0.0 && config.state_ttl_us > 0 &&
         config.request_ttl_us > 0;
}

struct CharacterizationTick {
  DriveRequest request{};
  CharacterizationPhase phase{CharacterizationPhase::Idle};
  std::uint32_t stop_reason{};
  double commanded_voltage_V{};
  bool has_request{};
};

class CharacterizationRunner {
 public:
  bool start(const CharacterizationConfig& config, const FrameHeader& header,
             const ModeSnapshot& mode, const RobotState& state,
             const OwnerToken& owner, bool bench_authorized) noexcept {
    reset();
    if (!bench_authorized || !validCharacterizationConfig(config) ||
        !validTestContext(header, mode, owner) ||
        !validState(state, header, config.state_ttl_us)) {
      phase_ = CharacterizationPhase::Aborted;
      stop_reason_ = kCharacterizationBadConfig;
      return false;
    }
    config_ = config;
    owner_ = owner;
    start_time_us_ = header.time_us;
    start_distance_m_ =
        0.5 * (state.left_distance_m + state.right_distance_m);
    phase_ = CharacterizationPhase::Running;
    return true;
  }

  CharacterizationTick tick(const FrameHeader& header,
                            const ModeSnapshot& mode,
                            const RobotState& state) noexcept {
    CharacterizationTick output{};
    output.phase = phase_;
    output.stop_reason = stop_reason_;
    if (phase_ == CharacterizationPhase::Idle ||
        phase_ == CharacterizationPhase::Complete ||
        phase_ == CharacterizationPhase::Aborted)
      return output;
    if (!validTestContext(header, mode, owner_)) {
      phase_ = CharacterizationPhase::Aborted;
      stop_reason_ |= kCharacterizationModeLost;
      output.phase = phase_;
      output.stop_reason = stop_reason_;
      return output;
    }
    if (header.time_us < start_time_us_) {
      phase_ = CharacterizationPhase::Aborted;
      stop_reason_ |= kCharacterizationStateInvalid;
      output.phase = phase_;
      output.stop_reason = stop_reason_;
      return output;
    }
    if (!validState(state, header, config_.state_ttl_us)) {
      phase_ = CharacterizationPhase::Stopping;
      stop_reason_ |= kCharacterizationStateInvalid;
    }

    const TimeUs elapsed_us = header.time_us - start_time_us_;
    const double distance =
        std::abs(0.5 * (state.left_distance_m + state.right_distance_m) -
                 start_distance_m_);
    if (elapsed_us >= config_.max_duration_us) {
      phase_ = CharacterizationPhase::Stopping;
      stop_reason_ |= kCharacterizationTimeLimit;
    }
    if (distance >= config_.max_distance_m) {
      phase_ = CharacterizationPhase::Stopping;
      stop_reason_ |= kCharacterizationDistanceLimit;
    }

    output.request.h = header;
    output.request.source = RequestSource::Test;
    output.request.owner = owner_;
    output.request.ttl_us = config_.request_ttl_us;
    if (phase_ == CharacterizationPhase::Stopping) {
      output.request.payload = BrakePayload{StopMode::Brake};
      output.has_request = true;
      phase_ = CharacterizationPhase::Complete;
      output.phase = CharacterizationPhase::Stopping;
      output.stop_reason = stop_reason_;
      return output;
    }

    const double direction =
        static_cast<double>(static_cast<std::int8_t>(config_.direction));
    const double elapsed_s = static_cast<double>(elapsed_us) * 1e-6;
    const double magnitude =
        config_.kind == CharacterizationKind::Quasistatic
            ? std::min(config_.max_voltage_V,
                       config_.ramp_V_per_s * elapsed_s)
            : config_.dynamic_step_V;
    output.commanded_voltage_V = direction * magnitude;
    output.request.payload = WheelVoltagePayload{output.commanded_voltage_V,
                                                 output.commanded_voltage_V};
    output.has_request = true;
    output.phase = phase_;
    return output;
  }

  void reset() noexcept {
    config_ = {};
    owner_ = {};
    start_time_us_ = 0;
    start_distance_m_ = 0.0;
    stop_reason_ = 0;
    phase_ = CharacterizationPhase::Idle;
  }

  CharacterizationPhase phase() const noexcept { return phase_; }

 private:
  static bool validTestContext(const FrameHeader& header,
                               const ModeSnapshot& mode,
                               const OwnerToken& owner) noexcept {
    return mode.enabled && mode.mode == CompetitionMode::Test &&
           mode.epoch == header.mode_epoch &&
           owner.mode_epoch == header.mode_epoch && owner.command_id != 0 &&
           (owner.requirements & Requirement::kDrivetrain) != 0;
  }

  static bool validState(const RobotState& state, const FrameHeader& header,
                         TimeUs state_ttl_us) noexcept {
    return state.h.mode_epoch == header.mode_epoch &&
           state.h.time_us <= header.time_us &&
           header.time_us - state.h.time_us <= state_ttl_us &&
           std::isfinite(state.left_distance_m) &&
           std::isfinite(state.right_distance_m);
  }

  CharacterizationConfig config_{};
  OwnerToken owner_{};
  TimeUs start_time_us_{};
  double start_distance_m_{};
  std::uint32_t stop_reason_{};
  CharacterizationPhase phase_{CharacterizationPhase::Idle};
};

}  // namespace robot
