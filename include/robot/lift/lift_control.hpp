#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robot/commands/scheduler.hpp"
#include "robot/config/robot_config.hpp"
#include "robot/drive/drive_request.hpp"
#include "robot/manual/input_shaping.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/state/controller_snapshot.hpp"
#include "robot/state/raw_inputs.hpp"

namespace robot {

struct LiftRequest {
  FrameHeader h{};
  RequestSource source{RequestSource::None};
  OwnerToken owner{};
  double normalized_output{};
  TimeUs ttl_us{};
};

struct LiftCommissioningConfig {
  AxisShapeConfig axis_shape{};
  double maximum_voltage_V{12.0};
  TimeUs request_ttl_us{30000};
  TimeUs maximum_sample_age_us{30000};
  double upper_guard_rad{};
  double slowdown_zone_rad{};
  double maximum_disagreement_rad{};
};

inline LiftCommissioningConfig make1690XLiftCommissioningConfig() noexcept {
  LiftCommissioningConfig config{};
  // Axis2 is proportional after a remapped neutral deadband. There is no
  // cubic component, so stick travel remains approximately linear.
  config.axis_shape = {0.0, 0.06, 0.0};
  config.maximum_voltage_V = 12.0;
  config.request_ttl_us = 30000;
  config.maximum_sample_age_us = 30000;
  // HIL candidates: command motion stops ten motor degrees before the
  // physical 830-degree endpoint, and speed tapers over the last 90 degrees.
  config.upper_guard_rad = units::degreesToRadians(10.0);
  config.slowdown_zone_rad = units::degreesToRadians(90.0);
  config.maximum_disagreement_rad = units::degreesToRadians(25.0);
  return config;
}

inline bool validLiftCommissioningConfig(
    const LiftCommissioningConfig& config,
    const LiftHardwareConfig& hardware) noexcept {
  const double travel =
      hardware.maximum_position_rad - hardware.minimum_position_rad;
  return hardware.installed && validAxisShape(config.axis_shape) &&
         std::isfinite(config.maximum_voltage_V) &&
         config.maximum_voltage_V > 0.0 &&
         config.maximum_voltage_V <= 12.0 && config.request_ttl_us > 0 &&
         config.maximum_sample_age_us > 0 && std::isfinite(travel) &&
         travel > 0.0 && std::isfinite(config.upper_guard_rad) &&
         config.upper_guard_rad > 0.0 && config.upper_guard_rad < travel &&
         std::isfinite(config.slowdown_zone_rad) &&
         config.slowdown_zone_rad > config.upper_guard_rad &&
         config.slowdown_zone_rad < travel &&
         std::isfinite(config.maximum_disagreement_rad) &&
         config.maximum_disagreement_rad > 0.0 &&
         config.maximum_disagreement_rad < travel;
}

class LiftManualMapper {
 public:
  LiftManualMapper(LiftCommissioningConfig config,
                   LiftHardwareConfig hardware) noexcept
      : config_(config), hardware_(hardware) {}

  bool update(const FrameHeader& header, const ModeSnapshot& mode,
              const ControllerSnapshot& controller,
              const OwnerToken& owner, LiftRequest& request) const noexcept {
    request = {};
    const bool valid_frame = controller.h.time_us == header.time_us &&
                             controller.h.sequence == header.sequence &&
                             controller.h.mode_epoch == header.mode_epoch;
    const bool valid_owner =
        owner.command_id != 0 &&
        (owner.requirements & Requirement::kLift) != 0 &&
        owner.mode_epoch == header.mode_epoch;
    const bool valid_mode = mode.enabled &&
                            mode.mode == CompetitionMode::Test &&
                            !mode.field_connected &&
                            mode.epoch == header.mode_epoch;
    if (!validLiftCommissioningConfig(config_, hardware_) || !valid_frame ||
        !valid_owner || !valid_mode || !controller.connected ||
        !controller.api_ok || !std::isfinite(controller.right_y)) {
      return false;
    }

    const double centered = centeredAxis(controller.right_y,
                                         config_.axis_shape);
    const double shaped = shapeCenteredAxis(centered, config_.axis_shape);
    if (!std::isfinite(shaped) || std::abs(shaped) > 1.0) return false;
    request.h = header;
    request.source = RequestSource::Test;
    request.owner = owner;
    request.normalized_output = shaped;
    request.ttl_us = config_.request_ttl_us;
    return true;
  }

 private:
  LiftCommissioningConfig config_{};
  LiftHardwareConfig hardware_{};
};

enum LiftLimit : std::uint32_t {
  kLiftLimitNone = 0,
  kLiftNoRequest = 1u << 0,
  kLiftBadRequest = 1u << 1,
  kLiftStaleRequest = 1u << 2,
  kLiftBadPosition = 1u << 3,
  kLiftPositionDisagreement = 1u << 4,
  kLiftLowerLimit = 1u << 5,
  kLiftUpperLimit = 1u << 6,
  kLiftSlowdownApplied = 1u << 7,
};

struct LiftControlResult {
  double voltage_V{};
  StopMode zero_behavior{StopMode::Hold};
  double normalized_output{};
  double minimum_position_rad{};
  double maximum_position_rad{};
  double applied_scale{};
  std::uint32_t limit_bits{};
  bool position_valid{};
  bool request_valid{};
};

class LiftSafetyGate {
 public:
  LiftSafetyGate(LiftCommissioningConfig config,
                 LiftHardwareConfig hardware) noexcept
      : config_(config), hardware_(hardware) {}

  LiftControlResult apply(const LiftRequest* request,
                          const std::array<MotorSample, kLiftMotorCount>& raw,
                          const ModeSnapshot& mode,
                          TimeUs now_us) const noexcept {
    LiftControlResult result{};
    if (request == nullptr) {
      result.limit_bits |= kLiftNoRequest;
      return result;
    }
    if (!validLiftCommissioningConfig(config_, hardware_) || !mode.enabled ||
        mode.mode != CompetitionMode::Test || mode.field_connected ||
        request->source != RequestSource::Test ||
        request->h.mode_epoch != mode.epoch ||
        request->owner.mode_epoch != mode.epoch ||
        request->owner.command_id == 0 ||
        (request->owner.requirements & Requirement::kLift) == 0 ||
        request->ttl_us == 0 ||
        !std::isfinite(request->normalized_output) ||
        std::abs(request->normalized_output) > 1.0 ||
        request->h.time_us > now_us) {
      result.limit_bits |= kLiftBadRequest;
      return result;
    }
    if (now_us - request->h.time_us > request->ttl_us) {
      result.limit_bits |= kLiftStaleRequest;
      return result;
    }
    result.request_valid = true;
    result.normalized_output = request->normalized_output;

    double positions[kLiftMotorCount]{};
    for (std::size_t i = 0; i < kLiftMotorCount; ++i) {
      const ScalarSample& sample = raw[i].position_rad;
      if (raw[i].smart_port !=
              static_cast<std::uint8_t>(hardware_.motors[i].smart_port) ||
          !sample.api_ok || !std::isfinite(sample.value) ||
          sample.sample_time_us > now_us ||
          now_us - sample.sample_time_us > config_.maximum_sample_age_us) {
        result.limit_bits |= kLiftBadPosition;
        return result;
      }
      positions[i] = sample.value;
    }
    result.position_valid = true;
    result.minimum_position_rad = std::min(positions[0], positions[1]);
    result.maximum_position_rad = std::max(positions[0], positions[1]);
    if (result.maximum_position_rad - result.minimum_position_rad >
        config_.maximum_disagreement_rad) {
      result.limit_bits |= kLiftPositionDisagreement;
      return result;
    }

    if (request->normalized_output == 0.0) return result;
    double remaining{};
    if (request->normalized_output > 0.0) {
      const double command_ceiling =
          hardware_.maximum_position_rad - config_.upper_guard_rad;
      remaining = command_ceiling - result.maximum_position_rad;
      if (remaining <= 0.0) {
        result.limit_bits |= kLiftUpperLimit;
        return result;
      }
    } else {
      remaining = result.minimum_position_rad -
                  hardware_.minimum_position_rad;
      if (remaining <= 0.0) {
        result.limit_bits |= kLiftLowerLimit;
        return result;
      }
    }

    result.applied_scale =
        std::clamp(remaining / config_.slowdown_zone_rad, 0.0, 1.0);
    if (result.applied_scale < 1.0)
      result.limit_bits |= kLiftSlowdownApplied;
    result.voltage_V = request->normalized_output *
                       config_.maximum_voltage_V * result.applied_scale;
    if (!std::isfinite(result.voltage_V) ||
        std::abs(result.voltage_V) > config_.maximum_voltage_V) {
      result.voltage_V = 0.0;
      result.limit_bits |= kLiftBadRequest;
    }
    return result;
  }

 private:
  LiftCommissioningConfig config_{};
  LiftHardwareConfig hardware_{};
};

}  // namespace robot
