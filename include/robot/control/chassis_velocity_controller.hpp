#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robot/control/engineering_pid.hpp"
#include "robot/control/feedforward.hpp"
#include "robot/drive/drive_request.hpp"
#include "robot/drive/kinematics.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/state/robot_state.hpp"

namespace robot {

struct ChassisVelocityControllerConfig {
  double track_width_m{};
  double max_wheel_speed_mps{};
  double max_target_acceleration_mps2{};
  double max_output_voltage_V{};
  double degraded_target_scale{};
  TimeUs max_state_age_us{};
  TimeUs output_request_ttl_us{};
  bool allow_degraded_state{};
  PidConfig left_pid{};
  PidConfig right_pid{};
  DifferentialFeedforwardConfig feedforward{};
};

enum ChassisVelocityReject : std::uint32_t {
  kChassisVelocityAccepted = 0,
  kChassisVelocityBadConfig = 1u << 0,
  kChassisVelocityBadRequest = 1u << 1,
  kChassisVelocityWrongMode = 1u << 2,
  kChassisVelocityEpoch = 1u << 3,
  kChassisVelocityStateFuture = 1u << 4,
  kChassisVelocityStateStale = 1u << 5,
  kChassisVelocityStateInvalid = 1u << 6,
  kChassisVelocityCapabilityLocked = 1u << 7,
  kChassisVelocityTargetOutOfRange = 1u << 8,
  kChassisVelocityControllerInvalid = 1u << 9,
};

enum ChassisVelocityLimit : std::uint32_t {
  kChassisVelocityNoLimit = 0,
  kChassisVelocityDegradedScale = 1u << 0,
  kChassisVelocityLeftAccelerationLimit = 1u << 1,
  kChassisVelocityRightAccelerationLimit = 1u << 2,
  kChassisVelocityVoltageClamp = 1u << 3,
};

struct ChassisVelocityControlInput {
  FrameHeader output_header{};
  ModeSnapshot mode{};
  const RobotState* state{};
  TimeUs now_us{};
  double dt_s{};
  DriveCapabilities capabilities{};
};

struct ChassisVelocityControlResult {
  DriveRequest request{};
  WheelSpeeds target{};
  DifferentialFeedforwardResult feedforward{};
  PidResult left_pid{};
  PidResult right_pid{};
  std::uint32_t reject_bits{};
  std::uint32_t applied_limits{};
  bool has_request{};
};

inline bool validChassisVelocityControllerConfig(
    const ChassisVelocityControllerConfig& config) noexcept {
  return std::isfinite(config.track_width_m) && config.track_width_m > 0.0 &&
         std::isfinite(config.max_wheel_speed_mps) &&
         config.max_wheel_speed_mps > 0.0 &&
         std::isfinite(config.max_target_acceleration_mps2) &&
         config.max_target_acceleration_mps2 > 0.0 &&
         std::isfinite(config.max_output_voltage_V) &&
         config.max_output_voltage_V > 0.0 &&
         config.max_output_voltage_V <= 12.0 &&
         std::isfinite(config.degraded_target_scale) &&
         config.degraded_target_scale > 0.0 &&
         config.degraded_target_scale <= 1.0 && config.max_state_age_us > 0 &&
         config.output_request_ttl_us > 0 && validPidConfig(config.left_pid) &&
         validPidConfig(config.right_pid) &&
         validFeedforwardGains(config.feedforward.left) &&
         validFeedforwardGains(config.feedforward.right) &&
         config.left_pid.output_min >= -config.max_output_voltage_V &&
         config.left_pid.output_max <= config.max_output_voltage_V &&
         config.right_pid.output_min >= -config.max_output_voltage_V &&
         config.right_pid.output_max <= config.max_output_voltage_V;
}

class ChassisVelocityController {
 public:
  explicit ChassisVelocityController(
      ChassisVelocityControllerConfig config) noexcept
      : config_(config),
        kinematics_(config.track_width_m),
        left_pid_(config.left_pid),
        right_pid_(config.right_pid) {}

  bool valid() const noexcept {
    return validChassisVelocityControllerConfig(config_) &&
           kinematics_.valid();
  }

  ChassisVelocityControlResult update(
      const DriveRequest& selected_request,
      const ChassisVelocityControlInput& input) noexcept {
    ChassisVelocityControlResult result{};
    if (!valid()) result.reject_bits |= kChassisVelocityBadConfig;
    const auto* velocity =
        std::get_if<ChassisVelocityPayload>(&selected_request.payload);
    if (velocity == nullptr ||
        selected_request.source != RequestSource::FutureAutonomy ||
        selected_request.owner.command_id == 0 ||
        (selected_request.owner.requirements & Requirement::kDrivetrain) == 0 ||
        selected_request.ttl_us == 0 ||
        !finitePayload(selected_request.payload))
      result.reject_bits |= kChassisVelocityBadRequest;
    if (selected_request.h.time_us > input.now_us ||
        (selected_request.h.time_us <= input.now_us &&
         input.now_us - selected_request.h.time_us > selected_request.ttl_us))
      result.reject_bits |= kChassisVelocityBadRequest;
    if (!input.mode.enabled ||
        input.mode.mode != CompetitionMode::AutonomousInterface)
      result.reject_bits |= kChassisVelocityWrongMode;
    if (selected_request.h.mode_epoch != input.mode.epoch ||
        selected_request.owner.mode_epoch != input.mode.epoch ||
        input.output_header.mode_epoch != input.mode.epoch)
      result.reject_bits |= kChassisVelocityEpoch;
    if (input.output_header.time_us > input.now_us)
      result.reject_bits |= kChassisVelocityControllerInvalid;
    if (!input.capabilities.autonomous_chassis_velocity)
      result.reject_bits |= kChassisVelocityCapabilityLocked;
    if (input.state == nullptr) {
      result.reject_bits |= kChassisVelocityStateInvalid;
    } else {
      if (input.state->h.mode_epoch != input.mode.epoch)
        result.reject_bits |= kChassisVelocityEpoch;
      if (input.state->h.time_us > input.now_us)
        result.reject_bits |= kChassisVelocityStateFuture;
      else if (input.now_us - input.state->h.time_us >
               config_.max_state_age_us)
        result.reject_bits |= kChassisVelocityStateStale;
      const bool quality_invalid =
          input.state->translation_quality == Quality::Invalid ||
          input.state->heading_quality == Quality::Invalid;
      const bool degraded =
          input.state->translation_quality == Quality::Degraded ||
          input.state->heading_quality == Quality::Degraded;
      if (quality_invalid || (degraded && !config_.allow_degraded_state) ||
          !std::isfinite(input.state->left_velocity_mps) ||
          !std::isfinite(input.state->right_velocity_mps))
        result.reject_bits |= kChassisVelocityStateInvalid;
    }
    if (!std::isfinite(input.dt_s) ||
        input.dt_s < config_.left_pid.min_dt_s ||
        input.dt_s > config_.left_pid.max_dt_s ||
        input.dt_s < config_.right_pid.min_dt_s ||
        input.dt_s > config_.right_pid.max_dt_s)
      result.reject_bits |= kChassisVelocityControllerInvalid;
    if (result.reject_bits != kChassisVelocityAccepted) {
      reset();
      return result;
    }

    if (!initialized_ || epoch_ != input.mode.epoch) {
      reset();
      initialized_ = true;
      epoch_ = input.mode.epoch;
    }
    ChassisSpeeds requested{velocity->vx_mps, velocity->omega_radps};
    if (input.state->translation_quality == Quality::Degraded ||
        input.state->heading_quality == Quality::Degraded) {
      requested.vx_mps *= config_.degraded_target_scale;
      requested.omega_radps *= config_.degraded_target_scale;
      result.applied_limits |= kChassisVelocityDegradedScale;
    }
    if (!kinematics_.inverse(requested, result.target) ||
        std::abs(result.target.left_mps) > config_.max_wheel_speed_mps ||
        std::abs(result.target.right_mps) > config_.max_wheel_speed_mps) {
      result.reject_bits |= kChassisVelocityTargetOutOfRange;
      reset();
      return result;
    }

    double left_acceleration{};
    double right_acceleration{};
    if (have_target_) {
      left_acceleration =
          (result.target.left_mps - last_target_.left_mps) / input.dt_s;
      right_acceleration =
          (result.target.right_mps - last_target_.right_mps) / input.dt_s;
      left_acceleration = limitAcceleration(
          left_acceleration, kChassisVelocityLeftAccelerationLimit,
          result.applied_limits);
      right_acceleration = limitAcceleration(
          right_acceleration, kChassisVelocityRightAccelerationLimit,
          result.applied_limits);
    }
    result.feedforward = calculateDifferentialFeedforward(
        config_.feedforward, result.target.left_mps, left_acceleration,
        result.target.right_mps, right_acceleration);
    if (!result.feedforward.valid) {
      result.reject_bits |= kChassisVelocityControllerInvalid;
      reset();
      return result;
    }
    result.left_pid =
        left_pid_.update(result.target.left_mps,
                         input.state->left_velocity_mps, input.dt_s,
                         result.feedforward.left_V);
    result.right_pid =
        right_pid_.update(result.target.right_mps,
                          input.state->right_velocity_mps, input.dt_s,
                          result.feedforward.right_V);
    if (!result.left_pid.valid || !result.right_pid.valid) {
      result.reject_bits |= kChassisVelocityControllerInvalid;
      reset();
      return result;
    }
    const double left_V = std::clamp(result.left_pid.output,
                                     -config_.max_output_voltage_V,
                                     config_.max_output_voltage_V);
    const double right_V = std::clamp(result.right_pid.output,
                                      -config_.max_output_voltage_V,
                                      config_.max_output_voltage_V);
    if (left_V != result.left_pid.output || right_V != result.right_pid.output)
      result.applied_limits |= kChassisVelocityVoltageClamp;
    result.request.h = input.output_header;
    result.request.source = RequestSource::FutureAutonomy;
    result.request.owner = selected_request.owner;
    result.request.ttl_us = config_.output_request_ttl_us;
    result.request.payload = WheelVoltagePayload{left_V, right_V};
    result.has_request = true;
    last_target_ = result.target;
    have_target_ = true;
    return result;
  }

  void reset() noexcept {
    left_pid_.reset();
    right_pid_.reset();
    last_target_ = {};
    epoch_ = 0;
    initialized_ = false;
    have_target_ = false;
  }

 private:
  double limitAcceleration(double acceleration, std::uint32_t flag,
                           std::uint32_t& applied_limits) const noexcept {
    const double limited =
        std::clamp(acceleration, -config_.max_target_acceleration_mps2,
                   config_.max_target_acceleration_mps2);
    if (limited != acceleration) applied_limits |= flag;
    return limited;
  }

  ChassisVelocityControllerConfig config_{};
  DifferentialKinematics kinematics_;
  EngineeringPid left_pid_;
  EngineeringPid right_pid_;
  WheelSpeeds last_target_{};
  std::uint32_t epoch_{};
  bool initialized_{};
  bool have_target_{};
};

}  // namespace robot
