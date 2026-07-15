#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robot/drive/drive_request.hpp"
#include "robot/manual/heading_assist.hpp"
#include "robot/manual/input_shaping.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/state/controller_snapshot.hpp"
#include "robot/state/robot_state.hpp"

namespace robot {

enum class ControllerAxis : std::uint8_t {
  LeftX,
  LeftY,
  RightX,
  RightY,
};

enum ManualReject : std::uint32_t {
  kManualAccepted = 0,
  kManualBadConfig = 1u << 0,
  kManualWrongMode = 1u << 1,
  kManualControllerDisconnected = 1u << 2,
  kManualControllerApi = 1u << 3,
  kManualFrameMismatch = 1u << 4,
  kManualNonfiniteInput = 1u << 5,
  kManualOwnerInvalid = 1u << 6,
};

struct ManualDriveConfig {
  ControllerAxis throttle_axis{ControllerAxis::RightY};
  ControllerAxis turn_axis{ControllerAxis::RightX};
  AxisShapeConfig throttle_shape{};
  AxisShapeConfig turn_shape{};
  double throttle_rise_per_s{};
  double throttle_fall_per_s{};
  double turn_rise_per_s{};
  double turn_fall_per_s{};
  double max_dt_s{};
  double curvature_gain{};
  double quick_turn_gain{};
  double quick_turn_max_throttle{};
  double quick_turn_max_speed_mps{};
  std::uint32_t quick_turn_button{};
  TimeUs state_ttl_us{};
  TimeUs request_ttl_us{};
  HeadingAssistConfig heading{};
};

inline bool validManualDriveConfig(const ManualDriveConfig& config) noexcept {
  return config.throttle_axis != config.turn_axis &&
         validAxisShape(config.throttle_shape) &&
         validAxisShape(config.turn_shape) &&
         std::isfinite(config.throttle_rise_per_s) &&
         config.throttle_rise_per_s > 0.0 &&
         std::isfinite(config.throttle_fall_per_s) &&
         config.throttle_fall_per_s > 0.0 &&
         std::isfinite(config.turn_rise_per_s) &&
         config.turn_rise_per_s > 0.0 &&
         std::isfinite(config.turn_fall_per_s) &&
         config.turn_fall_per_s > 0.0 &&
         std::isfinite(config.max_dt_s) && config.max_dt_s > 0.0 &&
         std::isfinite(config.curvature_gain) &&
         config.curvature_gain > 0.0 && config.curvature_gain <= 1.0 &&
         std::isfinite(config.quick_turn_gain) &&
         config.quick_turn_gain > 0.0 && config.quick_turn_gain <= 1.0 &&
         std::isfinite(config.quick_turn_max_throttle) &&
         config.quick_turn_max_throttle >= 0.0 &&
         config.quick_turn_max_throttle <= 1.0 &&
         std::isfinite(config.quick_turn_max_speed_mps) &&
         config.quick_turn_max_speed_mps > 0.0 &&
         config.quick_turn_button != 0 && config.state_ttl_us > 0 &&
         config.request_ttl_us > 0 && validHeadingAssistConfig(config.heading);
}

struct ManualDriveDiagnostics {
  double raw_throttle{};
  double raw_turn{};
  double centered_throttle{};
  double centered_turn{};
  double shaped_throttle{};
  double shaped_turn{};
  double slewed_throttle{};
  double slewed_turn{};
  bool quick_turn_requested{};
  bool quick_turn_active{};
  HeadingAssistOutput heading{};
  std::uint32_t reject_bits{};
};

struct ManualDriveResult {
  DriveRequest request{};
  ManualDriveDiagnostics diagnostics{};
  bool valid{};
};

class ManualDrive {
 public:
  explicit ManualDrive(ManualDriveConfig config) noexcept
      : config_(config),
        throttle_slew_(config.throttle_rise_per_s,
                       config.throttle_fall_per_s, config.max_dt_s),
        turn_slew_(config.turn_rise_per_s, config.turn_fall_per_s,
                   config.max_dt_s),
        heading_assist_(config.heading) {}

  bool valid() const noexcept { return validManualDriveConfig(config_); }

  ManualDriveResult update(const FrameHeader& header,
                           const ModeSnapshot& mode,
                           const ControllerSnapshot& controller,
                           const RobotState& state, double dt_s,
                           const OwnerToken& owner) noexcept {
    ManualDriveResult result{};
    if (!valid()) result.diagnostics.reject_bits |= kManualBadConfig;
    if (!mode.enabled || mode.mode != CompetitionMode::Driver)
      result.diagnostics.reject_bits |= kManualWrongMode;
    if (!controller.connected)
      result.diagnostics.reject_bits |= kManualControllerDisconnected;
    if (!controller.api_ok)
      result.diagnostics.reject_bits |= kManualControllerApi;
    if (controller.h.time_us != header.time_us ||
        controller.h.sequence != header.sequence ||
        controller.h.mode_epoch != header.mode_epoch ||
        mode.epoch != header.mode_epoch)
      result.diagnostics.reject_bits |= kManualFrameMismatch;
    if (owner.command_id == 0 ||
        (owner.requirements & Requirement::kDrivetrain) == 0 ||
        owner.mode_epoch != header.mode_epoch)
      result.diagnostics.reject_bits |= kManualOwnerInvalid;

    result.diagnostics.raw_throttle = axis(controller, config_.throttle_axis);
    result.diagnostics.raw_turn = axis(controller, config_.turn_axis);
    if (!std::isfinite(result.diagnostics.raw_throttle) ||
        !std::isfinite(result.diagnostics.raw_turn) ||
        !std::isfinite(dt_s) || dt_s < 0.0 || dt_s > config_.max_dt_s) {
      result.diagnostics.reject_bits |= kManualNonfiniteInput;
    }
    if (result.diagnostics.reject_bits != kManualAccepted) {
      reset(header.mode_epoch);
      return result;
    }

    if (!initialized_ || epoch_ != header.mode_epoch) {
      reset(header.mode_epoch);
      initialized_ = true;
    }
    result.diagnostics.centered_throttle =
        centeredAxis(result.diagnostics.raw_throttle, config_.throttle_shape);
    result.diagnostics.centered_turn =
        centeredAxis(result.diagnostics.raw_turn, config_.turn_shape);
    result.diagnostics.shaped_throttle = shapeCenteredAxis(
        result.diagnostics.centered_throttle, config_.throttle_shape);
    result.diagnostics.shaped_turn =
        shapeCenteredAxis(result.diagnostics.centered_turn,
                          config_.turn_shape);
    if (!throttle_slew_.update(result.diagnostics.shaped_throttle, dt_s,
                               result.diagnostics.slewed_throttle) ||
        !turn_slew_.update(result.diagnostics.shaped_turn, dt_s,
                           result.diagnostics.slewed_turn)) {
      result.diagnostics.reject_bits |= kManualNonfiniteInput;
      reset(header.mode_epoch);
      return result;
    }

    const bool state_fresh =
        state.h.mode_epoch == header.mode_epoch &&
        state.h.time_us <= header.time_us &&
        header.time_us - state.h.time_us <= config_.state_ttl_us;
    result.diagnostics.quick_turn_requested =
        (controller.buttons & config_.quick_turn_button) != 0;
    result.diagnostics.quick_turn_active =
        result.diagnostics.quick_turn_requested && state_fresh &&
        state.translation_quality != Quality::Invalid &&
        std::abs(result.diagnostics.slewed_throttle) <=
            config_.quick_turn_max_throttle &&
        std::abs(state.body_velocity.vx_mps) <=
            config_.quick_turn_max_speed_mps;

    result.diagnostics.heading = heading_assist_.update(
        {result.diagnostics.centered_turn,
         result.diagnostics.shaped_turn,
         result.diagnostics.slewed_throttle,
         state.pose.theta_rad,
         state.body_velocity.omega_radps,
         state.body_velocity.vx_mps,
         state.heading_quality,
         state.translation_quality,
         result.diagnostics.quick_turn_active,
         state_fresh,
         dt_s});

    DriverCurvaturePayload payload{};
    payload.forward = result.diagnostics.slewed_throttle;
    if (result.diagnostics.heading.active) {
      payload.steering = result.diagnostics.heading.turn;
      payload.steering_mode = DriverSteeringMode::HeadingAssist;
      payload.allocation = AllocationPolicy::PreserveTurn;
    } else if (result.diagnostics.quick_turn_active) {
      payload.steering = std::clamp(
          result.diagnostics.slewed_turn * config_.quick_turn_gain, -1.0,
          1.0);
      payload.steering_mode = DriverSteeringMode::QuickTurn;
      payload.allocation = AllocationPolicy::RatioPreserving;
    } else {
      payload.steering = std::clamp(
          result.diagnostics.slewed_turn * config_.curvature_gain, -1.0,
          1.0);
      payload.steering_mode = DriverSteeringMode::Curvature;
      payload.allocation = AllocationPolicy::RatioPreserving;
    }

    result.request.h = header;
    result.request.source = RequestSource::Driver;
    result.request.owner = owner;
    result.request.ttl_us = config_.request_ttl_us;
    result.request.payload = payload;
    result.valid = finitePayload(result.request.payload);
    if (!result.valid) {
      result.diagnostics.reject_bits |= kManualNonfiniteInput;
      reset(header.mode_epoch);
    }
    return result;
  }

  void reset(std::uint32_t epoch = 0) noexcept {
    throttle_slew_.reset();
    turn_slew_.reset();
    heading_assist_.reset();
    epoch_ = epoch;
    initialized_ = false;
  }

 private:
  static double axis(const ControllerSnapshot& controller,
                     ControllerAxis selected) noexcept {
    switch (selected) {
      case ControllerAxis::LeftX:
        return controller.left_x;
      case ControllerAxis::LeftY:
        return controller.left_y;
      case ControllerAxis::RightX:
        return controller.right_x;
      case ControllerAxis::RightY:
        return controller.right_y;
    }
    return 0.0;
  }

  ManualDriveConfig config_{};
  AsymmetricAxisSlew throttle_slew_;
  AsymmetricAxisSlew turn_slew_;
  HeadingAssist heading_assist_;
  std::uint32_t epoch_{};
  bool initialized_{};
};

}  // namespace robot
