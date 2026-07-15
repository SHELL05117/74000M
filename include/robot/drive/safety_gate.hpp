#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <variant>

#include "robot/drive/actuator_frame.hpp"
#include "robot/drive/output_slew.hpp"
#include "robot/runtime/mode.hpp"

namespace robot {

enum AppliedLimit : std::uint32_t {
  kAppliedNone = 0,
  kAppliedProportionalDesaturation = 1u << 0,
  kAppliedPreserveTurn = 1u << 1,
  kAppliedDerate = 1u << 2,
  kAppliedFinalClamp = 1u << 3,
  kAppliedInvalidRequestStop = 1u << 4,
  kAppliedDisabledStop = 1u << 5,
  kAppliedUnsupportedStop = 1u << 6,
  kAppliedNoRequestStop = 1u << 7,
};

struct SafetyGateConfig {
  double max_command_voltage_V{};
  TimeUs max_request_ttl_us{};
  OutputSlewConfig output_slew{};
  StopMode disabled_stop_mode{StopMode::Brake};
  StopMode fault_stop_mode{StopMode::Brake};
  StopMode no_request_stop_mode{StopMode::Brake};
};

struct SafetyGateInput {
  FrameHeader output_header{};
  ModeSnapshot mode{};
  TimeUs now_us{};
  double math_dt_s{};
  double output_derate{};
  DriveCapabilities capabilities{};
};

class SafetyGate {
 public:
  explicit SafetyGate(SafetyGateConfig config) noexcept
      : config_(config), output_slew_(config.output_slew) {}

  bool valid() const noexcept {
    return std::isfinite(config_.max_command_voltage_V) &&
           config_.max_command_voltage_V > 0.0 &&
           config_.max_command_voltage_V <= 12.0 &&
           config_.max_request_ttl_us > 0 && output_slew_.valid();
  }

  ActuatorFrame apply(const DriveRequest* request,
                      const SafetyGateInput& input) noexcept {
    if (!valid() || !validInput(input)) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, config_.fault_stop_mode,
                       kAppliedInvalidRequestStop);
    }
    if (!input.mode.enabled) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, config_.disabled_stop_mode,
                       kAppliedDisabledStop);
    }
    if (request == nullptr) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, config_.no_request_stop_mode,
                       kAppliedNoRequestStop);
    }
    if (!validRequest(*request, input)) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, config_.fault_stop_mode,
                       kAppliedInvalidRequestStop);
    }
    if (!supportedPayload(request->payload, input.capabilities,
                          request->source) ||
        std::holds_alternative<ChassisVelocityPayload>(request->payload)) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, config_.fault_stop_mode,
                       kAppliedUnsupportedStop);
    }

    if (const auto* brake = std::get_if<BrakePayload>(&request->payload)) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, brake->mode, kAppliedNone);
    }

    const double allowed_V =
        config_.max_command_voltage_V * input.output_derate;
    if (!validVoltageLimit(allowed_V)) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, config_.fault_stop_mode,
                       kAppliedInvalidRequestStop);
    }

    WheelVoltages desired{};
    AllocationPolicy policy{AllocationPolicy::RatioPreserving};
    if (!mapRequest(*request, desired, policy)) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, config_.fault_stop_mode,
                       kAppliedInvalidRequestStop);
    }

    const VoltageAllocation allocation =
        allocateVoltage(desired, allowed_V, policy);
    if (!allocation.valid) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, config_.fault_stop_mode,
                       kAppliedInvalidRequestStop);
    }

    if (!slew_initialized_ || slew_epoch_ != input.mode.epoch) {
      output_slew_.reset();
      slew_initialized_ = true;
      slew_epoch_ = input.mode.epoch;
    }
    WheelVoltages slewed{};
    if (!output_slew_.update(allocation.output, input.math_dt_s, slewed)) {
      resetForStop(input.mode.epoch);
      return stopFrame(input.output_header, config_.fault_stop_mode,
                       kAppliedInvalidRequestStop);
    }

    ActuatorFrame frame{};
    frame.h = input.output_header;
    frame.left_V = std::clamp(slewed.left_V, -allowed_V, allowed_V);
    frame.right_V = std::clamp(slewed.right_V, -allowed_V, allowed_V);
    frame.zero_behavior = StopMode::Brake;
    frame.owner = request->source;
    frame.owner_id = request->owner.command_id;
    frame.owner_lease = request->owner.lease_generation;
    if (allocation.limited) {
      frame.applied_limits |=
          policy == AllocationPolicy::RatioPreserving
              ? kAppliedProportionalDesaturation
              : kAppliedPreserveTurn;
    }
    if (input.output_derate < 1.0) frame.applied_limits |= kAppliedDerate;
    if (frame.left_V != slewed.left_V || frame.right_V != slewed.right_V)
      frame.applied_limits |= kAppliedFinalClamp;
    return frame;
  }

  void reset() noexcept {
    output_slew_.reset();
    slew_initialized_ = false;
    slew_epoch_ = 0;
  }

 private:
  static ActuatorFrame stopFrame(const FrameHeader& header, StopMode mode,
                                 std::uint32_t reason) noexcept {
    ActuatorFrame frame{};
    frame.h = header;
    frame.left_V = 0.0;
    frame.right_V = 0.0;
    frame.zero_behavior = mode;
    frame.owner = RequestSource::Safety;
    frame.applied_limits = reason;
    return frame;
  }

  bool validInput(const SafetyGateInput& input) const noexcept {
    return input.output_header.mode_epoch == input.mode.epoch &&
           input.output_header.time_us <= input.now_us &&
           std::isfinite(input.math_dt_s) && input.math_dt_s >= 0.0 &&
           input.math_dt_s <= config_.output_slew.max_dt_s &&
           std::isfinite(input.output_derate) &&
           input.output_derate >= 0.0 && input.output_derate <= 1.0;
  }

  bool validRequest(const DriveRequest& request,
                    const SafetyGateInput& input) const noexcept {
    if (!finitePayload(request.payload) ||
        request.h.mode_epoch != input.mode.epoch ||
        request.owner.mode_epoch != input.mode.epoch ||
        request.h.time_us > input.now_us || request.ttl_us == 0 ||
        request.ttl_us > config_.max_request_ttl_us ||
        input.now_us - request.h.time_us > request.ttl_us ||
        request.source == RequestSource::None) {
      return false;
    }
    if (request.source != RequestSource::Safety &&
        (request.owner.command_id == 0 ||
         (request.owner.requirements & Requirement::kDrivetrain) == 0)) {
      return false;
    }
    switch (request.source) {
      case RequestSource::Driver:
        return input.mode.mode == CompetitionMode::Driver;
      case RequestSource::FutureAutonomy:
        return input.mode.mode == CompetitionMode::AutonomousInterface;
      case RequestSource::Test:
        return input.mode.mode == CompetitionMode::Test;
      case RequestSource::Safety:
        return true;
      case RequestSource::None:
        return false;
    }
    return false;
  }

  bool mapRequest(const DriveRequest& request, WheelVoltages& desired,
                  AllocationPolicy& policy) const noexcept {
    if (const auto* wheel =
            std::get_if<WheelVoltagePayload>(&request.payload)) {
      desired = {wheel->left_V, wheel->right_V};
      policy = AllocationPolicy::RatioPreserving;
      return true;
    }
    const auto* driver =
        std::get_if<DriverCurvaturePayload>(&request.payload);
    if (driver == nullptr) return false;

    const double common =
        driver->forward * config_.max_command_voltage_V;
    double differential{};
    switch (driver->steering_mode) {
      case DriverSteeringMode::Curvature:
        differential = std::abs(driver->forward) * driver->steering *
                       config_.max_command_voltage_V;
        break;
      case DriverSteeringMode::QuickTurn:
      case DriverSteeringMode::HeadingAssist:
        differential =
            driver->steering * config_.max_command_voltage_V;
        break;
    }
    desired = {common - differential, common + differential};
    policy = driver->allocation;
    return std::isfinite(desired.left_V) &&
           std::isfinite(desired.right_V);
  }

  void resetForStop(std::uint32_t epoch) noexcept {
    output_slew_.reset();
    slew_initialized_ = false;
    slew_epoch_ = epoch;
  }

  SafetyGateConfig config_{};
  DifferentialOutputSlew output_slew_;
  bool slew_initialized_{};
  std::uint32_t slew_epoch_{};
};

}  // namespace robot
