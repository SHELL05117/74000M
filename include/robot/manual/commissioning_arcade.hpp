#pragma once

#include <cmath>
#include <cstdint>

#include "robot/commands/drive_request_arbiter.hpp"
#include "robot/commands/request_sink.hpp"
#include "robot/drive/safety_gate.hpp"
#include "robot/manual/input_shaping.hpp"
#include "robot/runtime/control_loop.hpp"

namespace robot {

enum class CommissioningDriveState : std::uint8_t {
  Locked,
  HoldingArmChord,
  AwaitingChordRelease,
  Armed,
  LatchedStop,
};

struct CommissioningArcadeConfig {
  AxisShapeConfig throttle_shape{};
  AxisShapeConfig turn_shape{};
  double throttle_rise_per_s{};
  double throttle_fall_per_s{};
  double turn_rise_per_s{};
  double turn_fall_per_s{};
  double max_dt_s{};
  double max_voltage_V{};
  TimeUs request_ttl_us{};
  TimeUs arm_hold_us{};
  double arm_neutral_threshold{};
  std::uint32_t arm_chord{};
  std::uint32_t emergency_stop_button{};
  OutputSlewConfig output_slew{};
};

inline CommissioningArcadeConfig make1690XCommissioningArcadeConfig() {
  CommissioningArcadeConfig config{};
  config.throttle_shape = {0.0, 0.06, 0.15};
  config.turn_shape = {0.0, 0.06, 0.15};
  config.throttle_rise_per_s = 3.0;
  config.throttle_fall_per_s = 6.0;
  config.turn_rise_per_s = 4.0;
  config.turn_fall_per_s = 8.0;
  config.max_dt_s = 0.05;
  config.max_voltage_V = 4.0;
  config.request_ttl_us = 30000;
  config.arm_hold_us = 1000000;
  config.arm_neutral_threshold = 0.05;
  config.arm_chord = kButtonL1 | kButtonL2 | kButtonR1 | kButtonR2;
  config.emergency_stop_button = kButtonB;
  config.output_slew = {16.0, 32.0, 0.05};
  return config;
}

inline bool validCommissioningArcadeConfig(
    const CommissioningArcadeConfig& config) noexcept {
  return validAxisShape(config.throttle_shape) &&
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
         std::isfinite(config.max_voltage_V) &&
         config.max_voltage_V > 0.0 && config.max_voltage_V <= 4.0 &&
         config.request_ttl_us > 0 && config.arm_hold_us > 0 &&
         std::isfinite(config.arm_neutral_threshold) &&
         config.arm_neutral_threshold >= 0.0 &&
         config.arm_neutral_threshold < 1.0 && config.arm_chord != 0 &&
         config.emergency_stop_button != 0 &&
         (config.arm_chord & config.emergency_stop_button) == 0 &&
         AsymmetricVoltageSlew(config.output_slew).valid();
}

struct CommissioningArcadeResult {
  DriveRequest request{};
  CommissioningDriveState state{CommissioningDriveState::Locked};
  double shaped_throttle{};
  double shaped_turn{};
  bool valid{};
};

class CommissioningArcadeMapper {
 public:
  explicit CommissioningArcadeMapper(
      CommissioningArcadeConfig config) noexcept
      : config_(config),
        throttle_slew_(config.throttle_rise_per_s,
                       config.throttle_fall_per_s, config.max_dt_s),
        turn_slew_(config.turn_rise_per_s, config.turn_fall_per_s,
                   config.max_dt_s) {}

  CommissioningArcadeResult update(
      const FrameHeader& header, const ModeSnapshot& mode,
      const ControllerSnapshot& controller, double dt_s,
      const OwnerToken& owner) noexcept {
    CommissioningArcadeResult result{};
    if (header.mode_epoch != epoch_) reset(header.mode_epoch);

    const bool valid_frame =
        controller.h.time_us == header.time_us &&
        controller.h.sequence == header.sequence &&
        controller.h.mode_epoch == header.mode_epoch;
    const bool valid_owner =
        owner.command_id != 0 &&
        (owner.requirements & Requirement::kDrivetrain) != 0 &&
        owner.mode_epoch == header.mode_epoch;
    const bool valid_mode = mode.enabled &&
                            mode.mode == CompetitionMode::Test &&
                            !mode.field_connected &&
                            mode.epoch == header.mode_epoch;
    if (!validCommissioningArcadeConfig(config_) || !valid_frame ||
        !valid_owner || !valid_mode || !controller.connected ||
        !controller.api_ok || !std::isfinite(dt_s) || dt_s < 0.0 ||
        dt_s > config_.max_dt_s) {
      if (state_ != CommissioningDriveState::LatchedStop)
        reset(header.mode_epoch);
      result.state = state_;
      return result;
    }

    if ((controller.buttons & config_.emergency_stop_button) != 0) {
      latchStop();
      result.state = state_;
      return result;
    }
    if (state_ == CommissioningDriveState::LatchedStop) {
      result.state = state_;
      return result;
    }

    const bool neutral =
        std::abs(controller.left_y) <= config_.arm_neutral_threshold &&
        std::abs(controller.left_x) <= config_.arm_neutral_threshold;
    const bool chord_held =
        (controller.buttons & config_.arm_chord) == config_.arm_chord;
    const bool chord_released =
        (controller.buttons & config_.arm_chord) == 0;

    if (state_ == CommissioningDriveState::Locked ||
        state_ == CommissioningDriveState::HoldingArmChord) {
      if (!neutral || !chord_held) {
        state_ = CommissioningDriveState::Locked;
        arm_start_us_ = 0;
        arm_timer_started_ = false;
        resetSlew();
        result.state = state_;
        return result;
      }
      if (!arm_timer_started_) {
        arm_start_us_ = header.time_us;
        arm_timer_started_ = true;
      }
      state_ = CommissioningDriveState::HoldingArmChord;
      if (header.time_us - arm_start_us_ < config_.arm_hold_us) {
        result.state = state_;
        return result;
      }
      state_ = CommissioningDriveState::AwaitingChordRelease;
    }

    if (state_ == CommissioningDriveState::AwaitingChordRelease) {
      if (!neutral) {
        reset(header.mode_epoch);
        result.state = state_;
        return result;
      }
      if (!chord_released) {
        result.state = state_;
        return result;
      }
      state_ = CommissioningDriveState::Armed;
      resetSlew();
    }

    const double centered_throttle =
        centeredAxis(controller.left_y, config_.throttle_shape);
    const double centered_turn =
        centeredAxis(controller.left_x, config_.turn_shape);
    const double throttle_target =
        shapeCenteredAxis(centered_throttle, config_.throttle_shape);
    const double turn_target =
        shapeCenteredAxis(centered_turn, config_.turn_shape);
    if (!throttle_slew_.update(throttle_target, dt_s,
                               result.shaped_throttle) ||
        !turn_slew_.update(turn_target, dt_s, result.shaped_turn)) {
      latchStop();
      result.state = state_;
      return result;
    }

    // Single-left-stick Arcade: positive Left X requests a right turn.
    const WheelVoltages normalized{
        result.shaped_throttle + result.shaped_turn,
        result.shaped_throttle - result.shaped_turn};
    const VoltageAllocation allocation =
        desaturateProportional(normalized, 1.0);
    if (!allocation.valid) {
      latchStop();
      result.state = state_;
      return result;
    }

    result.request.h = header;
    result.request.source = RequestSource::Test;
    result.request.owner = owner;
    result.request.ttl_us = config_.request_ttl_us;
    result.request.payload = WheelVoltagePayload{
        allocation.output.left_V * config_.max_voltage_V,
        allocation.output.right_V * config_.max_voltage_V};
    result.valid = finitePayload(result.request.payload);
    if (!result.valid) latchStop();
    result.state = state_;
    return result;
  }

  void reset(std::uint32_t epoch = 0) noexcept {
    epoch_ = epoch;
    state_ = CommissioningDriveState::Locked;
    arm_start_us_ = 0;
    arm_timer_started_ = false;
    resetSlew();
  }

  CommissioningDriveState state() const noexcept { return state_; }
  bool latchedStop() const noexcept {
    return state_ == CommissioningDriveState::LatchedStop;
  }

 private:
  void resetSlew() noexcept {
    throttle_slew_.reset();
    turn_slew_.reset();
  }

  void latchStop() noexcept {
    state_ = CommissioningDriveState::LatchedStop;
    resetSlew();
  }

  CommissioningArcadeConfig config_{};
  AsymmetricAxisSlew throttle_slew_;
  AsymmetricAxisSlew turn_slew_;
  TimeUs arm_start_us_{};
  std::uint32_t epoch_{};
  CommissioningDriveState state_{CommissioningDriveState::Locked};
  bool arm_timer_started_{};
};

class CommissioningDriveLeaseCommand final : public Command {
 public:
  CommandId id() const noexcept override { return 0x1690; }
  RequirementMask requirements() const noexcept override {
    return Requirement::kDrivetrain;
  }
  bool allowedInMode(CompetitionMode mode) const noexcept override {
    return mode == CompetitionMode::Test;
  }
  void initialize(const CommandContext&, const OwnerToken& owner) noexcept
      override {
    owner_ = owner;
    active_ = true;
  }
  CommandRunState execute(const CommandContext&,
                          const OwnerToken&) noexcept override {
    return CommandRunState::Running;
  }
  void end(const CommandContext&, const OwnerToken&,
           CommandEndReason) noexcept override {
    owner_ = {};
    active_ = false;
  }

  const OwnerToken& owner() const noexcept { return owner_; }
  bool active() const noexcept { return active_; }

 private:
  OwnerToken owner_{};
  bool active_{};
};

// Restricted physical commissioning cycle. Official RobotCapabilities remain
// false; only a bounded RequestSource::Test voltage payload is enabled here.
class CommissioningControlCycle final : public ControlCycle {
 public:
  explicit CommissioningControlCycle(CommissioningArcadeConfig config)
      : config_(config),
        mapper_(config),
        arbiter_({config.request_ttl_us}),
        safety_gate_(makeSafetyConfig(config)) {
    capabilities_.controlled_test_voltage = true;
  }

  ActuatorFrame update(const FrameHeader& header, const ModeSnapshot& mode,
                       const RawDriveInputs&,
                       const ControllerSnapshot& controller,
                       const TimingSample& timing) override {
    state_.h = header;
    state_.competition = mode;
    state_.controller_connected = controller.connected;
    const CommandContext context{header, mode, &state_, timing.math_dt_s};

    scheduler_.tick(context);
    const bool controller_ready = controller.connected && controller.api_ok;
    if (!controller_ready || !mode.enabled ||
        mode.mode != CompetitionMode::Test || mode.field_connected ||
        mapper_.latchedStop()) {
      scheduler_.cancelAll(context, CommandEndReason::Cancelled);
    } else if (scheduler_.activeCount() == 0) {
      scheduler_.schedule(lease_command_, context,
                          ConflictPolicy::RejectIncoming);
    }

    sink_.beginFrame(header);
    const CommissioningArcadeResult mapped = mapper_.update(
        header, mode, controller, timing.math_dt_s,
        lease_command_.owner());
    if (mapper_.latchedStop() || !controller_ready) {
      scheduler_.cancelAll(context, CommandEndReason::Cancelled);
    } else if (mapped.valid && lease_command_.active()) {
      sink_.publish(mapped.request, scheduler_);
    }

    DriveRequest request{};
    const bool request_present = sink_.read(request);
    std::array<DriveRequestCandidate, 1> candidates{{
        {request, request_present},
    }};
    const ArbitrationResult selected = arbiter_.select(
        candidates, mode, header.time_us, capabilities_, scheduler_);
    const DriveRequest* selected_request =
        selected.has_selection ? &selected.selected : nullptr;
    const SafetyGateInput gate_input{header, mode, header.time_us,
                                     timing.math_dt_s, 1.0,
                                     capabilities_};
    return safety_gate_.apply(selected_request, gate_input);
  }

  CommissioningDriveState state() const noexcept { return mapper_.state(); }

 private:
  static SafetyGateConfig makeSafetyConfig(
      const CommissioningArcadeConfig& config) noexcept {
    return {config.max_voltage_V,
            config.request_ttl_us,
            config.output_slew,
            StopMode::Brake,
            StopMode::Brake,
            StopMode::Brake};
  }

  CommissioningArcadeConfig config_{};
  CommissioningArcadeMapper mapper_;
  StaticScheduler<1> scheduler_{Requirement::kDrivetrain};
  CommissioningDriveLeaseCommand lease_command_{};
  DriveRequestSink sink_{};
  DriveRequestArbiter arbiter_;
  SafetyGate safety_gate_;
  DriveCapabilities capabilities_{};
  RobotState state_{};
};

}  // namespace robot
