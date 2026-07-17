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
  Driving,
  Coasting,
};

inline constexpr StopMode kCommissioningStopMode = StopMode::Coast;

struct CommissioningCurvatureConfig {
  AxisShapeConfig throttle_shape{};
  AxisShapeConfig turn_shape{};
  double throttle_rise_per_s{};
  double throttle_fall_per_s{};
  double turn_rise_per_s{};
  double turn_fall_per_s{};
  double max_dt_s{};
  double curvature_gain{};
  double quick_turn_gain{};
  double quick_turn_enter_throttle{};
  double quick_turn_exit_throttle{};
  double max_voltage_V{};
  TimeUs request_ttl_us{};
  std::uint32_t coast_button{};
  OutputSlewConfig output_slew{};
};

inline CommissioningCurvatureConfig
make1690XCommissioningCurvatureConfig() {
  CommissioningCurvatureConfig config{};
  // Match the original 1690X driver's linear throttle response while keeping
  // a remapped neutral deadband. Turn retains mild cubic shaping because
  // Curvature already provides the requested high-speed steering semantics.
  config.throttle_shape = {0.0, 0.06, 0.0};
  config.turn_shape = {0.0, 0.06, 0.15};
  config.throttle_rise_per_s = 100.0;
  config.throttle_fall_per_s = 100.0;
  config.turn_rise_per_s = 100.0;
  config.turn_fall_per_s = 100.0;
  config.max_dt_s = 0.05;
  config.curvature_gain = 1.0;
  config.quick_turn_gain = 1.0;
  config.quick_turn_enter_throttle = 0.15;
  config.quick_turn_exit_throttle = 0.25;
  config.max_voltage_V = 12.0;
  config.request_ttl_us = 30000;
  config.coast_button = kButtonB;
  // Aggressive HIL candidate: throttle, turn, and final voltage can reach the
  // legal full-scale command in one nominal 10 ms frame while retaining a
  // finite Slew boundary and the hard 12 V command ceiling.
  config.output_slew = {1200.0, 1200.0, 0.05};
  return config;
}

inline bool validCommissioningCurvatureConfig(
    const CommissioningCurvatureConfig& config) noexcept {
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
         std::isfinite(config.curvature_gain) &&
         config.curvature_gain > 0.0 && config.curvature_gain <= 1.0 &&
         std::isfinite(config.quick_turn_gain) &&
         config.quick_turn_gain > 0.0 && config.quick_turn_gain <= 1.0 &&
         std::isfinite(config.quick_turn_enter_throttle) &&
         config.quick_turn_enter_throttle >= 0.0 &&
         std::isfinite(config.quick_turn_exit_throttle) &&
         config.quick_turn_exit_throttle >
             config.quick_turn_enter_throttle &&
         config.quick_turn_exit_throttle <= 1.0 &&
         std::isfinite(config.max_voltage_V) &&
         config.max_voltage_V > 0.0 && config.max_voltage_V <= 12.0 &&
         config.request_ttl_us > 0 && config.coast_button != 0 &&
         AsymmetricVoltageSlew(config.output_slew).valid();
}

struct CommissioningCurvatureResult {
  DriveRequest request{};
  CommissioningDriveState state{CommissioningDriveState::Coasting};
  double shaped_throttle{};
  double shaped_turn{};
  bool quick_turn_active{};
  bool valid{};
};

class CommissioningCurvatureMapper {
 public:
  explicit CommissioningCurvatureMapper(
      CommissioningCurvatureConfig config) noexcept
      : config_(config),
        throttle_slew_(config.throttle_rise_per_s,
                       config.throttle_fall_per_s, config.max_dt_s),
        turn_slew_(config.turn_rise_per_s, config.turn_fall_per_s,
                   config.max_dt_s) {}

  CommissioningCurvatureResult update(
      const FrameHeader& header, const ModeSnapshot& mode,
      const ControllerSnapshot& controller, double dt_s,
      const OwnerToken& owner) noexcept {
    CommissioningCurvatureResult result{};
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
    if (!validCommissioningCurvatureConfig(config_) || !valid_frame ||
        !valid_owner || !valid_mode || !controller.connected ||
        !controller.api_ok || !std::isfinite(dt_s) || dt_s < 0.0 ||
        dt_s > config_.max_dt_s) {
      reset(header.mode_epoch);
      result.state = state_;
      return result;
    }

    if ((controller.buttons & config_.coast_button) != 0) {
      resetControlState();
      state_ = CommissioningDriveState::Coasting;
      result.request = makeCoastRequest(header, owner);
      result.valid = true;
      result.state = state_;
      return result;
    }

    const double centered_throttle =
        centeredAxis(controller.left_y, config_.throttle_shape);
    const double centered_turn =
        centeredAxis(controller.left_x, config_.turn_shape);
    const double throttle_target =
        shapeCenteredAxis(centered_throttle, config_.throttle_shape);
    const double turn_target =
        shapeCenteredAxis(centered_turn, config_.turn_shape);
    if (throttle_target == 0.0 && turn_target == 0.0) {
      resetControlState();
      state_ = CommissioningDriveState::Coasting;
      result.request = makeCoastRequest(header, owner);
      result.valid = true;
      result.state = state_;
      return result;
    }
    if (!throttle_slew_.update(throttle_target, dt_s,
                               result.shaped_throttle) ||
        !turn_slew_.update(turn_target, dt_s, result.shaped_turn)) {
      resetSlew();
      state_ = CommissioningDriveState::Coasting;
      result.state = state_;
      return result;
    }

    const double throttle_magnitude = std::abs(throttle_target);
    if (turn_target == 0.0) {
      quick_turn_active_ = false;
    } else if (quick_turn_active_) {
      if (throttle_magnitude >= config_.quick_turn_exit_throttle)
        quick_turn_active_ = false;
    } else if (throttle_magnitude <=
               config_.quick_turn_enter_throttle) {
      quick_turn_active_ = true;
    }
    result.quick_turn_active = quick_turn_active_;

    // Single-left-stick Curvature: normal steering scales with |throttle|.
    // Positive Left X requests a right turn. Low commanded throttle and a
    // nonzero turn command automatically select full-gain Quick Turn.
    const double differential_turn = std::clamp(
        result.shaped_turn *
            (result.quick_turn_active
                 ? config_.quick_turn_gain
                 : std::abs(result.shaped_throttle) *
                       config_.curvature_gain),
        -1.0, 1.0);
    const WheelVoltages normalized{
        result.shaped_throttle + differential_turn,
        result.shaped_throttle - differential_turn};
    const VoltageAllocation allocation =
        desaturateProportional(normalized, 1.0);
    if (!allocation.valid) {
      resetControlState();
      state_ = CommissioningDriveState::Coasting;
      result.state = state_;
      return result;
    }

    if (allocation.output.left_V == 0.0 &&
        allocation.output.right_V == 0.0) {
      state_ = CommissioningDriveState::Coasting;
      result.request = makeCoastRequest(header, owner);
      result.valid = true;
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
    state_ = result.valid ? CommissioningDriveState::Driving
                          : CommissioningDriveState::Coasting;
    if (!result.valid) resetSlew();
    result.state = state_;
    return result;
  }

  void reset(std::uint32_t epoch = 0) noexcept {
    epoch_ = epoch;
    state_ = CommissioningDriveState::Coasting;
    resetControlState();
  }

  CommissioningDriveState state() const noexcept { return state_; }

 private:
  DriveRequest makeCoastRequest(const FrameHeader& header,
                                const OwnerToken& owner) const noexcept {
    DriveRequest request{};
    request.h = header;
    request.source = RequestSource::Test;
    request.owner = owner;
    request.ttl_us = config_.request_ttl_us;
    request.payload = BrakePayload{kCommissioningStopMode};
    return request;
  }

  void resetSlew() noexcept {
    throttle_slew_.reset();
    turn_slew_.reset();
  }

  void resetControlState() noexcept {
    resetSlew();
    quick_turn_active_ = false;
  }

  CommissioningCurvatureConfig config_{};
  AsymmetricAxisSlew throttle_slew_;
  AsymmetricAxisSlew turn_slew_;
  std::uint32_t epoch_{};
  CommissioningDriveState state_{CommissioningDriveState::Coasting};
  bool quick_turn_active_{};
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
  explicit CommissioningControlCycle(CommissioningCurvatureConfig config)
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
        mode.mode != CompetitionMode::Test || mode.field_connected) {
      scheduler_.cancelAll(context, CommandEndReason::Cancelled);
    } else if (scheduler_.activeCount() == 0) {
      scheduler_.schedule(lease_command_, context,
                          ConflictPolicy::RejectIncoming);
    }

    sink_.beginFrame(header);
    const CommissioningCurvatureResult mapped = mapper_.update(
        header, mode, controller, timing.math_dt_s,
        lease_command_.owner());
    if (!controller_ready) {
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
      const CommissioningCurvatureConfig& config) noexcept {
    return {config.max_voltage_V,
            config.request_ttl_us,
            config.output_slew,
            kCommissioningStopMode,
            kCommissioningStopMode,
            kCommissioningStopMode};
  }

  CommissioningCurvatureConfig config_{};
  CommissioningCurvatureMapper mapper_;
  StaticScheduler<1> scheduler_{Requirement::kDrivetrain};
  CommissioningDriveLeaseCommand lease_command_{};
  DriveRequestSink sink_{};
  DriveRequestArbiter arbiter_;
  SafetyGate safety_gate_;
  DriveCapabilities capabilities_{};
  RobotState state_{};
};

}  // namespace robot
