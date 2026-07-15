#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robot/commands/request_sink.hpp"
#include "robot/commands/scheduler.hpp"
#include "robot/control/motion_profile.hpp"
#include "robot/control/termination.hpp"
#include "robot/drive/kinematics.hpp"
#include "robot/state/pose2d.hpp"

namespace robot {

class DriveRequestPublisher {
 public:
  virtual ~DriveRequestPublisher() = default;
  virtual bool publish(const DriveRequest& request) noexcept = 0;
};

class SinkDriveRequestPublisher final : public DriveRequestPublisher {
 public:
  SinkDriveRequestPublisher(DriveRequestSink& sink,
                            const LeaseAuthority& authority) noexcept
      : sink_(sink), authority_(authority) {}

  void beginFrame(const FrameHeader& header) noexcept { sink_.beginFrame(header); }
  bool publish(const DriveRequest& request) noexcept override {
    return sink_.publish(request, authority_);
  }

 private:
  DriveRequestSink& sink_;
  const LeaseAuthority& authority_;
};

enum class AutonomousMotionState : std::uint8_t {
  Init,
  Running,
  Settling,
  Success,
  Timeout,
  Stalled,
  StateInvalid,
  Interrupted,
};

struct MotionPrimitiveConfig {
  TrapezoidProfileConfig linear_profile{};
  TrapezoidProfileConfig angular_profile{};
  double max_linear_speed_mps{};
  double max_angular_speed_radps{};
  double linear_error_kp_per_s{};
  double heading_error_kp_per_s{};
  double final_heading_kp_per_s{};
  double heading_error_weight_m_per_rad{};
  double degraded_speed_scale{};
  TimeUs max_state_age_us{};
  TimeUs request_ttl_us{};
  bool allow_degraded_state{};
  MotionTerminationConfig termination{};
};

inline bool validMotionPrimitiveConfig(
    const MotionPrimitiveConfig& config) noexcept {
  return std::isfinite(config.linear_profile.max_velocity) &&
         config.linear_profile.max_velocity > 0.0 &&
         std::isfinite(config.linear_profile.max_acceleration) &&
         config.linear_profile.max_acceleration > 0.0 &&
         std::isfinite(config.angular_profile.max_velocity) &&
         config.angular_profile.max_velocity > 0.0 &&
         std::isfinite(config.angular_profile.max_acceleration) &&
         config.angular_profile.max_acceleration > 0.0 &&
         std::isfinite(config.max_linear_speed_mps) &&
         config.max_linear_speed_mps > 0.0 &&
         std::isfinite(config.max_angular_speed_radps) &&
         config.max_angular_speed_radps > 0.0 &&
         std::isfinite(config.linear_error_kp_per_s) &&
         config.linear_error_kp_per_s >= 0.0 &&
         std::isfinite(config.heading_error_kp_per_s) &&
         config.heading_error_kp_per_s >= 0.0 &&
         std::isfinite(config.final_heading_kp_per_s) &&
         config.final_heading_kp_per_s >= 0.0 &&
         std::isfinite(config.heading_error_weight_m_per_rad) &&
         config.heading_error_weight_m_per_rad >= 0.0 &&
         std::isfinite(config.degraded_speed_scale) &&
         config.degraded_speed_scale > 0.0 &&
         config.degraded_speed_scale <= 1.0 && config.max_state_age_us > 0 &&
         config.request_ttl_us > 0 &&
         validMotionTerminationConfig(config.termination);
}

struct MotionControlStep {
  ChassisSpeeds requested{};
  double terminal_error{};
  double velocity_metric{};
  double effort_metric{};
  bool brake{};
  bool valid{};
};

class AutonomousMotionCommandBase : public Command {
 public:
  AutonomousMotionCommandBase(CommandId id, MotionPrimitiveConfig config,
                              DriveRequestPublisher& publisher) noexcept
      : id_(id),
        config_(config),
        publisher_(publisher),
        termination_(config.termination) {}

  CommandId id() const noexcept override { return id_; }
  RequirementMask requirements() const noexcept override {
    return Requirement::kDrivetrain;
  }
  bool allowedInMode(CompetitionMode mode) const noexcept override {
    return mode == CompetitionMode::AutonomousInterface;
  }

  void initialize(const CommandContext& context,
                  const OwnerToken& owner) noexcept final {
    owner_ = owner;
    state_ = AutonomousMotionState::Init;
    last_brake_sequence_ = 0;
    initialized_ = false;
    if (!validMotionPrimitiveConfig(config_) || context.state == nullptr ||
        owner.command_id != id_ ||
        (owner.requirements & Requirement::kDrivetrain) == 0 ||
        owner.mode_epoch != context.mode.epoch ||
        !validContext(context, requiresTranslation(), requiresHeading()) ||
        !termination_.start(context.h.time_us)) {
      state_ = AutonomousMotionState::StateInvalid;
      return;
    }
    start_pose_ = context.state->pose;
    start_time_us_ = context.h.time_us;
    initialized_ = initializeMotion(context);
    if (!initialized_) state_ = AutonomousMotionState::StateInvalid;
  }

  CommandRunState execute(const CommandContext& context,
                          const OwnerToken& owner) noexcept final {
    if (!initialized_ || !sameLease(owner_, owner) ||
        owner_.mode_epoch != context.mode.epoch ||
        !validContext(context, requiresTranslation(), requiresHeading()) ||
        context.h.time_us < start_time_us_) {
      state_ = AutonomousMotionState::StateInvalid;
      emitBrake(context.h);
      return CommandRunState::Failed;
    }
    MotionControlStep step = calculate(context);
    if (!step.valid || !std::isfinite(step.terminal_error) ||
        !std::isfinite(step.velocity_metric) ||
        !std::isfinite(step.effort_metric) ||
        !std::isfinite(step.requested.vx_mps) ||
        !std::isfinite(step.requested.omega_radps)) {
      state_ = AutonomousMotionState::StateInvalid;
      emitBrake(context.h);
      return CommandRunState::Failed;
    }
    if (degraded(context)) {
      step.requested.vx_mps *= config_.degraded_speed_scale;
      step.requested.omega_radps *= config_.degraded_speed_scale;
    }
    step.requested.vx_mps =
        std::clamp(step.requested.vx_mps, -config_.max_linear_speed_mps,
                   config_.max_linear_speed_mps);
    step.requested.omega_radps =
        std::clamp(step.requested.omega_radps,
                   -config_.max_angular_speed_radps,
                   config_.max_angular_speed_radps);

    const MotionTerminationState termination = termination_.update(
        context.h.time_us, step.terminal_error, step.velocity_metric,
        step.effort_metric, true);
    if (termination == MotionTerminationState::Succeeded) {
      state_ = AutonomousMotionState::Success;
      emitBrake(context.h);
      return CommandRunState::Finished;
    }
    if (termination == MotionTerminationState::TimedOut) {
      state_ = AutonomousMotionState::Timeout;
      emitBrake(context.h);
      return CommandRunState::Failed;
    }
    if (termination == MotionTerminationState::Stalled) {
      state_ = AutonomousMotionState::Stalled;
      emitBrake(context.h);
      return CommandRunState::Failed;
    }
    if (termination == MotionTerminationState::StateInvalid) {
      state_ = AutonomousMotionState::StateInvalid;
      emitBrake(context.h);
      return CommandRunState::Failed;
    }
    state_ = termination == MotionTerminationState::Settling
                 ? AutonomousMotionState::Settling
                 : AutonomousMotionState::Running;
    if (step.brake) {
      if (!emitBrake(context.h)) {
        state_ = AutonomousMotionState::StateInvalid;
        return CommandRunState::Failed;
      }
    } else if (!emitVelocity(context.h, step.requested)) {
      state_ = AutonomousMotionState::StateInvalid;
      emitBrake(context.h);
      return CommandRunState::Failed;
    }
    return CommandRunState::Running;
  }

  void end(const CommandContext& context, const OwnerToken& owner,
           CommandEndReason reason) noexcept final {
    if (sameLease(owner_, owner) &&
        reason != CommandEndReason::Completed &&
        state_ != AutonomousMotionState::Timeout &&
        state_ != AutonomousMotionState::Stalled &&
        state_ != AutonomousMotionState::StateInvalid)
      state_ = AutonomousMotionState::Interrupted;
    emitBrake(context.h);
    initialized_ = false;
    termination_.reset();
  }

  AutonomousMotionState motionState() const noexcept { return state_; }
  const OwnerToken& owner() const noexcept { return owner_; }

 protected:
  virtual bool initializeMotion(const CommandContext& context) noexcept = 0;
  virtual MotionControlStep calculate(
      const CommandContext& context) noexcept = 0;
  virtual bool requiresTranslation() const noexcept { return true; }
  virtual bool requiresHeading() const noexcept { return true; }

  double elapsedSeconds(const CommandContext& context) const noexcept {
    return static_cast<double>(context.h.time_us - start_time_us_) * 1e-6;
  }
  const Pose2d& startPose() const noexcept { return start_pose_; }
  const MotionPrimitiveConfig& config() const noexcept { return config_; }

  static double clampUnitEffort(const ChassisSpeeds& speeds,
                                const MotionPrimitiveConfig& config) noexcept {
    return std::min(
        1.0, std::max(std::abs(speeds.vx_mps) / config.max_linear_speed_mps,
                      std::abs(speeds.omega_radps) /
                          config.max_angular_speed_radps));
  }

 private:
  bool validContext(const CommandContext& context, bool needs_translation,
                    bool needs_heading) const noexcept {
    if (context.state == nullptr || !context.mode.enabled ||
        context.mode.mode != CompetitionMode::AutonomousInterface ||
        context.h.mode_epoch != context.mode.epoch ||
        context.state->h.mode_epoch != context.mode.epoch ||
        context.state->h.time_us > context.h.time_us ||
        context.h.time_us - context.state->h.time_us >
            config_.max_state_age_us ||
        !valid(context.state->pose) ||
        !std::isfinite(context.state->body_velocity.vx_mps) ||
        !std::isfinite(context.state->body_velocity.omega_radps))
      return false;
    if (needs_translation &&
        context.state->translation_quality == Quality::Invalid)
      return false;
    if (needs_heading && context.state->heading_quality == Quality::Invalid)
      return false;
    if (!config_.allow_degraded_state && degraded(context)) return false;
    return true;
  }

  bool degraded(const CommandContext& context) const noexcept {
    return context.state->translation_quality == Quality::Degraded ||
           context.state->heading_quality == Quality::Degraded;
  }

  bool emitVelocity(const FrameHeader& header,
                    const ChassisSpeeds& speeds) noexcept {
    DriveRequest request{};
    request.h = header;
    request.source = RequestSource::FutureAutonomy;
    request.owner = owner_;
    request.ttl_us = config_.request_ttl_us;
    request.payload = ChassisVelocityPayload{speeds.vx_mps,
                                             speeds.omega_radps};
    return publisher_.publish(request);
  }

  bool emitBrake(const FrameHeader& header) noexcept {
    if (last_brake_sequence_ == header.sequence) return true;
    DriveRequest request{};
    request.h = header;
    request.source = RequestSource::FutureAutonomy;
    request.owner = owner_;
    request.ttl_us = config_.request_ttl_us;
    request.payload = BrakePayload{StopMode::Brake};
    const bool published = publisher_.publish(request);
    if (published) last_brake_sequence_ = header.sequence;
    return published;
  }

  CommandId id_{};
  MotionPrimitiveConfig config_{};
  DriveRequestPublisher& publisher_;
  MotionTerminationMonitor termination_;
  OwnerToken owner_{};
  Pose2d start_pose_{};
  TimeUs start_time_us_{};
  std::uint32_t last_brake_sequence_{};
  AutonomousMotionState state_{AutonomousMotionState::Init};
  bool initialized_{};
};

class DriveDistanceCommand final : public AutonomousMotionCommandBase {
 public:
  DriveDistanceCommand(CommandId id, double distance_m,
                       MotionPrimitiveConfig config,
                       DriveRequestPublisher& publisher) noexcept
      : AutonomousMotionCommandBase(id, config, publisher),
        distance_m_(distance_m) {}

 protected:
  bool initializeMotion(const CommandContext&) noexcept override {
    return std::isfinite(distance_m_) &&
           profile_.configure(distance_m_, config().linear_profile);
  }

  MotionControlStep calculate(const CommandContext& context) noexcept override {
    const auto reference = profile_.sample(elapsedSeconds(context));
    if (!reference.valid) return {};
    const double dx = context.state->pose.x_m - startPose().x_m;
    const double dy = context.state->pose.y_m - startPose().y_m;
    const double travelled = dx * std::cos(startPose().theta_rad) +
                             dy * std::sin(startPose().theta_rad);
    ChassisSpeeds speeds{
        reference.velocity +
            config().linear_error_kp_per_s *
                (reference.position - travelled),
        config().heading_error_kp_per_s *
            wrapPi(startPose().theta_rad - context.state->pose.theta_rad)};
    return {speeds,
            distance_m_ - travelled,
            std::abs(context.state->body_velocity.vx_mps),
            clampUnitEffort(speeds, config()), false, true};
  }

 private:
  double distance_m_{};
  TrapezoidProfile profile_{};
};

class TurnToHeadingCommand final : public AutonomousMotionCommandBase {
 public:
  TurnToHeadingCommand(CommandId id, double target_heading_rad,
                       MotionPrimitiveConfig config,
                       DriveRequestPublisher& publisher) noexcept
      : AutonomousMotionCommandBase(id, config, publisher),
        target_heading_rad_(target_heading_rad) {}

 protected:
  bool initializeMotion(const CommandContext&) noexcept override {
    delta_heading_rad_ = wrapPi(target_heading_rad_ - startPose().theta_rad);
    return std::isfinite(target_heading_rad_) &&
           profile_.configure(delta_heading_rad_, config().angular_profile);
  }

  MotionControlStep calculate(const CommandContext& context) noexcept override {
    const auto reference = profile_.sample(elapsedSeconds(context));
    if (!reference.valid) return {};
    const double reference_heading =
        startPose().theta_rad + reference.position;
    ChassisSpeeds speeds{
        0.0, reference.velocity +
                 config().heading_error_kp_per_s *
                     wrapPi(reference_heading -
                            context.state->pose.theta_rad)};
    return {speeds,
            wrapPi(target_heading_rad_ - context.state->pose.theta_rad),
            std::abs(context.state->body_velocity.omega_radps),
            clampUnitEffort(speeds, config()), false, true};
  }

  bool requiresTranslation() const noexcept override { return false; }

 private:
  double target_heading_rad_{};
  double delta_heading_rad_{};
  TrapezoidProfile profile_{};
};

class DriveArcCommand final : public AutonomousMotionCommandBase {
 public:
  DriveArcCommand(CommandId id, double radius_m, double angle_rad,
                  int travel_direction, MotionPrimitiveConfig config,
                  DriveRequestPublisher& publisher) noexcept
      : AutonomousMotionCommandBase(id, config, publisher),
        radius_m_(radius_m),
        angle_rad_(angle_rad),
        travel_direction_(travel_direction) {}

 protected:
  bool initializeMotion(const CommandContext&) noexcept override {
    if (!std::isfinite(radius_m_) || !std::isfinite(angle_rad_) ||
        radius_m_ <= 0.0 || angle_rad_ == 0.0 ||
        (travel_direction_ != -1 && travel_direction_ != 1))
      return false;
    signed_distance_m_ = static_cast<double>(travel_direction_) *
                         radius_m_ * std::abs(angle_rad_);
    curvature_per_m_ = angle_rad_ / signed_distance_m_;
    return profile_.configure(signed_distance_m_, config().linear_profile);
  }

  MotionControlStep calculate(const CommandContext& context) noexcept override {
    const auto reference = profile_.sample(elapsedSeconds(context));
    if (!reference.valid) return {};
    const double relative_heading = curvature_per_m_ * reference.position;
    const double body_x = std::sin(relative_heading) / curvature_per_m_;
    const double body_y = (1.0 - std::cos(relative_heading)) / curvature_per_m_;
    const Vec2 world = rotate({body_x, body_y}, startPose().theta_rad);
    const Pose2d reference_pose{startPose().x_m + world.x_m,
                                startPose().y_m + world.y_m,
                                startPose().theta_rad + relative_heading};
    const double along_error =
        std::cos(reference_pose.theta_rad) *
            (reference_pose.x_m - context.state->pose.x_m) +
        std::sin(reference_pose.theta_rad) *
            (reference_pose.y_m - context.state->pose.y_m);
    ChassisSpeeds speeds{
        reference.velocity + config().linear_error_kp_per_s * along_error,
        curvature_per_m_ * reference.velocity +
            config().heading_error_kp_per_s *
                wrapPi(reference_pose.theta_rad -
                       context.state->pose.theta_rad)};

    const double final_body_x = std::sin(angle_rad_) / curvature_per_m_;
    const double final_body_y =
        (1.0 - std::cos(angle_rad_)) / curvature_per_m_;
    const Vec2 final_world =
        rotate({final_body_x, final_body_y}, startPose().theta_rad);
    const double position_error = std::hypot(
        startPose().x_m + final_world.x_m - context.state->pose.x_m,
        startPose().y_m + final_world.y_m - context.state->pose.y_m);
    const double heading_error =
        wrapPi(startPose().theta_rad + angle_rad_ -
               context.state->pose.theta_rad);
    const double velocity_metric =
        std::abs(context.state->body_velocity.vx_mps) +
        config().heading_error_weight_m_per_rad *
            std::abs(context.state->body_velocity.omega_radps);
    return {speeds,
            position_error + config().heading_error_weight_m_per_rad *
                                 std::abs(heading_error),
            velocity_metric, clampUnitEffort(speeds, config()), false, true};
  }

 private:
  double radius_m_{};
  double angle_rad_{};
  int travel_direction_{};
  double signed_distance_m_{};
  double curvature_per_m_{};
  TrapezoidProfile profile_{};
};

class DriveToPoseCommand final : public AutonomousMotionCommandBase {
 public:
  DriveToPoseCommand(CommandId id, Pose2d target, bool allow_reverse,
                     MotionPrimitiveConfig config,
                     DriveRequestPublisher& publisher) noexcept
      : AutonomousMotionCommandBase(id, config, publisher),
        target_(target),
        allow_reverse_(allow_reverse) {}

 protected:
  bool initializeMotion(const CommandContext&) noexcept override {
    return valid(target_);
  }

  MotionControlStep calculate(const CommandContext& context) noexcept override {
    const double dx = target_.x_m - context.state->pose.x_m;
    const double dy = target_.y_m - context.state->pose.y_m;
    const double distance = std::hypot(dx, dy);
    double travel_heading =
        distance > 1e-9 ? std::atan2(dy, dx) : target_.theta_rad;
    double bearing_error =
        wrapPi(travel_heading - context.state->pose.theta_rad);
    double direction = 1.0;
    if (allow_reverse_ && std::abs(bearing_error) > 0.5 * units::kPi) {
      direction = -1.0;
      travel_heading = wrapPi(travel_heading + units::kPi);
      bearing_error =
          wrapPi(travel_heading - context.state->pose.theta_rad);
    }
    const double final_heading_error =
        wrapPi(target_.theta_rad - context.state->pose.theta_rad);
    ChassisSpeeds speeds{
        direction * config().linear_error_kp_per_s * distance,
        config().heading_error_kp_per_s * bearing_error +
            config().final_heading_kp_per_s * final_heading_error};
    const double velocity_metric =
        std::abs(context.state->body_velocity.vx_mps) +
        config().heading_error_weight_m_per_rad *
            std::abs(context.state->body_velocity.omega_radps);
    return {speeds,
            distance + config().heading_error_weight_m_per_rad *
                           std::abs(final_heading_error),
            velocity_metric, clampUnitEffort(speeds, config()), false, true};
  }

 private:
  Pose2d target_{};
  bool allow_reverse_{};
};

class BrakeCommand final : public AutonomousMotionCommandBase {
 public:
  BrakeCommand(CommandId id, MotionPrimitiveConfig config,
               DriveRequestPublisher& publisher) noexcept
      : AutonomousMotionCommandBase(id, config, publisher) {}

 protected:
  bool initializeMotion(const CommandContext&) noexcept override { return true; }
  MotionControlStep calculate(const CommandContext& context) noexcept override {
    const double velocity_metric =
        std::abs(context.state->body_velocity.vx_mps) +
        config().heading_error_weight_m_per_rad *
            std::abs(context.state->body_velocity.omega_radps);
    return {{}, 0.0, velocity_metric, 0.0, true, true};
  }
};

}  // namespace robot
