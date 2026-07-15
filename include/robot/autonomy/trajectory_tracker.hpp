#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "robot/autonomy/trajectory.hpp"
#include "robot/control/termination.hpp"
#include "robot/drive/drive_request.hpp"
#include "robot/drive/kinematics.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/state/robot_state.hpp"

namespace robot {

enum class TrajectoryTrackerState : std::uint8_t {
  Idle,
  Tracking,
  Degraded,
  Settling,
  Success,
  Timeout,
  Stalled,
  SlipAbort,
  DeviationAbort,
  StateInvalid,
  Interrupted,
};

struct TrajectoryTrackerConfig {
  double longitudinal_error_kp_per_s{};
  double lateral_error_gain_per_m{};
  double heading_error_kp_per_s{};
  double heading_error_weight_m_per_rad{};
  double max_linear_velocity_mps{};
  double max_angular_velocity_radps{};
  double max_curvature_per_m{};
  double max_position_deviation_m{};
  double max_heading_deviation_rad{};
  double degraded_speed_scale{};
  double slip_speed_scale{};
  TimeUs max_state_age_us{};
  TimeUs request_ttl_us{};
  TimeUs slip_abort_time_us{};
  bool allow_degraded_state{};
  MotionTerminationConfig termination{};
};

inline bool validTrajectoryTrackerConfig(
    const TrajectoryTrackerConfig& config) noexcept {
  return std::isfinite(config.longitudinal_error_kp_per_s) &&
         config.longitudinal_error_kp_per_s >= 0.0 &&
         std::isfinite(config.lateral_error_gain_per_m) &&
         config.lateral_error_gain_per_m >= 0.0 &&
         std::isfinite(config.heading_error_kp_per_s) &&
         config.heading_error_kp_per_s >= 0.0 &&
         std::isfinite(config.heading_error_weight_m_per_rad) &&
         config.heading_error_weight_m_per_rad >= 0.0 &&
         std::isfinite(config.max_linear_velocity_mps) &&
         config.max_linear_velocity_mps > 0.0 &&
         std::isfinite(config.max_angular_velocity_radps) &&
         config.max_angular_velocity_radps > 0.0 &&
         std::isfinite(config.max_curvature_per_m) &&
         config.max_curvature_per_m > 0.0 &&
         std::isfinite(config.max_position_deviation_m) &&
         config.max_position_deviation_m > 0.0 &&
         std::isfinite(config.max_heading_deviation_rad) &&
         config.max_heading_deviation_rad > 0.0 &&
         config.max_heading_deviation_rad <= units::kPi &&
         std::isfinite(config.degraded_speed_scale) &&
         config.degraded_speed_scale > 0.0 &&
         config.degraded_speed_scale <= 1.0 &&
         std::isfinite(config.slip_speed_scale) &&
         config.slip_speed_scale > 0.0 &&
         config.slip_speed_scale <= 1.0 && config.max_state_age_us > 0 &&
         config.request_ttl_us > 0 && config.slip_abort_time_us > 0 &&
         validMotionTerminationConfig(config.termination);
}

struct TrajectoryTrackerInput {
  FrameHeader output_header{};
  ModeSnapshot mode{};
  const RobotState* state{};
  OwnerToken owner{};
  DriveCapabilities capabilities{};
  Quality slip_quality{Quality::Good};
  TimeUs now_us{};
};

struct TrajectoryTrackerResult {
  DriveRequest request{};
  TrajectorySample target{};
  Vec2 body_position_error{};
  double heading_error_rad{};
  double progress{};
  std::uint32_t applied_limit_bits{};
  TrajectoryTrackerState state{TrajectoryTrackerState::Idle};
  bool has_request{};
};

enum TrajectoryTrackerLimit : std::uint32_t {
  kTrackerNoLimit = 0,
  kTrackerStateDegradedScale = 1u << 0,
  kTrackerSlipScale = 1u << 1,
  kTrackerLinearClamp = 1u << 2,
  kTrackerCurvatureClamp = 1u << 3,
  kTrackerAngularClamp = 1u << 4,
};

template <std::size_t TrajectoryCapacity>
class TrajectoryTracker {
 public:
  explicit TrajectoryTracker(TrajectoryTrackerConfig config) noexcept
      : config_(config), termination_(config.termination) {}

  bool start(const FixedTrajectory<TrajectoryCapacity>& trajectory,
             const TrajectoryTrackerInput& input) noexcept {
    reset();
    if (!validTrajectoryTrackerConfig(config_) || trajectory.size() < 2 ||
        !(trajectory.totalTime() > 0.0) || !validInput(input))
      return false;
    trajectory_ = &trajectory;
    owner_ = input.owner;
    epoch_ = input.mode.epoch;
    start_time_us_ = input.now_us;
    last_time_us_ = input.now_us;
    state_ = TrajectoryTrackerState::Tracking;
    active_ = termination_.start(input.now_us);
    return active_;
  }

  TrajectoryTrackerResult update(
      const TrajectoryTrackerInput& input) noexcept {
    TrajectoryTrackerResult result{};
    result.state = state_;
    if (!active_ || trajectory_ == nullptr || !validInput(input) ||
        !sameLease(owner_, input.owner) || input.mode.epoch != epoch_ ||
        input.now_us < last_time_us_) {
      return fail(input.output_header, TrajectoryTrackerState::StateInvalid);
    }
    last_time_us_ = input.now_us;
    if (input.slip_quality == Quality::Invalid)
      return fail(input.output_header, TrajectoryTrackerState::SlipAbort);
    if (input.slip_quality == Quality::Degraded) {
      if (!slip_timing_) {
        slip_timing_ = true;
        slip_start_us_ = input.now_us;
      } else if (input.now_us - slip_start_us_ >=
                 config_.slip_abort_time_us) {
        return fail(input.output_header, TrajectoryTrackerState::SlipAbort);
      }
    } else {
      slip_timing_ = false;
    }

    const double elapsed_s =
        static_cast<double>(input.now_us - start_time_us_) * 1e-6;
    result.target = trajectory_->sampleAt(elapsed_s);
    if (!result.target.valid)
      return fail(input.output_header, TrajectoryTrackerState::StateInvalid);
    const double total_distance =
        (*trajectory_)[trajectory_->size() - 1].distance_m;
    result.progress = total_distance > 0.0
                          ? std::clamp(result.target.distance_m / total_distance,
                                       0.0, 1.0)
                          : 1.0;
    const Vec2 world_error{result.target.pose.x_m - input.state->pose.x_m,
                           result.target.pose.y_m - input.state->pose.y_m};
    result.body_position_error =
        inverseRotate(world_error, input.state->pose.theta_rad);
    result.heading_error_rad =
        wrapPi(result.target.pose.theta_rad - input.state->pose.theta_rad);
    const double position_deviation =
        std::hypot(result.body_position_error.x_m,
                   result.body_position_error.y_m);
    if (position_deviation > config_.max_position_deviation_m ||
        std::abs(result.heading_error_rad) >
            config_.max_heading_deviation_rad)
      return fail(input.output_header,
                  TrajectoryTrackerState::DeviationAbort);

    ChassisSpeeds requested{
        result.target.linear_velocity_mps *
                std::cos(result.heading_error_rad) +
            config_.longitudinal_error_kp_per_s *
                result.body_position_error.x_m,
        result.target.angular_velocity_radps +
            config_.lateral_error_gain_per_m *
                result.target.linear_velocity_mps *
                result.body_position_error.y_m +
            config_.heading_error_kp_per_s * result.heading_error_rad};

    const bool state_degraded =
        input.state->translation_quality == Quality::Degraded ||
        input.state->heading_quality == Quality::Degraded;
    if (state_degraded) {
      requested.vx_mps *= config_.degraded_speed_scale;
      requested.omega_radps *= config_.degraded_speed_scale;
      result.applied_limit_bits |= kTrackerStateDegradedScale;
    }
    if (input.slip_quality == Quality::Degraded) {
      requested.vx_mps *= config_.slip_speed_scale;
      requested.omega_radps *= config_.slip_speed_scale;
      result.applied_limit_bits |= kTrackerSlipScale;
    }
    const double linear =
        std::clamp(requested.vx_mps, -config_.max_linear_velocity_mps,
                   config_.max_linear_velocity_mps);
    if (linear != requested.vx_mps)
      result.applied_limit_bits |= kTrackerLinearClamp;
    requested.vx_mps = linear;
    if (std::abs(requested.vx_mps) > 1e-9) {
      const double curvature = requested.omega_radps / requested.vx_mps;
      const double limited_curvature =
          std::clamp(curvature, -config_.max_curvature_per_m,
                     config_.max_curvature_per_m);
      if (limited_curvature != curvature)
        result.applied_limit_bits |= kTrackerCurvatureClamp;
      requested.omega_radps = limited_curvature * requested.vx_mps;
    }
    const double angular =
        std::clamp(requested.omega_radps,
                   -config_.max_angular_velocity_radps,
                   config_.max_angular_velocity_radps);
    if (angular != requested.omega_radps)
      result.applied_limit_bits |= kTrackerAngularClamp;
    requested.omega_radps = angular;

    const auto& final = (*trajectory_)[trajectory_->size() - 1];
    const double final_position_error =
        std::hypot(final.pose.x_m - input.state->pose.x_m,
                   final.pose.y_m - input.state->pose.y_m);
    const double final_heading_error =
        wrapPi(final.pose.theta_rad - input.state->pose.theta_rad);
    double termination_error =
        final_position_error +
        config_.heading_error_weight_m_per_rad *
            std::abs(final_heading_error);
    if (elapsed_s < trajectory_->totalTime())
      termination_error = std::max(
          termination_error, 2.0 * config_.termination.error_band);
    const double velocity_metric =
        std::abs(input.state->body_velocity.vx_mps) +
        config_.heading_error_weight_m_per_rad *
            std::abs(input.state->body_velocity.omega_radps);
    const double effort = std::min(
        1.0, std::max(std::abs(requested.vx_mps) /
                          config_.max_linear_velocity_mps,
                      std::abs(requested.omega_radps) /
                          config_.max_angular_velocity_radps));
    const auto termination = termination_.update(
        input.now_us, termination_error, velocity_metric, effort, true);
    if (termination == MotionTerminationState::Succeeded)
      return fail(input.output_header, TrajectoryTrackerState::Success);
    if (termination == MotionTerminationState::TimedOut)
      return fail(input.output_header, TrajectoryTrackerState::Timeout);
    if (termination == MotionTerminationState::Stalled)
      return fail(input.output_header, TrajectoryTrackerState::Stalled);
    if (termination == MotionTerminationState::StateInvalid)
      return fail(input.output_header, TrajectoryTrackerState::StateInvalid);

    state_ = termination == MotionTerminationState::Settling
                 ? TrajectoryTrackerState::Settling
                 : ((state_degraded ||
                     input.slip_quality == Quality::Degraded)
                        ? TrajectoryTrackerState::Degraded
                        : TrajectoryTrackerState::Tracking);
    result.state = state_;
    result.request = velocityRequest(input.output_header, requested);
    result.has_request = true;
    return result;
  }

  TrajectoryTrackerResult cancel(const FrameHeader& header) noexcept {
    return fail(header, TrajectoryTrackerState::Interrupted);
  }

  void reset() noexcept {
    trajectory_ = nullptr;
    termination_.reset();
    owner_ = {};
    epoch_ = 0;
    start_time_us_ = 0;
    last_time_us_ = 0;
    slip_start_us_ = 0;
    state_ = TrajectoryTrackerState::Idle;
    active_ = false;
    slip_timing_ = false;
  }

  TrajectoryTrackerState state() const noexcept { return state_; }

 private:
  bool validInput(const TrajectoryTrackerInput& input) const noexcept {
    if (input.state == nullptr || !input.mode.enabled ||
        input.mode.mode != CompetitionMode::AutonomousInterface ||
        input.output_header.mode_epoch != input.mode.epoch ||
        input.owner.command_id == 0 ||
        (input.owner.requirements & Requirement::kDrivetrain) == 0 ||
        input.owner.mode_epoch != input.mode.epoch ||
        !input.capabilities.autonomous_chassis_velocity ||
        input.output_header.time_us > input.now_us ||
        input.state->h.mode_epoch != input.mode.epoch ||
        input.state->h.time_us > input.now_us ||
        input.now_us - input.state->h.time_us > config_.max_state_age_us ||
        !valid(input.state->pose) ||
        !std::isfinite(input.state->body_velocity.vx_mps) ||
        !std::isfinite(input.state->body_velocity.omega_radps) ||
        input.state->translation_quality == Quality::Invalid ||
        input.state->heading_quality == Quality::Invalid)
      return false;
    const bool degraded =
        input.state->translation_quality == Quality::Degraded ||
        input.state->heading_quality == Quality::Degraded;
    return config_.allow_degraded_state || !degraded;
  }

  DriveRequest velocityRequest(const FrameHeader& header,
                               const ChassisSpeeds& requested) const noexcept {
    DriveRequest request{};
    request.h = header;
    request.source = RequestSource::FutureAutonomy;
    request.owner = owner_;
    request.ttl_us = config_.request_ttl_us;
    request.payload =
        ChassisVelocityPayload{requested.vx_mps, requested.omega_radps};
    return request;
  }

  TrajectoryTrackerResult fail(const FrameHeader& header,
                               TrajectoryTrackerState state) noexcept {
    state_ = state;
    active_ = false;
    TrajectoryTrackerResult result{};
    result.state = state;
    result.request.h = header;
    result.request.source = RequestSource::FutureAutonomy;
    result.request.owner = owner_;
    result.request.ttl_us = config_.request_ttl_us;
    result.request.payload = BrakePayload{StopMode::Brake};
    result.has_request = true;
    return result;
  }

  TrajectoryTrackerConfig config_{};
  MotionTerminationMonitor termination_;
  const FixedTrajectory<TrajectoryCapacity>* trajectory_{};
  OwnerToken owner_{};
  std::uint32_t epoch_{};
  TimeUs start_time_us_{};
  TimeUs last_time_us_{};
  TimeUs slip_start_us_{};
  TrajectoryTrackerState state_{TrajectoryTrackerState::Idle};
  bool active_{};
  bool slip_timing_{};
};

}  // namespace robot
