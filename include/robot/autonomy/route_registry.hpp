#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/autonomy/motion_commands.hpp"
#include "robot/hmi/selection_manager.hpp"

namespace robot {

struct RouteBuildContext {
  void* user_context{};
};

using RouteCommandFactory = Command* (*)(RouteBuildContext&) noexcept;

struct RouteFactoryDescriptor {
  RouteId route_id{RouteIds::kDoNothing};
  RouteCommandFactory factory{};
};

struct RouteResolutionConfig {
  double start_position_tolerance_m{};
  double start_heading_tolerance_rad{};
};

enum RouteResolutionReject : std::uint32_t {
  kRouteResolutionAccepted = 0,
  kRouteResolutionRegistry = 1u << 0,
  kRouteResolutionSelection = 1u << 1,
  kRouteResolutionMode = 1u << 2,
  kRouteResolutionRevision = 1u << 3,
  kRouteResolutionNotApproved = 1u << 4,
  kRouteResolutionQuality = 1u << 5,
  kRouteResolutionStartPose = 1u << 6,
  kRouteResolutionNoFactory = 1u << 7,
  kRouteResolutionFactoryFailed = 1u << 8,
};

struct RouteResolution {
  Command* command{};
  RouteId resolved_route{RouteIds::kDoNothing};
  std::uint32_t reject_bits{};
  bool fallback_to_do_nothing{true};
};

inline bool qualitySatisfies(RouteQualityRequirement requirement,
                             const RobotState& state) noexcept {
  switch (requirement) {
    case RouteQualityRequirement::None:
      return true;
    case RouteQualityRequirement::HeadingOnly:
      return state.heading_quality != Quality::Invalid;
    case RouteQualityRequirement::FullPoseDegradedAllowed:
      return state.translation_quality != Quality::Invalid &&
             state.heading_quality != Quality::Invalid;
    case RouteQualityRequirement::FullPoseGood:
      return state.translation_quality == Quality::Good &&
             state.heading_quality == Quality::Good;
  }
  return false;
}

template <std::size_t RouteCapacity, std::size_t FactoryCapacity>
class AutonomousRouteRegistry {
 public:
  AutonomousRouteRegistry(
      const RouteRegistry<RouteCapacity>& routes,
      const std::array<RouteFactoryDescriptor, FactoryCapacity>& factories,
      Command& do_nothing, RouteResolutionConfig config) noexcept
      : routes_(routes),
        factories_(factories),
        do_nothing_(do_nothing),
        config_(config) {}

  bool valid() const noexcept {
    if (!routes_.valid() || do_nothing_.id() == 0 ||
        !std::isfinite(config_.start_position_tolerance_m) ||
        config_.start_position_tolerance_m < 0.0 ||
        !std::isfinite(config_.start_heading_tolerance_rad) ||
        config_.start_heading_tolerance_rad < 0.0 ||
        config_.start_heading_tolerance_rad > units::kPi)
      return false;
    for (std::size_t i = 0; i < FactoryCapacity; ++i) {
      if (factories_[i].route_id == RouteIds::kDoNothing ||
          factories_[i].factory == nullptr ||
          routes_.find(factories_[i].route_id) == nullptr)
        return false;
      for (std::size_t j = i + 1; j < FactoryCapacity; ++j)
        if (factories_[i].route_id == factories_[j].route_id) return false;
    }
    return true;
  }

  RouteResolution resolve(const AutonSelectionSnapshot& locked_selection,
                          const ModeSnapshot& mode, const RobotState& state,
                          RouteBuildContext& build_context) const noexcept {
    RouteResolution result{&do_nothing_, RouteIds::kDoNothing,
                           kRouteResolutionAccepted, true};
    if (!valid()) result.reject_bits |= kRouteResolutionRegistry;
    if (!locked_selection.valid || locked_selection.fallback_to_do_nothing)
      result.reject_bits |= kRouteResolutionSelection;
    if (!mode.enabled || mode.mode != CompetitionMode::AutonomousInterface ||
        locked_selection.mode_epoch != mode.epoch ||
        state.h.mode_epoch != mode.epoch)
      result.reject_bits |= kRouteResolutionMode;
    if (locked_selection.route_registry_revision != routes_.revision())
      result.reject_bits |= kRouteResolutionRevision;
    const RouteDescriptor* route = routes_.find(locked_selection.route_id);
    if (route == nullptr || !route->implemented ||
        !route->competition_approved)
      result.reject_bits |= kRouteResolutionNotApproved;
    if (route != nullptr && route->id != RouteIds::kDoNothing &&
        !routeCompatible(*route, locked_selection.alliance,
                         locked_selection.start_side))
      result.reject_bits |= kRouteResolutionSelection;
    if (route != nullptr &&
        !qualitySatisfies(route->required_quality, state))
      result.reject_bits |= kRouteResolutionQuality;
    if (route != nullptr && route->id != RouteIds::kDoNothing) {
      const double start_error =
          std::hypot(state.pose.x_m - route->expected_start_pose.x_m,
                     state.pose.y_m - route->expected_start_pose.y_m);
      const double heading_error = std::abs(
          wrapPi(state.pose.theta_rad - route->expected_start_pose.theta_rad));
      const double snapshot_position_error = std::hypot(
          locked_selection.expected_start_pose.x_m -
              route->expected_start_pose.x_m,
          locked_selection.expected_start_pose.y_m -
              route->expected_start_pose.y_m);
      const double snapshot_heading_error = std::abs(wrapPi(
          locked_selection.expected_start_pose.theta_rad -
          route->expected_start_pose.theta_rad));
      if (!robot::valid(state.pose) ||
          start_error > config_.start_position_tolerance_m ||
          heading_error > config_.start_heading_tolerance_rad ||
          snapshot_position_error > 1e-9 || snapshot_heading_error > 1e-9)
        result.reject_bits |= kRouteResolutionStartPose;
    }
    const RouteFactoryDescriptor* factory =
        route == nullptr ? nullptr : findFactory(route->id);
    if (route != nullptr && route->id != RouteIds::kDoNothing &&
        factory == nullptr)
      result.reject_bits |= kRouteResolutionNoFactory;
    if (result.reject_bits != kRouteResolutionAccepted || route == nullptr ||
        route->id == RouteIds::kDoNothing)
      return result;
    Command* command = factory->factory(build_context);
    if (command == nullptr) {
      result.reject_bits |= kRouteResolutionFactoryFailed;
      return result;
    }
    result.command = command;
    result.resolved_route = route->id;
    result.fallback_to_do_nothing = false;
    return result;
  }

 private:
  const RouteFactoryDescriptor* findFactory(RouteId id) const noexcept {
    for (const auto& factory : factories_)
      if (factory.route_id == id) return &factory;
    return nullptr;
  }

  const RouteRegistry<RouteCapacity>& routes_;
  std::array<RouteFactoryDescriptor, FactoryCapacity> factories_{};
  Command& do_nothing_;
  RouteResolutionConfig config_{};
};

class DoNothingCommand final : public Command {
 public:
  DoNothingCommand(CommandId id, TimeUs request_ttl_us,
                   DriveRequestPublisher& publisher) noexcept
      : id_(id), request_ttl_us_(request_ttl_us), publisher_(publisher) {}
  CommandId id() const noexcept override { return id_; }
  RequirementMask requirements() const noexcept override {
    return Requirement::kDrivetrain;
  }
  bool allowedInMode(CompetitionMode mode) const noexcept override {
    return mode == CompetitionMode::AutonomousInterface;
  }
  void initialize(const CommandContext&, const OwnerToken& owner) noexcept override {
    owner_ = owner;
  }
  CommandRunState execute(const CommandContext& context,
                          const OwnerToken& owner) noexcept override {
    if (!sameLease(owner_, owner) || request_ttl_us_ == 0)
      return CommandRunState::Failed;
    DriveRequest request{};
    request.h = context.h;
    request.source = RequestSource::FutureAutonomy;
    request.owner = owner_;
    request.ttl_us = request_ttl_us_;
    request.payload = BrakePayload{StopMode::Brake};
    return publisher_.publish(request) ? CommandRunState::Running
                                       : CommandRunState::Failed;
  }
  void end(const CommandContext&, const OwnerToken&,
           CommandEndReason) noexcept override {}

 private:
  CommandId id_{};
  TimeUs request_ttl_us_{};
  DriveRequestPublisher& publisher_;
  OwnerToken owner_{};
};

}  // namespace robot
