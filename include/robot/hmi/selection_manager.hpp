#pragma once

#include <cstdint>

#include "robot/hmi/registry.hpp"

namespace robot {

struct AutonSelectionSnapshot {
  RouteId route_id{RouteIds::kDoNothing};
  Alliance alliance{Alliance::None};
  StartSide start_side{StartSide::None};
  Pose2d expected_start_pose{};
  std::uint32_t selection_revision{};
  std::uint32_t route_registry_revision{};
  std::uint32_t mode_epoch{};
  bool valid{};
  bool fallback_to_do_nothing{};
};

template <std::size_t Capacity>
class SelectionManager {
 public:
  explicit SelectionManager(const RouteRegistry<Capacity>& registry) noexcept
      : registry_(registry) {}

  bool stage(RouteId route_id, Alliance alliance, StartSide start,
             const ModeSnapshot& mode) noexcept {
    if (mode.enabled || locked_ || !registry_.valid()) return false;
    const RouteDescriptor* route = registry_.find(route_id);
    if (route == nullptr || !routeCompatible(*route, alliance, start))
      return false;
    draft_route_ = route_id;
    draft_alliance_ = alliance;
    draft_start_ = start;
    drafting_ = true;
    return true;
  }

  bool confirm(const ModeSnapshot& mode) noexcept {
    if (mode.enabled || locked_ || !drafting_) return false;
    const RouteDescriptor* route = registry_.find(draft_route_);
    if (route == nullptr || !route->implemented ||
        !route->competition_approved ||
        !routeCompatible(*route, draft_alliance_, draft_start_))
      return false;
    confirmed_route_ = draft_route_;
    confirmed_alliance_ = draft_alliance_;
    confirmed_start_ = draft_start_;
    confirmed_ = true;
    drafting_ = false;
    ++selection_revision_;
    return true;
  }

  void cancelDraft() noexcept {
    draft_route_ = confirmed_route_;
    draft_alliance_ = confirmed_alliance_;
    draft_start_ = confirmed_start_;
    drafting_ = false;
  }

  AutonSelectionSnapshot lockForEnable(const ModeSnapshot& mode) noexcept {
    locked_ = true;
    AutonSelectionSnapshot snapshot{};
    snapshot.selection_revision = selection_revision_;
    snapshot.route_registry_revision = registry_.revision();
    snapshot.mode_epoch = mode.epoch;
    const RouteDescriptor* route = registry_.find(confirmed_route_);
    const bool valid_selection =
        confirmed_ && route != nullptr && route->implemented &&
        route->competition_approved &&
        routeCompatible(*route, confirmed_alliance_, confirmed_start_);
    if (!valid_selection) {
      route = &registry_.doNothing();
      snapshot.route_id = RouteIds::kDoNothing;
      snapshot.fallback_to_do_nothing = true;
      snapshot.valid = registry_.valid();
      locked_snapshot_ = snapshot;
      return snapshot;
    }
    snapshot.route_id = route->id;
    snapshot.alliance = confirmed_alliance_;
    snapshot.start_side = confirmed_start_;
    snapshot.expected_start_pose = route->expected_start_pose;
    snapshot.valid = true;
    locked_snapshot_ = snapshot;
    return snapshot;
  }

  void onDisabled() noexcept {
    locked_ = false;
    cancelDraft();
  }

  RouteId draftRoute() const noexcept { return draft_route_; }
  RouteId confirmedRoute() const noexcept { return confirmed_route_; }
  bool confirmed() const noexcept { return confirmed_; }
  bool locked() const noexcept { return locked_; }
  std::uint32_t revision() const noexcept { return selection_revision_; }
  const AutonSelectionSnapshot& lockedSnapshot() const noexcept {
    return locked_snapshot_;
  }

 private:
  const RouteRegistry<Capacity>& registry_;
  RouteId draft_route_{RouteIds::kDoNothing};
  RouteId confirmed_route_{RouteIds::kDoNothing};
  Alliance draft_alliance_{Alliance::None};
  Alliance confirmed_alliance_{Alliance::None};
  StartSide draft_start_{StartSide::None};
  StartSide confirmed_start_{StartSide::None};
  AutonSelectionSnapshot locked_snapshot_{};
  std::uint32_t selection_revision_{};
  bool drafting_{};
  bool confirmed_{};
  bool locked_{};
};

}  // namespace robot
