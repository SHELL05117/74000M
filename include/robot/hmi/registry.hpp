#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "robot/calibration/calibration_tools.hpp"
#include "robot/state/pose2d.hpp"
#include "robot/ui/registry_ids.hpp"

namespace robot {

enum class Alliance : std::uint8_t { None, Red, Blue };
enum class StartSide : std::uint8_t { None, Near, Far };

enum AllianceMask : std::uint8_t {
  kAllianceNone = 0,
  kAllianceRed = 1u << 0,
  kAllianceBlue = 1u << 1,
  kAllianceAny = kAllianceRed | kAllianceBlue,
};

enum StartMask : std::uint8_t {
  kStartNone = 0,
  kStartNear = 1u << 0,
  kStartFar = 1u << 1,
  kStartAny = kStartNear | kStartFar,
};

enum class RouteQualityRequirement : std::uint8_t {
  None,
  HeadingOnly,
  FullPoseDegradedAllowed,
  FullPoseGood,
};

struct RouteDescriptor {
  RouteId id{};
  const char* short_name{};
  const char* long_name{};
  std::uint8_t alliance_mask{};
  std::uint8_t start_mask{};
  Pose2d expected_start_pose{};
  RouteQualityRequirement required_quality{RouteQualityRequirement::None};
  std::uint32_t tag_bits{};
  bool implemented{};
  bool competition_approved{};
};

inline bool boundedAscii(const char* text, std::size_t max_visible) noexcept {
  if (text == nullptr || max_visible == 0) return false;
  for (std::size_t i = 0; i <= max_visible; ++i) {
    const unsigned char value = static_cast<unsigned char>(text[i]);
    if (value == 0) return i > 0;
    if (value < 0x20 || value > 0x7E || i == max_visible) return false;
  }
  return false;
}

inline bool routeCompatible(const RouteDescriptor& route, Alliance alliance,
                            StartSide start) noexcept {
  const std::uint8_t alliance_bit =
      alliance == Alliance::Red
          ? kAllianceRed
          : (alliance == Alliance::Blue ? kAllianceBlue : kAllianceNone);
  const std::uint8_t start_bit =
      start == StartSide::Near
          ? kStartNear
          : (start == StartSide::Far ? kStartFar : kStartNone);
  return (route.alliance_mask & alliance_bit) != 0 &&
         (route.start_mask & start_bit) != 0;
}

template <std::size_t Capacity>
class RouteRegistry {
 public:
  RouteRegistry(const std::array<RouteDescriptor, Capacity>& routes,
                std::uint32_t revision) noexcept
      : routes_(routes), revision_(revision) {}

  bool valid() const noexcept {
    if (revision_ == 0) return false;
    bool found_do_nothing = false;
    for (std::size_t i = 0; i < Capacity; ++i) {
      const auto& route = routes_[i];
      if (!boundedAscii(route.short_name, 12) ||
          !boundedAscii(route.long_name, 40) ||
          route.alliance_mask == kAllianceNone ||
          route.start_mask == kStartNone || !robot::valid(route.expected_start_pose) ||
          (route.competition_approved && !route.implemented))
        return false;
      if (route.id == RouteIds::kDoNothing) {
        if (found_do_nothing || !route.implemented ||
            !route.competition_approved)
          return false;
        found_do_nothing = true;
      }
      for (std::size_t j = i + 1; j < Capacity; ++j)
        if (route.id == routes_[j].id) return false;
    }
    return found_do_nothing;
  }

  const RouteDescriptor* find(RouteId id) const noexcept {
    for (const auto& route : routes_)
      if (route.id == id) return &route;
    return nullptr;
  }

  const RouteDescriptor& doNothing() const noexcept {
    const auto* route = find(RouteIds::kDoNothing);
    return route != nullptr ? *route : routes_[0];
  }

  std::uint32_t revision() const noexcept { return revision_; }
  const std::array<RouteDescriptor, Capacity>& routes() const noexcept {
    return routes_;
  }

 private:
  std::array<RouteDescriptor, Capacity> routes_{};
  std::uint32_t revision_{};
};

using ParameterGroupId = std::uint8_t;

enum class ParameterUnit : std::uint8_t {
  None,
  Volt,
  Meter,
  Radian,
  Second,
  PerSecond,
  VoltSecondPerMeter,
  VoltSecond2PerMeter,
};

enum class ParameterAccess : std::uint8_t {
  ReadOnly,
  BenchEditable,
  CalibrationOnly,
};

enum class ApplyPolicy : std::uint8_t {
  ImmediateWhenDisabled,
  ResetControllerHistory,
  NextEnable,
  RebootRequired,
};

enum class PersistencePolicy : std::uint8_t {
  SessionOnly,
  UserProfile,
  CalibrationProfile,
};

struct ParameterDescriptor {
  ParameterId id{};
  ParameterGroupId group{};
  const char* short_name{};
  const char* long_name{};
  ParameterUnit unit{ParameterUnit::None};
  double min_value{};
  double max_value{};
  double fine_step{};
  double coarse_step{};
  double built_in_default{};
  ParameterAccess access{ParameterAccess::ReadOnly};
  ApplyPolicy apply_policy{ApplyPolicy::ImmediateWhenDisabled};
  PersistencePolicy persistence{PersistencePolicy::SessionOnly};
  bool available{};
};

inline bool validParameterDescriptor(
    const ParameterDescriptor& descriptor) noexcept {
  return descriptor.id != ParameterIds::kInvalid &&
         boundedAscii(descriptor.short_name, 12) &&
         boundedAscii(descriptor.long_name, 40) &&
         std::isfinite(descriptor.min_value) &&
         std::isfinite(descriptor.max_value) &&
         descriptor.min_value <= descriptor.max_value &&
         std::isfinite(descriptor.fine_step) && descriptor.fine_step > 0.0 &&
         std::isfinite(descriptor.coarse_step) &&
         descriptor.coarse_step >= descriptor.fine_step &&
         std::isfinite(descriptor.built_in_default) &&
         descriptor.built_in_default >= descriptor.min_value &&
         descriptor.built_in_default <= descriptor.max_value;
}

template <std::size_t Capacity>
class ParameterRegistry {
 public:
  ParameterRegistry(const std::array<ParameterDescriptor, Capacity>& values,
                    std::uint32_t revision) noexcept
      : values_(values), revision_(revision) {}

  bool valid() const noexcept {
    if (revision_ == 0) return false;
    for (std::size_t i = 0; i < Capacity; ++i) {
      if (!validParameterDescriptor(values_[i])) return false;
      for (std::size_t j = i + 1; j < Capacity; ++j)
        if (values_[i].id == values_[j].id) return false;
    }
    return true;
  }

  const ParameterDescriptor* find(ParameterId id) const noexcept {
    for (const auto& value : values_)
      if (value.id == id) return &value;
    return nullptr;
  }

  std::uint32_t revision() const noexcept { return revision_; }

 private:
  std::array<ParameterDescriptor, Capacity> values_{};
  std::uint32_t revision_{};
};

enum class PageSurface : std::uint8_t { Brain, Controller, Both };
enum class HmiPermission : std::uint8_t { ReadOnly, Disabled, Bench };

struct PageDescriptor {
  PageId id{};
  const char* short_name{};
  const char* long_name{};
  PageSurface surface{PageSurface::Both};
  bool visible_when_enabled{};
  HmiPermission required_permission{HmiPermission::ReadOnly};
};

template <std::size_t Capacity>
inline bool validPageRegistry(
    const std::array<PageDescriptor, Capacity>& pages) noexcept {
  for (std::size_t i = 0; i < Capacity; ++i) {
    if (pages[i].id == 0 || !boundedAscii(pages[i].short_name, 12) ||
        !boundedAscii(pages[i].long_name, 32))
      return false;
    for (std::size_t j = i + 1; j < Capacity; ++j)
      if (pages[i].id == pages[j].id) return false;
  }
  return true;
}

}  // namespace robot
