#pragma once

#include <cmath>
#include <cstdint>
#include <type_traits>
#include <variant>

#include "robot/core/frame.hpp"

namespace robot {

using RequirementMask = std::uint32_t;
using CommandId = std::uint16_t;

namespace Requirement {
constexpr RequirementMask kDrivetrain = 1u << 0;
constexpr RequirementMask kIntake = 1u << 1;
constexpr RequirementMask kLift = 1u << 2;
}  // namespace Requirement

struct OwnerToken {
  CommandId command_id{};
  RequirementMask requirements{};
  std::uint32_t lease_generation{};
  std::uint32_t mode_epoch{};
};

inline bool sameLease(const OwnerToken& left,
                      const OwnerToken& right) noexcept {
  return left.command_id == right.command_id &&
         left.requirements == right.requirements &&
         left.lease_generation == right.lease_generation &&
         left.mode_epoch == right.mode_epoch;
}

enum class RequestSource : std::uint8_t {
  None,
  Driver,
  FutureAutonomy,
  Test,
  Safety,
};

enum class DriverSteeringMode : std::uint8_t {
  Curvature,
  QuickTurn,
  HeadingAssist,
};

enum class AllocationPolicy : std::uint8_t {
  RatioPreserving,
  PreserveTurn,
};

enum class StopMode : std::uint8_t { Coast, Brake, Hold };

struct DriverCurvaturePayload {
  double forward{};
  double steering{};
  DriverSteeringMode steering_mode{DriverSteeringMode::Curvature};
  AllocationPolicy allocation{AllocationPolicy::RatioPreserving};
};

struct ChassisVelocityPayload {
  double vx_mps{};
  double omega_radps{};
};

struct WheelVoltagePayload {
  double left_V{};
  double right_V{};
};

struct BrakePayload {
  StopMode mode{StopMode::Brake};
};

using DrivePayload =
    std::variant<DriverCurvaturePayload, ChassisVelocityPayload,
                 WheelVoltagePayload, BrakePayload>;

struct DriveRequest {
  FrameHeader h;
  RequestSource source{RequestSource::None};
  OwnerToken owner;
  TimeUs ttl_us{};
  DrivePayload payload{BrakePayload{}};
};

struct DriveCapabilities {
  bool driver_curvature{};
  bool controlled_test_voltage{};
  bool autonomous_chassis_velocity{};
};

inline bool finitePayload(const DrivePayload& payload) noexcept {
  return std::visit(
      [](const auto& value) noexcept {
        using Payload = std::decay_t<decltype(value)>;
        if constexpr (std::is_same_v<Payload, DriverCurvaturePayload>) {
          if (!std::isfinite(value.forward) ||
              !std::isfinite(value.steering) ||
              std::abs(value.forward) > 1.0 ||
              std::abs(value.steering) > 1.0) {
            return false;
          }
          if (value.steering_mode == DriverSteeringMode::HeadingAssist) {
            return value.allocation == AllocationPolicy::PreserveTurn;
          }
          return value.allocation == AllocationPolicy::RatioPreserving;
        } else if constexpr (std::is_same_v<Payload,
                                            ChassisVelocityPayload>) {
          return std::isfinite(value.vx_mps) &&
                 std::isfinite(value.omega_radps);
        } else if constexpr (std::is_same_v<Payload, WheelVoltagePayload>) {
          return std::isfinite(value.left_V) && std::isfinite(value.right_V) &&
                 std::abs(value.left_V) <= 12.0 &&
                 std::abs(value.right_V) <= 12.0;
        } else {
          return true;
        }
      },
      payload);
}

inline bool supportedPayload(const DrivePayload& payload,
                             const DriveCapabilities& capabilities) noexcept {
  if (std::holds_alternative<DriverCurvaturePayload>(payload))
    return capabilities.driver_curvature;
  if (std::holds_alternative<ChassisVelocityPayload>(payload))
    return capabilities.autonomous_chassis_velocity;
  if (std::holds_alternative<WheelVoltagePayload>(payload))
    return capabilities.controlled_test_voltage;
  return true;
}

inline bool supportedPayload(const DrivePayload& payload,
                             const DriveCapabilities& capabilities,
                             RequestSource source) noexcept {
  if (std::holds_alternative<DriverCurvaturePayload>(payload))
    return source == RequestSource::Driver && capabilities.driver_curvature;
  if (std::holds_alternative<ChassisVelocityPayload>(payload))
    return source == RequestSource::FutureAutonomy &&
           capabilities.autonomous_chassis_velocity;
  if (std::holds_alternative<WheelVoltagePayload>(payload))
    return (source == RequestSource::Test &&
            capabilities.controlled_test_voltage) ||
           (source == RequestSource::FutureAutonomy &&
            capabilities.autonomous_chassis_velocity);
  return std::holds_alternative<BrakePayload>(payload);
}

}  // namespace robot
