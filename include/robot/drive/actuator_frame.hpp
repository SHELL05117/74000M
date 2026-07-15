#pragma once

#include <cmath>
#include <cstdint>

#include "robot/core/frame.hpp"
#include "robot/drive/drive_request.hpp"

namespace robot {

struct ActuatorFrame {
  FrameHeader h;
  double left_V{};
  double right_V{};
  StopMode zero_behavior{StopMode::Brake};
  RequestSource owner{RequestSource::None};
  CommandId owner_id{};
  std::uint32_t owner_lease{};
  std::uint32_t applied_limits{};
};

inline bool finiteAndBounded(const ActuatorFrame& frame,
                             double max_voltage_V) noexcept {
  return std::isfinite(frame.left_V) && std::isfinite(frame.right_V) &&
         std::isfinite(max_voltage_V) && max_voltage_V > 0.0 &&
         max_voltage_V <= 12.0 &&
         std::abs(frame.left_V) <= max_voltage_V &&
         std::abs(frame.right_V) <= max_voltage_V;
}

}  // namespace robot
