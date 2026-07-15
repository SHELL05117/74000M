#pragma once

#include <algorithm>
#include <cmath>

#include "robot/drive/drive_request.hpp"

namespace robot {

struct WheelVoltages {
  double left_V{};
  double right_V{};
};

struct VoltageAllocation {
  WheelVoltages output{};
  bool valid{};
  bool limited{};
};

inline bool validVoltageLimit(double limit_V) noexcept {
  return std::isfinite(limit_V) && limit_V >= 0.0 && limit_V <= 12.0;
}

inline VoltageAllocation desaturateProportional(WheelVoltages input,
                                                double limit_V) noexcept {
  VoltageAllocation result{};
  if (!validVoltageLimit(limit_V) || !std::isfinite(input.left_V) ||
      !std::isfinite(input.right_V)) {
    return result;
  }
  result.valid = true;
  const double peak =
      std::max(std::abs(input.left_V), std::abs(input.right_V));
  if (peak <= limit_V || peak == 0.0) {
    result.output = input;
    return result;
  }
  result.limited = true;
  const double scale = limit_V / peak;
  result.output = {input.left_V * scale, input.right_V * scale};
  return result;
}

inline VoltageAllocation desaturatePreserveTurn(WheelVoltages input,
                                                double limit_V) noexcept {
  VoltageAllocation result{};
  if (!validVoltageLimit(limit_V) || !std::isfinite(input.left_V) ||
      !std::isfinite(input.right_V)) {
    return result;
  }
  result.valid = true;
  double common = 0.5 * (input.left_V + input.right_V);
  double turn = 0.5 * (input.right_V - input.left_V);
  const double original_common = common;
  const double original_turn = turn;
  turn = std::clamp(turn, -limit_V, limit_V);
  const double common_headroom = limit_V - std::abs(turn);
  common = std::clamp(common, -common_headroom, common_headroom);
  result.output = {common - turn, common + turn};
  result.limited = common != original_common || turn != original_turn;
  return result;
}

inline VoltageAllocation allocateVoltage(WheelVoltages input,
                                         double limit_V,
                                         AllocationPolicy policy) noexcept {
  switch (policy) {
    case AllocationPolicy::RatioPreserving:
      return desaturateProportional(input, limit_V);
    case AllocationPolicy::PreserveTurn:
      return desaturatePreserveTurn(input, limit_V);
  }
  return {};
}

}  // namespace robot
