#pragma once

#include <cstdint>

namespace robot {

using FaultBits = std::uint64_t;

enum class Fault : FaultBits {
  None = 0,
  Configuration = FaultBits{1} << 0,
  SensorInvalid = FaultBits{1} << 1,
  TimingDeadline = FaultBits{1} << 2,
  RequestRejected = FaultBits{1} << 3,
  OutputStale = FaultBits{1} << 4,
  ControllerDisconnected = FaultBits{1} << 5,
  ThermalDerate = FaultBits{1} << 6,
  Stall = FaultBits{1} << 7,
};

constexpr FaultBits faultBit(Fault fault) noexcept {
  return static_cast<FaultBits>(fault);
}

constexpr bool hasFault(FaultBits bits, Fault fault) noexcept {
  return (bits & faultBit(fault)) != 0;
}

}  // namespace robot
