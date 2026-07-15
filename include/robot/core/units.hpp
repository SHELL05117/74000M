#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace robot::units {

constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kRadPerDeg = kPi / 180.0;
constexpr double kRadPerCentideg = kPi / 18000.0;
constexpr double kRadpsPerRpm = 2.0 * kPi / 60.0;

constexpr double degreesToRadians(double degrees) noexcept {
  return degrees * kRadPerDeg;
}

constexpr double centidegreesToRadians(double centidegrees) noexcept {
  return centidegrees * kRadPerCentideg;
}

constexpr double rpmToRadiansPerSecond(double rpm) noexcept {
  return rpm * kRadpsPerRpm;
}

constexpr double millivoltsToVolts(std::int32_t millivolts) noexcept {
  return static_cast<double>(millivolts) * 1e-3;
}

constexpr double milliampsToAmps(std::int32_t milliamps) noexcept {
  return static_cast<double>(milliamps) * 1e-3;
}

inline std::int32_t voltsToMotorMillivolts(double volts) noexcept {
  if (!std::isfinite(volts)) return 0;
  const double bounded = std::clamp(volts, -12.0, 12.0);
  return static_cast<std::int32_t>(std::lround(bounded * 1000.0));
}

}  // namespace robot::units
