#pragma once

#include <cstdint>

#include "robot/core/frame.hpp"

namespace robot {

enum class CompetitionMode : std::uint8_t {
  Boot,
  Calibrating,
  Disabled,
  Driver,
  AutonomousInterface,
  Test,
  FaultStop,
};

struct ModeSnapshot {
  CompetitionMode mode{CompetitionMode::Boot};
  bool enabled{};
  bool field_connected{};
  std::uint32_t epoch{};
  TimeUs transition_time_us{};
  std::uint32_t fault_bits{};
};

}  // namespace robot
