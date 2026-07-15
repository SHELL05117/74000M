#pragma once

#include <cstdint>

#include "robot/core/fault.hpp"
#include "robot/core/frame.hpp"
#include "robot/core/quality.hpp"
#include "robot/runtime/mode.hpp"
#include "robot/state/pose2d.hpp"

namespace robot {

struct RobotState {
  FrameHeader h;
  ModeSnapshot competition;
  Pose2d pose;
  BodyVelocity2d body_velocity;
  double left_distance_m{};
  double right_distance_m{};
  double left_velocity_mps{};
  double right_velocity_mps{};
  double battery_V{};
  double output_derate{};
  bool controller_connected{};
  Quality translation_quality{Quality::Invalid};
  Quality heading_quality{Quality::Invalid};
  FaultBits sensor_faults{};
  FaultBits timing_faults{};
  std::uint32_t reset_generation{};
};

using StateSnapshot = RobotState;

}  // namespace robot
