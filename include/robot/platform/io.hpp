#pragma once

#include <cstdint>

#include "robot/core/frame.hpp"
#include "robot/drive/drive_request.hpp"
#include "robot/state/controller_snapshot.hpp"
#include "robot/state/raw_inputs.hpp"

namespace robot {

class DriveIO {
 public:
  virtual bool initialize() = 0;
  virtual bool beginImuCalibration() = 0;
  virtual RawDriveInputs readAll(const FrameHeader& header) = 0;
  virtual bool writeVoltage(double left_V, double right_V) = 0;
  virtual bool stop(StopMode mode) = 0;
  virtual bool zeroLiftAtLowerLimit() = 0;
  virtual bool writeLiftVoltage(double voltage_V) = 0;
  virtual bool stopLift(StopMode mode) = 0;
  virtual ~DriveIO() = default;
};

class ControllerIO {
 public:
  virtual ControllerSnapshot readOnce(const FrameHeader& header) = 0;
  virtual ~ControllerIO() = default;
};

class ControllerDisplayIO {
 public:
  virtual bool writeLine(std::uint8_t row, const char* text) = 0;
  virtual bool rumble(const char* pattern) = 0;
  virtual ~ControllerDisplayIO() = default;
};

class CompetitionIO {
 public:
  virtual CompetitionSnapshot readOnce(const FrameHeader& header) = 0;
  virtual ~CompetitionIO() = default;
};

class Clock {
 public:
  virtual TimeUs nowUs() const = 0;
  virtual std::uint32_t nowMs() const = 0;
  virtual void delayUntilMs(std::uint32_t& previous_ms,
                            std::uint32_t period_ms) = 0;
  virtual ~Clock() = default;
};

}  // namespace robot
