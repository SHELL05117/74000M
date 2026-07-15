#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "robot/config/robot_config.hpp"
#include "robot/platform/io.hpp"

namespace robot {

class ProsClock final : public Clock {
 public:
  TimeUs nowUs() const override;
  std::uint32_t nowMs() const override;
  void delayUntilMs(std::uint32_t& previous_ms,
                    std::uint32_t period_ms) override;
};

class ProsCompetitionIO final : public CompetitionIO {
 public:
  CompetitionSnapshot readOnce(const FrameHeader& header) override;
};

class ProsControllerIO final : public ControllerIO {
 public:
  ControllerSnapshot readOnce(const FrameHeader& header) override;
};

class ProsDriveIO final : public DriveIO {
 public:
  ProsDriveIO(const RobotConfig& config, const char* expected_robot_id,
              std::uint32_t expected_schema);

  bool initialize() override;
  bool beginImuCalibration() override;
  RawDriveInputs readAll(const FrameHeader& header) override;
  bool writeVoltage(double left_V, double right_V) override;
  bool stop(StopMode mode) override;

  bool structurallyValid() const noexcept { return structurally_valid_; }

 private:
  MotorSample readMotor(const MotorPortConfig& motor);
  bool configureMotor(const MotorPortConfig& motor);
  bool writeSide(const std::array<MotorPortConfig, kMotorsPerSide>& side,
                 double voltage_V);

  HardwareConfig hardware_{};
  ElectricalConfig electrical_{};
  bool structurally_valid_{};
};

}  // namespace robot
