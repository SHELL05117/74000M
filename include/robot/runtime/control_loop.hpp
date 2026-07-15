#pragma once

#include <cstdint>

#include "robot/platform/io.hpp"
#include "robot/runtime/mailbox.hpp"
#include "robot/runtime/mode_manager.hpp"
#include "robot/runtime/timing_monitor.hpp"

namespace robot {

class ControlCycle {
 public:
  virtual ActuatorFrame update(const FrameHeader& header,
                               const ModeSnapshot& mode,
                               const RawDriveInputs& raw,
                               const ControllerSnapshot& controller,
                               const TimingSample& timing) = 0;
  virtual ~ControlCycle() = default;
};

class SafeStopControlCycle final : public ControlCycle {
 public:
  ActuatorFrame update(const FrameHeader& header, const ModeSnapshot&,
                       const RawDriveInputs&, const ControllerSnapshot&,
                       const TimingSample&) override {
    ActuatorFrame frame{};
    frame.h = header;
    frame.zero_behavior = StopMode::Brake;
    frame.owner = RequestSource::Safety;
    return frame;
  }
};

struct ControlLoopConfig {
  std::uint32_t period_ms{10};
  // Commissioning-only mode: use RequestSource::Test while not connected to a
  // field. A field connection forces Disabled instead of enabling this path.
  bool commissioning_test_mode{};
};

class ControlLoop {
 public:
  ControlLoop(Clock& clock, DriveIO& drive, ControllerIO& controller,
              CompetitionIO& competition, ModeManager& modes,
              ActuatorStore& actuator_store, TimingMonitor& timing,
              ControlCycle& cycle, ControlLoopConfig config)
      : clock_(clock),
        drive_(drive),
        controller_(controller),
        competition_(competition),
        modes_(modes),
        actuator_store_(actuator_store),
        timing_(timing),
        cycle_(cycle),
        config_(config) {}

  TimingSample tickOnce() {
    const TimeUs start_us = clock_.nowUs();
    ModeSnapshot before = modes_.snapshot();
    const FrameHeader probe{start_us, sequence_ + 1, before.epoch};
    const CompetitionSnapshot competition = competition_.readOnce(probe);
    CompetitionMode requested{CompetitionMode::Disabled};
    if (!competition.disabled) {
      if (config_.commissioning_test_mode) {
        requested = competition.field_connected
                        ? CompetitionMode::Disabled
                        : CompetitionMode::Test;
      } else {
        requested = competition.autonomous
                        ? CompetitionMode::AutonomousInterface
                        : CompetitionMode::Driver;
      }
    }
    modes_.transitionTo(requested, start_us, 0,
                        competition.field_connected);
    const ModeSnapshot mode = modes_.snapshot();
    const FrameHeader header{start_us, ++sequence_, mode.epoch};
    TimingSample sample = timing_.begin(header);

    const RawDriveInputs raw = drive_.readAll(header);
    const ControllerSnapshot controller = controller_.readOnce(header);
    ActuatorFrame frame =
        cycle_.update(header, mode, raw, controller, sample);
    frame.h = header;
    actuator_store_.publish(frame);

    if (modes_.boundaryStopPending())
      modes_.acknowledgeBoundaryStop(header.mode_epoch);
    timing_.finish(sample, clock_.nowUs());
    return sample;
  }

  [[noreturn]] void run() {
    std::uint32_t wake_ms = clock_.nowMs();
    while (true) {
      tickOnce();
      clock_.delayUntilMs(wake_ms, config_.period_ms);
    }
  }

 private:
  Clock& clock_;
  DriveIO& drive_;
  ControllerIO& controller_;
  CompetitionIO& competition_;
  ModeManager& modes_;
  ActuatorStore& actuator_store_;
  TimingMonitor& timing_;
  ControlCycle& cycle_;
  ControlLoopConfig config_{};
  std::uint32_t sequence_{};
};

}  // namespace robot
