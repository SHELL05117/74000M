#pragma once

#include <cstdint>

#include "robot/platform/io.hpp"
#include "robot/runtime/global_control_event.hpp"
#include "robot/runtime/mailbox.hpp"
#include "robot/runtime/mode_manager.hpp"
#include "robot/runtime/output_status.hpp"
#include "robot/runtime/timing_monitor.hpp"
#include "robot/telemetry/flight_recorder.hpp"

namespace robot {

class ControlCycle {
 public:
  virtual void acceptGlobalEvent(
      const GlobalControlEvent&) noexcept {}
  virtual std::uint32_t consumedGlobalEventBits() const noexcept {
    return kGlobalControlNoEvent;
  }
  virtual ActuatorFrame update(const FrameHeader& header,
                               const ModeSnapshot& mode,
                               const RawDriveInputs& raw,
                               const ControllerSnapshot& controller,
                               const TimingSample& timing) = 0;
  virtual void populateLogFrame(LogFrame&) const noexcept {}
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
              ControlCycle& cycle, ControlLoopConfig config,
              FlightRecorderPort* recorder = nullptr,
              std::uint32_t run_id_hash = 0,
              const OutputStatusStore* output_status = nullptr)
      : clock_(clock),
        drive_(drive),
        controller_(controller),
        competition_(competition),
        modes_(modes),
        actuator_store_(actuator_store),
        timing_(timing),
        cycle_(cycle),
        config_(config),
        recorder_(recorder),
        run_id_hash_(run_id_hash),
        output_status_(output_status) {}

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
    const GlobalControlEvent global_event =
        global_event_detector_.observe(header, controller);
    cycle_.acceptGlobalEvent(global_event);
    ActuatorFrame frame =
        cycle_.update(header, mode, raw, controller, sample);
    frame.h = header;
    actuator_store_.publish(frame);

    if (modes_.boundaryStopPending())
      modes_.acknowledgeBoundaryStop(header.mode_epoch);
    const TimeUs finish_us = clock_.nowUs();
    timing_.finish(sample, finish_us);
    if (sample.deadline_missed && overrun_total_ != UINT32_MAX)
      ++overrun_total_;
    if (recorder_ != nullptr) {
      LogFrame log = makeControlLogFrame(
          header, run_id_hash_, mode, raw, controller, frame, sample, 0, 0,
          finish_us);
      log.timing.overrun_total = overrun_total_;
      log.trace.availability_bits |= kTraceCompetitionInput;
      log.trace.competition_disabled = competition.disabled;
      log.trace.competition_autonomous = competition.autonomous;
      log.trace.competition_api_ok = competition.api_ok;
      OutputStatus status{};
      if (output_status_ != nullptr &&
          output_status_->readLatest(status)) {
        log.trace.availability_bits |= kTraceOutputStatus;
        log.trace.output_action =
            static_cast<std::uint8_t>(status.action);
        log.actuator.last_written_sequence = status.actuator_sequence;
        log.actuator.write_attempted = status.write_attempted;
        log.actuator.write_ok = status.io_ok;
        log.actuator.write_reject_bits = status.reject_bits;
      }
      cycle_.populateLogFrame(log);
      log.system_event.event_sequence = global_event.event_sequence;
      log.system_event.event_bits = global_event.event_bits;
      log.system_event.drive_consumed =
          (cycle_.consumedGlobalEventBits() &
           kGlobalCoastOnce) != 0;
      recorder_->capture(mode, controller, log);
    }
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
  FlightRecorderPort* recorder_{};
  std::uint32_t run_id_hash_{};
  const OutputStatusStore* output_status_{};
  GlobalControlEventDetector global_event_detector_{};
  std::uint32_t sequence_{};
  std::uint32_t overrun_total_{};
};

}  // namespace robot
