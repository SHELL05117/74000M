#include "robot/drive/output_service.hpp"
#include "robot/runtime/mode_manager.hpp"
#include "robot/runtime/timing_monitor.hpp"
#include "test_framework.hpp"

#include <limits>

namespace {

class MemoryModeStore final : public robot::ModeStore {
 public:
  void publish(const robot::ModeSnapshot& mode) override { value = mode; }
  robot::ModeSnapshot read() const override { return value; }
  robot::ModeSnapshot value{};
};

class RecordingDriveIO final : public robot::DriveIO {
 public:
  bool initialize() override { return true; }
  bool beginImuCalibration() override { return true; }
  robot::RawDriveInputs readAll(const robot::FrameHeader&) override { return {}; }
  bool writeVoltage(double left, double right) override {
    ++write_count;
    left_V = left;
    right_V = right;
    return true;
  }
  bool stop(robot::StopMode mode) override {
    ++stop_count;
    stop_mode = mode;
    return true;
  }
  bool zeroLiftAtLowerLimit() override { return true; }
  bool writeLiftVoltage(double voltage) override {
    ++lift_write_count;
    lift_V = voltage;
    return true;
  }
  bool stopLift(robot::StopMode mode) override {
    ++lift_stop_count;
    lift_stop_mode = mode;
    return true;
  }
  int write_count{};
  int stop_count{};
  int lift_write_count{};
  int lift_stop_count{};
  double left_V{};
  double right_V{};
  double lift_V{};
  robot::StopMode stop_mode{robot::StopMode::Coast};
  robot::StopMode lift_stop_mode{robot::StopMode::Hold};
};

robot::ActuatorFrame frameAt(robot::TimeUs time_us, std::uint32_t sequence,
                             std::uint32_t epoch, double left, double right) {
  robot::ActuatorFrame frame{};
  frame.h = {time_us, sequence, epoch};
  frame.left_V = left;
  frame.right_V = right;
  return frame;
}

}  // namespace

ROBOT_TEST("mode manager uses break before make and increments epoch once") {
  MemoryModeStore store;
  robot::ModeManager modes(store);
  ROBOT_REQUIRE(modes.transitionTo(robot::CompetitionMode::Driver, 100));
  const auto entering = modes.snapshot();
  ROBOT_REQUIRE(entering.epoch == 1);
  ROBOT_REQUIRE(!entering.enabled);
  ROBOT_REQUIRE(modes.boundaryStopPending());
  ROBOT_REQUIRE(!modes.transitionTo(robot::CompetitionMode::Driver, 101));
  ROBOT_REQUIRE(modes.acknowledgeBoundaryStop(1));
  ROBOT_REQUIRE(modes.snapshot().enabled);
}

ROBOT_TEST("old epoch full voltage cannot cross a disable transition") {
  RecordingDriveIO io;
  robot::OutputService output(io, {30000, 12.0, 1e-9,
                                   robot::StopMode::Brake});
  robot::ModeSnapshot mode{robot::CompetitionMode::Disabled, false, false, 2,
                           200, 0};
  auto old = frameAt(200, 7, 1, 12.0, 12.0);
  const auto result = output.tick(mode, &old, 200);
  ROBOT_REQUIRE(result.action == robot::OutputAction::Stopped);
  ROBOT_REQUIRE((result.reject_bits & robot::kOutputModeDisabled) != 0);
  ROBOT_REQUIRE((result.reject_bits & robot::kOutputEpochMismatch) != 0);
  ROBOT_REQUIRE(io.write_count == 0);
  ROBOT_REQUIRE(io.stop_count == 1);
  ROBOT_REQUIRE(io.lift_write_count == 0);
  ROBOT_REQUIRE(io.lift_stop_count == 1);
  ROBOT_REQUIRE(io.lift_stop_mode == robot::StopMode::Hold);
}

ROBOT_TEST("output watchdog rejects future stale and nonfinite frames") {
  RecordingDriveIO io;
  robot::OutputService output(io, {20000, 12.0, 1e-9,
                                   robot::StopMode::Brake});
  const robot::ModeSnapshot mode{robot::CompetitionMode::Driver, true, false,
                                 3, 0, 0};
  auto future = frameAt(101, 1, 3, 1.0, 1.0);
  ROBOT_REQUIRE((output.tick(mode, &future, 100).reject_bits &
                 robot::kOutputFutureTimestamp) != 0);
  auto stale = frameAt(100, 2, 3, 1.0, 1.0);
  ROBOT_REQUIRE((output.tick(mode, &stale, 20101).reject_bits &
                 robot::kOutputStale) != 0);
  auto nan_frame = frameAt(200, 3, 3,
                           std::numeric_limits<double>::quiet_NaN(), 1.0);
  ROBOT_REQUIRE((output.tick(mode, &nan_frame, 200).reject_bits &
                 robot::kOutputNonfinite) != 0);
  ROBOT_REQUIRE(io.write_count == 0);
}

ROBOT_TEST("one zero side is a drive command while double zero is a stop") {
  RecordingDriveIO io;
  robot::OutputService output(io, {30000, 12.0, 1e-9,
                                   robot::StopMode::Brake});
  const robot::ModeSnapshot mode{robot::CompetitionMode::Driver, true, false,
                                 4, 0, 0};
  auto pivot = frameAt(100, 1, 4, 0.0, 5.0);
  ROBOT_REQUIRE(output.tick(mode, &pivot, 100).action ==
                robot::OutputAction::WroteVoltage);
  ROBOT_REQUIRE(io.write_count == 1);
  ROBOT_REQUIRE(io.stop_count == 0);

  auto zero = frameAt(110, 2, 4, 0.0, 0.0);
  zero.zero_behavior = robot::StopMode::Hold;
  ROBOT_REQUIRE(output.tick(mode, &zero, 110).action ==
                robot::OutputAction::Stopped);
  ROBOT_REQUIRE(io.stop_count == 1);
  ROBOT_REQUIRE(io.stop_mode == robot::StopMode::Hold);
}

ROBOT_TEST("timing monitor preserves raw dt while bounding math dt") {
  robot::TimingMonitor monitor({0.010, 0.005, 0.020, 0.015});
  auto first = monitor.begin({10000, 1, 1});
  monitor.finish(first, 12000);
  auto delayed = monitor.begin({50000, 2, 1});
  monitor.finish(delayed, 52000);
  ROBOT_REQUIRE_NEAR(delayed.raw_dt_s, 0.040, 1e-12);
  ROBOT_REQUIRE_NEAR(delayed.math_dt_s, 0.020, 1e-12);
  ROBOT_REQUIRE(delayed.deadline_missed);
  ROBOT_REQUIRE(monitor.summary().missed_count == 1);
}
