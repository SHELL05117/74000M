#include "robot/platform/fake_io.hpp"
#include "robot/runtime/control_loop.hpp"
#include "robot/telemetry/integrity.hpp"
#include "robot/telemetry/replay.hpp"
#include "robot/telemetry/spsc_ring.hpp"
#include "robot/telemetry/telemetry_task.hpp"
#include "test_framework.hpp"

#include <array>

namespace {

class MemoryModeStore final : public robot::ModeStore {
 public:
  void publish(const robot::ModeSnapshot& next) override { mode = next; }
  robot::ModeSnapshot read() const override { return mode; }
  robot::ModeSnapshot mode{};
};

class MemoryActuatorStore final : public robot::ActuatorStore {
 public:
  void publish(const robot::ActuatorFrame& next) override {
    frame = next;
    initialized = true;
  }
  bool readLatest(robot::ActuatorFrame& output) const override {
    if (!initialized) return false;
    output = frame;
    return true;
  }
  robot::ActuatorFrame frame{};
  bool initialized{};
};

class FailingSink final : public robot::TelemetrySink {
 public:
  bool write(const robot::LogFrame*, std::size_t count) override {
    observed += count;
    return false;
  }
  std::size_t observed{};
};

robot::HardwareConfig hardwareConfig() {
  robot::HardwareConfig hardware{};
  hardware.left = {{{1, false, 200}, {2, false, 200}, {3, false, 200}}};
  hardware.right = {{{4, false, 200}, {5, false, 200}, {6, false, 200}}};
  hardware.imu_port = 7;
  return hardware;
}

}  // namespace

ROBOT_TEST("SPSC ring drops new frames without overwriting queued data") {
  robot::SpscRing<std::uint32_t, 4> ring;
  ROBOT_REQUIRE(ring.tryPush(10));
  ROBOT_REQUIRE(ring.tryPush(11));
  ROBOT_REQUIRE(ring.tryPush(12));
  ROBOT_REQUIRE(!ring.tryPush(13));
  ROBOT_REQUIRE(ring.dropped() == 1);
  ROBOT_REQUIRE(ring.depth() == 3);
  ROBOT_REQUIRE(ring.highWatermark() == 3);
  std::uint32_t value{};
  ROBOT_REQUIRE(ring.tryPop(value) && value == 10);
  ROBOT_REQUIRE(ring.tryPop(value) && value == 11);
  ROBOT_REQUIRE(ring.tryPop(value) && value == 12);
  ROBOT_REQUIRE(!ring.tryPop(value));
}

ROBOT_TEST("telemetry sink failure is counted after a bounded drain") {
  robot::SpscRing<robot::LogFrame, 8> ring;
  for (std::uint32_t i = 0; i < 3; ++i)
    ROBOT_REQUIRE(ring.tryPush(robot::makeEmptyLogFrame({i * 10000, i, 1}, 9)));
  FailingSink sink;
  robot::TelemetryDrain<8, 2> drain(ring, sink);
  ROBOT_REQUIRE(drain.drainOnce() == 2);
  ROBOT_REQUIRE(drain.sinkFailureCount() == 1);
  ROBOT_REQUIRE(drain.discardedFrames() == 2);
  ROBOT_REQUIRE(ring.depth() == 1);
}

ROBOT_TEST("log integrity detects gaps time regression and run mismatch") {
  robot::LogIntegrityTracker tracker(42);
  auto first = robot::makeEmptyLogFrame({100, 1, 1}, 42);
  ROBOT_REQUIRE(tracker.observe(first) == robot::kIntegrityOk);
  auto second = robot::makeEmptyLogFrame({90, 4, 1}, 99);
  const auto faults = tracker.observe(second);
  ROBOT_REQUIRE((faults & robot::kLogSequenceGap) != 0);
  ROBOT_REQUIRE((faults & robot::kLogTimeRegression) != 0);
  ROBOT_REQUIRE((faults & robot::kLogRunMismatch) != 0);
  ROBOT_REQUIRE(tracker.report().missing_frames == 2);
}

ROBOT_TEST("fake IO keeps port identity and supports isolated fault injection") {
  robot::FakeDriveIO drive(hardwareConfig(), {2.0, 12.0});
  ROBOT_REQUIRE(drive.initialize());
  ROBOT_REQUIRE(drive.writeVoltage(3.0, 4.0));
  drive.advance(0.1);
  drive.injectLeftMotorFailure(1, true);
  const auto raw = drive.readAll({100000, 1, 1});
  ROBOT_REQUIRE(raw.left.motor[0].smart_port == 1);
  ROBOT_REQUIRE(raw.left.motor[1].smart_port == 2);
  ROBOT_REQUIRE(raw.left.motor[0].position_rad.value > 0.0);
  ROBOT_REQUIRE(raw.left.motor[0].position_rad.api_ok);
  ROBOT_REQUIRE(!raw.left.motor[1].position_rad.api_ok);
  ROBOT_REQUIRE(raw.right.motor[0].position_rad.api_ok);
}

ROBOT_TEST("control loop samples each IO once and publishes a boundary stop") {
  robot::FakeClock clock;
  clock.setNowUs(10000);
  robot::FakeDriveIO drive(hardwareConfig(), {2.0, 12.0});
  ROBOT_REQUIRE(drive.initialize());
  robot::FakeControllerIO controller;
  robot::ControllerSnapshot pad{};
  pad.connected = true;
  pad.api_ok = true;
  controller.set(pad);
  robot::FakeCompetitionIO competition;
  robot::CompetitionSnapshot competition_state{};
  competition_state.disabled = false;
  competition_state.autonomous = false;
  competition_state.api_ok = true;
  competition.set(competition_state);
  MemoryModeStore mode_store;
  robot::ModeManager modes(mode_store);
  MemoryActuatorStore actuators;
  robot::TimingMonitor timing({0.010, 0.001, 0.050, 0.015});
  robot::SafeStopControlCycle cycle;
  robot::ControlLoop loop(clock, drive, controller, competition, modes,
                          actuators, timing, cycle, {10});
  loop.tickOnce();
  ROBOT_REQUIRE(drive.readCount() == 1);
  ROBOT_REQUIRE(controller.readCount() == 1);
  ROBOT_REQUIRE(competition.readCount() == 1);
  ROBOT_REQUIRE(actuators.initialized);
  ROBOT_REQUIRE(actuators.frame.left_V == 0.0);
  ROBOT_REQUIRE(actuators.frame.right_V == 0.0);
  ROBOT_REQUIRE(modes.snapshot().enabled);
}

ROBOT_TEST("raw replay preserves recorded timestamps and sequence") {
  std::array<robot::RawDriveInputs, 2> frames{};
  frames[0].h = {10000, 8, 3};
  frames[1].h = {25000, 9, 3};
  robot::RawInputReplay<2> replay(frames);
  robot::RawDriveInputs output{};
  ROBOT_REQUIRE(replay.next(output));
  ROBOT_REQUIRE(output.h.time_us == 10000);
  ROBOT_REQUIRE(replay.next(output));
  ROBOT_REQUIRE(output.h.time_us == 25000);
  ROBOT_REQUIRE(output.h.sequence == 9);
  ROBOT_REQUIRE(!replay.next(output));
}
