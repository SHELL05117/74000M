#include "robot/platform/hardware_self_test.hpp"
#include "robot/state/raw_inputs.hpp"
#include "test_framework.hpp"

#include <type_traits>

namespace {

robot::HardwareConfig hardwareConfig() {
  robot::HardwareConfig hardware{};
  hardware.left = {{{1, false, 200}, {2, false, 200}, {3, false, 200}}};
  hardware.right = {{{4, true, 200}, {5, true, 200}, {6, true, 200}}};
  hardware.imu = {true, 7};
  return hardware;
}

}  // namespace

ROBOT_TEST("raw input keeps every motor identity before aggregation") {
  robot::RawDriveInputs raw{};
  for (std::size_t i = 0; i < robot::kMotorsPerSide; ++i) {
    raw.left.motor[i].smart_port = static_cast<std::uint8_t>(i + 1);
    raw.right.motor[i].smart_port = static_cast<std::uint8_t>(i + 4);
  }
  ROBOT_REQUIRE(raw.left.motor[2].smart_port == 3);
  ROBOT_REQUIRE(raw.right.motor[0].smart_port == 4);
  ROBOT_REQUIRE(std::is_trivially_copyable_v<robot::RawDriveInputs>);
}

ROBOT_TEST("direction test cannot start without disabled bench authorization") {
  robot::DirectionSelfTest test;
  const robot::DirectionSelfTestLimits limits{1.0, 50000, 0.01};
  ROBOT_REQUIRE(!test.begin(hardwareConfig(), limits, false, true));
  ROBOT_REQUIRE(test.state() == robot::DirectionTestState::Locked);
  ROBOT_REQUIRE(!test.begin(hardwareConfig(), limits, true, false));
  ROBOT_REQUIRE(test.state() == robot::DirectionTestState::Locked);
}

ROBOT_TEST("direction test emits one bounded intent and requires positive feedback") {
  robot::DirectionSelfTest test;
  const robot::DirectionSelfTestLimits limits{1.0, 50000, 0.01};
  ROBOT_REQUIRE(test.begin(hardwareConfig(), limits, true, true));
  robot::DirectionPulseIntent intent{};
  ROBOT_REQUIRE(test.nextIntent(intent));
  ROBOT_REQUIRE(intent.smart_port == 1);
  ROBOT_REQUIRE(intent.voltage_V == 1.0);
  ROBOT_REQUIRE(test.observe(1, 0.02, true, false));
  ROBOT_REQUIRE(test.state() == robot::DirectionTestState::Ready);
}

ROBOT_TEST("direction test locks failure to the affected port") {
  robot::DirectionSelfTest test;
  ROBOT_REQUIRE(test.begin(hardwareConfig(), {1.0, 50000, 0.01}, true, true));
  robot::DirectionPulseIntent intent{};
  ROBOT_REQUIRE(test.nextIntent(intent));
  ROBOT_REQUIRE(!test.observe(intent.smart_port, -0.02, true, false));
  ROBOT_REQUIRE(test.state() == robot::DirectionTestState::Failed);
  ROBOT_REQUIRE(test.failedPort() == 1);
}
