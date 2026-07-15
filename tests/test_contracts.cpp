#include "robot/core/snapshot_box.hpp"
#include "robot/drive/actuator_frame.hpp"
#include "robot/drive/drive_request.hpp"
#include "robot/ui/registry_ids.hpp"
#include "test_framework.hpp"

#include <limits>
#include <mutex>

ROBOT_TEST("snapshot box publishes complete copies") {
  robot::SnapshotBox<robot::FrameHeader, std::mutex> box;
  ROBOT_REQUIRE(!box.initialized());
  box.publish({1234, 7, 3});
  const auto value = box.read();
  ROBOT_REQUIRE(box.initialized());
  ROBOT_REQUIRE(value.time_us == 1234);
  ROBOT_REQUIRE(value.sequence == 7);
  ROBOT_REQUIRE(value.mode_epoch == 3);
}

ROBOT_TEST("drive payload validation rejects illegal numeric values") {
  robot::DrivePayload valid = robot::DriverCurvaturePayload{
      0.8, -0.2, robot::DriverSteeringMode::Curvature,
      robot::AllocationPolicy::RatioPreserving};
  ROBOT_REQUIRE(robot::finitePayload(valid));

  robot::DrivePayload invalid = robot::WheelVoltagePayload{
      std::numeric_limits<double>::quiet_NaN(), 1.0};
  ROBOT_REQUIRE(!robot::finitePayload(invalid));
}

ROBOT_TEST("reserved autonomous payload stays unavailable by default") {
  const robot::DriveCapabilities capabilities{};
  const robot::DrivePayload payload = robot::ChassisVelocityPayload{1.0, 0.2};
  ROBOT_REQUIRE(robot::finitePayload(payload));
  ROBOT_REQUIRE(!robot::supportedPayload(payload, capabilities));
}

ROBOT_TEST("actuator frame enforces physical voltage and finite values") {
  robot::ActuatorFrame frame{};
  frame.left_V = 6.0;
  frame.right_V = -6.0;
  ROBOT_REQUIRE(robot::finiteAndBounded(frame, 12.0));
  frame.right_V = 12.1;
  ROBOT_REQUIRE(!robot::finiteAndBounded(frame, 12.0));
}

ROBOT_TEST("stable safety registry identifiers do not depend on array order") {
  ROBOT_REQUIRE(robot::RouteIds::kDoNothing == 0x0000);
  ROBOT_REQUIRE(robot::ParameterIds::kInvalid == 0x0000);
  ROBOT_REQUIRE(robot::PageIds::kDashboard != robot::PageIds::kAbout);
}
