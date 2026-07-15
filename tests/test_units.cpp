#include "robot/core/units.hpp"
#include "test_framework.hpp"

#include <limits>

ROBOT_TEST("unit conversions preserve signs and endpoints") {
  ROBOT_REQUIRE_NEAR(robot::units::degreesToRadians(180.0),
                     robot::units::kPi, 1e-12);
  ROBOT_REQUIRE_NEAR(robot::units::centidegreesToRadians(-18000.0),
                     -robot::units::kPi, 1e-12);
  ROBOT_REQUIRE_NEAR(robot::units::rpmToRadiansPerSecond(60.0),
                     2.0 * robot::units::kPi, 1e-12);
  ROBOT_REQUIRE(robot::units::voltsToMotorMillivolts(-12.0) == -12000);
  ROBOT_REQUIRE(robot::units::voltsToMotorMillivolts(0.0) == 0);
  ROBOT_REQUIRE(robot::units::voltsToMotorMillivolts(12.0) == 12000);
  ROBOT_REQUIRE(robot::units::voltsToMotorMillivolts(99.0) == 12000);
}

ROBOT_TEST("nonfinite motor voltage has a final zero conversion fallback") {
  ROBOT_REQUIRE(robot::units::voltsToMotorMillivolts(
                    std::numeric_limits<double>::quiet_NaN()) == 0);
  ROBOT_REQUIRE(robot::units::voltsToMotorMillivolts(
                    std::numeric_limits<double>::infinity()) == 0);
}
