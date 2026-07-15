#include "robot/state/pose2d.hpp"
#include "test_framework.hpp"

ROBOT_TEST("coordinate convention rotates body forward toward world left") {
  const auto rotated = robot::rotate({1.0, 0.0}, robot::units::kPi / 2.0);
  ROBOT_REQUIRE_NEAR(rotated.x_m, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(rotated.y_m, 1.0, 1e-12);
}

ROBOT_TEST("pose composition inverse and relative direction are consistent") {
  const robot::Pose2d a{1.2, -0.4, 0.7};
  const robot::Pose2d b{0.3, 0.2, -0.1};
  const auto identity = robot::compose(a, robot::inverse(a));
  ROBOT_REQUIRE_NEAR(identity.x_m, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(identity.y_m, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(identity.theta_rad, 0.0, 1e-12);

  const auto composed = robot::compose(a, b);
  const auto recovered = robot::relativeTo(composed, a);
  ROBOT_REQUIRE_NEAR(recovered.x_m, b.x_m, 1e-12);
  ROBOT_REQUIRE_NEAR(recovered.y_m, b.y_m, 1e-12);
  ROBOT_REQUIRE_NEAR(recovered.theta_rad, b.theta_rad, 1e-12);
}

ROBOT_TEST("SE2 exponential follows a positive left-turn arc") {
  const robot::Twist2d twist{1.0, 0.0, 0.5};
  const auto increment = robot::exp(twist);
  ROBOT_REQUIRE(increment.y_m > 0.0);
  robot::Twist2d recovered{};
  ROBOT_REQUIRE(robot::log(increment, recovered));
  ROBOT_REQUIRE_NEAR(recovered.dx_m, twist.dx_m, 1e-12);
  ROBOT_REQUIRE_NEAR(recovered.dy_m, twist.dy_m, 1e-12);
  ROBOT_REQUIRE_NEAR(recovered.dtheta_rad, twist.dtheta_rad, 1e-12);
}

ROBOT_TEST("SE2 zero angle branch is finite and exact for translation") {
  const auto increment = robot::exp({0.25, -0.1, 0.0});
  ROBOT_REQUIRE_NEAR(increment.x_m, 0.25, 1e-12);
  ROBOT_REQUIRE_NEAR(increment.y_m, -0.1, 1e-12);
  ROBOT_REQUIRE_NEAR(increment.theta_rad, 0.0, 1e-12);
}
