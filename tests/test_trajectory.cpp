#include "robot/autonomy/trajectory.hpp"
#include "test_framework.hpp"

#include <array>
#include <cmath>

namespace {

robot::TrajectoryConstraints constraints(double spatial_step = 0.1) {
  return {1.2,
          2.0,
          2.5,
          2.0,
          2.0,
          3.0,
          12.0,
          0.4,
          spatial_step,
          0.2,
          0.0,
          {{0.1, 0.12, 0.5, 0.05}, {0.11, 0.13, 0.52, 0.05}}};
}

template <std::size_t Capacity>
void requireConstraints(const robot::FixedTrajectory<Capacity>& trajectory,
                        const robot::TrajectoryConstraints& limits) {
  ROBOT_REQUIRE(trajectory.size() >= 2);
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const auto& sample = trajectory[i];
    ROBOT_REQUIRE(sample.valid);
    ROBOT_REQUIRE(std::abs(sample.linear_velocity_mps) <=
                  limits.max_velocity_mps + 1e-8);
    ROBOT_REQUIRE(std::abs(sample.angular_velocity_radps) <=
                  limits.max_angular_velocity_radps + 1e-8);
    ROBOT_REQUIRE(sample.linear_velocity_mps *
                      robot::directionSign(sample.direction) >=
                  -1e-12);
    const double centripetal =
        sample.linear_velocity_mps * sample.linear_velocity_mps *
        std::abs(sample.path_curvature_per_m);
    ROBOT_REQUIRE(centripetal <=
                  limits.max_centripetal_acceleration_mps2 + 1e-8);
    ROBOT_REQUIRE(std::abs(sample.predicted_left_voltage_V) <=
                  limits.max_voltage_V + 1e-8);
    ROBOT_REQUIRE(std::abs(sample.predicted_right_voltage_V) <=
                  limits.max_voltage_V + 1e-8);
    if (i > 0) {
      ROBOT_REQUIRE(sample.time_s >= trajectory[i - 1].time_s);
      ROBOT_REQUIRE(sample.distance_m >= trajectory[i - 1].distance_m);
      ROBOT_REQUIRE(std::hypot(sample.pose.x_m - trajectory[i - 1].pose.x_m,
                               sample.pose.y_m - trajectory[i - 1].pose.y_m) <
                    0.25);
    }
  }
}

}  // namespace

ROBOT_TEST("fixed trajectory preserves line endpoints and nonzero initial speed") {
  std::array<robot::PathWaypoint, 2> waypoints{{
      {{0.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Forward},
      {{2.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Forward},
  }};
  robot::FixedTrajectoryGenerator<2, 128> generator;
  robot::FixedTrajectory<128> trajectory;
  const auto limits = constraints();
  ROBOT_REQUIRE(generator.generate(waypoints, 2, limits, trajectory) ==
                robot::TrajectoryGenerationStatus::Success);
  ROBOT_REQUIRE_NEAR(trajectory[0].pose.x_m, 0.0, 0.0);
  ROBOT_REQUIRE_NEAR(trajectory[0].linear_velocity_mps, 0.2, 1e-12);
  const auto& end = trajectory[trajectory.size() - 1];
  ROBOT_REQUIRE_NEAR(end.pose.x_m, 2.0, 1e-12);
  ROBOT_REQUIRE_NEAR(end.pose.y_m, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(end.pose.theta_rad, 0.0, 1e-12);
  ROBOT_REQUIRE_NEAR(end.linear_velocity_mps, 0.0, 0.0);
  requireConstraints(trajectory, limits);
}

ROBOT_TEST("fixed trajectory supports reverse while preserving robot heading") {
  std::array<robot::PathWaypoint, 2> waypoints{{
      {{0.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Reverse},
      {{-2.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Reverse},
  }};
  robot::FixedTrajectoryGenerator<2, 128> generator;
  robot::FixedTrajectory<128> trajectory;
  const auto limits = constraints();
  ROBOT_REQUIRE(generator.generate(waypoints, 2, limits, trajectory) ==
                robot::TrajectoryGenerationStatus::Success);
  ROBOT_REQUIRE(trajectory[0].linear_velocity_mps < 0.0);
  const auto& end = trajectory[trajectory.size() - 1];
  ROBOT_REQUIRE_NEAR(end.pose.x_m, -2.0, 1e-12);
  ROBOT_REQUIRE_NEAR(robot::wrapPi(end.pose.theta_rad), 0.0, 1e-12);
  requireConstraints(trajectory, limits);
}

ROBOT_TEST("curved Hermite path honors angular traction and voltage limits") {
  std::array<robot::PathWaypoint, 2> waypoints{{
      {{0.0, 0.0, 0.0}, 1.4, robot::TravelDirection::Forward},
      {{1.0, 1.0, 0.5 * robot::units::kPi}, 1.4,
       robot::TravelDirection::Forward},
  }};
  robot::FixedTrajectoryGenerator<2, 128> generator;
  robot::FixedTrajectory<128> trajectory;
  auto limits = constraints(0.08);
  limits.initial_speed_mps = 0.0;
  ROBOT_REQUIRE(generator.generate(waypoints, 2, limits, trajectory) ==
                robot::TrajectoryGenerationStatus::Success);
  const auto& end = trajectory[trajectory.size() - 1];
  ROBOT_REQUIRE_NEAR(end.pose.x_m, 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(end.pose.y_m, 1.0, 1e-12);
  ROBOT_REQUIRE_NEAR(end.pose.theta_rad, 0.5 * robot::units::kPi, 1e-12);
  bool observed_curvature = false;
  for (std::size_t i = 0; i < trajectory.size(); ++i)
    observed_curvature |=
        std::abs(trajectory[i].path_curvature_per_m) > 0.1;
  ROBOT_REQUIRE(observed_curvature);
  requireConstraints(trajectory, limits);
}

ROBOT_TEST("multi-waypoint S path is finite continuous and fixed capacity") {
  std::array<robot::PathWaypoint, 3> waypoints{{
      {{0.0, 0.0, 0.0}, 1.0, robot::TravelDirection::Forward},
      {{1.0, 0.5, 0.0}, 1.0, robot::TravelDirection::Forward},
      {{2.0, 0.0, 0.0}, 1.0, robot::TravelDirection::Forward},
  }};
  robot::FixedTrajectoryGenerator<3, 128> generator;
  robot::FixedTrajectory<128> trajectory;
  auto limits = constraints(0.1);
  limits.initial_speed_mps = 0.0;
  ROBOT_REQUIRE(generator.generate(waypoints, 3, limits, trajectory) ==
                robot::TrajectoryGenerationStatus::Success);
  ROBOT_REQUIRE_NEAR(trajectory[trajectory.size() - 1].pose.x_m, 2.0,
                     1e-12);
  ROBOT_REQUIRE_NEAR(trajectory[trajectory.size() - 1].pose.y_m, 0.0,
                     1e-12);
  requireConstraints(trajectory, limits);
}

ROBOT_TEST("trajectory resampling periods preserve source and endpoint") {
  std::array<robot::PathWaypoint, 2> waypoints{{
      {{0.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Forward},
      {{2.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Forward},
  }};
  robot::FixedTrajectoryGenerator<2, 128> generator;
  robot::FixedTrajectory<128> source;
  auto limits = constraints(0.05);
  ROBOT_REQUIRE(generator.generate(waypoints, 2, limits, source) ==
                robot::TrajectoryGenerationStatus::Success);
  robot::FixedTrajectory<512> ten_ms;
  robot::FixedTrajectory<256> twenty_ms;
  ROBOT_REQUIRE(robot::resampleTrajectory(source, 0.01, ten_ms));
  ROBOT_REQUIRE(robot::resampleTrajectory(source, 0.02, twenty_ms));
  ROBOT_REQUIRE_NEAR(ten_ms.totalTime(), source.totalTime(), 1e-12);
  ROBOT_REQUIRE_NEAR(twenty_ms.totalTime(), source.totalTime(), 1e-12);
  const auto a = ten_ms.sampleAt(1.0);
  const auto b = twenty_ms.sampleAt(1.0);
  ROBOT_REQUIRE_NEAR(a.pose.x_m, b.pose.x_m, 0.005);
  ROBOT_REQUIRE_NEAR(a.linear_velocity_mps, b.linear_velocity_mps, 0.01);
  ROBOT_REQUIRE_NEAR(ten_ms[ten_ms.size() - 1].pose.x_m, 2.0, 1e-12);
  ROBOT_REQUIRE_NEAR(twenty_ms[twenty_ms.size() - 1].pose.x_m, 2.0, 1e-12);
}

ROBOT_TEST("trajectory generation rejects capacity direction and input errors") {
  std::array<robot::PathWaypoint, 2> waypoints{{
      {{0.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Forward},
      {{2.0, 0.0, 0.0}, 0.0, robot::TravelDirection::Forward},
  }};
  robot::FixedTrajectoryGenerator<2, 8> small_generator;
  robot::FixedTrajectory<8> small;
  ROBOT_REQUIRE(small_generator.generate(waypoints, 2, constraints(0.01),
                                         small) ==
                robot::TrajectoryGenerationStatus::CapacityExceeded);

  waypoints[1].direction = robot::TravelDirection::Reverse;
  robot::FixedTrajectoryGenerator<2, 64> generator;
  robot::FixedTrajectory<64> output;
  ROBOT_REQUIRE(generator.generate(waypoints, 2, constraints(), output) ==
                robot::TrajectoryGenerationStatus::DirectionChangeUnsupported);
  auto invalid = constraints();
  invalid.max_voltage_V = 13.0;
  ROBOT_REQUIRE(generator.generate(waypoints, 2, invalid, output) ==
                robot::TrajectoryGenerationStatus::InvalidInput);
}
