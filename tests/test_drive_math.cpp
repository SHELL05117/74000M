#include "robot/drive/kinematics.hpp"
#include "robot/drive/output_slew.hpp"
#include "robot/drive/safety_gate.hpp"
#include "robot/drive/voltage_allocation.hpp"
#include "test_framework.hpp"

#include <cmath>
#include <limits>

namespace {

robot::SafetyGateConfig gateConfig() {
  return {12.0, 40000, {10000.0, 10000.0, 0.050},
          robot::StopMode::Brake, robot::StopMode::Brake,
          robot::StopMode::Brake};
}

robot::ModeSnapshot driverMode(std::uint32_t epoch = 3) {
  return {robot::CompetitionMode::Driver, true, false, epoch, 0, 0};
}

robot::DriveRequest driverRequest(robot::TimeUs time_us,
                                  std::uint32_t epoch,
                                  robot::DriverCurvaturePayload payload) {
  robot::DriveRequest request{};
  request.h = {time_us, 7, epoch};
  request.source = robot::RequestSource::Driver;
  request.owner = {1, robot::Requirement::kDrivetrain, 2, epoch};
  request.ttl_us = 40000;
  request.payload = payload;
  return request;
}

robot::SafetyGateInput gateInput(robot::TimeUs now_us,
                                 std::uint32_t epoch = 3) {
  robot::SafetyGateInput input{};
  input.output_header = {now_us, 9, epoch};
  input.mode = driverMode(epoch);
  input.now_us = now_us;
  input.math_dt_s = 0.010;
  input.output_derate = 1.0;
  input.capabilities.driver_curvature = true;
  return input;
}

}  // namespace

ROBOT_TEST("differential kinematics has the frozen signs and round trips") {
  robot::DifferentialKinematics kinematics(0.30);
  const robot::ChassisSpeeds cases[] = {
      {0.0, 0.0}, {1.0, 0.0}, {0.0, 2.0}, {-0.7, 1.3}, {0.4, -3.0}};
  for (const auto& input : cases) {
    robot::WheelSpeeds wheels{};
    ROBOT_REQUIRE(kinematics.inverse(input, wheels));
    robot::ChassisSpeeds output{};
    ROBOT_REQUIRE(kinematics.forward(wheels, output));
    ROBOT_REQUIRE_NEAR(output.vx_mps, input.vx_mps, 1e-12);
    ROBOT_REQUIRE_NEAR(output.omega_radps, input.omega_radps, 1e-12);
  }
  robot::WheelSpeeds turn{};
  ROBOT_REQUIRE(kinematics.inverse({0.0, 2.0}, turn));
  ROBOT_REQUIRE(turn.left_mps < 0.0);
  ROBOT_REQUIRE(turn.right_mps > 0.0);
}

ROBOT_TEST("kinematics rejects unknown or nonfinite geometry") {
  robot::WheelSpeeds output{1.0, 1.0};
  ROBOT_REQUIRE(!robot::DifferentialKinematics(0.0).inverse({1.0, 0.0},
                                                            output));
  ROBOT_REQUIRE_NEAR(output.left_mps, 0.0, 0.0);
  robot::DifferentialKinematics valid(0.3);
  ROBOT_REQUIRE(!valid.inverse(
      {std::numeric_limits<double>::quiet_NaN(), 0.0}, output));
}

ROBOT_TEST("voltage allocation preserves the selected invariant") {
  const auto proportional =
      robot::desaturateProportional({10.0, 14.0}, 12.0);
  ROBOT_REQUIRE(proportional.valid && proportional.limited);
  ROBOT_REQUIRE_NEAR(proportional.output.left_V, 10.0 * 12.0 / 14.0,
                     1e-12);
  ROBOT_REQUIRE_NEAR(proportional.output.right_V, 12.0, 1e-12);

  const auto turn = robot::desaturatePreserveTurn({10.0, 14.0}, 12.0);
  ROBOT_REQUIRE(turn.valid && turn.limited);
  ROBOT_REQUIRE_NEAR(turn.output.left_V, 8.0, 1e-12);
  ROBOT_REQUIRE_NEAR(turn.output.right_V, 12.0, 1e-12);
  ROBOT_REQUIRE_NEAR((turn.output.right_V - turn.output.left_V) * 0.5,
                     2.0, 1e-12);
}

ROBOT_TEST("allocation properties hold over a deterministic input grid") {
  for (int left = -24; left <= 24; ++left) {
    for (int right = -24; right <= 24; ++right) {
      const robot::WheelVoltages input{static_cast<double>(left),
                                       static_cast<double>(right)};
      const auto proportional =
          robot::desaturateProportional(input, 12.0);
      const auto turn = robot::desaturatePreserveTurn(input, 12.0);
      ROBOT_REQUIRE(proportional.valid && turn.valid);
      ROBOT_REQUIRE(std::abs(proportional.output.left_V) <= 12.0);
      ROBOT_REQUIRE(std::abs(proportional.output.right_V) <= 12.0);
      ROBOT_REQUIRE(std::abs(turn.output.left_V) <= 12.0);
      ROBOT_REQUIRE(std::abs(turn.output.right_V) <= 12.0);
      const double input_difference = 0.5 * (input.right_V - input.left_V);
      const double output_difference =
          0.5 * (turn.output.right_V - turn.output.left_V);
      ROBOT_REQUIRE_NEAR(output_difference,
                         std::clamp(input_difference, -12.0, 12.0), 1e-12);
    }
  }
}

ROBOT_TEST("asymmetric slew crosses reversal through zero") {
  robot::AsymmetricVoltageSlew slew({10.0, 20.0, 0.050});
  double output{};
  ROBOT_REQUIRE(slew.update(12.0, 0.050, output));
  ROBOT_REQUIRE_NEAR(output, 0.5, 1e-12);
  ROBOT_REQUIRE(slew.update(-12.0, 0.050, output));
  ROBOT_REQUIRE_NEAR(output, 0.0, 1e-12);
  ROBOT_REQUIRE(slew.update(-12.0, 0.050, output));
  ROBOT_REQUIRE_NEAR(output, -0.5, 1e-12);
}

ROBOT_TEST("safety gate maps curvature and applies proportional allocation") {
  robot::SafetyGate gate(gateConfig());
  auto request = driverRequest(
      100000, 3,
      {1.0, 0.5, robot::DriverSteeringMode::Curvature,
       robot::AllocationPolicy::RatioPreserving});
  const auto frame = gate.apply(&request, gateInput(100000));
  ROBOT_REQUIRE_NEAR(frame.left_V, 4.0, 1e-12);
  ROBOT_REQUIRE_NEAR(frame.right_V, 12.0, 1e-12);
  ROBOT_REQUIRE((frame.applied_limits &
                 robot::kAppliedProportionalDesaturation) != 0);
}

ROBOT_TEST("safety gate preserves heading-assist differential under derate") {
  robot::SafetyGate gate(gateConfig());
  auto request = driverRequest(
      100000, 3,
      {1.0, 0.25, robot::DriverSteeringMode::HeadingAssist,
       robot::AllocationPolicy::PreserveTurn});
  auto input = gateInput(100000);
  input.output_derate = 0.5;
  const auto frame = gate.apply(&request, input);
  ROBOT_REQUIRE(std::abs(frame.left_V) <= 6.0);
  ROBOT_REQUIRE(std::abs(frame.right_V) <= 6.0);
  ROBOT_REQUIRE_NEAR((frame.right_V - frame.left_V) * 0.5, 3.0, 1e-12);
  ROBOT_REQUIRE((frame.applied_limits & robot::kAppliedDerate) != 0);
  ROBOT_REQUIRE((frame.applied_limits & robot::kAppliedPreserveTurn) != 0);
}

ROBOT_TEST("safety gate stops stale epoch-disabled and unsupported requests") {
  robot::SafetyGate gate(gateConfig());
  auto request = driverRequest(
      100000, 3,
      {0.5, 0.0, robot::DriverSteeringMode::Curvature,
       robot::AllocationPolicy::RatioPreserving});
  auto stale_input = gateInput(140001);
  auto frame = gate.apply(&request, stale_input);
  ROBOT_REQUIRE(frame.owner == robot::RequestSource::Safety);
  ROBOT_REQUIRE((frame.applied_limits & robot::kAppliedInvalidRequestStop) !=
                0);

  auto disabled_input = gateInput(150000);
  disabled_input.mode.enabled = false;
  frame = gate.apply(&request, disabled_input);
  ROBOT_REQUIRE((frame.applied_limits & robot::kAppliedDisabledStop) != 0);

  robot::DriveRequest velocity = request;
  velocity.h.time_us = 160000;
  velocity.payload = robot::ChassisVelocityPayload{1.0, 0.0};
  auto velocity_input = gateInput(160000);
  velocity_input.capabilities.autonomous_chassis_velocity = true;
  velocity_input.mode.mode = robot::CompetitionMode::AutonomousInterface;
  velocity.source = robot::RequestSource::FutureAutonomy;
  frame = gate.apply(&velocity, velocity_input);
  ROBOT_REQUIRE((frame.applied_limits & robot::kAppliedUnsupportedStop) != 0);
}

ROBOT_TEST("safety gate resets slew state on a mode epoch boundary") {
  robot::SafetyGateConfig config = gateConfig();
  config.output_slew = {10.0, 20.0, 0.050};
  robot::SafetyGate gate(config);
  auto request = driverRequest(
      100000, 3,
      {1.0, 0.0, robot::DriverSteeringMode::Curvature,
       robot::AllocationPolicy::RatioPreserving});
  auto frame = gate.apply(&request, gateInput(100000));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.1, 1e-12);

  request = driverRequest(
      110000, 4,
      {1.0, 0.0, robot::DriverSteeringMode::Curvature,
       robot::AllocationPolicy::RatioPreserving});
  frame = gate.apply(&request, gateInput(110000, 4));
  ROBOT_REQUIRE_NEAR(frame.left_V, 0.1, 1e-12);
}
