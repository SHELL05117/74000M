#include "robot/commands/drive_request_arbiter.hpp"
#include "robot/drive/output_service.hpp"
#include "robot/drive/safety_gate.hpp"
#include "robot/platform/fake_io.hpp"
#include "test_framework.hpp"

#include <array>
#include <limits>

namespace {

robot::HardwareConfig fakeHardware() {
  robot::HardwareConfig hardware{};
  hardware.left = {{{1, false, 200}, {2, false, 200}, {3, false, 200}}};
  hardware.right = {{{4, true, 200}, {5, true, 200}, {6, true, 200}}};
  hardware.imu = {true, 7};
  return hardware;
}

class OneLease final : public robot::LeaseAuthority {
 public:
  explicit OneLease(robot::OwnerToken owner) noexcept : owner_(owner) {}
  bool owns(const robot::OwnerToken& token) const noexcept override {
    return robot::sameLease(owner_, token);
  }

 private:
  robot::OwnerToken owner_{};
};

robot::DriveRequest driverRequest(robot::TimeUs time_us,
                                  std::uint32_t sequence,
                                  std::uint32_t epoch,
                                  const robot::OwnerToken& owner) {
  robot::DriveRequest request{};
  request.h = {time_us, sequence, epoch};
  request.source = robot::RequestSource::Driver;
  request.owner = owner;
  request.ttl_us = 30000;
  request.payload = robot::DriverCurvaturePayload{
      0.5, 0.2, robot::DriverSteeringMode::Curvature,
      robot::AllocationPolicy::RatioPreserving};
  return request;
}

robot::SafetyGateInput gateInput(robot::TimeUs now_us,
                                 std::uint32_t sequence,
                                 const robot::ModeSnapshot& mode) {
  return {{now_us, sequence, mode.epoch}, mode, now_us, 0.01, 1.0,
          {true, false, false}};
}

}  // namespace

ROBOT_TEST("freeze chain routes an owned request through both safety TTLs") {
  robot::FakeDriveIO io(fakeHardware(), {5.0, 8.0});
  ROBOT_REQUIRE(io.initialize());
  robot::DriveRequestArbiter arbiter({40000});
  robot::SafetyGate gate({12.0, 40000, {1000.0, 1000.0, 0.05},
                          robot::StopMode::Brake, robot::StopMode::Brake,
                          robot::StopMode::Brake});
  robot::OutputService output(
      io, {20000, 12.0, 1e-9, robot::StopMode::Brake});
  const robot::ModeSnapshot driver{robot::CompetitionMode::Driver, true, false,
                                   7, 0, 0};
  const robot::OwnerToken owner{1, robot::Requirement::kDrivetrain, 3, 7};
  const OneLease authority(owner);
  robot::DriveCapabilities capabilities{};
  capabilities.driver_curvature = true;
  std::array<robot::DriveRequestCandidate, 1> candidates{{
      {driverRequest(10000, 1, 7, owner), true},
  }};

  auto selected =
      arbiter.select(candidates, driver, 10000, capabilities, authority);
  ROBOT_REQUIRE(selected.has_selection);
  auto frame = gate.apply(&selected.selected, gateInput(10000, 1, driver));
  ROBOT_REQUIRE(frame.left_V > 0.0 && frame.right_V > frame.left_V);
  const auto first = output.tick(driver, &frame, 10000);
  ROBOT_REQUIRE(first.action == robot::OutputAction::WroteVoltage);
  ROBOT_REQUIRE(io.writeCount() == 1);

  selected = arbiter.select(candidates, driver, 40001, capabilities, authority);
  ROBOT_REQUIRE(!selected.has_selection);
  ROBOT_REQUIRE((selected.reject_bits & robot::kArbitrationStale) != 0);
  frame = gate.apply(nullptr, gateInput(40001, 2, driver));
  ROBOT_REQUIRE((frame.applied_limits & robot::kAppliedNoRequestStop) != 0);
  ROBOT_REQUIRE(output.tick(driver, &frame, 40001).action ==
                robot::OutputAction::Stopped);
  ROBOT_REQUIRE(io.leftCommandV() == 0.0 && io.rightCommandV() == 0.0);

  candidates[0].request = driverRequest(50000, 2, 7, owner);
  selected = arbiter.select(candidates, driver, 50000, capabilities, authority);
  ROBOT_REQUIRE(selected.has_selection);
  frame = gate.apply(&selected.selected, gateInput(50000, 3, driver));
  const auto stale_output = output.tick(driver, &frame, 70001);
  ROBOT_REQUIRE(stale_output.action == robot::OutputAction::Stopped);
  ROBOT_REQUIRE((stale_output.reject_bits & robot::kOutputStale) != 0);
}

ROBOT_TEST("freeze chain rejects disabled old epoch and malformed requests") {
  robot::FakeDriveIO io(fakeHardware(), {5.0, 8.0});
  ROBOT_REQUIRE(io.initialize());
  robot::DriveRequestArbiter arbiter({40000});
  robot::SafetyGate gate({12.0, 40000, {1000.0, 1000.0, 0.05},
                          robot::StopMode::Brake, robot::StopMode::Brake,
                          robot::StopMode::Brake});
  robot::OutputService output(
      io, {20000, 12.0, 1e-9, robot::StopMode::Brake});
  const robot::OwnerToken old_owner{1, robot::Requirement::kDrivetrain, 3, 7};
  const OneLease authority(old_owner);
  robot::DriveCapabilities capabilities{};
  capabilities.driver_curvature = true;
  std::array<robot::DriveRequestCandidate, 1> candidates{{
      {driverRequest(10000, 1, 7, old_owner), true},
  }};
  const robot::ModeSnapshot next_driver{robot::CompetitionMode::Driver, true,
                                        false, 8, 0, 0};
  auto selected = arbiter.select(candidates, next_driver, 10000, capabilities,
                                 authority);
  ROBOT_REQUIRE(!selected.has_selection);
  ROBOT_REQUIRE((selected.reject_bits & robot::kArbitrationEpochMismatch) != 0);

  robot::ActuatorFrame old_frame{{10000, 1, 7}, 12.0, 12.0,
                                 robot::StopMode::Brake,
                                 robot::RequestSource::Driver, 1, 3, 0};
  const auto epoch_stop = output.tick(next_driver, &old_frame, 10000);
  ROBOT_REQUIRE(epoch_stop.action == robot::OutputAction::Stopped);
  ROBOT_REQUIRE((epoch_stop.reject_bits & robot::kOutputEpochMismatch) != 0);
  ROBOT_REQUIRE(io.writeCount() == 0);

  const robot::ModeSnapshot disabled{robot::CompetitionMode::Disabled, false,
                                     false, 8, 0, 0};
  selected =
      arbiter.select(candidates, disabled, 10000, capabilities, authority);
  ROBOT_REQUIRE(!selected.has_selection);
  ROBOT_REQUIRE((selected.reject_bits & robot::kArbitrationModeDisabled) != 0);
  const auto stop_frame = gate.apply(nullptr, gateInput(10000, 2, disabled));
  ROBOT_REQUIRE((stop_frame.applied_limits & robot::kAppliedDisabledStop) != 0);

  auto malformed = driverRequest(10000, 2, 7, old_owner);
  std::get<robot::DriverCurvaturePayload>(malformed.payload).forward =
      std::numeric_limits<double>::quiet_NaN();
  candidates[0].request = malformed;
  const robot::ModeSnapshot old_driver{robot::CompetitionMode::Driver, true,
                                       false, 7, 0, 0};
  selected =
      arbiter.select(candidates, old_driver, 10000, capabilities, authority);
  ROBOT_REQUIRE(!selected.has_selection);
  ROBOT_REQUIRE((selected.reject_bits & robot::kArbitrationBadPayload) != 0);
}
