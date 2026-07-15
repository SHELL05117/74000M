#include "robot/calibration/calibration_tools.hpp"
#include "robot/calibration/characterization_runner.hpp"
#include "test_framework.hpp"

#include <cmath>

ROBOT_TEST("line fit separates training and validation metrics") {
  robot::FixedDataset<robot::LineFitSample, 8> data;
  for (int i = 0; i < 6; ++i)
    ROBOT_REQUIRE(data.add({static_cast<double>(i),
                            2.0 * static_cast<double>(i) + 0.5, i < 4,
                            true}));
  const auto fit = robot::fitLine(data);
  ROBOT_REQUIRE(fit.valid);
  ROBOT_REQUIRE_NEAR(fit.slope, 2.0, 1e-12);
  ROBOT_REQUIRE_NEAR(fit.intercept, 0.5, 1e-12);
  ROBOT_REQUIRE(fit.training.sample_count == 4);
  ROBOT_REQUIRE(fit.validation.sample_count == 2);
}

ROBOT_TEST("geometry helpers preserve calibration signs") {
  robot::FixedDataset<robot::LineFitSample, 6> distance;
  robot::FixedDataset<robot::LineFitSample, 6> parallel;
  robot::FixedDataset<robot::LineFitSample, 6> lateral;
  for (int i = 1; i <= 5; ++i) {
    const double input = static_cast<double>(i);
    const bool training = i <= 3;
    distance.add({input, 1.1 * input, training, true});
    parallel.add({input, -0.2 * input, training, true});
    lateral.add({input, 0.1 * input, training, true});
  }
  const auto scale = robot::fitDistanceScale(distance);
  const auto y_offset = robot::fitParallelTrackingOffset(parallel);
  const auto x_offset = robot::fitLateralTrackingOffset(lateral);
  ROBOT_REQUIRE(scale.valid && y_offset.valid && x_offset.valid);
  ROBOT_REQUIRE_NEAR(scale.value, 1.1, 1e-12);
  ROBOT_REQUIRE_NEAR(y_offset.value, 0.2, 1e-12);
  ROBOT_REQUIRE_NEAR(x_offset.value, 0.1, 1e-12);
}

ROBOT_TEST("feedforward fit recovers an exact synthetic model") {
  robot::FixedDataset<robot::SysIdSample, 32> data;
  for (int i = -8; i <= 8; ++i) {
    if (i == 0) continue;
    const double velocity = static_cast<double>(i) * 0.2;
    const double acceleration = static_cast<double>((i % 3) - 1) * 0.7;
    const double voltage = std::copysign(0.4, velocity) +
                           2.0 * velocity + 0.3 * acceleration;
    data.add({voltage, velocity, acceleration, std::abs(i) <= 6, true, 0});
  }
  const auto fit = robot::fitFeedforward(data, 0.01);
  ROBOT_REQUIRE(fit.valid);
  ROBOT_REQUIRE_NEAR(fit.kS_V, 0.4, 1e-12);
  ROBOT_REQUIRE_NEAR(fit.kV_Vs_per_m, 2.0, 1e-12);
  ROBOT_REQUIRE_NEAR(fit.kA_Vs2_per_m, 0.3, 1e-12);
  ROBOT_REQUIRE(fit.validation.sample_count > 0);
}

ROBOT_TEST("calibration provenance cannot reuse training as validation") {
  robot::CalibrationProvenance provenance{};
  provenance.schema_version = 1;
  provenance.robot_id_hash = 1;
  provenance.software_commit_hash = 2;
  provenance.training_dataset_hash = 3;
  provenance.validation_dataset_hash = 3;
  provenance.conditions_hash = 4;
  provenance.profile_state = robot::ProfileState::Draft;
  provenance.verification = robot::VerificationLevel::PCValidated;
  ROBOT_REQUIRE(!robot::validCalibrationProvenance(provenance));
  provenance.validation_dataset_hash = 5;
  ROBOT_REQUIRE(robot::validCalibrationProvenance(provenance));
}

ROBOT_TEST("characterization runner emits only bounded test requests") {
  robot::CharacterizationRunner runner;
  const robot::CharacterizationConfig config{
      robot::CharacterizationKind::Quasistatic,
      robot::CharacterizationDirection::Forward, 2.0, 4.0, 6.0, 1000000,
      2.0, 20000, 30000};
  const robot::FrameHeader start_header{100000, 1, 7};
  const robot::ModeSnapshot mode{robot::CompetitionMode::Test, true, false, 7,
                                 0, 0};
  robot::RobotState state{};
  state.h = start_header;
  const robot::OwnerToken owner{9, robot::Requirement::kDrivetrain, 3, 7};
  ROBOT_REQUIRE(runner.start(config, start_header, mode, state, owner, true));

  const robot::FrameHeader tick_header{600000, 2, 7};
  state.h = tick_header;
  const auto tick = runner.tick(tick_header, mode, state);
  ROBOT_REQUIRE(tick.has_request);
  ROBOT_REQUIRE(tick.request.source == robot::RequestSource::Test);
  ROBOT_REQUIRE_NEAR(tick.commanded_voltage_V, 1.0, 1e-12);
  const auto* voltage =
      std::get_if<robot::WheelVoltagePayload>(&tick.request.payload);
  ROBOT_REQUIRE(voltage != nullptr);
  ROBOT_REQUIRE_NEAR(voltage->left_V, 1.0, 1e-12);
}

ROBOT_TEST("characterization runner brakes at a configured boundary") {
  robot::CharacterizationRunner runner;
  const robot::CharacterizationConfig config{
      robot::CharacterizationKind::Dynamic,
      robot::CharacterizationDirection::Reverse, 2.0, 4.0, 6.0, 1000000,
      1.0, 20000, 30000};
  const robot::FrameHeader start_header{100000, 1, 7};
  const robot::ModeSnapshot mode{robot::CompetitionMode::Test, true, false, 7,
                                 0, 0};
  robot::RobotState state{};
  state.h = start_header;
  const robot::OwnerToken owner{9, robot::Requirement::kDrivetrain, 3, 7};
  ROBOT_REQUIRE(runner.start(config, start_header, mode, state, owner, true));
  const robot::FrameHeader stop_header{200000, 2, 7};
  state.h = stop_header;
  state.left_distance_m = 1.1;
  state.right_distance_m = 1.1;
  const auto stop = runner.tick(stop_header, mode, state);
  ROBOT_REQUIRE(stop.has_request);
  ROBOT_REQUIRE(stop.phase == robot::CharacterizationPhase::Stopping);
  ROBOT_REQUIRE((stop.stop_reason & robot::kCharacterizationDistanceLimit) !=
                0);
  ROBOT_REQUIRE(std::holds_alternative<robot::BrakePayload>(
      stop.request.payload));
}

ROBOT_TEST("characterization runner cannot bypass test authorization") {
  robot::CharacterizationRunner runner;
  const robot::CharacterizationConfig config{
      robot::CharacterizationKind::Dynamic,
      robot::CharacterizationDirection::Forward, 1.0, 2.0, 3.0, 100000,
      1.0, 20000, 20000};
  const robot::FrameHeader header{1000, 1, 1};
  const robot::ModeSnapshot driver{robot::CompetitionMode::Driver, true, false,
                                   1, 0, 0};
  robot::RobotState state{};
  state.h = header;
  const robot::OwnerToken owner{1, robot::Requirement::kDrivetrain, 1, 1};
  ROBOT_REQUIRE(!runner.start(config, header, driver, state, owner, true));
  ROBOT_REQUIRE(runner.phase() == robot::CharacterizationPhase::Aborted);
}
