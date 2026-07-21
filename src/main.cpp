#include "pros/llemu.hpp"
#include "pros/rtos.h"

#include "robot/config/robot_config.hpp"
#include "robot/core/snapshot_box.hpp"
#include "robot/drive/output_service.hpp"
#include "robot/hmi/render_models.hpp"
#include "robot/manual/commissioning_curvature.hpp"
#include "robot/platform/hardware_self_test.hpp"
#include "robot/platform/pros_adapters.hpp"
#include "robot/platform/pros_recording_sink.hpp"
#include "robot/runtime/control_loop.hpp"
#include "robot/runtime/mailbox.hpp"
#include "robot/runtime/mode_manager.hpp"
#include "robot/runtime/output_task.hpp"
#include "robot/runtime/timing_monitor.hpp"
#include "robot/telemetry/flight_recorder.hpp"
#include "robot/telemetry/recording.hpp"

#ifndef ROBOT_SOURCE_COMMIT
#define ROBOT_SOURCE_COMMIT "UNKNOWN"
#endif

#ifndef ROBOT_BUILD_DIRTY
#define ROBOT_BUILD_DIRTY 1
#endif

namespace {

constexpr char kExpectedRobotId[] = "1690X";
constexpr std::uint32_t kExpectedConfigSchema = 3;
constexpr std::uint32_t kStartupSelfCheckPollMs = 10;
constexpr std::uint32_t kControllerHmiPeriodMs = 100;
constexpr std::uint32_t kTelemetryPeriodMs = 5;
constexpr std::size_t kTelemetryRingCapacity = 128;
constexpr std::size_t kTelemetryBatchSize = 8;

class ProsMutex {
 public:
  ProsMutex() : handle_(pros::c::mutex_create()) {}
  void lock() { pros::c::mutex_take(handle_, TIMEOUT_MAX); }
  void unlock() { pros::c::mutex_give(handle_); }

 private:
  pros::mutex_t handle_{};
};

class RobotRuntime {
 public:
  RobotRuntime()
      : config_(robot::make1690XCommissioningConfig()),
        drive_(config_, kExpectedRobotId, kExpectedConfigSchema),
        modes_(mode_store_),
        timing_({config_.runtime.nominal_period_s,
                 config_.runtime.min_math_dt_s,
                 config_.runtime.max_math_dt_s, 0.015}),
        cycle_(robot::make1690XCommissioningCurvatureConfig(),
               robot::make1690XLiftCommissioningConfig(),
               config_.hardware.lift),
        recording_metadata_(robot::makeRecordingMetadata(
            config_, clock_.nowUs(), ROBOT_SOURCE_COMMIT,
            ROBOT_BUILD_DIRTY != 0)),
        recorder_producer_(recording_control_, telemetry_ring_,
                           recording_metadata_.boot_id),
        control_loop_(clock_, drive_, controller_, competition_, modes_,
                      actuator_store_, timing_, cycle_, {10, true},
                      &recorder_producer_,
                      recording_metadata_.robot_id_hash, &output_status_),
        output_(drive_, {config_.runtime.output_ttl_us,
                         config_.electrical.max_command_voltage_V, 1e-9,
                         robot::kCommissioningStopMode}),
        output_task_(clock_, mode_store_, actuator_store_, output_, 5,
                     &output_status_),
        recording_sink_(recording_metadata_),
        recording_worker_(recording_control_, telemetry_ring_,
                          recording_sink_) {
    robot::HmiModel model{};
    model.controller_page = robot::ControllerPage::PoseDebug;
    model.translation_quality = robot::Quality::Invalid;
    model.heading_quality = robot::Quality::Invalid;
    robot::copyFixed(config_.identity.team_number, model.team_number);
    hmi_model_.publish(model);
    hmi_alert_sequence_.publish(0);
  }

  bool initializeHardware() {
    const robot::ConfigCheck check = robot::validateConfig(
        config_, kExpectedRobotId, kExpectedConfigSchema);
    config_valid_ = check.structurally_valid;
    hardware_initialized_ = config_valid_ && drive_.initialize();
    return hardware_initialized_;
  }

  robot::StartupSelfCheckStatus runStartupSelfCheck() {
    forceDisabled();
    const bool imu_calibration_started =
        config_.hardware.imu.installed && drive_.beginImuCalibration();
    self_check_.begin(clock_.nowUs(), config_.hardware, config_valid_,
                      hardware_initialized_, imu_calibration_started);

    std::uint32_t sequence{};
    robot::StartupSelfCheckStatus status{};
    do {
      const robot::TimeUs now_us = clock_.nowUs();
      const robot::ModeSnapshot mode = modes_.snapshot();
      const robot::FrameHeader header{now_us, ++sequence, mode.epoch};
      const robot::RawDriveInputs raw = drive_.readAll(header);
      status = self_check_.tick(clock_.nowUs(), raw);
      if (!status.complete) pros::c::task_delay(kStartupSelfCheckPollMs);
    } while (!status.complete);

    self_check_complete_ = true;
    self_check_fault_bits_ = status.fault_bits;
    if (!status.healthy) hmi_alert_sequence_.publish(1);
    return status;
  }

  bool ready() const noexcept {
    return hardware_initialized_ && self_check_complete_;
  }

  std::uint32_t selfCheckFaultBits() const noexcept {
    return self_check_fault_bits_;
  }

  void setRecorderAvailable(bool available) noexcept {
    recording_control_.setAvailable(available);
  }

  [[noreturn]] void runControl() { control_loop_.run(); }
  [[noreturn]] void runOutput() { output_task_.run(); }

  [[noreturn]] void runTelemetry() {
    std::uint32_t handled_alert_sequence{};
    std::uint32_t wake_ms = clock_.nowMs();
    while (true) {
      recording_worker_.tickOnce();
      const std::uint32_t requested =
          recording_control_.alertSequence();
      if (requested != handled_alert_sequence) {
        hmi_alert_sequence_.publish(hmi_alert_sequence_.read() + 1);
        handled_alert_sequence = requested;
      }
      clock_.delayUntilMs(wake_ms, kTelemetryPeriodMs);
    }
  }

  [[noreturn]] void runControllerHmi() {
    std::uint32_t handled_alert_sequence{};
    std::uint32_t wake_ms = clock_.nowMs();
    while (true) {
      const std::uint32_t requested_alert = hmi_alert_sequence_.read();
      if (requested_alert != handled_alert_sequence) {
        // Three distinct short pulses. The event remains pending and is
        // retried if the Controller was not connected at the first attempt.
        if (controller_display_.rumble(". . ."))
          handled_alert_sequence = requested_alert;
      } else {
        controller_renderer_.tick(hmi_model_.read(), controller_display_);
      }
      clock_.delayUntilMs(wake_ms, kControllerHmiPeriodMs);
    }
  }

  void forceDisabled() {
    recording_control_.requestStop();
    modes_.transitionTo(robot::CompetitionMode::Disabled, clock_.nowUs());
  }

 private:
  robot::RobotConfig config_;
  robot::ProsClock clock_{};
  robot::ProsDriveIO drive_;
  robot::ProsControllerIO controller_{};
  robot::ProsControllerDisplayIO controller_display_{};
  robot::ProsCompetitionIO competition_{};
  robot::LockedModeStore<ProsMutex> mode_store_{};
  robot::LockedActuatorStore<ProsMutex> actuator_store_{};
  robot::ModeManager modes_;
  robot::TimingMonitor timing_;
  robot::CommissioningControlCycle cycle_;
  robot::RecordingControl recording_control_{};
  robot::SpscRing<robot::LogFrame, kTelemetryRingCapacity> telemetry_ring_{};
  robot::RecordingMetadata recording_metadata_{};
  robot::FlightRecorderProducer<kTelemetryRingCapacity>
      recorder_producer_;
  robot::AtomicOutputStatusStore output_status_{};
  robot::ControlLoop control_loop_;
  robot::OutputService output_;
  robot::OutputTask output_task_;
  robot::ProsRecordingSessionSink recording_sink_;
  robot::RecordingWorker<kTelemetryRingCapacity, kTelemetryBatchSize>
      recording_worker_;
  robot::StartupSelfCheck self_check_{};
  robot::ControllerPoseRenderer controller_renderer_{};
  robot::SnapshotBox<robot::HmiModel, ProsMutex> hmi_model_{};
  robot::SnapshotBox<std::uint32_t, ProsMutex> hmi_alert_sequence_{};
  std::uint32_t self_check_fault_bits_{};
  bool config_valid_{};
  bool hardware_initialized_{};
  bool self_check_complete_{};
};

RobotRuntime* runtime{};
pros::task_t output_task{};
pros::task_t controller_hmi_task{};
pros::task_t telemetry_task{};

void outputTaskEntry(void* parameter) {
  static_cast<RobotRuntime*>(parameter)->runOutput();
}

void controllerHmiTaskEntry(void* parameter) {
  static_cast<RobotRuntime*>(parameter)->runControllerHmi();
}

void telemetryTaskEntry(void* parameter) {
  static_cast<RobotRuntime*>(parameter)->runTelemetry();
}

}  // namespace

extern "C" void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "1690X 6-MOTOR SAMPLE");
  pros::lcd::set_text(1, "COMMISSIONING: 12V MAX");
  pros::lcd::set_text(2, "NO IMU / AUTO LOCKED");

  runtime = new RobotRuntime();
  const bool hardware_initialized = runtime->initializeHardware();

  if (hardware_initialized) {
    output_task = pros::c::task_create(outputTaskEntry, runtime,
                                       TASK_PRIORITY_DEFAULT + 1,
                                       TASK_STACK_DEPTH_DEFAULT,
                                       "drive_output");
  }

  controller_hmi_task = pros::c::task_create(
      controllerHmiTaskEntry, runtime, TASK_PRIORITY_DEFAULT - 1,
      TASK_STACK_DEPTH_DEFAULT, "controller_hmi");
  telemetry_task = pros::c::task_create(
      telemetryTaskEntry, runtime, TASK_PRIORITY_DEFAULT - 1,
      TASK_STACK_DEPTH_DEFAULT, "flight_recorder");
  runtime->setRecorderAvailable(telemetry_task != nullptr);

  pros::lcd::set_text(3, "SELF CHECK: RUNNING");
  const robot::StartupSelfCheckStatus self_check =
      runtime->runStartupSelfCheck();

  if (!hardware_initialized) {
    pros::lcd::set_text(3, "CONFIG/HARDWARE ERROR");
  } else if (output_task == nullptr) {
    pros::lcd::set_text(3, "OUTPUT TASK ERROR");
  } else if (controller_hmi_task == nullptr) {
    pros::lcd::set_text(3, "CONTROLLER HMI ERROR");
  } else if (telemetry_task == nullptr) {
    pros::lcd::set_text(3, "RECORDER TASK ERROR");
  } else if (!self_check.healthy) {
    pros::lcd::set_text(3, "READY: SENSOR WARNING");
  } else {
    pros::lcd::set_text(3, "READY: SELF CHECK OK");
  }
  pros::lcd::set_text(4, "DRIVE COAST / LIFT HOLD");
  pros::lcd::set_text(5, "LEFT Y/X DRIVE; RIGHT Y LIFT");
  pros::lcd::set_text(6, "BOOT LIFT AT LOWER STOP");
}

extern "C" void disabled() {
  if (runtime != nullptr) runtime->forceDisabled();
  pros::lcd::set_text(7, "DISABLED / COAST");
}

extern "C" void competition_initialize() {
  if (runtime != nullptr) runtime->forceDisabled();
}

extern "C" void autonomous() {
  if (runtime != nullptr) runtime->forceDisabled();
  pros::lcd::set_text(7, "AUTO: DO NOTHING");
}

extern "C" void opcontrol() {
  if (runtime == nullptr || !runtime->ready() || output_task == nullptr ||
      controller_hmi_task == nullptr) {
    pros::lcd::set_text(7, "DRIVE: LOCKED");
    return;
  }
  pros::lcd::set_text(7, "TEST: LEFT COAST / Y REC");
  runtime->runControl();
}
