#include "pros/llemu.hpp"
#include "pros/rtos.h"

#include "robot/config/robot_config.hpp"
#include "robot/core/snapshot_box.hpp"
#include "robot/drive/output_service.hpp"
#include "robot/hmi/render_models.hpp"
#include "robot/manual/commissioning_arcade.hpp"
#include "robot/platform/hardware_self_test.hpp"
#include "robot/platform/pros_adapters.hpp"
#include "robot/runtime/control_loop.hpp"
#include "robot/runtime/mailbox.hpp"
#include "robot/runtime/mode_manager.hpp"
#include "robot/runtime/output_task.hpp"
#include "robot/runtime/timing_monitor.hpp"

namespace {

constexpr char kExpectedRobotId[] = "1690X";
constexpr std::uint32_t kExpectedConfigSchema = 2;
constexpr std::uint32_t kStartupSelfCheckPollMs = 10;
constexpr std::uint32_t kControllerHmiPeriodMs = 100;

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
        cycle_(robot::make1690XCommissioningArcadeConfig()),
        control_loop_(clock_, drive_, controller_, competition_, modes_,
                      actuator_store_, timing_, cycle_, {10, true}),
        output_(drive_, {config_.runtime.output_ttl_us,
                         config_.electrical.max_command_voltage_V, 1e-9,
                         robot::kCommissioningStopMode}),
        output_task_(clock_, mode_store_, actuator_store_, output_, 5) {
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

  [[noreturn]] void runControl() { control_loop_.run(); }
  [[noreturn]] void runOutput() { output_task_.run(); }

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
  robot::ControlLoop control_loop_;
  robot::OutputService output_;
  robot::OutputTask output_task_;
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

void outputTaskEntry(void* parameter) {
  static_cast<RobotRuntime*>(parameter)->runOutput();
}

void controllerHmiTaskEntry(void* parameter) {
  static_cast<RobotRuntime*>(parameter)->runControllerHmi();
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

  pros::lcd::set_text(3, "SELF CHECK: RUNNING");
  const robot::StartupSelfCheckStatus self_check =
      runtime->runStartupSelfCheck();

  if (!hardware_initialized) {
    pros::lcd::set_text(3, "CONFIG/HARDWARE ERROR");
  } else if (output_task == nullptr) {
    pros::lcd::set_text(3, "OUTPUT TASK ERROR");
  } else if (controller_hmi_task == nullptr) {
    pros::lcd::set_text(3, "CONTROLLER HMI ERROR");
  } else if (!self_check.healthy) {
    pros::lcd::set_text(3, "READY: SENSOR WARNING");
  } else {
    pros::lcd::set_text(3, "READY: SELF CHECK OK");
  }
  pros::lcd::set_text(4, "ALL STOPS = COAST");
  pros::lcd::set_text(5, "LEFT Y/X ARCADE");
  pros::lcd::set_text(6, "B = HOLD TO COAST");
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
  pros::lcd::set_text(7, "TEST: DIRECT / B COAST");
  runtime->runControl();
}
