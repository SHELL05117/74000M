#include "pros/llemu.hpp"
#include "pros/rtos.h"

#include "robot/config/robot_config.hpp"
#include "robot/drive/output_service.hpp"
#include "robot/manual/commissioning_arcade.hpp"
#include "robot/platform/pros_adapters.hpp"
#include "robot/runtime/control_loop.hpp"
#include "robot/runtime/mailbox.hpp"
#include "robot/runtime/mode_manager.hpp"
#include "robot/runtime/output_task.hpp"
#include "robot/runtime/timing_monitor.hpp"

namespace {

constexpr char kExpectedRobotId[] = "1690X";
constexpr std::uint32_t kExpectedConfigSchema = 2;

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
        output_task_(clock_, mode_store_, actuator_store_, output_, 5) {}

  bool initializeHardware() {
    const robot::ConfigCheck check = robot::validateConfig(
        config_, kExpectedRobotId, kExpectedConfigSchema);
    ready_ = check.structurally_valid && drive_.initialize();
    return ready_;
  }

  bool ready() const noexcept { return ready_; }

  [[noreturn]] void runControl() { control_loop_.run(); }
  [[noreturn]] void runOutput() { output_task_.run(); }

  void forceDisabled() {
    modes_.transitionTo(robot::CompetitionMode::Disabled, clock_.nowUs());
  }

 private:
  robot::RobotConfig config_;
  robot::ProsClock clock_{};
  robot::ProsDriveIO drive_;
  robot::ProsControllerIO controller_{};
  robot::ProsCompetitionIO competition_{};
  robot::LockedModeStore<ProsMutex> mode_store_{};
  robot::LockedActuatorStore<ProsMutex> actuator_store_{};
  robot::ModeManager modes_;
  robot::TimingMonitor timing_;
  robot::CommissioningControlCycle cycle_;
  robot::ControlLoop control_loop_;
  robot::OutputService output_;
  robot::OutputTask output_task_;
  bool ready_{};
};

RobotRuntime* runtime{};
pros::task_t output_task{};

void outputTaskEntry(void* parameter) {
  static_cast<RobotRuntime*>(parameter)->runOutput();
}

}  // namespace

extern "C" void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "1690X 6-MOTOR SAMPLE");
  pros::lcd::set_text(1, "COMMISSIONING: 12V MAX");
  pros::lcd::set_text(2, "NO IMU / AUTO LOCKED");

  runtime = new RobotRuntime();
  if (!runtime->initializeHardware()) {
    pros::lcd::set_text(3, "CONFIG/HARDWARE ERROR");
    return;
  }

  output_task = pros::c::task_create(outputTaskEntry, runtime,
                                     TASK_PRIORITY_DEFAULT + 1,
                                     TASK_STACK_DEPTH_DEFAULT,
                                     "drive_output");
  if (output_task == nullptr) {
    runtime->forceDisabled();
    pros::lcd::set_text(3, "OUTPUT TASK ERROR");
    return;
  }
  pros::lcd::set_text(3, "READY: DIRECT DRIVE");
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
  if (runtime == nullptr || !runtime->ready() || output_task == nullptr) {
    pros::lcd::set_text(7, "DRIVE: LOCKED");
    return;
  }
  pros::lcd::set_text(7, "TEST: DIRECT / B COAST");
  runtime->runControl();
}
