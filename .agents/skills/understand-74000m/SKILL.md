---
name: understand-74000m
description: Rapid project orientation for the 74000M VEX V5 PROS/C++17 repository. Use before any agent plans, reads, reviews, debugs, tests, documents, or modifies files under C:\Users\alexh\Documents\override\74000M\74000M to understand the architecture, technology stack, directory ownership, key APIs, coding patterns, PC tests, PROS builds, current commissioning limits, and Git/change-log workflow.
---

# Understand 74000M

Read this file completely before working in the repository. Then follow the stricter safety and document gates in `../../../74000pros.md`; this guide explains the codebase but never relaxes those rules.

## Project snapshot

- Target: VEX V5 differential-drive robot, current sample identity `1690X`.
- Runtime: PROS kernel, C++17, ARM firmware built with the PROS Makefiles.
- Host validation: CMake 3.20+ and CTest on Windows or another desktop C++17 compiler.
- Current chassis: six motors, three per side, mixed 600 RPM and 200 RPM cartridges with external gearing normalized to the same nominal wheel speed.
- Current executable composition: sensorless sample commissioning firmware, not competition-approved production firmware. IMU, tracking wheels, pose-good capability, autonomous motion, and competition routes remain locked; `autonomous()` is `DoNothing`.
- All formal `RobotCapabilities` are false. The only output exception is the narrow `CommissioningControlCycle`, which enables `controlled_test_voltage` locally only in non-field `Test` mode and still uses the scheduler, arbiter, safety gate, TTLs, and sole output service.
- Driver test: left-stick single-stick Curvature, hold `R1` for low-throttle Quick Turn, 12 V command ceiling, Coast stops, hold `B` to force Coast.
- Startup: at least three seconds of self-check. The current profile intentionally declares no IMU, so every boot is expected to finish with a sensor warning, request three Controller rumble pulses, and display `X/Y/H:ERR`; any other unhealthy configured-device check uses the same warning event.
- Source of current hardware truth: `include/robot/config/robot_config.hpp` plus the traceable manifest `config/hardware_profile.yaml`. The YAML is not loaded at runtime.

Re-read the current config and `src/main.cpp` before reporting hardware or capability status; this snapshot can become stale after later commits.

## Technology stack

| Area | Technology |
|---|---|
| Robot OS/API | PROS for VEX V5; current project metadata identifies kernel 4.2.2 |
| Language | C++17; fixed-capacity, allocation-free control-path style |
| Firmware build | `Makefile`, `common.mk`, ARM `arm-none-eabi` toolchain |
| PC build/test | CMake, CTest, the small framework in `tests/test_framework.hpp` |
| CI | `.github/workflows/pc-tests.yml` |
| Units | SI in core code; degree, RPM, mV, and device conventions only in platform adapters |
| Documentation | Project docs in `docs/`; mandatory design library in read-only `C:\Users\alexh\Documents\VEX0713` |

`include/pros/`, `include/liblvgl/`, and `firmware/` are vendor/framework content. Do not modify them for normal application work.

## Runtime architecture

The only legal drivetrain output chain is:

```text
PROS lifecycle / Controller snapshot
  -> ControlLoop
  -> Command or ManualDrive
  -> DriveRequestSink
  -> Scheduler / DriveRequestArbiter
  -> drivetrain control
  -> SafetyGate
  -> ActuatorFrame mailbox
  -> independent OutputTask
  -> OutputService
  -> DriveIO
  -> PROS motor API
```

Key ownership rules:

- `src/main.cpp` is the composition root. It constructs long-lived objects and implements `initialize`, `disabled`, `competition_initialize`, `autonomous`, and `opcontrol`.
- `ControlLoop` samples each hardware input once per nominal 10 ms tick and publishes an `ActuatorFrame`; it does not write motors.
- `OutputTask` runs independently and calls `OutputService`; this is the only drivetrain motor writer and enforces mode, epoch, TTL, finite-value, and voltage checks.
- `ModeManager` performs break-before-make transitions with a mode epoch.
- Controller display work runs in a slower task and consumes snapshots; it never owns motor output.
- Core algorithms depend on project interfaces and value types, never directly on PROS devices.

## Directory map

| Path | Responsibility |
|---|---|
| `src/main.cpp` | PROS lifecycle, task creation, startup self-check, current 1690X composition |
| `src/platform/pros_adapters.cpp` | The physical PROS boundary and device/unit conversions |
| `src/robot.cpp` | Small host-linkable library translation unit |
| `include/robot/robot.hpp` | Public umbrella header |
| `include/robot/core/` | Units, frame headers, quality, faults, build info, snapshot box |
| `include/robot/config/` | Robot identity, ports, geometry, calibration, capabilities, config validation |
| `include/robot/platform/` | IO interfaces, Fake IO, PROS declarations, hardware self-tests |
| `include/robot/state/` | Raw inputs, Controller snapshot, pose, complete robot state |
| `include/robot/sensors/` | Validation, filtering, port-level fault isolation |
| `include/robot/drive/` | Request/output types, kinematics, allocation, Slew, safety gate, unique output service |
| `include/robot/runtime/` | Mode lifecycle, control/output loops, timing, cross-task mailboxes |
| `include/robot/manual/` | Input shaping, manual drive, heading assist, 1690X commissioning Curvature cycle |
| `include/robot/commands/` | Static command scheduler, requirements, leases, request sink, command groups |
| `include/robot/control/` | PID, feedforward, motion profiles, termination, chassis velocity controller |
| `include/robot/odometry/` | IMU-first drive/tracking-wheel pose estimation |
| `include/robot/autonomy/` | Motion commands, trajectory generation/tracking, routes, autonomous safety |
| `include/robot/health/` | Fault state machine, derating, motor thermal/current/stall protection |
| `include/robot/calibration/` | Geometry calibration and SysId/characterization tools |
| `include/robot/telemetry/` | Fixed log frames, SPSC ring, integrity, replay, background task logic |
| `include/robot/hmi/` | HMI model/events, rendering, edit leases, transactions, selection, persistence |
| `include/robot/ui/` | Stable route and parameter IDs |
| `tests/` | PC unit tests, Fake IO, fault injection, replay, and full-chain offline tests |
| `config/` | Human-readable hardware manifest; not a runtime parser input |
| `docs/` | Local implementation, commissioning, validation, and release evidence |
| `.agents/skills/` | Mandatory project skills |
| `build/`, `bin/` | Generated PC and firmware outputs; do not hand-edit or commit unless explicitly tracked |

Use `include/robot/commands/`; the empty legacy `include/robot/command/` directory is not the command implementation.

## Key APIs

Configuration and hardware:

- `make1690XCommissioningConfig()` creates the current locked sample configuration.
- `validateConfig()` checks identity, schema, ports, geometry, transmission, runtime values, voltage, routes, and capability dependencies.
- `DriveIO`, `ControllerIO`, `ControllerDisplayIO`, `CompetitionIO`, and `Clock` define platform boundaries.
- `ProsDriveIO` is the V5 implementation; `FakeDriveIO` is the deterministic PC implementation.
- `StartupSelfCheck` validates configured devices without becoming a second motor writer.

Requests, scheduling, and output:

- `StaticScheduler<N>` manages static commands, requirements, owner tokens, and cancellation.
- `DriveRequestSink` accepts a request only from the current valid owner.
- `DriveRequestArbiter::select()` applies mode, source, capability, lease, epoch, and TTL rules.
- `SafetyGate::apply()` performs final bounds, Slew, derating, and stop semantics.
- `OutputService::tick()` is the sole hardware write point.

Control and motion:

- `ManualDrive::update()` and `CommissioningControlCycle::update()` generate manual requests.
- `Odometry::update()` consumes validated motion inputs and maintains pose quality.
- `EngineeringPid::update()`, `calculateFeedforward()`, and `ChassisVelocityController::update()` provide closed-loop foundations.
- `DriveDistanceCommand`, `TurnToHeadingCommand`, `DriveArcCommand`, and `DriveToPoseCommand` are non-blocking motion primitives.
- `FixedTrajectoryGenerator`, `TrajectoryTracker`, and `AutonomousRouteRegistry` implement the offline autonomous path.

HMI and configuration UI:

- `HmiModel` is the immutable render input.
- `SelectionManager` enforces Draft -> Confirmed -> Locked route selection.
- `ParameterTransaction` stages, applies, or rolls back bounded parameter changes.
- `ControllerPoseRenderer` writes at most one changed Controller row per slow tick.

Prefer `#include "robot/robot.hpp"` for broad consumers. Include a narrow subsystem header in low-level files and tests to keep dependencies explicit.

## How to write code here

1. Read `74000pros.md`, this skill, and every external document required by the affected plan blocks before changing code.
2. Run `git status --short`; preserve unrelated and user-owned changes. Stage only files belonging to the task.
3. Search with `rg`/`rg --files` before adding new types or paths.
4. Keep math, state machines, validation, and control algorithms platform-independent under `include/robot/`.
5. Put PROS calls and non-SI conversions only in `src/platform/pros_adapters.cpp` or the composition boundary.
6. Use bounded arrays/static objects in real-time paths. Do not allocate, perform file I/O, format long strings, refresh displays, or wait for calibration in the 10 ms control tick.
7. Add `FrameHeader`/epoch/TTL to cross-task data and publish immutable snapshots through the existing mailbox/snapshot patterns.
8. Add or update a PC test for pure logic, boundaries, timeouts, invalid values, and recovery paths.
9. Run both PC tests and the ARM PROS build when changing `src/main.cpp`, `src/platform/pros_adapters.cpp`, `include/robot/platform/pros_adapters.hpp`, `include/robot/platform/io.hpp`, `project.pros`, `Makefile`, `common.mk`, firmware metadata, or any declaration that changes the PROS-facing build. Do not normally edit vendor `include/pros/`, `include/liblvgl/`, or `firmware/` content.
10. Keep unverified values explicit and related capabilities false. A successful build is not HIL or field validation.
11. Finish the Git/change-log workflow required by the default `74000pros` skill and update root `CHANGELOG.md` in English.

Never add a second drivetrain writer, call motor APIs from commands/HMI, create blocking autonomous motion loops, amplify with `12 / battery_V`, or bypass the scheduler/arbiter/safety gate.

## PC tests

Run the complete host suite from the project root:

```powershell
cmake -S . -B build/pc -DBUILD_TESTING=ON
cmake --build build/pc --config Release --parallel
ctest --test-dir build/pc -C Release --output-on-failure
```

Important limitation: the CMake target compiles the platform-independent core and tests. It does not prove that `src/main.cpp` or `src/platform/pros_adapters.cpp` compiles for V5.

Add a new test source to `CMakeLists.txt`. Use `ROBOT_TEST`, `ROBOT_REQUIRE`, and `ROBOT_REQUIRE_NEAR` from `tests/test_framework.hpp`. Prefer FakeClock/Fake IO and deterministic timestamps over sleeps.

## PROS firmware build

With the PROS toolchain on `PATH`:

```powershell
make
```

If `make` is not on `PATH`, locate the installed PROS extension toolchain and prepend its `usr/bin` directory. One known local location is under `%APPDATA%\Windsurf\User\globalStorage\sigbots.pros\install\pros-toolchain-windows\usr\bin`; do not hard-code it without checking the current machine.

A successful build creates `bin/hot.package.bin`. It proves ARM compile/link only, not device behavior.

## Fast task checklist

```text
read mandatory skills/docs
-> inspect status and affected files
-> identify plan blocks and capabilities
-> make the smallest architecture-consistent change
-> run focused tests
-> run full PC tests
-> run PROS build when platform-facing code changed
-> commit task files only
-> append one simple English CHANGELOG entry with the commit hash
-> commit the log update separately
-> report unverified HIL/field items
```
