# Project Change Log

This file records implementation commits in simple English. Dedicated `chore(log)` commits are intentionally omitted to avoid recursive log entries.

## 2026-07-21

- 4cdafb0b86d66223d7011bdda6b56e7b74932929 — Redesigned the PC status monitor frontend as a dark engineering-instrument console while preserving all analysis workflows.

- 72a888df1e44944910346b98d4da8b6e22321fe1 — Prevented local flight recordings and generated analysis evidence from being committed or pushed.
- 213b4134f24692ec4e71cbb820dd1701d44b5d3c — Added the guided TF session workflow with a three-choice home, step-by-step import, history, and automatic results.
- df2acd5d8bd0bd2e8ead867dc092665a7a52bacc — Added the Chinese Swiss-grid desktop interface while preserving all PC analysis workflows.
- 63245809280c3f20286663223d6ae3715afc742a — Added the complete PC flight-log import, integrity, analysis, GUI, comparison, replay, reporting, and Windows packaging system.

## 2026-07-20

- 1aa41ea9cff09736a04b82e25236ac39868ed477 — Added schema 3.1 causal trace logging for all currently available recording signals.
- 39cfe0e7610857c92457371450c25e5a99aa44c6 — Added the streaming V5L integrity inspector for TF card recordings.
- 8d09e40f9455a02b290170df4df8b5a8930c39d5 — Added identity-bound V5L2 flight recording, global one-shot Coast events, and offline recovery verification.
- a6a9d4c5fb981f7a4eea9a1c5998a575431ecda1 — Added controller-triggered flight recording with one-shot Coast and recoverable TF card logs.
- 3cc3be479eb41c1d2778e9ddd475757c8cf8c76c — Added the ultimate VEX V5 flight recorder and post-run diagnostics plan.
- dc5ff01ce8a835cc8196c25da4093255b29a5c9c — Added the telemetry status monitor plan document.
- ee3cf1ca398cc76f081c14677e4f1e2258c0e749 — Added the VEX V5 PC status monitoring, recording, analysis, and tuning system implementation plan.

## 2026-07-17

- fb85ec4e0b8866d1e2c84033e6f682e10a6f7f41 — Made the commissioning throttle linear and able to reach full voltage in one nominal control frame.

## 2026-07-16

- 2f2db8e4a9ba508e8cb86d3dfce61c06edc80c73 — Added automatic full-gain Quick Turn with aggressive 50 ms response and removed team identifiers from the README.
- 889825fe28e698502aca16e3d9dfeacea638d2a7 — Switched the 1690X commissioning manual drive from Arcade to Curvature with R1 Quick Turn.

## 2026-07-15

- 4cea04d385c16639334823e223b1b89fd7c25851 — Added the mandatory project guide, enforced it in the default agent rules, and created the English change log.
- 6b19422fdea9ceec24405c5b051b6e4a15704c5c — Added the startup self-check, three-pulse Controller warning, and pose-only Controller display.
- 0ee2cf998312a4ec0c788f700c897eca0fc88978 — Increased the commissioning throttle and output rise rates for faster acceleration.
- 8b76c4a70934022b1fadf619a41d815b0101208d — Removed drive arming and made commissioning stops use Coast.
- 72268f0004dbc46c11d3cfec0ad114c5e001597f — Corrected all six drivetrain directions and enabled the 12 V commissioning limit.
- c96e23e5449d1bf5ccbfe34386f3939b17a18fd7 — Added the restricted 1690X single-stick Arcade commissioning drive.
- 9e1d1c3c18489d6a5a0d2d3aa2ff122a19e251a6 — Recorded the six drivetrain motor ports, gearsets, directions, and transmissions.
- d93e05220e4175fa09c868a44f07e5ec126b5d13 — Recorded the offline release validation evidence.
- 8b573049f1b14e59ff392a2ba5b88471c1f646fd — Added the offline full-chain release validation gate.
- a9b97efb9c9958aaedb837c30a5e51a71c7473f2 — Added latched autonomous failure handling and retry policy.
- a1edb95549f3e5938e1019081d3646c09bb0ab7a — Added fixed command groups and the safe autonomous route registry.
- d8ec0a59638880f76b2f38e2631c871bfc443099 — Added the quality-gated chassis trajectory follower.
- c8cf1ada63d77360d7024d48ecc675718e55a3fe — Added fixed-capacity trajectory generation with motion constraints.
- 5f05826d2bc35de64bc3d62036ca12f951bb238a — Added non-blocking autonomous motion primitives.
- 4ab55d3f80e0431c84eb89e741dd22c27fad23e5 — Added the gated differential wheel-speed controller.
- b28484207e46116750394e53ddc89b54f1aa77ed — Added platform-independent PID, feedforward, profiling, and termination foundations.
- 7335fda510588abbf3c940af3e13c54b0deac2d6 — Added tests that freeze the offline drivetrain safety interfaces.
- 05ad2cc966d1c7568842c303d93ad43587812cd6 — Added transactional Brain and Controller HMI foundations.
- fad6999b2600621e0229f688d5c95e86c31fce28 — Added offline geometry calibration, SysId, and test runner tools.
- 33bca9d514bd2c3ba60f80ae957805f7207ba781 — Added fault hysteresis, motor protection, and output derating.
- f85b9ffef3f2d6ea054b6fedc2b5b3a83fe3bb92 — Added the static scheduler, owner leases, request sinks, and arbitration.
- f0c004199f84c0ab320162fc5613fceec4fcaa38 — Added shaped manual drive, Curvature Drive, Quick Turn, and heading assist.
- 924fabeb9f86fb6533a8ca9329b613981dabeb6e — Added IMU-first odometry for multiple sensor layouts.
- 0290dcd0bd81658a60b0a8639dc442c1f79668bf — Added validated sensor snapshots, filters, and port-level fault isolation.
- 93101a4e38dc2757e43f948b2878d432f7e639d9 — Added drivetrain kinematics, voltage allocation, Slew, and the safety gate.
- e431fecb5b01131bd9ecca979878862a1c893a34 — Added the fixed telemetry ring, Fake IO, and input replay.
- a2550495afe932126d288b517c60b4f474b2b041 — Added epoch-based lifecycle handling and the independent output watchdog.
- ed8859128e05c0266117df9a89194b3cc9bcc0e5 — Added C++17 PROS adapters and the bench self-test contract.
- cd2ce780bee1d781896d79a9e636ba8ece0979c7 — Added validated core contracts and locked capability configuration.
- 22056910ba072278108830f013ab533a977a06a4 — Added the C++17 PC and PROS build and test skeleton.
- 2dd67d87533594951d633fc0df6e02a65efd86a1 — Froze the initial offline hardware and safety contract.
- 06f3f5670bf64322a7b9eec1bd0cbdc6fa2b3421 — Established the project protocol and PROS baseline.
