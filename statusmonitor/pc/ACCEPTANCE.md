# PC application acceptance record

> Authored by OpenAI GPT-5.6 (Codex)
> Scope: PC software only; no real V5 Brain, Controller, motor, or microSD HIL was
> performed in this implementation run.

## Acceptance result

| Area | Evidence | Result |
|---|---|---|
| V5L2 / schema 3.1 ABI | NumPy size assertions; current C++ fixture cross-check | PASS |
| Integrity hard gate | CRC, format/endian/schema/size, block/frame sequence, time, footer, drops, truncation tests | PASS |
| Session identity | SQLite CRUD, required fields, template round-trip, human/robot identity separation | PASS |
| Media import | Recursive V5L/TMP scan, SHA-256 copy verification, read-only raw archive, duplicate binding guard | PASS |
| Large-file behavior | Streaming block verification and Parquet writing; 360,000-frame / 60-minute / 100 Hz stress import | PASS |
| Parquet and index | PyArrow Zstd Parquet + SQLite run/session/analysis index | PASS |
| Segmentation | Deterministic output/request heuristic with method/version recorded | PASS (offline heuristic) |
| Motion analysis | True-timestamp velocity source, robust acceleration/jerk, gap separation | PASS |
| PID gating | Missing PID terms become NOT AVAILABLE; no zero-value inference | PASS |
| Pose gating | Missing/invalid pose forbids trajectory PASS and generates a placeholder | PASS |
| Motor/energy/timing | Six-port statistics, spread, battery/output/derate, dt/exec/jitter/drop metrics | PASS |
| Frequency analysis | FFT, Welch PSD, STFT peak track, coherence with observational limitation | PASS |
| Filter A/B | EMA, moving average, Savitzky–Golay method/causality/delay reporting | PASS |
| Periodic Fourier | Explicit periodic-test + valid-pose gate, harmonic coefficients and residual | PASS |
| SysId | Fixed-vector solver test; runtime requires calibrated linear velocity and train/validation labels | PASS / gated |
| Run comparison | Identity/config-aware summary and delta output, PNG summary | PASS |
| Recorded replay | V5L↔Parquet equivalence and time-indexed causal-chain snapshot | PASS |
| Human GUI | Eight Chinese Swiss-grid pages, numbered navigation/shortcuts, worker-thread import/analysis, interactive Parquet plots, offscreen smoke test | PASS |
| LLM pack | Report, dictionary, timeline, metrics, events, evidence CSVs, SHA-256 manifest | PASS |
| CLI | doctor/session/window/scan/check/import/analyze/compare/replay/report/gui | PASS |
| Windows packaging | PyInstaller one-directory build; packaged EXE offscreen startup/clean exit | PASS |

## 2026-07-21 offline test record

- Ruff: PASS.
- PC tests: 18 PASS, 1 stress test skipped by default.
- Explicit stress test: PASS, 360,000 frames at 100 Hz (60 minutes).
- Packaged executable:
  `dist/VEXFlightStatusMonitor/VEXFlightStatusMonitor.exe`, startup/exit code 0.
- Native PC regression: CMake Release build PASS; CTest 5/5 PASS.
- PROS CLI/ARM rebuild: NOT RUN in this shell because `pros` is not installed
  or on `PATH`. This does not invalidate the PC-only acceptance evidence, and
  no robot firmware source was changed by this work.

## Automated commands

```powershell
.\statusmonitor\pc\.venv\Scripts\python.exe -m ruff check `
  statusmonitor\pc\src statusmonitor\pc\tests

$env:QT_QPA_PLATFORM = "offscreen"
.\statusmonitor\pc\.venv\Scripts\python.exe -m pytest `
  statusmonitor\pc\tests -q

$env:STATUSMONITOR_RUN_STRESS = "1"
.\statusmonitor\pc\.venv\Scripts\python.exe -m pytest `
  statusmonitor\pc\tests\test_stress.py -q

.\statusmonitor\pc\.venv\Scripts\statusmonitor.exe doctor
```

## Explicitly deferred evidence

The following are not PC software defects, but remain release gates from the
ultimate plan:

- 16 GB FAT32 target microSD/TF continuous-write HIL;
- real card removal, full disk, write protection, slow-write and power-loss HIL;
- Controller Y/Left physical timing, vibration, and human-factor validation;
- physical sensor units, pose quality, PID terms, mechanism/pneumatic signals;
- field thresholds, calibration, SysId datasets and parameter approval;
- `HILValidated`, `FieldValidated`, and `CompetitionApproved`.

All of these remain `NOT TESTED`. The PC application does not unlock any robot
capability and never writes candidate parameters to the robot.
