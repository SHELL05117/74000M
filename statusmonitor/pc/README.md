# VEX Flight Log Status Monitor

This is the PC half of the 74000M VEX V5 flight-recorder system. It imports
V5L2 recordings from a microSD/TF card, verifies integrity before analysis,
archives the original bytes by SHA-256, creates derived columnar data, computes
diagnostic metrics and plots, and emits both a desktop GUI summary and an LLM
evidence pack.

The application never writes robot parameters or commands a robot.

## Desktop interface

Version 0.3 uses a guided Chinese workflow derived from the selected J concept.
The home screen contains only three choices: create a TF recording session,
continue an existing session, or open history. Creation and import use a
three-step form, then integrity checking, analysis, plots, and the LLM evidence
pack are generated in one background operation. The detailed tools remain
available inside the selected session instead of competing for attention on
the home screen.

## Install and run

```powershell
cd statusmonitor\pc
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -e ".[dev]"
.\.venv\Scripts\statusmonitor.exe doctor
.\.venv\Scripts\statusmonitor-gui.exe
```

For a headless workflow:

```powershell
statusmonitor new-session --team 74000M --operator Alex --test-type manual
statusmonitor scan-media E:\
statusmonitor import <session-id> E:\FLIGHT
statusmonitor analyze <run-id>
statusmonitor report <run-id>
```

Artifacts default to `statusmonitor/artifacts`. Raw recordings are copied once,
hashed, and made read-only. Derived files are replaceable because their
algorithm version and source hash are recorded.

## Capability boundary

The current robot schema is V5L2 / LogFrame 3.1 (1536 bytes). Signals whose
availability bits are false are reported as `NOT AVAILABLE`; zeros are never
invented as measurements. A run with an invalid or unavailable pose does not
receive a trajectory-performance pass.
